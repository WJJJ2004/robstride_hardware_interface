#include "robstride_rdk_ros2/CanTransport.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <fcntl.h>
#include <cerrno>
#include <thread>

CanTransport::CanTransport() {}

CanTransport::~CanTransport()
{
    close();
}

bool CanTransport::open(const std::string& interface_name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    interface_name_ = interface_name;

    // 소캣 생성
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        perror("Socket creation failed");
        return false;
    }

    // 인터페이스 이름 복사
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    // 복사한 인터페이스 이름으로 인덱스 가져오기
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("Interface index retrieval failed");
        close();
        return false;
    }

    // 소캣 주소 설정 및 바인딩
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("Socket bind failed");
        close();
        return false;
    }

    int recv_own_msgs = 0;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0)
    {
        perror("Failed to set CAN_RAW_RECV_OWN_MSGS");
    }

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0)
    {
        perror("fcntl(F_GETFL) failed");
    }
    else if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        perror("fcntl(F_SETFL, O_NONBLOCK) failed");
    }

    // Increase TX buffer to reduce ENOBUFS under high transmission rates
    int sndbuf_size = 1048576; // 1MB
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size)) < 0)
    {
        perror("Failed to set SO_SNDBUF");
    }

    std::cout << "[CanTransport] Opened " << interface_name << " successfully." << std::endl;
    return true;
}

void CanTransport::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

// Non-blocking send: drops frame on EAGAIN/EWOULDBLOCK/ENOBUFS instead of blocking.
// This prevents TX queue congestion from freezing the control loop.
bool CanTransport::send(uint32_t can_id, const std::vector<uint8_t>& data)
{
    if (socket_fd_ < 0)
    {
        errno = ENOTCONN;
        return false;
    }

    if (data.size() > 8)
    {
        errno = EINVAL;
        return false;
    }

    struct can_frame frame{};
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = static_cast<uint8_t>(data.size());
    std::memcpy(frame.data, data.data(), data.size());

    std::lock_guard<std::mutex> lock(mutex_);

    const int nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes == static_cast<int>(sizeof(frame)))
    {
        return true;
    }

    if (nbytes < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)
        {
            return false; // Silent drop, don't block
        }
        return false;
    }

    // write partially succeeded (abnormal, treat as error)
    errno = EIO;
    return false;
}

// bool CanTransport::send(uint32_t can_id, const std::vector<uint8_t>& data)
// {
//     // 소캣이 열려있는지, 데이터 크기가 8바이트 이하인지 확인 (표준 CAN 프레임은 최대 8바이트)
//     if (socket_fd_ < 0) return false;
//     if (data.size() > 8) {
//         std::cerr << "Error: Data size > 8 bytes" << std::endl;
//         return false;
//     }

//     // CAN 프레임 구성
//     struct can_frame frame;
//     frame.can_id = can_id | CAN_EFF_FLAG; // 항상 확장 프레임 사용
//     frame.can_dlc = static_cast<uint8_t>(data.size());
//     std::memcpy(frame.data, data.data(), data.size());

//     std::lock_guard<std::mutex> lock(mutex_);

//     // ENOBUFS 발생 시 최대 3회 재시도 (CAN TX 버퍼 오버플로 대응)
//     constexpr int max_retries = 3;
//     for (int attempt = 0; attempt < max_retries; attempt++)
//     {
//         int nbytes = write(socket_fd_, &frame, sizeof(frame));
//         if (nbytes == sizeof(frame))
//         {
//             return true;
//         }

//         if (errno == ENOBUFS)
//         {
//             // 버퍼가 가득 찬 경우 잠시 대기 후 재시도
//             std::this_thread::sleep_for(std::chrono::microseconds(500));
//             continue;
//         }
//         else
//         {
//             perror("Write failed");
//             return false;
//         }
//     }

//     std::cerr << "Write failed after " << max_retries << " retries (ENOBUFS)" << std::endl;
//     return false;
// }

bool CanTransport::receive(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms)
{
    if (socket_fd_ < 0) throw std::runtime_error("CAN socket is not open");

    // pollfd 구조체 설정 (감시리스트)
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN; // 읽기 이벤트 감시

    // poll을 사용하여 타임아웃 처리 (CPU 점유율 낮춤)
    int ret = poll(&pfd, 1, timeout_ms);

    if (ret < 0)
    {
        perror("Poll error");
        return false;
    }
    else if (ret == 0)
    {
        return false;
    }

    // revent -> poll()이 채운 실제 발생한 이벤트
    if (pfd.revents & POLLIN)
    {
        struct can_frame frame;
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

        if (nbytes < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return false; 
            }
            perror("Read error");
            return false;
        }

        if (nbytes != static_cast<ssize_t>(sizeof(frame)))
        {
            return false;
        }

        // Verify extended frame format
        if (!(frame.can_id & CAN_EFF_FLAG))
        {
            return false;
        }

        can_id = frame.can_id & CAN_EFF_MASK;
        data.assign(frame.data, frame.data + frame.can_dlc);
        return true;
    }

    return false;
}
