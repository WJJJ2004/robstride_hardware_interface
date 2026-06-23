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

// NOTE: temporal debug send()
bool CanTransport::send(
    uint32_t can_id,
    const std::vector<uint8_t>& data)
{
    if (socket_fd_ < 0)
    {
        errno = ENOTCONN;

        std::fprintf(
            stderr,
            "[CanTransport::send] Invalid socket: "
            "fd=%d errno=%d (%s)\n",
            socket_fd_,
            errno,
            std::strerror(errno));

        return false;
    }

    if (data.size() > 8)
    {
        errno = EINVAL;

        std::fprintf(
            stderr,
            "[CanTransport::send] Invalid CAN data size: "
            "size=%zu errno=%d (%s)\n",
            data.size(),
            errno,
            std::strerror(errno));

        return false;
    }

    struct can_frame frame{};
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = static_cast<uint8_t>(data.size());

    std::memcpy(
        frame.data,
        data.data(),
        data.size());

    std::lock_guard<std::mutex> lock(mutex_);

    const int nbytes =
        ::write(socket_fd_, &frame, sizeof(frame));

    if (nbytes == static_cast<int>(sizeof(frame)))
    {
        return true;
    }

    if (nbytes < 0)
    {
        // write 직후 errno를 반드시 저장
        const int saved_errno = errno;

        std::fprintf(
            stderr,
            "[CanTransport::send] CAN write failed: "
            "fd=%d can_id=0x%08X dlc=%u "
            "nbytes=%d errno=%d (%s)\n",
            socket_fd_,
            frame.can_id,
            static_cast<unsigned>(frame.can_dlc),
            nbytes,
            saved_errno,
            std::strerror(saved_errno));

        // 상위 safeSendCommand()에서도 같은 errno를 확인하도록 복원
        errno = saved_errno;

        return false;
    }

    // CAN frame은 부분 쓰기가 정상적으로 발생하면 안 됨
    errno = EIO;

    std::fprintf(
        stderr,
        "[CanTransport::send] Partial CAN write: "
        "fd=%d can_id=0x%08X dlc=%u "
        "nbytes=%d expected=%zu errno=%d (%s)\n",
        socket_fd_,
        frame.can_id,
        static_cast<unsigned>(frame.can_dlc),
        nbytes,
        sizeof(frame),
        errno,
        std::strerror(errno));

    return false;
}

// // Non-blocking send: drops frame on EAGAIN/EWOULDBLOCK/ENOBUFS instead of blocking.
// // This prevents TX queue congestion from freezing the control loop.
// bool CanTransport::send(uint32_t can_id, const std::vector<uint8_t>& data)
// {
//     if (socket_fd_ < 0)
//     {
//         errno = ENOTCONN;
//         return false;
//     }

//     if (data.size() > 8)
//     {
//         errno = EINVAL;
//         return false;
//     }

//     struct can_frame frame{};
//     frame.can_id = can_id | CAN_EFF_FLAG;
//     frame.can_dlc = static_cast<uint8_t>(data.size());
//     std::memcpy(frame.data, data.data(), data.size());

//     std::lock_guard<std::mutex> lock(mutex_);

//     const int nbytes = write(socket_fd_, &frame, sizeof(frame));
//     if (nbytes == static_cast<int>(sizeof(frame)))
//     {
//         return true;
//     }

//     if (nbytes < 0)
//     {
//         if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)
//         {
//             return false; // Silent drop, don't block
//         }
//         return false;
//     }

//     // write partially succeeded (abnormal, treat as error)
//     errno = EIO;
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
