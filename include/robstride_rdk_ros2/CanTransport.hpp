#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>

struct CanTxFrameData
{
    uint32_t can_id{0};
    std::vector<uint8_t> data;
    uint16_t motor_id{0};
    const char* operation{"UNKNOWN"};
};

struct CanBatchSendResult
{
    std::size_t requested{0};
    std::size_t queued{0};
    int error_number{0};
};

class CanTransport {
public:
    CanTransport();
    ~CanTransport();

    // CAN 인터페이스 열기
    bool open(const std::string& interface_name);
    void close();

    // 데이터 전송
    bool send(const CanTxFrameData& frame_data);
    CanBatchSendResult sendBatch(const std::vector<CanTxFrameData>& frames);

    // 데이터 수신
    bool receive(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms = 10);

    bool isOpen() const { return socket_fd_ >= 0; }
    const std::string& getInterfaceName() const { return interface_name_; }

private:
    void closeUnlocked();

    int socket_fd_ = -1;
    std::string interface_name_;
    std::mutex mutex_;
};
