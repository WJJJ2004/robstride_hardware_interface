#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>

class CanTransport {
public:
    CanTransport();
    ~CanTransport();

    // CAN 인터페이스 열기
    bool open(const std::string& interface_name);
    void close();

    // 데이터 전송
    bool send(uint32_t can_id, const std::vector<uint8_t>& data);

    // 데이터 수신
    bool receive(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms = 10);

    bool isOpen() const { return socket_fd_ >= 0; }

private:
    void closeUnlocked();

    int socket_fd_ = -1;
    std::string interface_name_;
    std::mutex mutex_;
};
