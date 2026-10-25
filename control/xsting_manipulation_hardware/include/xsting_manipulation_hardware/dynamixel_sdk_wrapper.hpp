#ifndef XSTING_MANIPULATION_HARDWARE__DYNAMIXEL_SDK_WRAPPER_HPP_
#define XSTING_MANIPULATION_HARDWARE__DYNAMIXEL_SDK_WRAPPER_HPP_

#include <cstdint>
#include <string>
#include <mutex>
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace xsting_manipulation_hardware
{
class DynamixelSDKWrapper
{
public:
    explicit DynamixelSDKWrapper(const uint8_t & id);
    virtual ~DynamixelSDKWrapper();

    // inline void error_print(const char * log);
    bool open_port(const std::string & usb_port);
    bool set_baud_rate(const uint32_t & baud_rate);
    uint16_t ping();
    bool read(
        const uint16_t & address,
        const uint16_t & length,
        uint8_t * data);
    uint8_t read_byte(const uint16_t & address);
    bool write(
        const uint16_t & address,
        const uint16_t & length,
        uint8_t * data);
    void write_byte(const uint16_t & address, uint8_t data);

private:
    uint8_t id_;
    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
    std::mutex sdk_handler_m_;
};
}

#endif