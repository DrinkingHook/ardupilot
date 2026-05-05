/*
 * KST CAN Servo Driver for ArduPilot
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>

#ifndef HAL_KST_CAN_ENABLE
#define HAL_KST_CAN_ENABLE 0
#endif

#if HAL_KST_CAN_ENABLE

#define KST_MSG_RATE_HZ_MIN      1
#define KST_MSG_RATE_HZ_MAX      500
#define KST_MSG_RATE_HZ_DEFAULT  50
#define KST_MAX_NUM_SERVO        32

#define KST_BASE_ID              0x600U
#define KST_CMD_BYTE0            0x22
#define KST_CMD_BYTE1            0x03
#define KST_CMD_BYTE2            0x60
#define KST_CMD_BYTE3            0x00
#define KST_PWM_CENTER_US        1500
#define KST_PWM_RANGE_US         500
#define KST_ANGLE_MAX_DEG        100
#define KST_RAW_SCALE            10

struct KST_Servo_t {
    uint16_t command;
    bool     newCommand;
    bool     present;
    bool     enabled;
    uint32_t last_rx_us;
    int16_t  position;
    int16_t  current;
    int16_t  voltage;
    int8_t   temperature;

    KST_Servo_t() :
        command(1500), newCommand(false),
        present(false), enabled(false), last_rx_us(0),
        position(0), current(0), voltage(0), temperature(0) {}

    bool is_connected(uint32_t timeout_us = 2000000) const {
        return present && ((AP_HAL::micros() - last_rx_us) < timeout_us);
    }
};

class AP_KSTCAN : public AP_CANDriver
{
public:
    AP_KSTCAN();
    ~AP_KSTCAN() = default;
    CLASS_NO_COPY(AP_KSTCAN);

    static const struct AP_Param::GroupInfo var_info[];
    static AP_KSTCAN *get_pcan(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface *can_iface) override;
    void update();
    bool pre_arm_check(char *reason, uint8_t reason_len);
    bool is_servo_channel_active(uint8_t chan) const;
    bool is_servo_present(uint8_t chan, uint32_t timeout_us = 2000000) const;
    bool is_servo_enabled(uint8_t chan) const;

private:
    void loop();
    bool write_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us);
    bool read_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us);
    void build_position_cmd(AP_HAL::CANFrame &frame, uint8_t chan, uint16_t pwm_us);
    void send_servo_messages();
    void handle_servo_message(const AP_HAL::CANFrame &frame);

    bool                _initialized;
    uint8_t             _driver_index;
    char                _thread_name[16];
    AP_HAL::CANIface   *_can_iface;
    HAL_BinarySemaphore _sem_handle;
    HAL_Semaphore       _telem_sem;

    KST_Servo_t _servos[KST_MAX_NUM_SERVO];

    AP_Int32 _srv_bm;
    AP_Int16 _srv_hz;
};
#endif // HAL_KST_CAN_ENABLE
