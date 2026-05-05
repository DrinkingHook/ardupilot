/*
 * KST CAN Servo Driver for ArduPilot
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_KSTCAN.h"
#include <AP_CANManager/AP_CANManager.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#if HAL_KST_CAN_ENABLE

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "KST_CAN", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

const AP_Param::GroupInfo AP_KSTCAN::var_info[] = {
    // @Param: SRV_BM
    // @DisplayName: Servo channel bitmask
    // @Description: Bitmask of servo channels to transmit over KST CAN
    // @Bitmask: 0:Servo1,1:Servo2,...,31:Servo32
    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 1, AP_KSTCAN, _srv_bm, 0xFFFF),

    // @Param: SRV_RT
    // @DisplayName: Servo command rate
    // @Description: Rate at which servo position commands are transmitted
    // @Units: Hz
    // @Range: 1 500
    // @User: Advanced
    AP_GROUPINFO("SRV_RT", 2, AP_KSTCAN, _srv_hz, KST_MSG_RATE_HZ_DEFAULT),

    AP_GROUPEND
};

AP_KSTCAN::AP_KSTCAN() :
    _initialized(false),
    _driver_index(0),
    _can_iface(nullptr)
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(_thread_name, 0, sizeof(_thread_name));
}

AP_KSTCAN *AP_KSTCAN::get_pcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::KSTCAN) {
        return nullptr;
    }
    return static_cast<AP_KSTCAN *>(AP::can().get_driver(driver_index));
}

bool AP_KSTCAN::add_interface(AP_HAL::CANIface *can_iface)
{
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "multiple interfaces not supported\n\r");
        return false;
    }
    _can_iface = can_iface;
    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN driver not found\n\r");
        return false;
    }
    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN driver not initialized\n\r");
        return false;
    }
    if (!_can_iface->set_event_handle(&_sem_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "cannot add event handle\n\r");
        return false;
    }
    return true;
}

void AP_KSTCAN::init(uint8_t driver_index, bool /*enable_filters*/)
{
    _driver_index = driver_index;
    debug_can(AP_CANManager::LOG_DEBUG, "KST_CAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "already initialized\n\r");
        return;
    }

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_KSTCAN::loop, void),
            _thread_name, 4096,
            AP_HAL::Scheduler::PRIORITY_CAN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "failed to create thread\n\r");
        return;
    }
    _initialized = true;

    snprintf(_thread_name, sizeof(_thread_name), "KST_CAN%u", driver_index);

    debug_can(AP_CANManager::LOG_DEBUG, "KST_CAN: init done\n\r");
}

void AP_KSTCAN::loop()
{
    uint16_t servo_tx_counter = 0;

    while (true) {
        if (!_initialized) {
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        _srv_hz.set(constrain_int16(_srv_hz, KST_MSG_RATE_HZ_MIN, KST_MSG_RATE_HZ_MAX));
        const uint16_t servo_rate_ms = 1000 / _srv_hz;

        hal.scheduler->delay_microseconds(1000);

        if (servo_tx_counter++ >= servo_rate_ms) {
            servo_tx_counter = 0;
            send_servo_messages();
        }

        AP_HAL::CANFrame rx_frame {};
        while (read_frame(rx_frame, 0)) {
            if (rx_frame.id & AP_HAL::CANFrame::FlagEFF) { continue; }
            const uint32_t id = rx_frame.id & 0x7FFU;
            if (id >= 0x581U && id <= 0x590U) { handle_servo_message(rx_frame); }
        }
    }
}

bool AP_KSTCAN::write_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us)
{
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }
    frame.id &= ~AP_HAL::CANFrame::FlagEFF;

    bool read_select  = false;
    bool write_select = true;
    const uint64_t deadline = AP_HAL::micros64() + timeout_us;

    if (!_can_iface->select(read_select, write_select, &frame, deadline) || !write_select) {
        return false;
    }
    return (_can_iface->send(frame, deadline, AP_HAL::CANIface::AbortOnError) == 1);
}

bool AP_KSTCAN::read_frame(AP_HAL::CANFrame &frame, uint32_t timeout_us)
{
    if (!_initialized || _can_iface == nullptr) {
        return false;
    }
    bool read_select  = true;
    bool write_select = false;
    if (!_can_iface->select(read_select, write_select, nullptr,
                             AP_HAL::micros64() + timeout_us) || !read_select) {
        return false;
    }
    uint64_t ts {};
    AP_HAL::CANIface::CanIOFlags flags {};
    return (_can_iface->receive(frame, ts, flags) == 1);
}

static int16_t pwm_to_kst_raw(uint16_t pwm_us)
{
    // PWM 1000~2000us -> raw -1000~+1000 (0.1 deg/LSB, int16)
    return (int16_t)constrain_int32(((int32_t)pwm_us - 1500) * 2, -1000, 1000);
}

void AP_KSTCAN::build_position_cmd(AP_HAL::CANFrame &frame, uint8_t chan, uint16_t pwm_us)
{
    const uint8_t node_id = chan + 1;
    const int16_t raw     = pwm_to_kst_raw(pwm_us);
    frame.id      = KST_BASE_ID + node_id;
    frame.dlc     = 8;
    frame.data[0] = KST_CMD_BYTE0;
    frame.data[1] = KST_CMD_BYTE1;
    frame.data[2] = KST_CMD_BYTE2;
    frame.data[3] = KST_CMD_BYTE3;
    frame.data[4] = static_cast<uint8_t>(raw & 0xFF);
    frame.data[5] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
}

void AP_KSTCAN::send_servo_messages()
{
    if (_srv_bm == 0) {
        return;
    }
    AP_HAL::CANFrame tx_frame {};
    for (uint8_t ii = 0; ii < KST_MAX_NUM_SERVO; ii++) {
        if (!is_servo_channel_active(ii)) {
            continue;
        }
        // Always send - servo response sets present=true for pre-arm check
        uint16_t pwm = _servos[ii].command;
        const SRV_Channel::Aux_servo_function_t fn = SRV_Channels::channel_function(ii);
        SRV_Channels::get_output_pwm(fn, pwm);
        build_position_cmd(tx_frame, ii, pwm);
        if (write_frame(tx_frame, 1000)) {
            _servos[ii].newCommand = false;
        }
    }
}

void AP_KSTCAN::handle_servo_message(const AP_HAL::CANFrame &frame)
{
    const uint32_t id = frame.id & 0x7FFU;
    const uint8_t node_id = static_cast<uint8_t>(id - 0x580U);
    if (node_id == 0 || node_id > KST_MAX_NUM_SERVO) {
        return;
    }
    const uint8_t chan = node_id - 1;
    WITH_SEMAPHORE(_telem_sem);
    KST_Servo_t &srv = _servos[chan];
    srv.present    = true;
    srv.enabled    = true;
    srv.last_rx_us = AP_HAL::micros();
    if (frame.dlc >= 1 && frame.data[0] == 0x80) {
        debug_can(AP_CANManager::LOG_WARNING, "Servo %u SDO error response\n\r", node_id);
    }
}

void AP_KSTCAN::update()
{
    for (uint8_t ii = 0; ii < KST_MAX_NUM_SERVO; ii++) {
        if (!is_servo_channel_active(ii)) { continue; }
        const SRV_Channel::Aux_servo_function_t fn = SRV_Channels::channel_function(ii);
        uint16_t pwm = 0;
        if (SRV_Channels::get_output_pwm(fn, pwm)) {
            _servos[ii].command = pwm;
            _servos[ii].newCommand = true;
        }
    }
}

bool AP_KSTCAN::is_servo_channel_active(uint8_t chan) const
{
    if (chan >= KST_MAX_NUM_SERVO) return false;
    if (!((_srv_bm >> chan) & 0x01)) return false;
    const SRV_Channel::Aux_servo_function_t fn = SRV_Channels::channel_function(chan);
    if (fn <= SRV_Channel::k_none) return false;
    return true;
}

bool AP_KSTCAN::is_servo_present(uint8_t chan, uint32_t timeout_us) const
{
    if (chan >= KST_MAX_NUM_SERVO) return false;
    return _servos[chan].is_connected(timeout_us);
}

bool AP_KSTCAN::is_servo_enabled(uint8_t chan) const
{
    if (chan >= KST_MAX_NUM_SERVO) return false;
    if (!is_servo_present(chan)) return false;
    return _servos[chan].enabled;
}

bool AP_KSTCAN::pre_arm_check(char *reason, uint8_t reason_len)
{
   // for (uint8_t ii = 0; ii < KST_MAX_NUM_SERVO; ii++) {
   //     if (is_servo_channel_active(ii) && !is_servo_present(ii)) {
   //         snprintf(reason, reason_len, "KST Servo %u not detected on CAN", ii + 1);
   //         return false;
   //     }
   // }
   return true;
}
#endif // HAL_KST_CAN_ENABLE
