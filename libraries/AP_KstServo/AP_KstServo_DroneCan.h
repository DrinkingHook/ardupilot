#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_DroneCAN/AP_DroneCAN.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

struct Servo_Status_State {
    uint32_t last_updated_ms;
    uint8_t count;
    uint16_t voltage[8];
    uint16_t current[8];
    uint16_t power[8];
};

class AP_KstServo_DroneCan {
public:
    AP_KstServo_DroneCan();

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    static void trampoline_servo_status(AP_DroneCAN *ap_dronecan,
                                         const CanardRxTransfer& transfer,
                                         const ardupilot_equipment_kstservo_ServoStatus &msg);

    void handle_servo_status(const ardupilot_equipment_kstservo_ServoStatus &pkt);

    const Servo_Status_State& get_state() const { return state; }

    void send_mavlink_servo_status(mavlink_channel_t chan);

    static AP_KstServo_DroneCan* get_instance() { return instance; }

private:
    Servo_Status_State state;
    static AP_KstServo_DroneCan *instance;
};

#endif // HAL_ENABLE_DRONECAN_DRIVERS
