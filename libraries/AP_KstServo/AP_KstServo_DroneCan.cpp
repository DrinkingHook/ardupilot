#include <AP_HAL/AP_HAL.h>
#include <AP_DroneCAN/AP_DroneCAN.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include "AP_KstServo_DroneCan.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_KstServo_DroneCan *AP_KstServo_DroneCan::instance;

AP_KstServo_DroneCan::AP_KstServo_DroneCan()
{
    instance = this;
    memset(&state, 0, sizeof(state));
}

void AP_KstServo_DroneCan::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (instance == nullptr) {
        instance = new AP_KstServo_DroneCan();
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &trampoline_servo_status, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("servo_status_sub");
    }
}

void AP_KstServo_DroneCan::trampoline_servo_status(AP_DroneCAN *ap_dronecan,
                                                     const CanardRxTransfer& transfer,
                                                     const ardupilot_equipment_kstservo_ServoStatus &msg)
{
    if (instance == nullptr) {
        return;
    }
    instance->handle_servo_status(msg);
}

void AP_KstServo_DroneCan::handle_servo_status(const ardupilot_equipment_kstservo_ServoStatus &pkt)
{
    state.count = pkt.count;
    for (uint8_t i = 0; i < 8; i++) {
        state.voltage[i] = pkt.voltage[i];
        state.current[i] = pkt.current[i];
        state.power[i] = pkt.power[i];
    }
    state.last_updated_ms = AP_HAL::millis();

    GCS_SEND_MESSAGE(MSG_SERVO_STATUS);
}

void AP_KstServo_DroneCan::send_mavlink_servo_status(mavlink_channel_t chan)
{
    mavlink_msg_servo_status_send(
        chan,
        state.voltage,
        state.current,
        state.power,
        state.count
    );
}

#endif // HAL_ENABLE_DRONECAN_DRIVERS
