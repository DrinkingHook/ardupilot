/*
 * AP_Gripper_DroneCAN.cpp
 *
 *  Gripper backend using DroneCAN uavcan.equipment.hardpoint.Command
 */

#include "AP_Gripper_DroneCAN.h"

#if AP_GRIPPER_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_CANManager/AP_CANManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

AP_Gripper_DroneCAN::AP_Gripper_DroneCAN(struct AP_Gripper::Backend_Config &_config) :
    AP_Gripper_Backend(_config),
    _has_dronecan(false)
{
}

void AP_Gripper_DroneCAN::init_gripper()
{
    // check if any DroneCAN driver exists
    uint8_t can_num_drivers = AP::can().get_num_drivers();

    for (uint8_t i = 0; i < can_num_drivers; i++) {
        if (AP_DroneCAN::get_dronecan(i) != nullptr) {
            _has_dronecan = true;
            break;
        }
    }
}

void AP_Gripper_DroneCAN::send_command(uint16_t command)
{
    uavcan_equipment_hardpoint_Command msg {};
    msg.hardpoint_id = config.uavcan_hardpoint_id;
    msg.command = command;

    uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto *dronecan = AP_DroneCAN::get_dronecan(i);
        if (dronecan != nullptr) {
            dronecan->gripper_hardpoint.broadcast(msg);
        }
    }
}

void AP_Gripper_DroneCAN::grab()
{
    if (config.state == AP_Gripper::STATE_GRABBING ||
        config.state == AP_Gripper::STATE_GRABBED) {
        return;
    }

    config.state = AP_Gripper::STATE_GRABBING;
    _last_grab_or_release = AP_HAL::millis();

    send_command(1);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gripper load grabbing (DroneCAN)");
    LOGGER_WRITE_EVENT(LogEvent::GRIPPER_GRAB);
}

void AP_Gripper_DroneCAN::release()
{
    if (config.state == AP_Gripper::STATE_RELEASING ||
        config.state == AP_Gripper::STATE_RELEASED) {
        return;
    }

    config.state = AP_Gripper::STATE_RELEASING;
    _last_grab_or_release = AP_HAL::millis();

    send_command(0);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gripper load releasing (DroneCAN)");
    LOGGER_WRITE_EVENT(LogEvent::GRIPPER_RELEASE);
}

void AP_Gripper_DroneCAN::update_gripper()
{
    // after timeout, transition to final state
    if (AP_HAL::millis() - _last_grab_or_release > GRIPPER_DRONECAN_ACTUATION_MS) {
        if (config.state == AP_Gripper::STATE_GRABBING) {
            config.state = AP_Gripper::STATE_GRABBED;
        } else if (config.state == AP_Gripper::STATE_RELEASING) {
            config.state = AP_Gripper::STATE_RELEASED;
        }
    }
}

bool AP_Gripper_DroneCAN::valid() const
{
    if (_has_dronecan) {
        return true;
    }
    // also check dynamically in case CAN was not ready at init
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        if (AP_DroneCAN::get_dronecan(i) != nullptr) {
            return true;
        }
    }
    return false;
}

bool AP_Gripper_DroneCAN::grabbed() const
{
    return config.state == AP_Gripper::STATE_GRABBED;
}

bool AP_Gripper_DroneCAN::released() const
{
    return config.state == AP_Gripper::STATE_RELEASED;
}

#endif // AP_GRIPPER_DRONECAN_ENABLED
