/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * AP_PrerotaingCan.h
 *
 *      Author: Francisco Ferreira and Tom Pittenger
 */
 
#pragma once

#include <AP_PrerotaingCan/AP_PrerotaingCan_config.h>

#if AP_PrerotaingCan_ENABLED
#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include "AP_Motors/AP_MotorsHeli_RSC.h"

#define AP_PrerotaingCan_USE_EVENTS (defined(CH_CFG_USE_EVENTS) && CH_CFG_USE_EVENTS == TRUE)

#if AP_PrerotaingCan_USE_EVENTS
#include <ch.h>
#endif
#define DEFAULT_NUM_POLES 14

class AP_PrerotaingCan_Driver : public CANSensor
#if HAL_WITH_ESC_TELEM
, public AP_ESC_Telem_Backend
#endif
{
public:
    
    AP_PrerotaingCan_Driver();

    // called from SRV_Channels
    void update(const uint8_t num_poles);
    bool send_pre_rotate_can_packet();
private:

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    bool send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint16_t data);
    bool send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint8_t *data = nullptr, const uint8_t data_len = 0);

    void loop();

    struct {
        uint32_t detected_bitmask;
        uint32_t detected_bitmask_ms;
    } _init;

    union frame_id_t {
        struct PACKED {
            uint8_t object_address;
            uint8_t destination_id;
            uint8_t source_id;
            uint8_t priority:5;
            uint8_t unused:3;
        };
        uint32_t value;
    };
    static const uint8_t AUTOPILOT_NODE_ID = 0;
    static const uint8_t BROADCAST_NODE_ID = 1;
};

class AP_PrerotaingCan{
public:
    AP_PrerotaingCan();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PrerotaingCan);

    static const struct AP_Param::GroupInfo var_info[];
    void init();
    void update();

    static AP_PrerotaingCan *get_singleton() { return _singleton; }

private:
    static AP_PrerotaingCan *_singleton;
    AP_Int8 _num_poles;
    AP_PrerotaingCan_Driver *_driver;
};
namespace AP {
    AP_PrerotaingCan *PrerotaingCan();
};

#endif // AP_PrerotaingCan_ENABLED
