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
 * AP_PrerotaingCan.cpp
 *
 *      Author: Francisco Ferreira and Tom Pittenger
 */

#include "AP_PrerotaingCan.h"

#if AP_PrerotaingCan_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;

#define AP_PrerotaingCan_DEBUG 0

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_PrerotaingCan::var_info[] = {

    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_PrerotaingCan, _num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_PrerotaingCan::AP_PrerotaingCan()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_PrerotaingCan must be singleton");
    }
#endif
    _singleton = this;
}

void AP_PrerotaingCan::init()
{
    if (_driver != nullptr) {
            return;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrerotaingCan: init() started");
    
        for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
            if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::PrerotaingCan) {
                _driver = NEW_NOTHROW AP_PrerotaingCan_Driver();
                if (_driver != nullptr) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrerotaingCan: Driver CREATED on CAN bus %d", i);
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "PrerotaingCan: NEW_NOTHROW failed!");
                }
                return;
            }
        }
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "PrerotaingCan: NO matching protocol found! Check CAN_Dx_PROTOCOL");
}

void AP_PrerotaingCan::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update((uint8_t)_num_poles.get());
}

AP_PrerotaingCan_Driver::AP_PrerotaingCan_Driver() : CANSensor("PrerotaingCan")
{
    register_driver(AP_CAN::Protocol::PrerotaingCan);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrerotaingCan_Driver: Constructor OK, starting thread...");
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PrerotaingCan_Driver::loop, void), "PrerotaingCan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
    
}

// parse inbound frames
void AP_PrerotaingCan_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
            return;
        }
    #if AP_PrerotaingCan_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "PrerotaingCan: RX ext frame ID=0x%08X len=%d", (unsigned)frame.id, frame.dlc);
    #endif

}

void AP_PrerotaingCan_Driver::update(const uint8_t num_poles)
{
    if (_init.detected_bitmask == 0) {
        // nothing to do...
        return;
    }

}
bool AP_PrerotaingCan_Driver::send_pre_rotate_can_packet()
{
    // 指令数据 payload
    const uint8_t payload[8] = {0x00, 0x01, 0x73, 0x18, 0x00, 0x00, 0x00, 0x00};

    // 写法 A：最常见情况 - object_address 放低字节，destination_id 放高字节
    bool sent_ok = send_packet(
        0x03,           // object_address     → 低 8 bit
        0x00,           // destination_id     → 高 8 bit → 形成 0x0300 ? 交换后可能得 0x0003
        2000,           // timeout_us
        payload,
        8
    );
    // 调试输出
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "PreRotate CAN send ID expected 0x0003: %s",
                  sent_ok ? "OK" : "FAILED");

    return sent_ok;
}
void AP_PrerotaingCan_Driver::loop()
{
    while (true) {
            static uint32_t last_send = 0;
            uint32_t now = AP_HAL::millis();
    
            if (pre_rotate_can && (now - last_send >= 300)) {  // 每 300ms 发送一次
                bool success = send_pre_rotate_can_packet();
                last_send = now;
    
                // 可选：只在失败时多打印，或根据需要调整
                if (!success) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PreRotate CAN send failed!");
                }
            }
    
            // 这里可以加其他循环逻辑...
            hal.scheduler->delay(10);  // 避免 CPU 100%（根据需要调整）
        }
}

bool AP_PrerotaingCan_Driver::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint16_t data)
{
    const uint16_t data_be16 = htobe16(data);
    return send_packet(address, dest_id, timeout_us, (uint8_t*)&data_be16, 2);
}

bool AP_PrerotaingCan_Driver::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint8_t *data, const uint8_t data_len)
{
    // broadcast telemetry request frame
    const frame_id_t id {
        {
            .object_address = address,
            .destination_id = dest_id,
            .source_id = AUTOPILOT_NODE_ID,
            .priority = 0,
            .unused = 0
        }
    };

    AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);

    return write_frame(frame, timeout_us);
}

// singleton instance
AP_PrerotaingCan *AP_PrerotaingCan::_singleton;

namespace AP {
AP_PrerotaingCan *PrerotaingCan()
{
    return AP_PrerotaingCan::get_singleton();
}
};

#endif // AP_PrerotaingCan_ENABLED
