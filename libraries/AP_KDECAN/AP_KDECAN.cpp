// /*
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//  */
// /*
//  * AP_KDECAN.cpp
//  *
//  *      Author: Francisco Ferreira and Tom Pittenger
//  */

// #include "AP_KDECAN.h"

// #if AP_KDECAN_ENABLED
// #include <stdio.h>
// #include <AP_BoardConfig/AP_BoardConfig.h>
// #include <AP_HAL/utility/sparse-endian.h>
// #include <SRV_Channel/SRV_Channel.h>
// #include <GCS_MAVLink/GCS.h>
// #include <AP_Math/AP_Math.h>    // for MIN,MAX

// #include <AP_GPS/AP_GPS.h>
// #include <AP_AHRS/AP_AHRS.h>
// extern const AP_HAL::HAL& hal;

// #define AP_KDECAN_DEBUG 0

// // 初始化全局变量-----------------------------------------------------------------------------------------------------------------------------------
// uint8_t AP_KDECANUSE::mode_number = 0;
// bool AP_KDECANUSE::RC_failsafe = false;

// uint16_t AP_KDECANUSE::LD1 = 0;
// uint16_t AP_KDECANUSE::LD2 = 0;
// uint16_t AP_KDECANUSE::LD3 = 0;
// uint16_t AP_KDECANUSE::LD4 = 0;
// uint16_t AP_KDECANUSE::LD5 = 0;
// uint16_t AP_KDECANUSE::LD6 = 0;
// uint16_t AP_KDECANUSE::LD7 = 0;
// uint16_t AP_KDECANUSE::LD8 = 0;

// uint16_t AP_KDECANUSE::battery_V = 0;
// uint16_t AP_KDECANUSE::battery_A = 0;
// uint16_t AP_KDECANUSE::battery_T = 0;
// uint8_t AP_KDECANUSE::battery_bai = 0;
// uint8_t AP_KDECANUSE::VCU_status = 0;

// uint16_t AP_KDECANUSE::int1 = 0;
// uint16_t AP_KDECANUSE::int2 = 0;
// uint16_t AP_KDECANUSE::int3 = 0;
// uint16_t AP_KDECANUSE::int4 = 0;
// uint16_t AP_KDECANUSE::int5 = 0;
// uint16_t AP_KDECANUSE::int6 = 0;
// uint16_t AP_KDECANUSE::int7 = 0;
// uint16_t AP_KDECANUSE::int8 = 0;
// uint16_t AP_KDECANUSE::int9 = 0;
// uint16_t AP_KDECANUSE::int10 = 0;
// uint16_t AP_KDECANUSE::int11 = 0;
// uint16_t AP_KDECANUSE::int12 = 0;
// uint16_t AP_KDECANUSE::int13 = 0;
// uint16_t AP_KDECANUSE::int14 = 0;
// uint16_t AP_KDECANUSE::int15 = 0;
// uint16_t AP_KDECANUSE::int16 = 0;

// uint16_t AP_KDECANUSE::qgc_read1 = 0;
// uint16_t AP_KDECANUSE::qgc_read2 = 0;
// uint16_t AP_KDECANUSE::qgc_read3 = 0;
// uint16_t AP_KDECANUSE::qgc_read4 = 0;
// uint16_t AP_KDECANUSE::qgc_read5 = 0;
// uint16_t AP_KDECANUSE::qgc_read6 = 0;
// uint16_t AP_KDECANUSE::qgc_read7 = 0;
// uint16_t AP_KDECANUSE::qgc_read8 = 0;

// uint16_t AP_KDECANUSE::qgc_send1 = 0;
// uint16_t AP_KDECANUSE::qgc_send2 = 0;
// uint16_t AP_KDECANUSE::qgc_send3 = 0;
// uint16_t AP_KDECANUSE::qgc_send4 = 0;
// float AP_KDECANUSE::qgc_send5 = 0;
// float AP_KDECANUSE::qgc_send6 = 0;
// float AP_KDECANUSE::qgc_send7 = 0;
// float AP_KDECANUSE::qgc_send8 = 0;
// float AP_KDECANUSE::qgc_send9 = 0;
// float AP_KDECANUSE::qgc_send10 = 0;
// float AP_KDECANUSE::qgc_send11 = 0;
// float AP_KDECANUSE::qgc_send12 = 0;

// uint16_t AP_KDECANUSE::CAN_HZ = 0;

// uint8_t add_het = 0;

// // table of user settable CAN bus parameters
// const AP_Param::GroupInfo AP_KDECAN::var_info[] = {

//     // @Param: NPOLE
//     // @DisplayName: Number of motor poles
//     // @Description: Sets the number of motor poles to calculate the correct RPM value
//     AP_GROUPINFO("NPOLE", 1, AP_KDECAN, _num_poles, DEFAULT_NUM_POLES),

//     AP_GROUPEND
// };

// AP_KDECAN::AP_KDECAN()
// {
//     AP_Param::setup_object_defaults(this, var_info);
// #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
//     if (_singleton != nullptr) {
//         AP_HAL::panic("AP_KDECAN must be singleton");
//     }
// #endif
//     _singleton = this;
// }

// void AP_KDECAN::init()
// {
//     if (_driver != nullptr) {
//         // only allow one instance
//         return;
//     }

//     for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
//         if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::KDECAN) {
//             _driver = NEW_NOTHROW AP_KDECAN_Driver();
//             return;
//         }
//     }
// }

// void AP_KDECAN::update()
// {
//     if (_driver == nullptr) {
//         return;
//     }
//     _driver->update((uint8_t)_num_poles.get());
// }

// AP_KDECAN_Driver::AP_KDECAN_Driver() : CANSensor("KDECAN")
// {
//     register_driver(AP_CAN::Protocol::KDECAN);

//     // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
//     hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_KDECAN_Driver::loop, void), "kdecan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
// }

// // parse inbound frames
// void AP_KDECAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
// {
//     if (!frame.isExtended()) {
//         return;
//     }

//     const frame_id_t id { .value = frame.id & AP_HAL::CANFrame::MaskStdID }; // 获取帧ID

//     switch (id.object_address)
//     {          // 向飞控或地面站发送的数据------------------------------------------------------------------
//     case 0xBA: // 雷达数据   参数：雷达类型选择TeraRangerTower
//         AP_KDECANUSE::LD1 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::LD2 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::LD3 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::LD4 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xBB: // 雷达数据
//         AP_KDECANUSE::LD5 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::LD6 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::LD7 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::LD8 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xBC:                                                          // 电池数据
//         AP_KDECANUSE::battery_V = (frame.data[0] << 8) | frame.data[1]; // 获取电池电压
//         AP_KDECANUSE::battery_A = (frame.data[2] << 8) | frame.data[3]; // 获取电池电流
//         AP_KDECANUSE::battery_T = (frame.data[4] << 8) | frame.data[5]; // 获取电池温度
//         AP_KDECANUSE::battery_bai = frame.data[6];                      // 获取电池电量
//         AP_KDECANUSE::VCU_status = frame.data[7];                       // 获取VCU状态
//         break;
//     case 0xBD: // 接收发动机温度数据1～4
//         AP_KDECANUSE::int1 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::int2 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::int3 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::int4 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xBE: // 接收发动机温度数据5～8
//         AP_KDECANUSE::int5 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::int6 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::int7 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::int8 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xBF: // 自定义数据，16位
//         AP_KDECANUSE::int9 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::int10 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::int11 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::int12 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xC1: // 上传浮点数，65535转655.35
//         AP_KDECANUSE::int13 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::int14 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::int15 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::int16 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xd1:
//         AP_KDECANUSE::qgc_send1 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::qgc_send2 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::qgc_send3 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::qgc_send4 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xd2:
//         AP_KDECANUSE::qgc_send5 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::qgc_send6 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::qgc_send7 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::qgc_send8 = (frame.data[6] << 8) | frame.data[7];
//         break;
//     case 0xd3:
//         AP_KDECANUSE::qgc_send9 = (frame.data[0] << 8) | frame.data[1];
//         AP_KDECANUSE::qgc_send10 = (frame.data[2] << 8) | frame.data[3];
//         AP_KDECANUSE::qgc_send11 = (frame.data[4] << 8) | frame.data[5];
//         AP_KDECANUSE::qgc_send12 = (frame.data[6] << 8) | frame.data[7];
//         break;

//         // #if AP_KDECAN_DEBUG
//         //     if (id.object_address != TELEMETRY_OBJ_ADDR) {
//         //         GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"KDECAN: rx id:%d, src:%d, dest:%d, len:%d", (int)id.object_address, (int)id.source_id, (int)id.destination_id, (int)frame.dlc);
//         //     }
//         // #endif

//         //     if (id.destination_id != AUTOPILOT_NODE_ID || id.source_id < ESC_NODE_ID_FIRST) {
//         //         // not for us or invalid id (0 and 1 are invalid)
//         //         return;
//         //     }

//         //     // check if frame is valid: directed at autopilot, doesn't come from broadcast and ESC was detected before
//         //     switch (id.object_address) {
//         //         case ESC_INFO_OBJ_ADDR:
//         //             if (frame.dlc == 5 &&
//         //                 (id.source_id < (ARRAY_SIZE(_output.pwm) + ESC_NODE_ID_FIRST)))
//         //             {
//         //                 if (__builtin_popcount(_init.detected_bitmask) >= KDECAN_MAX_NUM_ESCS) {
//         //                     // we already have the maximum number of ESCs
//         //                     return;
//         //                 }
//         //                 const uint16_t bitmask = (1UL << (id.source_id - ESC_NODE_ID_FIRST));

//         //                 if ((bitmask & _init.detected_bitmask) != bitmask) {
//         //                     _init.detected_bitmask |= bitmask;
//         //                     GCS_SEND_TEXT(MAV_SEVERITY_INFO,"KDECAN: Found ESC id %u mapped to SERVO%u", id.source_id, id.source_id-1);
//         //                 }
//         //             }
//         //         break;

//         // #if HAL_WITH_ESC_TELEM
//         //         case TELEMETRY_OBJ_ADDR:
//         //             if (frame.dlc == 8 &&
//         //                 (1UL << (id.source_id - ESC_NODE_ID_FIRST) & _init.detected_bitmask))
//         //             {
//         //                 const uint8_t idx = id.source_id - ESC_NODE_ID_FIRST;
//         //                 const uint8_t num_poles = _telemetry.num_poles > 0 ? _telemetry.num_poles : DEFAULT_NUM_POLES;
//         //                 update_rpm(idx, uint16_t(uint16_t(frame.data[4] << 8 | frame.data[5]) * 60UL * 2 / num_poles));

//         //                 const TelemetryData t {
//         //                     .temperature_cdeg = int16_t(frame.data[6] * 100),
//         //                     .voltage = float(uint16_t(frame.data[0] << 8 | frame.data[1])) * 0.01f,
//         //                     .current = float(uint16_t(frame.data[2] << 8 | frame.data[3])) * 0.01f,
//         //                 };
//         //                 update_telem_data(idx, t,
//         //                     AP_ESC_Telem_Backend::TelemetryType::CURRENT |
//         //                     AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
//         //                     AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
//         //             }
//         //             break;
//         // #endif // HAL_WITH_ESC_TELEM
//         //     }
//     }
// }
// void AP_KDECAN_Driver::update(const uint8_t num_poles)
// {
//     if (_init.detected_bitmask == 0) {
//         // nothing to do...
//         return;
//     }

// #if HAL_WITH_ESC_TELEM
//     _telemetry.num_poles = num_poles;
// #endif
    
// //     WITH_SEMAPHORE(_output.sem);
// //     for (uint8_t i = 0; i < ARRAY_SIZE(_output.pwm); i++) {
// //         if ((_init.detected_bitmask & (1UL<<i)) == 0 || SRV_Channels::channel_function(i) <= SRV_Channel::Function::k_none) {
// //             _output.pwm[i] = 0;
// //             continue;
// //         }

// //         const SRV_Channel *c = SRV_Channels::srv_channel(i);
// //         if (c == nullptr) {
// //             _output.pwm[i] = 0;
// //             continue;
// //         }
// //         _output.pwm[i] = c->get_output_pwm();
// //     }

// //     _output.is_new = true;

// // #if AP_KDECAN_USE_EVENTS
// //     if (_output.thread_ctx != nullptr) {
// //         // trigger the thread to wake up immediately
// //         chEvtSignal(_output.thread_ctx, 1);
// //     }
// // #endif

// // #if AP_KDECAN_DEBUG
// //     static uint32_t last_send_ms = 0;
// //     const uint32_t now_ms = AP_HAL::millis();
// //     if (now_ms - last_send_ms > 1000) {
// //         last_send_ms = now_ms;
// //         GCS_SEND_TEXT(MAV_SEVERITY_INFO,"%u: %u, %u, %u, %u, %u, %u, %u, %u",
// //         (unsigned)_init.detected_bitmask,
// //         (unsigned)_output.pwm[0], (unsigned)_output.pwm[1], (unsigned)_output.pwm[2], (unsigned)_output.pwm[3],
// //         (unsigned)_output.pwm[4], (unsigned)_output.pwm[5], (unsigned)_output.pwm[6], (unsigned)_output.pwm[7]);
// //     }
// // #endif
// }

// void AP_KDECAN_Driver::loop()
// {
//     uint16_t pwm[ARRAY_SIZE(_output.pwm)] {};

// #if AP_KDECAN_USE_EVENTS
//     _output.thread_ctx = chThdGetSelfX();
// #endif

//     uint8_t broadcast_esc_info_boot_spam_count = 3;
//     uint32_t broadcast_esc_info_next_interval_ms = 100; // spam a few at boot at this rate

//     while (true) {
// #if AP_KDECAN_USE_EVENTS
//         // sleep until we get new data, but also wake up at 400Hz to send the old data again
//         chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
//  #else
//         hal.scheduler->delay_microseconds(2500); // 400Hz
// #endif

//         const uint32_t now_ms = AP_HAL::millis();

//         // This should run at 400Hz
//         {
//             WITH_SEMAPHORE(_output.sem);
//             if (_output.is_new) {
//                 _output.last_new_ms = now_ms;
//                 _output.is_new = false;
//                 memcpy(&pwm, &_output.pwm, sizeof(pwm));

//             } else if (_output.last_new_ms && now_ms - _output.last_new_ms > 1000) {
//                 // if we haven't gotten any PWM updates for a bit, zero it
//                 // out so we don't just keep sending the same values forever
//                 memset(&pwm, 0, sizeof(pwm));
//                 _output.last_new_ms = 0;
//             }
//         }

//         for (uint8_t i=0; i<ARRAY_SIZE(_output.pwm); i++) {
//             if ((_init.detected_bitmask & (1UL<<i)) != 0) {
//                 send_packet_uint16(SET_PWM_OBJ_ADDR, (i + ESC_NODE_ID_FIRST), 1000, pwm[i]);
//             }
//         }

// #if HAL_WITH_ESC_TELEM
//         // broadcast as request-telemetry msg to everyone
//         if (_init.detected_bitmask != 0 && now_ms - _telemetry.timer_ms >= TELEMETRY_INTERVAL_MS) {
//             if (send_packet(TELEMETRY_OBJ_ADDR, BROADCAST_NODE_ID, 10000)) {
//                 _telemetry.timer_ms = now_ms;
//             }
//         }
// #endif // HAL_WITH_ESC_TELEM

//         if ((_init.detected_bitmask == 0 || broadcast_esc_info_boot_spam_count > 0) && (now_ms - _init.detected_bitmask_ms >= broadcast_esc_info_next_interval_ms)) {
//             // broadcast an "anyone there?" quick at boot but then 1Hz forever until we see at least 1 esc respond
//             if (broadcast_esc_info_boot_spam_count > 0) {
//                 broadcast_esc_info_boot_spam_count--;
//             } else {
//                 broadcast_esc_info_next_interval_ms = 1000;
//             }

//             // if (send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100000)) {
//             //     _init.detected_bitmask_ms = now_ms;
//             // }
//             //-------------------------------------------------发送CAN消息-------------------------------------------------
//             // 使用信号量保护输出数组
//             WITH_SEMAPHORE(_output.sem); //
//             for (uint8_t i = 0; i < ARRAY_SIZE(_output.pwm); i++)
//             {

//                 // 获取当前通道的指针
//                 const SRV_Channel *c = SRV_Channels::srv_channel(i);
//                 if (c == nullptr)
//                 {
//                     // 如果获取通道指针失败，则输出PWM为0
//                     _output.pwm[i] = 0;
//                     continue;
//                 }
//                 // 获取当前通道的输出PWM值
//                 _output.pwm[i] = c->get_output_pwm();
//             }

//             // 标记输出数组为新的
//             _output.is_new = true;

//             // 发送1～4通道的PWM值   0xAA     --------------------------------------------------------------------
//             uint8_t test_data1[] = {

//                 (uint8_t)((_output.pwm[0]) >> 8 & 0xFF), (uint8_t)((_output.pwm[0]) & 0xFF),
//                 (uint8_t)((_output.pwm[1]) >> 8 & 0xFF), (uint8_t)((_output.pwm[1]) & 0xFF),
//                 (uint8_t)((_output.pwm[2]) >> 8 & 0xFF), (uint8_t)((_output.pwm[2]) & 0xFF),
//                 (uint8_t)((_output.pwm[3]) >> 8 & 0xFF), (uint8_t)((_output.pwm[3]) & 0xFF)

//             };

//             send_packet(0, 0xAA, 10, test_data1, sizeof(test_data1)); // send_packet(扩展帧8位, 标准帧8位, 超时时间或发送延迟,数据数组，数据长度)

//             // 发送5～8通道的PWM值  0xAB       --------------------------------------------------------
//             uint8_t test_data2[] = {

//                 (uint8_t)((_output.pwm[4]) >> 8 & 0xFF), (uint8_t)((_output.pwm[4]) & 0xFF),
//                 (uint8_t)((_output.pwm[5]) >> 8 & 0xFF), (uint8_t)((_output.pwm[5]) & 0xFF),
//                 (uint8_t)((_output.pwm[6]) >> 8 & 0xFF), (uint8_t)((_output.pwm[6]) & 0xFF),
//                 (uint8_t)((_output.pwm[7]) >> 8 & 0xFF), (uint8_t)((_output.pwm[7]) & 0xFF)

//             };

//             send_packet(0, 0xAB, 10, test_data2, sizeof(test_data2));

//             // 发送9～12通道的PWM值  0xAC     ------------------------------------------------------
//             uint8_t test_data3[] = {
//                 (uint8_t)((_output.pwm[8]) >> 8 & 0xFF), (uint8_t)((_output.pwm[8]) & 0xFF),
//                 (uint8_t)((_output.pwm[9]) >> 8 & 0xFF), (uint8_t)((_output.pwm[9]) & 0xFF),
//                 (uint8_t)((_output.pwm[10]) >> 8 & 0xFF), (uint8_t)((_output.pwm[10]) & 0xFF),
//                 (uint8_t)((_output.pwm[11]) >> 8 & 0xFF), (uint8_t)((_output.pwm[11]) & 0xFF)};

//             send_packet(0, 0xAC, 10, test_data3, sizeof(test_data3));

//             // 发送13～16通道的PWM值  0xAD    ----------------------------------------------
//             uint8_t test_data4[] = {

//                 (uint8_t)((_output.pwm[12]) >> 8 & 0xFF), (uint8_t)((_output.pwm[12]) & 0xFF),
//                 (uint8_t)((_output.pwm[13]) >> 8 & 0xFF), (uint8_t)((_output.pwm[13]) & 0xFF),
//                 (uint8_t)((_output.pwm[14]) >> 8 & 0xFF), (uint8_t)((_output.pwm[14]) & 0xFF),
//                 (uint8_t)((_output.pwm[15]) >> 8 & 0xFF), (uint8_t)((_output.pwm[15]) & 0xFF)

//             };

//             send_packet(0, 0xAD, 10, test_data4, sizeof(test_data4));

//             // 发送飞控状态数据    0xAE       ----------------------------------------------
//             const bool armed = hal.util->get_soft_armed(); // 更新解锁状态
//             uint8_t arm_sta = armed ? 0xAA : 0xDD;         // 使用三元运算符简化

//             add_het++; // 心跳包

//             uint8_t test_data5[] = {
//                 arm_sta,                                                               // 解锁状态
//                 AP_KDECANUSE::mode_number,                                             // 模式号
//                 AP::gps().status(),                                                    // gps状态
//                 AP::gps().num_sats(),                                                  // 卫星数
//                 static_cast<uint8_t>(((AP::ahrs().get_yaw()) + 3.2f) * 255.0f / 6.4f), // 航向
//                 AP_KDECANUSE::RC_failsafe,                                             // 失控状态
//                 0,
//                 add_het // 心跳包
//             };

//             send_packet(0, 0xAE, 10, test_data5, sizeof(test_data5));

//             // 读取地面站CMD数据并发送 0xB1       ----------------------------------------------
//             uint8_t test_data6[] = {

//                 (uint8_t)((AP_KDECANUSE::qgc_read1) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read1) & 0xFF),
//                 (uint8_t)((AP_KDECANUSE::qgc_read2) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read2) & 0xFF),
//                 (uint8_t)((AP_KDECANUSE::qgc_read3) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read3) & 0xFF),
//                 (uint8_t)((AP_KDECANUSE::qgc_read4) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read4) & 0xFF)

//             };

//             send_packet(0, 0xB1, 10, test_data6, sizeof(test_data6));

//             // 读取地面站CMD数据并发送  0xB2       ----------------------------------------------
//             uint8_t test_data7[] = {

//                 (uint8_t)((AP_KDECANUSE::qgc_read5) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read5) & 0xFF),
//                 (uint8_t)((AP_KDECANUSE::qgc_read6) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read6) & 0xFF),
//                 (uint8_t)((AP_KDECANUSE::qgc_read7) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read7) & 0xFF),
//                 (uint8_t)((AP_KDECANUSE::qgc_read8) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read8) & 0xFF) // CMD ID

//             };

//             send_packet(0, 0xB2, 10, test_data7, sizeof(test_data7));

//             //  uint8_t test_data6[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x11,0x22,0x33};

//             //  send_packet(0, 0xAF, 10,test_data6,sizeof(test_data6)) ;
//         }

//     } // while true
// }

// bool AP_KDECAN_Driver::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint16_t data)
// {
//     const uint16_t data_be16 = htobe16(data);
//     return send_packet(address, dest_id, timeout_us, (uint8_t*)&data_be16, 2);
// }

// bool AP_KDECAN_Driver::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint8_t *data, const uint8_t data_len)
// {
//     // broadcast telemetry request frame
//     const frame_id_t id {
//         {
            // .object_address = address,
            // .destination_id = dest_id,
            // .source_id = AUTOPILOT_NODE_ID,
            // .priority = 0,
            // .unused = 0
//             .object_address = dest_id,
//             .destination_id = 0,
//             .source_id = 0,
//             .priority = 0,
//             .unused = 0
//         }
//     };

//     AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);

//     return write_frame(frame, timeout_us);
// }

// // singleton instance
// AP_KDECAN *AP_KDECAN::_singleton;

// namespace AP {
// AP_KDECAN *kdecan()
// {
//     return AP_KDECAN::get_singleton();
// }
// };

// #endif // AP_KDECAN_ENABLED

/*
 * AP_KDECAN.cpp
 *
 *      Author: Francisco Ferreira and Tom Pittenger
 */

#include "AP_KDECAN.h"

#if AP_KDECAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h> // for MIN,MAX

#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL &hal;

#define AP_KDECAN_DEBUG 0

// 初始化全局变量-----------------------------------------------------------------------------------------------------------------------------------
uint8_t AP_KDECANUSE::mode_number = 0;
bool AP_KDECANUSE::RC_failsafe = false;

// uint16_t AP_KDECANUSE::LD1 = 0;
// uint16_t AP_KDECANUSE::LD2 = 0;
// uint16_t AP_KDECANUSE::LD3 = 0;
// uint16_t AP_KDECANUSE::LD4 = 0;
// uint16_t AP_KDECANUSE::LD5 = 0;
// uint16_t AP_KDECANUSE::LD6 = 0;
// uint16_t AP_KDECANUSE::LD7 = 0;
// uint16_t AP_KDECANUSE::LD8 = 0;

// uint16_t AP_KDECANUSE::battery_V = 0;
// uint16_t AP_KDECANUSE::battery_A = 0;
// uint16_t AP_KDECANUSE::battery_T = 0;
// uint8_t AP_KDECANUSE::battery_bai = 0;
// uint8_t AP_KDECANUSE::VCU_status = 0;

uint16_t AP_KDECANUSE::int0 = 0;
uint16_t AP_KDECANUSE::int1 = 0;
uint16_t AP_KDECANUSE::int2 = 0;
uint16_t AP_KDECANUSE::int3 = 0;
uint16_t AP_KDECANUSE::int4 = 0;
uint16_t AP_KDECANUSE::int5 = 0;
uint16_t AP_KDECANUSE::int6 = 0;
uint16_t AP_KDECANUSE::int7 = 0;
uint16_t AP_KDECANUSE::int8 = 0;
uint16_t AP_KDECANUSE::int9 = 0;
uint16_t AP_KDECANUSE::int10 = 0;
uint16_t AP_KDECANUSE::int11 = 0;
uint16_t AP_KDECANUSE::int12 = 0;
uint16_t AP_KDECANUSE::int13 = 0;
uint16_t AP_KDECANUSE::int14 = 0;
uint16_t AP_KDECANUSE::int15 = 0;

// uint16_t AP_KDECANUSE::qgc_read1 = 0;
// uint16_t AP_KDECANUSE::qgc_read2 = 0;
// uint16_t AP_KDECANUSE::qgc_read3 = 0;
// uint16_t AP_KDECANUSE::qgc_read4 = 0;
// uint16_t AP_KDECANUSE::qgc_read5 = 0;
// uint16_t AP_KDECANUSE::qgc_read6 = 0;
// uint16_t AP_KDECANUSE::qgc_read7 = 0;
// uint16_t AP_KDECANUSE::qgc_read8 = 0;

// uint16_t AP_KDECANUSE::qgc_send1 = 0;
// uint16_t AP_KDECANUSE::qgc_send2 = 0;
// uint16_t AP_KDECANUSE::qgc_send3 = 0;
// uint16_t AP_KDECANUSE::qgc_send4 = 0;
// float AP_KDECANUSE::qgc_send5 = 0;
// float AP_KDECANUSE::qgc_send6 = 0;
// float AP_KDECANUSE::qgc_send7 = 0;
// float AP_KDECANUSE::qgc_send8 = 0;
// float AP_KDECANUSE::qgc_send9 = 0;
// float AP_KDECANUSE::qgc_send10 = 0;
// float AP_KDECANUSE::qgc_send11 = 0;
// float AP_KDECANUSE::qgc_send12 = 0;

uint16_t AP_KDECANUSE::CAN_HZ = 0;

uint8_t add_het = 0;

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_KDECAN::var_info[] = { // 参数表

    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_KDECAN, _num_poles, DEFAULT_NUM_POLES), // 默认的极数

    AP_GROUPEND};

AP_KDECAN::AP_KDECAN()
{
    // 设置对象的默认值
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // 判断是否为单例模式
    if (_singleton != nullptr)
    {
        // 如果不是单例模式，则触发异常
        AP_HAL::panic("AP_KDECAN must be singleton");
    }
#endif

    // 将当前对象设置为单例
    _singleton = this;
}

void AP_KDECAN::init()
{
    if (_driver != nullptr)
    {
        // 如果已经存在实例，则不允许创建多个实例
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::KDECAN) {
            _driver = NEW_NOTHROW AP_KDECAN_Driver();
            return;
        }
    }
}

void AP_KDECAN::update()
{
    // 如果驱动为空，则直接返回
    if (_driver == nullptr)
    {
        return;
    }
    // 调用驱动对象的update方法，传入极数作为参数
    _driver->update((uint8_t)_num_poles.get());
}

AP_KDECAN_Driver::AP_KDECAN_Driver() : CANSensor("KDECAN") // 构造器
{
    register_driver(AP_CAN::Protocol::KDECAN); // 注册驱动

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack     // 启动线程，用于接收和发送CAN帧。测试表明我们使用大约640字节的堆栈
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_KDECAN_Driver::loop, void), "kdecan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

//-------------------------------------------------------接收CAN消息---------------------------------------------------------------------------

// parse inbound frames
void AP_KDECAN_Driver::handle_frame(AP_HAL::CANFrame &frame) // 处理帧
{

    if (frame.isExtended())
    { // 判断是否为扩展帧
        return;
    }

    const frame_id_t id{.value = frame.id & AP_HAL::CANFrame::MaskStdID}; // 获取帧ID

    switch (id.object_address)
    {          // 向飞控或地面站发送的数据------------------------------------------------------------------
    // case 0xBA: // 雷达数据   参数：雷达类型选择TeraRangerTower
    //     AP_KDECANUSE::LD1 = (frame.data[0] << 8) | frame.data[1];
    //     AP_KDECANUSE::LD2 = (frame.data[2] << 8) | frame.data[3];
    //     AP_KDECANUSE::LD3 = (frame.data[4] << 8) | frame.data[5];
    //     AP_KDECANUSE::LD4 = (frame.data[6] << 8) | frame.data[7];
    //     break;
    // case 0xBB: // 雷达数据
    //     AP_KDECANUSE::LD5 = (frame.data[0] << 8) | frame.data[1];
    //     AP_KDECANUSE::LD6 = (frame.data[2] << 8) | frame.data[3];
    //     AP_KDECANUSE::LD7 = (frame.data[4] << 8) | frame.data[5];
    //     AP_KDECANUSE::LD8 = (frame.data[6] << 8) | frame.data[7];
    //     break;
    // case 0xBC:                                                          // 电池数据
    //     AP_KDECANUSE::battery_V = (frame.data[0] << 8) | frame.data[1]; // 获取电池电压
    //     AP_KDECANUSE::battery_A = (frame.data[2] << 8) | frame.data[3]; // 获取电池电流
    //     AP_KDECANUSE::battery_T = (frame.data[4] << 8) | frame.data[5]; // 获取电池温度
    //     AP_KDECANUSE::battery_bai = frame.data[6];                      // 获取电池电量
    //     AP_KDECANUSE::VCU_status = frame.data[7];                       // 获取VCU状态
    //     break;
    case 0xBD: // 接收发动机温度数据1～4
        AP_KDECANUSE::int0 = (frame.data[0] << 8) | frame.data[1];
        AP_KDECANUSE::int1 = (frame.data[2] << 8) | frame.data[3];
        AP_KDECANUSE::int2 = (frame.data[4] << 8) | frame.data[5];
        AP_KDECANUSE::int3 = (frame.data[6] << 8) | frame.data[7]; 
        break;
    case 0xBE: // 接收发动机温度数据5～8
        AP_KDECANUSE::int4 = (frame.data[0] << 8) | frame.data[1];//滑油压力
        AP_KDECANUSE::int5 = (frame.data[2] << 8) | frame.data[3];//水温
        AP_KDECANUSE::int6 = (frame.data[4] << 8) | frame.data[5];//油温
        AP_KDECANUSE::int7 = (frame.data[6] << 8) | frame.data[7];//缸温
        break;
    // case 0xBF: // 自定义数据，16位
    //     AP_KDECANUSE::int9 = (frame.data[0] << 8) | frame.data[1];
    //     AP_KDECANUSE::int10 = (frame.data[2] << 8) | frame.data[3];
    //     AP_KDECANUSE::int11 = (frame.data[4] << 8) | frame.data[5];
    //     AP_KDECANUSE::int12 = (frame.data[6] << 8) | frame.data[7];
    //     break;
    // case 0xC1: // 上传浮点数，65535转655.35
    //     AP_KDECANUSE::int13 = (frame.data[0] << 8) | frame.data[1];
    //     AP_KDECANUSE::int14 = (frame.data[2] << 8) | frame.data[3];
    //     AP_KDECANUSE::int15 = (frame.data[4] << 8) | frame.data[5];
    //     AP_KDECANUSE::int16 = (frame.data[6] << 8) | frame.data[7];
    //     break;
    // case 0xd1:
    //     AP_KDECANUSE::qgc_send1 = (frame.data[0] << 8) | frame.data[1];
    //     AP_KDECANUSE::qgc_send2 = (frame.data[2] << 8) | frame.data[3];
    //     AP_KDECANUSE::qgc_send3 = (frame.data[4] << 8) | frame.data[5];
    //     AP_KDECANUSE::qgc_send4 = (frame.data[6] << 8) | frame.data[7];
    //     break;
    // case 0xd2:
    //     AP_KDECANUSE::qgc_send5 = (frame.data[0] << 8) | frame.data[1];
    //     AP_KDECANUSE::qgc_send6 = (frame.data[2] << 8) | frame.data[3];
    //     AP_KDECANUSE::qgc_send7 = (frame.data[4] << 8) | frame.data[5];
    //     AP_KDECANUSE::qgc_send8 = (frame.data[6] << 8) | frame.data[7];
    //     break;
    // case 0xd3:
    //     AP_KDECANUSE::qgc_send9 = (frame.data[0] << 8) | frame.data[1];
    //     AP_KDECANUSE::qgc_send10 = (frame.data[2] << 8) | frame.data[3];
    //     AP_KDECANUSE::qgc_send11 = (frame.data[4] << 8) | frame.data[5];
    //     AP_KDECANUSE::qgc_send12 = (frame.data[6] << 8) | frame.data[7];
    //     break;
    }
}

void AP_KDECAN_Driver::update(const uint8_t num_poles) // 更新极数
{
    if (_init.detected_bitmask == 0)
    {
        // 如果没有检测到任何设备，则无需执行任何操作
        // nothing to do...
        return;
    }

#if HAL_WITH_ESC_TELEM
    _telemetry.num_poles = num_poles;
#endif
    
    WITH_SEMAPHORE(_output.sem);
    for (uint8_t i = 0; i < ARRAY_SIZE(_output.pwm); i++) {
        if ((_init.detected_bitmask & (1UL<<i)) == 0 || SRV_Channels::channel_function(i) <= SRV_Channel::Aux_servo_function_t::k_none) {
            _output.pwm[i] = 0;
            continue;
        }

        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            _output.pwm[i] = 0;
            continue;
        }
        _output.pwm[i] = c->get_output_pwm();
    }

    _output.is_new = true;

#if AP_KDECAN_USE_EVENTS
    if (_output.thread_ctx != nullptr) {
        // trigger the thread to wake up immediately
        chEvtSignal(_output.thread_ctx, 1);
    }
#endif

#if AP_KDECAN_DEBUG
    static uint32_t last_send_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms > 1000) {
        last_send_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"%u: %u, %u, %u, %u, %u, %u, %u, %u",
        (unsigned)_init.detected_bitmask,
        (unsigned)_output.pwm[0], (unsigned)_output.pwm[1], (unsigned)_output.pwm[2], (unsigned)_output.pwm[3],
        (unsigned)_output.pwm[4], (unsigned)_output.pwm[5], (unsigned)_output.pwm[6], (unsigned)_output.pwm[7]);
    }
#endif
}

void AP_KDECAN_Driver::loop() // 驱动循环
{
//     // uint16_t pwm[ARRAY_SIZE(_output.pwm)]{}; // 定义PWM数组

// #if AP_KDECAN_USE_EVENTS                  // 如果定义了AP_KDECAN_USE_EVENTS
//     _output.thread_ctx = chThdGetSelfX(); // 获取当前线程上下文
// #endif

//     uint8_t broadcast_esc_info_boot_spam_count = 3;     // 启动时发送3次ESC信息
//     uint32_t broadcast_esc_info_next_interval_ms = 100; // spam a few at boot at this rate      // 启动时发送ESC信息间隔为100ms

//     while (true) {                    
// #if AP_KDECAN_USE_EVENTS // 如果定义了AP_KDECAN_USE_EVENTS
//         // sleep until we get new data, but also  wake up at 400Hz to send the old data again    // 睡眠，直到有新数据到来，并且也以400Hz频率唤醒
//         // chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));                        // 等待事件，超时时间为2500us
//         chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(1000000 / AP_KDECANUSE::CAN_HZ));
// #else
//         // hal.scheduler->delay_microseconds(2500); // 400Hz                    // 延迟2500us--------------g2.kdecan_send_hz--------------------------------------------------
//         //  hal.scheduler->delay_microseconds(10000); // 100Hz -------------------------------调整CAN发送帧率----------------------------------------------------------------
//         hal.scheduler->delay_microseconds(1000000 / AP_KDECANUSE::CAN_HZ); // 添加自定义参数调整CAN发送帧率   AP_KDECANUSE::CAN_HZ = g2.kdecan_send_hz;
// #endif

//         const uint32_t now_ms = AP_HAL::millis(); // 获取当前时间

//         // // This should run at 400Hz                            // 400Hz
//         // {
//         //     WITH_SEMAPHORE(_output.sem); // 使用信号量保护输出数组
//         //     if (_output.is_new)
//         //     {                                            // 如果输出数组为新数据
//         //         _output.last_new_ms = now_ms;            // 更新最后新数据的时间
//         //         _output.is_new = false;                  // 标记输出数组为旧数据
//         //         memcpy(&pwm, &_output.pwm, sizeof(pwm)); // 将输出数组复制到PWM数组中
//         //     }
//         //     else if (_output.last_new_ms && now_ms - _output.last_new_ms > 1000)
//         //     { // 如果输出数组为旧数据，并且距离上次新数据的时间超过1秒
//         //         // if we haven't gotten any PWM updates for a bit, zero it
//         //         // out so we don't just keep sending the same values forever
//         //         memset(&pwm, 0, sizeof(pwm)); // 将PWM数组清零
//         //         _output.last_new_ms = 0;      // 更新最后新数据的时间为0
//         //     }
//         // }

//         // for (uint8_t i = 0; i < ARRAY_SIZE(_output.pwm); i++)
//         // { // 遍历PWM数组
//         //     if ((_init.detected_bitmask & (1UL << i)) != 0)
//         //     {   // 如果检测到当前通道
//         //         // send_packet_uint16(SET_PWM_OBJ_ADDR, (i + ESC_NODE_ID_FIRST), 1, pwm[i]);    // 发送PWM值
//         //     }
//         // }

        for (uint8_t i=0; i<ARRAY_SIZE(_output.pwm); i++) {
            if ((_init.detected_bitmask & (1UL<<i)) != 0) {
                send_packet_uint16(SET_PWM_OBJ_ADDR, (i + ESC_NODE_ID_FIRST), 1000, pwm[i]);
            }
        }

#if HAL_WITH_ESC_TELEM
        // broadcast as request-telemetry msg to everyone
        if (_init.detected_bitmask != 0 && now_ms - _telemetry.timer_ms >= TELEMETRY_INTERVAL_MS) {
            if (send_packet(TELEMETRY_OBJ_ADDR, BROADCAST_NODE_ID, 10000)) {
                _telemetry.timer_ms = now_ms;
            }
        }
#endif // HAL_WITH_ESC_TELEM

//             //-------------------------------------------------发送CAN消息-------------------------------------------------

            if (send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100000)) {
                _init.detected_bitmask_ms = now_ms;
            }
        }

//             //     // 获取当前通道的指针
//             //     const SRV_Channel *c = SRV_Channels::srv_channel(i);
//             //     if (c == nullptr)
//             //     {
//             //         // 如果获取通道指针失败，则输出PWM为0
//             //         _output.pwm[i] = 0;
//             //         continue;
//             //     }
//             //     // 获取当前通道的输出PWM值
//             //     _output.pwm[i] = c->get_output_pwm();
//             // }

//             // 标记输出数组为新的
//             // _output.is_new = true;

//             // 发送1～4通道的PWM值   0xAA     --------------------------------------------------------------------
//             // uint8_t test_data1[] = {

//             //     (uint8_t)((_output.pwm[0]) >> 8 & 0xFF), (uint8_t)((_output.pwm[0]) & 0xFF),
//             //     (uint8_t)((_output.pwm[1]) >> 8 & 0xFF), (uint8_t)((_output.pwm[1]) & 0xFF),
//             //     (uint8_t)((_output.pwm[2]) >> 8 & 0xFF), (uint8_t)((_output.pwm[2]) & 0xFF),
//             //     (uint8_t)((_output.pwm[3]) >> 8 & 0xFF), (uint8_t)((_output.pwm[3]) & 0xFF)

//             // };

//             // send_packet(0, 0xAA, 10, test_data1, sizeof(test_data1)); // send_packet(扩展帧8位, 标准帧8位, 超时时间或发送延迟,数据数组，数据长度)

//             // // 发送5～8通道的PWM值  0xAB       --------------------------------------------------------
//             // uint8_t test_data2[] = {

//             //     (uint8_t)((_output.pwm[4]) >> 8 & 0xFF), (uint8_t)((_output.pwm[4]) & 0xFF),
//             //     (uint8_t)((_output.pwm[5]) >> 8 & 0xFF), (uint8_t)((_output.pwm[5]) & 0xFF),
//             //     (uint8_t)((_output.pwm[6]) >> 8 & 0xFF), (uint8_t)((_output.pwm[6]) & 0xFF),
//             //     (uint8_t)((_output.pwm[7]) >> 8 & 0xFF), (uint8_t)((_output.pwm[7]) & 0xFF)

//             // };

//             // send_packet(0, 0xAB, 10, test_data2, sizeof(test_data2));

//             // // 发送9～12通道的PWM值  0xAC     ------------------------------------------------------
//             // uint8_t test_data3[] = {
//             //     (uint8_t)((_output.pwm[8]) >> 8 & 0xFF), (uint8_t)((_output.pwm[8]) & 0xFF),
//             //     (uint8_t)((_output.pwm[9]) >> 8 & 0xFF), (uint8_t)((_output.pwm[9]) & 0xFF),
//             //     (uint8_t)((_output.pwm[10]) >> 8 & 0xFF), (uint8_t)((_output.pwm[10]) & 0xFF),
//             //     (uint8_t)((_output.pwm[11]) >> 8 & 0xFF), (uint8_t)((_output.pwm[11]) & 0xFF)};

//             // send_packet(0, 0xAC, 10, test_data3, sizeof(test_data3));

//             // // 发送13～16通道的PWM值  0xAD    ----------------------------------------------
//             // uint8_t test_data4[] = {

//             //     (uint8_t)((_output.pwm[12]) >> 8 & 0xFF), (uint8_t)((_output.pwm[12]) & 0xFF),
//             //     (uint8_t)((_output.pwm[13]) >> 8 & 0xFF), (uint8_t)((_output.pwm[13]) & 0xFF),
//             //     (uint8_t)((_output.pwm[14]) >> 8 & 0xFF), (uint8_t)((_output.pwm[14]) & 0xFF),
//             //     (uint8_t)((_output.pwm[15]) >> 8 & 0xFF), (uint8_t)((_output.pwm[15]) & 0xFF)

//             // };

//             // send_packet(0, 0xAD, 10, test_data4, sizeof(test_data4));

//             // 发送飞控状态数据    0xAE       ----------------------------------------------
//             const bool armed = hal.util->get_soft_armed(); // 更新解锁状态
//             uint8_t arm_sta = armed ? 0xAA : 0xDD;         // 使用三元运算符简化

//             add_het++; // 心跳包

//             uint8_t test_data5[] = {
//                 arm_sta,                                                               // 解锁状态
//                 AP_KDECANUSE::mode_number,                                             // 模式号
//                 AP::gps().status(),                                                    // gps状态
//                 AP::gps().num_sats(),                                                  // 卫星数
//                 static_cast<uint8_t>(((AP::ahrs().get_yaw()) + 3.2f) * 255.0f / 6.4f), // 航向
//                 AP_KDECANUSE::RC_failsafe,                                             // 失控状态
//                 0,
//                 add_het // 心跳包
//             };

//             send_packet(0, 0xAE, 10, test_data5, sizeof(test_data5));

//             // // 读取地面站CMD数据并发送 0xB1       ----------------------------------------------
//             // uint8_t test_data6[] = {

//             //     (uint8_t)((AP_KDECANUSE::qgc_read1) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read1) & 0xFF),
//             //     (uint8_t)((AP_KDECANUSE::qgc_read2) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read2) & 0xFF),
//             //     (uint8_t)((AP_KDECANUSE::qgc_read3) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read3) & 0xFF),
//             //     (uint8_t)((AP_KDECANUSE::qgc_read4) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read4) & 0xFF)

//             // };

//             // send_packet(0, 0xB1, 10, test_data6, sizeof(test_data6));

//             // // 读取地面站CMD数据并发送  0xB2       ----------------------------------------------
//             // uint8_t test_data7[] = {

//             //     (uint8_t)((AP_KDECANUSE::qgc_read5) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read5) & 0xFF),
//             //     (uint8_t)((AP_KDECANUSE::qgc_read6) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read6) & 0xFF),
//             //     (uint8_t)((AP_KDECANUSE::qgc_read7) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read7) & 0xFF),
//             //     (uint8_t)((AP_KDECANUSE::qgc_read8) >> 8 & 0xFF), (uint8_t)((AP_KDECANUSE::qgc_read8) & 0xFF) // CMD ID

//             // };

//             // send_packet(0, 0xB2, 10, test_data7, sizeof(test_data7));

//             //  uint8_t test_data6[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x11,0x22,0x33};

//             //  send_packet(0, 0xAF, 10,test_data6,sizeof(test_data6)) ;
//         }

//     } // while true
}

bool AP_KDECAN_Driver::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint16_t data)
{
    // 将数据转换为大端字节序
    const uint16_t data_be16 = htobe16(data);
    return send_packet(address, dest_id, timeout_us, (uint8_t*)&data_be16, 2);
}

bool AP_KDECAN_Driver::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint8_t *data, const uint8_t data_len)
{
    // 构造广播遥测请求帧的帧ID
    // broadcast telemetry request frame
    const frame_id_t id{
        {.object_address = dest_id,
         .destination_id = 0,
         .source_id = 0,
         .priority = 0,
         .unused = 0}};

    // 构造CAN帧
    AP_HAL::CANFrame frame = AP_HAL::CANFrame(id.value, data, data_len, false);

    return write_frame(frame, timeout_us);
}

// singleton instance
AP_KDECAN *AP_KDECAN::_singleton;

namespace AP
{
    AP_KDECAN *kdecan()
    {
        // 返回 AP_KDECAN 类的单例对象
        return AP_KDECAN::get_singleton();
    }
};

#endif // AP_KDECAN_ENABLED
