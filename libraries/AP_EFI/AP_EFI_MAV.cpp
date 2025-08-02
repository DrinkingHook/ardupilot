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

#include "AP_EFI_config.h"
#include "AP_KDECAN/AP_KDECAN.h"
#if AP_EFI_MAV_ENABLED

#include "AP_EFI_MAV.h"
#include <AP_Math/AP_Math.h>

//Called from frontend to update with the readings received by handler
void AP_EFI_MAV::update()
{
    // if (receivedNewData) {
    	copy_to_frontend();
    	// receivedNewData = false;
    // }
}

//Decode MavLink message
void AP_EFI_MAV::handle_EFI_message(const mavlink_message_t &msg)
{
    // mavlink_efi_status_t state;
    // mavlink_msg_efi_status_decode(&msg, &state);

    // internal_state.ecu_index = state.ecu_index;
    // internal_state.engine_speed_rpm = state.rpm;
    // internal_state.estimated_consumed_fuel_volume_cm3 = state.fuel_consumed;
    // internal_state.fuel_consumption_rate_cm3pm = state.fuel_flow;
    // internal_state.engine_load_percent = state.engine_load;
    // internal_state.throttle_position_percent = state.throttle_position;
    // internal_state.spark_dwell_time_ms = state.spark_dwell_time;
    // internal_state.atmospheric_pressure_kpa = state.barometric_pressure;
    // internal_state.intake_manifold_pressure_kpa = state.intake_manifold_pressure;
    // internal_state.intake_manifold_temperature = C_TO_KELVIN(state.intake_manifold_temperature);
    // internal_state.cylinder_status.cylinder_head_temperature = C_TO_KELVIN(state.cylinder_head_temperature);
    // internal_state.cylinder_status.ignition_timing_deg = state.ignition_timing;
    // internal_state.cylinder_status.injection_time_ms = state.injection_time;
    // internal_state.cylinder_status.exhaust_gas_temperature = C_TO_KELVIN(state.exhaust_gas_temperature);
    // internal_state.throttle_out = state.throttle_out;
    // internal_state.pt_compensation = state.pt_compensation;
    // //internal_state.??? = state.health;
    // internal_state.ignition_voltage = state.ignition_voltage;

    // receivedNewData = true;

    // mavlink_efi_status_t state;

    // mavlink_msg_efi_status_decode(&msg, &state);

    // internal_state.ecu_index = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int1)) / 3000.0f;
    // internal_state.engine_speed_rpm = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int2)) / 3000.0f;
    // internal_state.estimated_consumed_fuel_volume_cm3 = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int3)) / 3000.0f;
    // internal_state.fuel_consumption_rate_cm3pm = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int4)) / 3000.0f;
    // internal_state.engine_load_percent = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int5)) / 3000.0f;
    // internal_state.throttle_position_percent = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int6)) / 3000.0f;
    // internal_state.spark_dwell_time_ms = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int7)) / 3000.0f;
    // internal_state.atmospheric_pressure_kpa = static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int8)) / 3000.0f;
    // internal_state.intake_manifold_pressure_kpa = state.intake_manifold_pressure;
    // internal_state.intake_manifold_temperature = C_TO_KELVIN(state.intake_manifold_temperature);
    // internal_state.cylinder_status.cylinder_head_temperature = C_TO_KELVIN(state.cylinder_head_temperature);
    // internal_state.cylinder_status.ignition_timing_deg = state.ignition_timing;
    // internal_state.cylinder_status.injection_time_ms = state.injection_time;
    // internal_state.cylinder_status.exhaust_gas_temperature = C_TO_KELVIN(state.exhaust_gas_temperature);
    // internal_state.throttle_out = state.throttle_out;
    // internal_state.pt_compensation = state.pt_compensation;
    // // internal_state.??? = state.health;
    // internal_state.ignition_voltage = state.ignition_voltage;

    // receivedNewData = true;
    // mavlink_efi_status_t state;

    // state.ecu_index = (AP_KDECANUSE::int1) / 3000.0f;
    // state.rpm = (AP_KDECANUSE::int2) / 3000.0f;
    // state.fuel_consumed = (AP_KDECANUSE::int3) / 3000.0f;
    // state.fuel_flow = (AP_KDECANUSE::int4) / 3000.0f;
    // state.engine_load = (AP_KDECANUSE::int5) / 3000.0f;
    // state.throttle_position = (AP_KDECANUSE::int6) / 3000.0f;
    // state.spark_dwell_time = (AP_KDECANUSE::int7) / 3000.0f;
    // state.barometric_pressure = (AP_KDECANUSE::int8) / 3000.0f;
    // state.intake_manifold_pressure = AP_KDECANUSE::int9;
    // state.intake_manifold_temperature = AP_KDECANUSE::int10;
    // state.cylinder_head_temperature = AP_KDECANUSE::int11;
    // state.ignition_timing = AP_KDECANUSE::int12;
    // state.injection_time = AP_KDECANUSE::int13;
    // state.exhaust_gas_temperature = AP_KDECANUSE::int14;
    // state.throttle_out = AP_KDECANUSE::int15;
    // state.pt_compensation = AP_KDECANUSE::int16;
    // 其他字段按需赋值...

    // 用state更新internal_state
    internal_state.ecu_index = (AP_KDECANUSE::int1) / 3000.0f;
    internal_state.engine_speed_rpm = (AP_KDECANUSE::int2) / 3000.0f;
    internal_state.estimated_consumed_fuel_volume_cm3 = (AP_KDECANUSE::int3) / 3000.0f;
    internal_state.fuel_consumption_rate_cm3pm = (AP_KDECANUSE::int4) / 3000.0f;
    internal_state.engine_load_percent = (AP_KDECANUSE::int5) / 3000.0f;
    internal_state.throttle_position_percent = (AP_KDECANUSE::int6) / 3000.0f;
    internal_state.spark_dwell_time_ms = (AP_KDECANUSE::int7) / 3000.0f;
    internal_state.atmospheric_pressure_kpa = (AP_KDECANUSE::int8) / 3000.0f;
    internal_state.intake_manifold_pressure_kpa = 55.0f; // state.intake_manifold_pressure;
    internal_state.intake_manifold_temperature = 40.0f;//C_TO_KELVIN(state.intake_manifold_temperature);
    internal_state.cylinder_status.cylinder_head_temperature = 30.0f;
    //C_TO_KELVIN(state.cylinder_head_temperature);
    internal_state.cylinder_status.ignition_timing_deg = 20.0f; // state.ignition_timing;
    internal_state.cylinder_status.injection_time_ms = 45.0f;   // state.injection_time;
    internal_state.cylinder_status.exhaust_gas_temperature = 21.0f;
    //C_TO_KELVIN(state.exhaust_gas_temperature);
    internal_state.throttle_out = 31.0f;
    //state.throttle_out;
    internal_state.pt_compensation = 20.0f;
    //state.pt_compensation;
    // 其他字段按需赋值...

    receivedNewData = true;
    // mavlink_msg_efi_status_send(chan,
    //                             1,
    //                             //   AP_KDECANUSE::int1为uint16_t类型。转换为有符号整数后除以3000.0f 得到浮点数,保留两位小数
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int1)) / 3000.0f,
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int2)) / 3000.0f,
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int3)) / 3000.0f,
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int4)) / 3000.0f,
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int5)) / 3000.0f,
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int6)) / 3000.0f,
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int7)) / 3000.0f,
    //                             static_cast<float>(static_cast<int32_t>(AP_KDECANUSE::int8)) / 3000.0f,
    //                             AP_KDECANUSE::int9,  // intake_manifold_pressure
    //                             AP_KDECANUSE::int10, // intake_manifold_temperature
    //                             AP_KDECANUSE::int11, // cylinder_head_temperature
    //                             AP_KDECANUSE::int12, // ignition_timing
    //                             AP_KDECANUSE::int13, // injection_time
    //                             AP_KDECANUSE::int14, // exhaust_gas_temperature
    //                             AP_KDECANUSE::int15, // throttle_out
    //                             AP_KDECANUSE::int16, // pt_compensation
    //                             0,
    //                             0);
}

#endif  // AP_EFI_MAV_ENABLED
