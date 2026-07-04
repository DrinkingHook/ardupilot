#include <AP_HAL/AP_HAL.h>

#include "AP_EFI_config.h"

#if AP_EFI_DRONECAN_ENABLED
#include "AP_ENGINE_DroneCan.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

AP_ENGINE_DroneCan *AP_ENGINE_DroneCan::driver;

// constructor
AP_ENGINE_DroneCan::AP_ENGINE_DroneCan(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    driver = this;
}

// links the DroneCAN message to this backend
void AP_ENGINE_DroneCan::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &trampoline_status, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("engine_status_sub");
    }
}

// Called from frontend to update with the readings received by handler
void AP_ENGINE_DroneCan::update()
{
}

// DroneCAN message handler
void AP_ENGINE_DroneCan::trampoline_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_engine_Status &msg)
{
    if (driver == nullptr) {
        return;
    }
    driver->handle_status(msg);
}

/*
  handle custom engine status message from DroneCAN
 */
void AP_ENGINE_DroneCan::handle_status(const ardupilot_equipment_engine_Status &pkt)
{
    auto &istate = internal_state;

    istate.oil_pressure = pkt.oil_pressure;
    istate.fuel_pressure = pkt.lube_pressure;
    istate.estimated_consumed_fuel_volume_cm3 = pkt.fuel_quantity;
    istate.coolant_temperature = pkt.coolant_temp;
    istate.oil_temperature = pkt.oil_temp;
    istate.gearbox_oil_temperature = pkt.gearbox_oil_temp;
    istate.intake_manifold_pressure_kpa = pkt.turbo_pressure;
    istate.cylinder_status.exhaust_gas_temperature = pkt.egt[0];
    istate.cylinder_status.exhaust_gas_temperature2 = pkt.egt[1];
    istate.cylinder_status.exhaust_gas_temperature3 = pkt.egt[2];
    istate.cylinder_status.exhaust_gas_temperature4 = pkt.egt[3];
    istate.cylinder_status.lambda_coefficient = pkt.lambda;

    // Required for healthy message
    istate.last_updated_ms = AP_HAL::millis();

    copy_to_frontend();
}

#endif // AP_EFI_DRONECAN_ENABLED
