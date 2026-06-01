#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_DRONECAN_ENABLED
#include "AP_EFI.h"
#include "AP_EFI_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_ENGINE_DroneCan : public AP_EFI_Backend {
public:
    AP_ENGINE_DroneCan(AP_EFI &_frontend);

    void update() override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static void trampoline_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_engine_Status &msg);

private:
    void handle_status(const ardupilot_equipment_engine_Status &pkt);

    static AP_ENGINE_DroneCan *driver;
};

#endif // AP_EFI_DRONECAN_ENABLED
