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

#pragma once

#include "AP_Gripper.h"

#if AP_GRIPPER_DRONECAN_ENABLED

#include "AP_Gripper_Backend.h"

#define GRIPPER_DRONECAN_ACTUATION_MS  500     // ms after grab/release before state is finalized

/// @class AP_Gripper_DroneCAN
/// @brief Gripper backend using DroneCAN hardpoint command
class AP_Gripper_DroneCAN : public AP_Gripper_Backend {
public:
    AP_Gripper_DroneCAN(struct AP_Gripper::Backend_Config &_config);

    void init_gripper() override;
    void grab() override;
    void release() override;
    void update_gripper() override;
    bool valid() const override;
    bool grabbed() const override;
    bool released() const override;

private:

    void send_command(uint16_t command);

    bool _has_dronecan;
};

#endif // AP_GRIPPER_DRONECAN_ENABLED
