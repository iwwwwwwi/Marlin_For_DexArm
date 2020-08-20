/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/endstops.h"
#include "../../core/debug_out.h"
#include "../../module/update_manager.h"

typedef enum {
  NO_NEED_CONFIRM = 0U,
  NEED_CONFIRM,
  CONFIRMED,
}Update_Need_ConfirmTypeDef;

int need_confirm_state = NO_NEED_CONFIRM;

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
void GcodeSuite::M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
void GcodeSuite::M121() { endstops.enable_globally(false); }

void GcodeSuite::M2002()
{
	need_confirm_state = NEED_CONFIRM;
    DEBUG_ECHOLNPGM("Ready to enter update bootloader, please use M2003 confirm or M2004 cancel");
}

void GcodeSuite::M2003()
{
	if(need_confirm_state == NEED_CONFIRM){
		need_confirm_state = CONFIRMED;
		DEBUG_ECHOLNPGM("Reset to enter update bootloader");
		enter_update();
	}else{
		DEBUG_ECHOLNPGM("Inorder to confirm <enter update bootloader>, should be used after CMD M2002");
	}
}
