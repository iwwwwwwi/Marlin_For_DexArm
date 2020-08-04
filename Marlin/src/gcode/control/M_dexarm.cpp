#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/endstops.h"
#include "../../module/configuration_store.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"
#include "../../module/dexarm/dexarm.h"

typedef enum
{
	NO_NEED_CONFIRM = 0U,
	NEED_CONFIRM,
	CONFIRMED,
} Update_Need_ConfirmTypeDef;

int need_confirm_state = NO_NEED_CONFIRM;

void re_calibration(float distance)
{
	float y;
	front_module_offset -= 300 - ((distance / 2) / 0.25007f);
	MYSERIAL0.println("Re Calibration OFFSET_FRONT: ");
	MYSERIAL0.println(front_module_offset);
}

void GcodeSuite::M888(void)
{
	planner.synchronize();
	const bool p_set = parser.seen('P');
	if (p_set)
	{
		const float P_TMP = parser.floatval('P');
		switch ((int)P_TMP)
		{
		case 0:
			front_module_offset = PEN_MODULE_OFFSET;
			MYSERIAL0.println("THE CURRENT MODULE IS PEN");
			(void)settings.save();
			update_dexarm_offset();
			planner.buffer_line(current_position, 30, active_extruder);
			break;
		case 1:
			front_module_offset = LASER_MODULE_OFFSET;
			laser_protection_enable_flag = true;
			MYSERIAL0.println("THE CURRENT MODULE IS LASER");
			(void)settings.save();
			update_dexarm_offset();
			planner.buffer_line(current_position, 30, active_extruder);
			break;
		case 2:
			front_module_offset = PUMP_MODULE_OFFSET;
			MYSERIAL0.println("THE CURRENT MODULE IS PUMP");
			(void)settings.save();
			update_dexarm_offset();
			planner.buffer_line(current_position, 30, active_extruder);
			break;
		case 3:
			front_module_offset = _3D_MODULE_OFFSET;
			MYSERIAL0.println("THE CURRENT MODULE IS 3D");
			(void)settings.save();
			update_dexarm_offset();
			planner.buffer_line(current_position, 30, active_extruder);
			break;
		//case 4:  front_module_offset = CAMERA_MODULE_OFFSET;    MYSERIAL0.println("THE CURRENT MODULE IS Camera");    break;
		case 5:
		{
			const float DISTANCE = parser.floatval('S');
			re_calibration(DISTANCE);
			MYSERIAL0.println("THE CURRENT MODULE IS Custom Module");
			break;
		}
		case 10:
		{
			laser_protection_enable_flag = false;
			MYSERIAL0.println("Disable laser protection");
			break;
		}
		case 11:
		{
			laser_protection_enable_flag = true;
			MYSERIAL0.println("Enable laser protection");
			break;
		}
		case 12:
		{
			//To-Do
			break;
		}
		case 13:
		{
			if(laser_door_open_flag){
				SERIAL_ECHOPAIR("Warning!Laser protection door opened");
				SERIAL_EOL();
			}else{
				SERIAL_ECHOPAIR("Laser protection door closed");
				SERIAL_EOL();
			}
			break;
		}
		}
		destination = current_position;
	}
	else
	{
		print_current_module_type();
	}
}

void GcodeSuite::M889()
{
	DEBUG_ECHOLNPGM("M889");
	planner.synchronize();

	bool check_param = parser.seen('X') & parser.seen('Y') & parser.seen('Z');
	if (check_param)
	{
		calibration_position_sensor_value[0] = parser.intval('X', 999);
		calibration_position_sensor_value[1] = parser.intval('Y', 999);
		calibration_position_sensor_value[2] = parser.intval('Z', 999);
	}
	else
	{
		LOOP_XYZ(i) { calibration_position_sensor_value[i] = position_sensor_value_read(i); }
	}

	(void)settings.save();

	MYSERIAL0.print("SET X VALUE: ");
	MYSERIAL0.print(calibration_position_sensor_value[0]);

	MYSERIAL0.print("  Y VALUE: ");
	MYSERIAL0.print(calibration_position_sensor_value[1]);

	MYSERIAL0.print("  Z VALUE: ");
	MYSERIAL0.println(calibration_position_sensor_value[2]);
}

void GcodeSuite::M890()
{
	DEBUG_ECHOLNPGM("M890");
	planner.synchronize();
	int position_sensor_value[3];
	LOOP_XYZ(i) { position_sensor_value[i] = position_sensor_value_read(i); }

	MYSERIAL0.print("GET X VALUE: ");
	MYSERIAL0.print(position_sensor_value[0]);

	MYSERIAL0.print("	Y VALUE: ");
	MYSERIAL0.print(position_sensor_value[1]);

	MYSERIAL0.print("	Z VALUE: ");
	MYSERIAL0.println(position_sensor_value[2]);
}

void GcodeSuite::M891(void)
{
	planner.synchronize();
	float x, y;
	x = parser.floatval('X', 999);
	y = parser.floatval('Y', 999);

	if (x < X_AXIS_SLOPE_MAX && x > X_AXIS_SLOPE_MIN)
	{
		MYSERIAL0.print("SET X AXIS SLOPE IS ");
		MYSERIAL0.println(x, 5);
	}
	else
	{
		MYSERIAL0.print("X AXIS SLOPE OVER LIMIT ");
		return;
	}

	if (y < Y_AXIS_SLOPE_MAX && y > Y_AXIS_SLOPE_MIN)
	{
		MYSERIAL0.print("SET Y AXIS SLOPE IS ");
		MYSERIAL0.println(y, 5);
	}
	else
	{
		MYSERIAL0.print("Y AXIS SLOPE OVER LIMIT ");
		return;
	}
	x_axis_scaling_factor = x;
	y_axis_scaling_factor = y;
	(void)settings.save();
}

void GcodeSuite::M892()
{

	(void)settings.load(false);

	MYSERIAL0.print("GET X AXIS SLOPE IS ");
	MYSERIAL0.println(x_axis_scaling_factor, 5);

	MYSERIAL0.print("GET Y AXIS SLOPE IS ");
	MYSERIAL0.println(y_axis_scaling_factor, 5);
}

void GcodeSuite::M893(void)
{
	get_current_encoder();
}

void GcodeSuite::M894(void)
{
	int x, y, z;
	bool check_param = parser.seen('X') & parser.seen('Y') & parser.seen('Z');
	if (check_param)
	{
		MYSERIAL0.println("Param check is all ok");
		x = parser.floatval('X', 999);
		y = parser.floatval('Y', 999);
		z = parser.floatval('Z', 999);
		process_encoder(x, y, z);
	}
}

void GcodeSuite::M1000()
{
	planner.synchronize(); 
	MYSERIAL2.println("QB00");
}

void GcodeSuite::M1001()
{
	planner.synchronize(); 
	MYSERIAL2.println("QB01");
}

void GcodeSuite::M1002()
{
	planner.synchronize(); 
	MYSERIAL2.println("QB02");
}

void GcodeSuite::M1003()
{
	planner.synchronize(); 
	MYSERIAL2.println("QB03");
}

void GcodeSuite::M1111()
{
	planner.synchronize();
	DEBUG_ECHOLNPGM("M1111");
	position_M1111();
}

void GcodeSuite::M1112()
{
	DEBUG_ECHOLNPGM("M1112");
	planner.synchronize();
	xyz_pos_t position;
	bool check_param = parser.seen('X') & parser.seen('Y') & parser.seen('Z');
	if (check_param)
	{
		position[X_AXIS] = parser.floatval('X', 999);
		position[Y_AXIS] = parser.floatval('Y', 999);
		position[Z_AXIS] = parser.floatval('Z', 999);
		m1112_position(position);
	}
	else
	{
		position[X_AXIS] = 0;
		position[Y_AXIS] = 300;
		position[Z_AXIS] = 0;
		m1112_position(position);
	}
}

void GcodeSuite::M1113()
{
	DEBUG_ECHOLNPGM("M1113");
	planner.synchronize();
	xyz_pos_t position;
	bool check_param = parser.seen('X') & parser.seen('Y') & parser.seen('Z');
	if (check_param)
	{
		position[X_AXIS] = parser.floatval('X', 999);
		position[Y_AXIS] = parser.floatval('Y', 999);
		position[Z_AXIS] = parser.floatval('Z', 999);
		m1113_position(position);
	}
	else
	{
		position[X_AXIS] = 0;
		position[Y_AXIS] = 300;
		position[Z_AXIS] = 0;
		m1113_position(position);
	}
}

void GcodeSuite::M1114()
{
    forward_kinematics_DEXARM(
      planner.get_axis_position_degrees(A_AXIS),
      planner.get_axis_position_degrees(B_AXIS),
      planner.get_axis_position_degrees(C_AXIS));
}

void GcodeSuite::M2002()
{
	need_confirm_state = NEED_CONFIRM;
	DEBUG_ECHOLNPGM("Ready to enter update bootloader, please use M2003 confirm or M2004 cancel");
}

void GcodeSuite::M2003()
{
	if (need_confirm_state == NEED_CONFIRM)
	{
		need_confirm_state = CONFIRMED;
		DEBUG_ECHOLNPGM("Reset to enter update bootloader");
		enter_update();
	}
	else
	{
		DEBUG_ECHOLNPGM("Inorder to confirm <enter update bootloader>, should be used after CMD M2002");
	}
}

void GcodeSuite::M2004()
{
	if (need_confirm_state == NEED_CONFIRM)
	{
		DEBUG_ECHOLNPGM("Cancelled");
	}
	else
	{
		DEBUG_ECHOLNPGM("M2004 is <Cancel enter update> CMD, should be used after CMD M2002");
	}
}

void GcodeSuite::M2007()
{
	NVIC_SystemReset();
}

void GcodeSuite::M2010()
{
	SERIAL_ECHOPAIR(FIRMWARE_VERSION);
}

void GcodeSuite::M2011()
{
	SERIAL_ECHOPAIR(HARDWARE_VERSION);
}