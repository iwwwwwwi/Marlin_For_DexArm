#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/endstops.h"
#include "../../module/configuration_store.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"
#include "../../module/dexarm/dexarm.h"

#if ENABLED(SENSORLESS_HOMING)
  #include "../../feature/tmc_util.h"
#endif
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../module/endstops.h"

typedef enum
{
	NO_NEED_CONFIRM = 0U,
	NEED_CONFIRM,
	CONFIRMED,
} Update_Need_ConfirmTypeDef;

int need_confirm_state = NO_NEED_CONFIRM;

void re_calibration(float target_distance, float actual_distance)
{
	float angle_cos = 0;
	angle_cos = target_distance / 600;
	MYSERIAL0.println("angle: ");
	MYSERIAL0.println(angle_cos);
	front_module_offset -= 291 - ((actual_distance / 2) / angle_cos);
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
			//laser_fan_flag = false;
			//OUT_WRITE(HEATER_0_PIN, LOW);
			MYSERIAL0.println("THE CURRENT MODULE IS 3D");
			(void)settings.save();
			update_dexarm_offset();
			planner.buffer_line(current_position, 30, active_extruder);
			break;
		//case 4:  front_module_offset = CAMERA_MODULE_OFFSET;    MYSERIAL0.println("THE CURRENT MODULE IS Camera");    break;
		case 5:
		{
			const float target_distance = parser.floatval('T');
			const float actual_distance = parser.floatval('A');
			if(target_distance < 100){
				MYSERIAL0.println("Target distance must be greater than 100, 150 is suggested");
				break;
			}
			re_calibration(target_distance, actual_distance);
			MYSERIAL0.println("THE CURRENT MODULE IS Custom Module");
			(void)settings.save();
			update_dexarm_offset();
			planner.buffer_line(current_position, 30, active_extruder);
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
		case 14:
		{
			//SERIAL_ECHOPAIR("Laser protection Fun Start");
			//SERIAL_EOL();
			//laser_fan_flag = true;
			//OUT_WRITE(HEATER_0_PIN, HIGH);
			break;
		}
		case 15:
		{
			//SERIAL_ECHOPAIR("Laser protection Fun Stop");
			//SERIAL_EOL();
			//laser_fan_flag = false;
			//OUT_WRITE(HEATER_0_PIN, LOW);
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

void GcodeSuite::M895(void)
{
	xyz_pos_t position;
	get_current_position_from_position_sensor(position);
	SERIAL_ECHOLNPAIR("X:", position.x, " Y:", position.y, " Z:", position.z);
	SERIAL_EOL();
}

move_mode_t teach_play_move_mode = FAST_MODE;
float teach_play_jump_height = 50.0;
float teach_play_feedrate = 50.0;
void GcodeSuite::M896(void)
{
	int x, y, z;
	bool check_height_param = parser.seen('H');
	if (check_height_param)
	{
		teach_play_jump_height = parser.floatval('H', 999);
	}

	bool check_feedrate_param = parser.seen('F');
	if (check_height_param)
	{
		teach_play_feedrate = parser.floatval('F', 999);
	}

	bool check_mode_param = parser.seen('P');
	if (check_mode_param)
	{
		switch (parser.intval('P', 999))
		{
		case 0:
			teach_play_move_mode = FAST_MODE;
			SERIAL_ECHOLNPAIR("teach&play move mode set to fast mode");
			break;
		case 1:
			teach_play_move_mode = LINE_MODE;
			SERIAL_ECHOLNPAIR("teach&play move mode set to line mode");
			break;
		case 2:
			teach_play_move_mode = JUMP_MODE;
			SERIAL_ECHOLNPAIR("teach&play move mode set to jump mode");
			break;
		}
	}

	bool check_param = parser.seen('X') & parser.seen('Y') & parser.seen('Z');
	if (check_param)
	{
		const feedRate_t old_feedrate = feedrate_mm_s;
		feedrate_mm_s = teach_play_feedrate;
		if (!position_init_flag)
		{
    		enable_all_steppers();
    		position_init_flag = true;
			set_current_position_from_position_sensor();
		}
		destination.x = parser.floatval('X', 999);
		destination.y = parser.floatval('Y', 999);
		destination.z = parser.floatval('Z', 999);
		destination.e = 0;
		switch(teach_play_move_mode)
		{
			case FAST_MODE:
				prepare_fast_move_to_destination();
				break;
			case LINE_MODE:
				prepare_line_to_destination();
				break;
			case JUMP_MODE:
				prepare_jump_move_to_destination(teach_play_jump_height);
				break;
		}
	}

	if((!check_height_param)&(!check_feedrate_param)&(!check_param)){
		switch(teach_play_move_mode)
		{
			case FAST_MODE:
				teach_play_move_mode = FAST_MODE;
				SERIAL_ECHOLNPAIR("teach&play move mode is fast mode");
				break;
			case LINE_MODE:
				teach_play_move_mode = LINE_MODE;
				SERIAL_ECHOLNPAIR("teach&play move mode is line mode");
				break;
			case JUMP_MODE:
				teach_play_move_mode = JUMP_MODE;
				SERIAL_ECHOLNPAIR("teach&play move mode is jump mode");
				break;
		}		
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
	SERIAL_ECHOPAIR("M1111\r\n");
	position_M1111();
}

void GcodeSuite::M1112()
{
	SERIAL_ECHOPAIR("M1112\r\n");
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
		position[Y_AXIS] = 291;
		position[Z_AXIS] = 0;
		m1112_position(position);
	}
}

void GcodeSuite::M1113()
{
	SERIAL_ECHOPAIR("M1113\r\n");
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
		position[Y_AXIS] = 291;
		position[Z_AXIS] = 0;
		m1113_position(position);
	}
}

void GcodeSuite::M1114()
{
	abc_pos_t angle;
	//LOOP_ABC(axis) { angle[axis] = planner.get_axis_position_degrees(axis);}
	angle.a = planner.get_axis_position_degrees(A_AXIS);
	angle.b = planner.get_axis_position_degrees(B_AXIS);
	angle.c = planner.get_axis_position_degrees(C_AXIS);
    forward_kinematics_DEXARM(angle);
}

void GcodeSuite::M2000()
{
    SERIAL_ECHOPAIR("M2000\r\n");
    G0_MOVE_MODE = LINE_MODE;
}

void GcodeSuite::M2001()
{
    SERIAL_ECHOPAIR("M2001\r\n");
    G0_MOVE_MODE = FAST_MODE;
}

void GcodeSuite::M2002()
{
	need_confirm_state = NEED_CONFIRM;
	SERIAL_ECHOPAIR("Ready to enter update bootloader, please use M2003 confirm or M2004 cancel\r\n");
}

void GcodeSuite::M2003()
{
	if (need_confirm_state == NEED_CONFIRM)
	{
		need_confirm_state = CONFIRMED;
		 SERIAL_ECHOPAIR("Reset to enter update bootloader\r\n");
		enter_update();
	}
	else
	{
		SERIAL_ECHOPAIR("Inorder to confirm <enter update bootloader>, should be used after CMD M2002\r\n");
	}
}

void GcodeSuite::M2004()
{
	if (need_confirm_state == NEED_CONFIRM)
	{
		need_confirm_state = NO_NEED_CONFIRM;
		SERIAL_ECHOPAIR("Cancelled\r\n");
	}
	else
	{
		SERIAL_ECHOPAIR("M2004 is <Cancel enter update> CMD, should be used after CMD M2002\r\n");
	}
}

void GcodeSuite::M2005()
{
	feedRate_t home_feedrate_high = 30;
	feedRate_t home_feedrate_low = 10;
	int16_t homing_threshold_first = 60;
	int16_t homing_threshold_second = 60;
	bool check_param = parser.seen('X') & parser.seen('Y')& parser.seen('Z')& parser.seen('E');
	if (check_param)
	{
		home_feedrate_high = parser.intval('X', 999);
		home_feedrate_low = parser.intval('Y', 999);
		homing_threshold_first = parser.intval('Z', 999);
		homing_threshold_second = parser.intval('E', 999);
	}
	sliding_rail_home(home_feedrate_high, home_feedrate_low, homing_threshold_first, homing_threshold_second);
}

void GcodeSuite::M2006()
{

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
uint8_t front_rotation_init_flag = 0;
//front rotation init
void GcodeSuite::M2100()
{
	int init_speed = 20;
	int init_position = 10;
	front_rotation_init();
	set_motion_speed(SERO_1, init_speed);
	set_pos(SERO_1, init_position);
	pos_demo_test();
	front_rotation_init_flag = 1;
}

void GcodeSuite::M2101()
{
	static uint16_t speed = 50, positon = 500;
	static uint16_t torque_val = 1023;
	
	if(!front_rotation_init_flag){
		front_rotation_init();
		MYSERIAL0.println("front rotation init ok......\r\n");
		front_rotation_init_flag = 1;
	}

	bool s_seen = parser.seen('S');
	if(s_seen){
		speed = parser.intval('S');
		speed = scope_limit(1,speed,1023);
		set_motion_speed(SERO_1, speed);
		HAL_Delay(100);		
	}

	bool t_seen = parser.seen('T');
	if(t_seen){
		torque_val = parser.intval('T');
		torque_val = scope_limit(1,torque_val,1023);
		set_torque_limt(SERO_1,torque_val);
		HAL_Delay(100);	
	}

	bool p_seen = parser.seen('P');
	if(p_seen){
		positon = parser.intval('P');
		positon = scope_limit(1,positon,1023);
		set_pos(SERO_1, positon);
		HAL_Delay(100);		
	}

	positon = read_pos(SERO_1);
	HAL_Delay(100);
	speed = read_motion_speed(SERO_1);
	HAL_Delay(100);	

	char str[30];
	memset(&str,0,30);
	sprintf(str,"surrent positon = %d",positon);
	MYSERIAL0.println(str);
	sprintf(str,"surrent speed = %d",speed);
	MYSERIAL0.println(str);

	MYSERIAL0.println("ok");
}

void GcodeSuite::M2012()
{
	feedRate_t e_feedrate_mm_s;
	int direction;
  	if (parser.linearval('F') > 0)
    	e_feedrate_mm_s = parser.value_feedrate();
	if (parser.linearval('D') > 0)
    	direction =parser.floatval('D', 999);;
	rail_init(e_feedrate_mm_s, direction);
}

void GcodeSuite::M2013()
{
	SERIAL_ECHOPAIR("Conveyor belt stop\r\n");
	rail_disable();
}

void GcodeSuite::M5201314()
{
	while(1){
		OUT_WRITE(UART1_TX_PIN, HIGH); 
		delay(100);
		OUT_WRITE(UART1_RX_PIN, HIGH); 
		delay(100);
		TOGGLE(LASER_PWM_PIN);  // heartbeat indicator
		delay(100);
		OUT_WRITE(UART1_TX_PIN, LOW); 
		delay(100);
		OUT_WRITE(UART1_RX_PIN, LOW); 
		delay(100);
		TOGGLE(LASER_PWM_PIN);  // heartbeat indicator
		delay(100);
    }
}

void GcodeSuite::M5010000()
{
	SERIAL_ECHOPAIR("Reset UART3 GPIO Level, please connect touch screen to DexARM\r\n");
	while(1){
		OUT_WRITE(UART3_RX_PIN,HIGH);
		OUT_WRITE(UART3_TX_PIN,LOW);
	}
}
