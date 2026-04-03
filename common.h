/* ---------------------------------------------------------------
 * Copyright(C) 2024, BLUTEK Co., LTD. All Right Reserved.
 * ---------------------------------------------------------------
 * Author     : ChongHan Park (E-mail: chpark@blutek.co.kr)
 * Filename   : common.h
 * Created on : 20250718
 * Description:
 * ---------------------------------------------------------------
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdio.h>
#include <unistd.h>

#include <stdint.h>

#include <math.h>
#pragma pack(1)
/**************************** Type Definitions *******************************/
///////////////////////////////////////////////CAN-FD RPU0////////////////////////////////////////////////////////////////////
typedef struct
{
	int16_t IMU_roll;
	int16_t IMU_pitch;
	int16_t IMU_yaw;
	int16_t IMU_pitch_vel;
	int16_t IMU_acc_x;
	int16_t IMU_acc_y;
	int16_t IMU_acc_z;
	uint16_t hip_joint_loadcell;
	uint16_t Knee_loadcell;
	uint16_t hip_joint_potentiometer;
	uint16_t Knee_potentiometer;
	uint16_t thigh_flexible_sensor1;
	uint16_t thigh_flexible_sensor2;
	uint16_t thigh_flexible_sensor3;
	uint16_t reserved1;
	uint16_t reserved2;
}LegDataL_t;

typedef struct
{
	int16_t IMU_roll;
	int16_t IMU_pitch;
	int16_t IMU_yaw;
	int16_t IMU_pitch_vel;
	int16_t IMU_acc_x;
	int16_t IMU_acc_y;
	int16_t IMU_acc_z;
	uint16_t foot_floor_flexible_sensor1;
	uint16_t foot_floor_flexible_sensor2;
	uint16_t foot_floor_flexible_sensor3;
	uint16_t foot_floor_flexible_sensor4;
	uint16_t foot_floor_flexible_sensor5;
	uint16_t foot_floor_flexible_sensor6;
	uint16_t foot_floor_flexible_sensor7;
	uint16_t foot_floor_flexible_sensor8;
	uint16_t reserved;
}FootDataL_t;

typedef struct
{
	int16_t IMU_roll;
	int16_t IMU_pitch;
	int16_t IMU_yaw;
	int16_t IMU_pitch_vel;
	int16_t IMU_acc_x;
	int16_t IMU_acc_y;
	int16_t IMU_acc_z;
	uint16_t hip_joint_loadcell;
	uint16_t Knee_loadcell;
	uint16_t hip_joint_potentiometer;
	uint16_t Knee_potentiometer;
	uint16_t thigh_flexible_sensor1;
	uint16_t thigh_flexible_sensor2;
	uint16_t thigh_flexible_sensor3;
	uint16_t reserved1;
	uint16_t reserved2;
}LegDataR_t;

typedef struct
{
	int16_t IMU_roll;
	int16_t IMU_pitch;
	int16_t IMU_yaw;
	int16_t IMU_pitch_vel;
	int16_t IMU_acc_x;
	int16_t IMU_acc_y;
	int16_t IMU_acc_z;
	uint16_t foot_floor_flexible_sensor1;
	uint16_t foot_floor_flexible_sensor2;
	uint16_t foot_floor_flexible_sensor3;
	uint16_t foot_floor_flexible_sensor4;
	uint16_t foot_floor_flexible_sensor5;
	uint16_t foot_floor_flexible_sensor6;
	uint16_t foot_floor_flexible_sensor7;
	uint16_t foot_floor_flexible_sensor8;
	uint16_t reserved;
}FootDataR_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	LegDataL_t leg_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}LegDataMemL_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	FootDataL_t foot_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}FootDataMemL_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	LegDataR_t leg_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}LegDataMemR_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	FootDataR_t foot_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}FootDataMemR_t;



///////////////////////////////////////////////////CAN-2.0 RPU1/////////////////////////////////////////////////////////////////////////////////////////
///////////////// 상지 -> 주제어기  //////////////////////////////////
typedef struct
{
	uint16_t loadcell_1;
	uint16_t loadcell_2;
	uint16_t loadcell_3;
	int16_t shoulder_yaw_position;
}upperL_recv_110_t;

typedef struct
{
	uint16_t Potentiometer_1;
	uint16_t Potentiometer_2;
	uint16_t Potentiometer_3;
	int16_t elbow_yaw_position;
}upperL_recv_120_t;

typedef struct
{
	int16_t FT_sensor_Fx;
	int16_t FT_sensor_Fy;
	int16_t FT_sensor_Fz;
	uint8_t switch_on_off;
	uint8_t reserved;
}upperL_recv_130_t;

typedef struct
{
	int16_t FT_sensor_Fx;
	int16_t FT_sensor_Ty;
	int16_t FT_sensor_Tz;
	uint8_t mode_echo;
	uint8_t sub_mode_echo;
}upperL_recv_140_t;

typedef struct
{
	int16_t Potentiometer1_velocity;
	int16_t Potentiometer2_velocity;
	int16_t Potentiometer3_velocity;
	uint16_t  reserved;
}upperL_recv_150_t;

typedef struct
{
	uint16_t loadcell_1;
	uint16_t loadcell_2;
	uint16_t loadcell_3;
	int16_t shoulder_yaw_position;
}upperR_recv_110_t;

typedef struct
{
	uint16_t Potentiometer_1;
	uint16_t Potentiometer_2;
	uint16_t Potentiometer_3;
	int16_t elbow_yaw_position;
}upperR_recv_120_t;

typedef struct
{
	int16_t FT_sensor_Fx;
	int16_t FT_sensor_Fy;
	int16_t FT_sensor_Fz;
	uint8_t switch_on_off;
	uint8_t reserved;
}upperR_recv_130_t;

typedef struct
{
	int16_t FT_sensor_Fx;
	int16_t FT_sensor_Ty;
	int16_t FT_sensor_Tz;
	uint8_t mode_echo;
	uint8_t sub_mode_echo;
}upperR_recv_140_t;

typedef struct
{
	int16_t Potentiometer1_velocity;
	int16_t Potentiometer2_velocity;
	int16_t Potentiometer3_velocity;
	uint16_t  reserved;
}upperR_recv_150_t;

typedef struct
{
	uint8_t drive_mode;
	uint8_t drive_sub_mode;
	uint16_t reserved1;
	uint16_t reserved2;
	uint16_t reserved3;
}upperL_send_99_t;

typedef struct
{
	int16_t shoulder_roll_cmd;
	int16_t shoulder_pitch_cmd;
	int16_t elbow_pitch_cmd;
	int16_t elbow_yaw_cmd;
}upperL_send_100_t;

typedef struct
{
	uint8_t drive_mode;
	uint8_t drive_sub_mode;
	uint16_t reserved1;
	uint16_t reserved2;
	uint16_t reserved3;
}upperR_send_99_t;

typedef struct
{
	int16_t shoulder_roll_cmd;
	int16_t shoulder_pitch_cmd;
	int16_t elbow_pitch_cmd;
	int16_t elbow_yaw_cmd;
}upperR_send_100_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperL_recv_110_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperL_recv_110_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperL_recv_120_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperL_recv_120_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperL_recv_130_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperL_recv_130_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperL_recv_140_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperL_recv_140_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperL_recv_150_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperL_recv_150_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperR_recv_110_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperR_recv_110_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperR_recv_120_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperR_recv_120_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperR_recv_130_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperR_recv_130_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperR_recv_140_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperR_recv_140_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperR_recv_150_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperR_recv_150_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperL_send_99_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperL_send_99_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperL_send_100_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperL_send_100_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperR_send_99_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperR_send_99_mem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	upperR_send_100_t data;
	uint8_t padding[64 - 14]; // <-- 64바이트 패딩 (50바이트 추가)
}upperR_send_100_mem_t;

/////////////////////////////////////////////////////////////////////

typedef struct
{
	uint16_t seq;
	uint32_t id;
	uint8_t rxd[8];
}RecvData_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	uint8_t rxd[8];
}SendData_t;

//////////////////////상지 유연센서 보드 CAN-FD RPU1 ///////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	int16_t IMU_roll;
	int16_t IMU_pitch;
	int16_t IMU_yaw;
	int16_t IMU_pitch_vel;
	int16_t IMU_acc_x;
	int16_t IMU_acc_y;
	int16_t IMU_acc_z;
	uint16_t upper_flexible_sensor_1;
	uint16_t upper_flexible_sensor_2;
	uint16_t upper_flexible_sensor_3;
	uint16_t upper_flexible_sensor_4;
	uint16_t upper_flexible_sensor_5;
	uint16_t upper_flexible_sensor_6;
	uint16_t upper_flexible_sensor_7;
	uint16_t upper_flexible_sensor_8;
	uint16_t reserved;
}FlexibleSensorL_t;

typedef struct
{
	int16_t IMU_roll;
	int16_t IMU_pitch;
	int16_t IMU_yaw;
	int16_t IMU_pitch_vel;
	int16_t  IMU_acc_x;
	int16_t  IMU_acc_y;
	int16_t  IMU_acc_z;
	uint16_t upper_flexible_sensor_1;
	uint16_t upper_flexible_sensor_2;
	uint16_t upper_flexible_sensor_3;
	uint16_t upper_flexible_sensor_4;
	uint16_t upper_flexible_sensor_5;
	uint16_t upper_flexible_sensor_6;
	uint16_t upper_flexible_sensor_7;
	uint16_t upper_flexible_sensor_8;
	uint16_t reserved;
}FlexibleSensorR_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	FlexibleSensorL_t flexible_sensor_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}FlexibleSensorLMem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	FlexibleSensorR_t flexible_sensor_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}FlexibleSensorRMem_t;
///////////////// 유압 구동기  CAN-FD RPU1//////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	uint16_t pressure_sensor1;				//압력센서1
	uint16_t pressure_sensor2;				//압력센서2
	uint16_t temperature_sensor1;			//온도센서1
	uint16_t temperature_sensor2;			//압력센서2
	int16_t  hydraulic_pump_rpm;			//유압펌프RPM
	uint16_t swash_plate_angle1;			//사각판도1
	uint16_t swash_plate_angle2;			//사각판도2
	uint16_t sol_valve1_feedback;			//솔밸브1피드백
	uint16_t sol_valve2_feedback;			//솔밸브2피드백
	uint16_t sol_valve3_feedback;			//솔밸브3피드백
	uint16_t reserved1;
	uint16_t reserved2;
	uint16_t reserved3;
	uint16_t reserved4;
	uint16_t reserved5;
	uint16_t reserved6;
}HydraulicActuatorRecv_t;

typedef struct
{
	int16_t hip_joint_l_drv_cmd;   			// 고관절L 	구동명령
	int16_t Knee_l_drv_cmd;					// 무릎관절L 	구동명령
	int16_t hip_joint_r_drv_cmd;  			// 고관절R	구동명령
	int16_t	Knee_r_drv_cmd;					// 무릎관절R 	구동명령
	int16_t	pump_motor_velocity_cmd;		// 펌프모터 	속도명령
	uint16_t pump_motor_on_off_cmd;			// 펌프모터	on/off
	int16_t	pump_l_swash_plate_run_cmd;		// 펌프L 사판 구동명령
	int16_t pump_r_swash_plate_run_cmd;		// 펌프R 사판 구동명령
	uint8_t fan_1_velocity_cmd;				// FAN #1 속도명령
	uint8_t fan_2_velocity_cmd;				// FAN #2 속도명령
	uint8_t fan_3_velocity_cmd;				// FAN #3 속도명령
	uint8_t fan_4_velocity_cmd;				// FAN #4 속도명령
	uint16_t sol_valve1_drv_cmd;			// 솔 밸브1 구동명령
	uint16_t sol_valve2_drv_cmd;			// 솔 밸브2 구동명령
	uint16_t sol_valve3_drv_cmd;			// 솔 밸브3 구동명령
	uint16_t reserved1;
	uint16_t reserved2;
	uint16_t reserved3;
}HydraulicActuatorSend_t;
typedef struct
{
	uint16_t seq;
	uint32_t id;
	HydraulicActuatorRecv_t hydraulic_actuator_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}HydraulicActuatorRevcMem_t;

typedef struct
{
	uint16_t seq;
	uint32_t id;
	HydraulicActuatorSend_t hydraulic_actuator_data;
	uint8_t padding[64 - 38]; // <-- 64바이트 패딩 (26바이트 추가)
}HydraulicActuatorSendMem_t;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



typedef struct __status__
{
	uint32_t    status;
} status_t;


/*
 * @brief  sdadcValue
 * 0: X-ROLL
 * 1: Y-PITCH
 * 2: Z-YAW
 * 3: X-ACCEL
 * 4: Y-ACCEL
 * 5: Z-ACEEL
 * 6: X-ROLL VEL
 * 7: Y-PITCH VEL
 * 8: Z-YAW VEL
 */

typedef struct
{
	float X_ROLL;
	float Y_PITCH;
	float Z_YAW;
	float X_ACCEL;
	float Y_ACCEL;
	float Z_ACCEL;
	float X_ROLL_VEL;
	float Y_PITCH_VEL;
	float Z_YAW_VEL;
}MtiData_t;

typedef struct
{
	uint16_t  seq;
	MtiData_t mti_data;
}MtiDataMem_t;

typedef struct
{
	uint8_t	LED1;
	uint8_t	LED2;
	uint8_t	LED3;
	uint8_t	LED4;
	uint8_t	push_Spare[9];
}ConData_t;


typedef struct
{
	uint8_t		battery_status;
	uint8_t		error_code;
	uint16_t	pack_voltage;
	uint16_t	pack_current;
	int8_t		temperature;
	uint8_t  	SOC;
	uint8_t  	reserved1;
	uint8_t  	reserved2;
}BmsData_t;

typedef struct
{
	uint16_t  seq;
	BmsData_t bms_data;
	uint8_t padding[64 - 15]; // <-- 64바이트 패딩 (26바이트 추가)
}BmsDataMem_t;


typedef struct
{
	uint16_t  seq;
	ConData_t con_data;
	uint8_t padding[64 - 15]; // <-- 64바이트 패딩 (26바이트 추가)
}ConDataMem_t;

typedef struct
{
	uint8_t	battery_level_status;
	uint8_t  PowerLed;
	uint8_t  errorLed;
	uint8_t	Spare[10];
}ConSendData_t;

typedef struct
{
	uint16_t		 seq;
	ConSendData_t con_data;
	uint8_t padding[64 - 15]; // <-- 64바이트 패딩 (26바이트 추가)
}ConSendDataMem_t;


////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	uint16_t MSG_ID;
	uint8_t UdpDataPayload[64];
}UdpData_t;

typedef struct
{
	float IMU_roll;
	float IMU_pitch;
	float IMU_yaw;
	float IMU_pitch_vel;
	float IMU_acc_x;
	float IMU_acc_y;
	float IMU_acc_z;
	float hip_joint_loadcell;
	float Knee_loadcell;
	float hip_joint_potentiometer;
	float Knee_potentiometer;
	float thigh_flexible_sensor1;
	float thigh_flexible_sensor2;
	float thigh_flexible_sensor3;
	float Pressure_sensor;
	uint32_t Reserved1;
}UdpLensL_t;

typedef struct
{
	float IMU_roll;
	float IMU_pitch;
	float IMU_yaw;
	float IMU_pitch_vel;
	float IMU_acc_x;
	float IMU_acc_y;
	float IMU_acc_z;
	float hip_joint_loadcell;
	float Knee_loadcell;
	float hip_joint_potentiometer;
	float Knee_potentiometer;
	float thigh_flexible_sensor1;
	float thigh_flexible_sensor2;
	float thigh_flexible_sensor3;
	float Pressure_sensor;
	uint32_t Reserved1;
}UdpLensR_t;



typedef struct
{
	float IMU_roll;
	float IMU_pitch;
	float IMU_yaw;
	float IMU_pitch_vel;
	float IMU_acc_x;
	float IMU_acc_y;
	float IMU_acc_z;
	float force_sensor_Fx;
	float force_sensor_Fy;
	float force_sensor_Fz;
	float foot_floor_flexible_sensor1;
	float foot_floor_flexible_sensor2;
	float foot_floor_flexible_sensor3;
	float foot_floor_flexible_sensor4;
	float foot_floor_flexible_sensor5;
	float foot_floor_flexible_sensor6;
}UdpFootL_t;


typedef struct
{
	float IMU_roll;
	float IMU_pitch;
	float IMU_yaw;
	float IMU_pitch_vel;
	float IMU_acc_x;
	float IMU_acc_y;
	float IMU_acc_z;
	float force_sensor_Fx;
	float force_sensor_Fy;
	float force_sensor_Fz;
	float foot_floor_flexible_sensor1;
	float foot_floor_flexible_sensor2;
	float foot_floor_flexible_sensor3;
	float foot_floor_flexible_sensor4;
	float foot_floor_flexible_sensor5;
	float foot_floor_flexible_sensor6;
}UdpFootR_t;

typedef struct
{
	float upper_limb_flexible_sensor_1;
	float upper_limb_flexible_sensor_2;
	float upper_limb_flexible_sensor_3;
	float upper_limb_flexible_sensor_4;
	float upper_limb_flexible_sensor_5;
	float upper_limb_flexible_sensor_6;
	float upper_limb_flexible_sensor_7;
	float upper_limb_flexible_sensor_8;
	float FT_sensor_Mx;
	float FT_sensor_My;
	float FT_sensor_Mz;
	uint32_t Reserved1;
	uint32_t Reserved2;
	uint32_t Reserved3;
	uint32_t Reserved4;
	uint32_t Reserved5;
}UdpFlexibleSensorL_t;

typedef struct
{
	float upper_limb_flexible_sensor_1;
	float upper_limb_flexible_sensor_2;
	float upper_limb_flexible_sensor_3;
	float upper_limb_flexible_sensor_4;
	float upper_limb_flexible_sensor_5;
	float upper_limb_flexible_sensor_6;
	float upper_limb_flexible_sensor_7;
	float upper_limb_flexible_sensor_8;
	float FT_sensor_Fx;
	float FT_sensor_Fy;
	float FT_sensor_Fz;
	uint32_t Reserved1;
	uint32_t Reserved2;
	uint32_t Reserved3;
	uint32_t Reserved4;
	uint32_t Reserved5;
}UdpFlexibleSensorR_t;

typedef struct
{
	uint16_t pressure_sensor1;
	uint16_t pressure_sensor2;
	uint16_t temperature_sensor1;
	uint16_t temperature_sensor2;
	uint16_t hydraulic_pump_rpm;
	uint16_t FAN_1_velocity;
	uint16_t FAN_2_velocity;
	uint32_t Reserved1;
	uint32_t Reserved2;
	uint32_t Reserved3;
	uint32_t Reserved4;
	uint32_t Reserved5;
	uint32_t Reserved6;
	uint32_t Reserved7;
	uint32_t Reserved8;
	uint32_t Reserved9;
}UdpHydraulicActuatorRecv_t;


///////////////////////////////test////////////////////////
typedef struct
{
	uint64_t start_count;
	uint64_t end_count;
	uint64_t mesuer_count;
	uint8_t padding[64 - 24]; // <-- 64바이트 패딩 (26바이트 추가)
}MemoryData_t;
////////////////////////////////////////////////////////
#define CASH_LINE_64   64UL

////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma pack()
#define BASE_MEM_ADDRESS  				(0x70000000)
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
#define SHARED_MEM_UPPER_L_SEND_99 		(BASE_MEM_ADDRESS + 0U)
#define SHARED_MEM_UPPER_L_SEND_100		(CASH_LINE_64)
#define SHARED_MEM_UPPER_L_RECV_110 	(SHARED_MEM_UPPER_L_SEND_100 + CASH_LINE_64)
#define SHARED_MEM_UPPER_L_RECV_120 	(SHARED_MEM_UPPER_L_RECV_110 + CASH_LINE_64)
#define SHARED_MEM_UPPER_L_RECV_130 	(SHARED_MEM_UPPER_L_RECV_120 + CASH_LINE_64)
#define SHARED_MEM_UPPER_L_RECV_140 	(SHARED_MEM_UPPER_L_RECV_130 + CASH_LINE_64)
#define SHARED_MEM_UPPER_L_RECV_150 	(SHARED_MEM_UPPER_L_RECV_140 + CASH_LINE_64)
/////////////////////////////////////////////////////////////////////////////////////////////
#define SHARED_MEM_UPPER_R_SEND_99		(SHARED_MEM_UPPER_L_RECV_150 + CASH_LINE_64)
#define SHARED_MEM_UPPER_R_SEND_100		(SHARED_MEM_UPPER_R_SEND_99 +  CASH_LINE_64)
#define SHARED_MEM_UPPER_R_RECV_110 	(SHARED_MEM_UPPER_R_SEND_100 + CASH_LINE_64)
#define SHARED_MEM_UPPER_R_RECV_120 	(SHARED_MEM_UPPER_R_RECV_110 + CASH_LINE_64)
#define SHARED_MEM_UPPER_R_RECV_130 	(SHARED_MEM_UPPER_R_RECV_120 + CASH_LINE_64)
#define SHARED_MEM_UPPER_R_RECV_140 	(SHARED_MEM_UPPER_R_RECV_130 + CASH_LINE_64)
#define SHARED_MEM_UPPER_R_RECV_150 	(SHARED_MEM_UPPER_R_RECV_140 + CASH_LINE_64)
/////////////////////////////RPU-0///////////////////////////////////////////////////////////
#define SHARED_MEM_LEG_L 				(SHARED_MEM_UPPER_R_RECV_150 + CASH_LINE_64)
#define SHARED_MEM_FOOT_L 				(SHARED_MEM_LEG_L + CASH_LINE_64)
#define SHARED_MEM_LEG_R 				(SHARED_MEM_FOOT_L + CASH_LINE_64)
#define SHARED_MEM_FOOT_R 				(SHARED_MEM_LEG_R + CASH_LINE_64)
////////////////////////////////////////////////////////////////////////////////////////////
#define SHARED_MEM_FS_L 				(SHARED_MEM_FOOT_R + CASH_LINE_64)
#define SHARED_MEM_FS_R 				(SHARED_MEM_FS_L + CASH_LINE_64)
#define SHARED_MEM_HA_RECV 				(SHARED_MEM_FS_R + CASH_LINE_64)
#define SHARED_MEM_HA_SEND 				(SHARED_MEM_HA_RECV + CASH_LINE_64)
////////////////////////////////////////////////////////////////////////////////////////////
#define SHARED_MEM_IMU					(SHARED_MEM_HA_SEND + CASH_LINE_64)
#define SHARED_MEM_CON					(SHARED_MEM_IMU + CASH_LINE_64)
#define SHARED_MEM_BMS					(SHARED_MEM_CON + CASH_LINE_64)
#define SHARED_MEM_CON_SEND				(SHARED_MEM_BMS + CASH_LINE_64)
#define SHARED_MEM_PWM					(SHARED_MEM_CON_SEND + CASH_LINE_64)
///////////////////////////test code/////////////////////////////////////
#define SHARED_MEM_COUNT				(SHARED_MEM_PWM + CASH_LINE_64)
///////////////////////////test code - end/////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
#define LEG_DATA_SIZE sizeof(LegData_t)
#define FOOT_DATA_SIZE sizeof(FootData_t)
/////////////////////////////////////////////////////////////////////////
#define FS_L_DATA_SIZE sizeof(FlexibleSensorL_t)
#define FS_R_DATA_SIZE sizeof(FlexibleSensorR_t)
/////////////////////////////////////////////////////////////////////////
#define HA_RECV_DATA_SIZE  sizeof(HydraulicActuatorRecv_t)
#define HA_SEND_DATA_SIZE  sizeof(HydraulicActuatorSend_t)
/////////////////////////////////////////////////////////////////////////
#define LEG_SIZE sizeof(LegDataMem_t)
#define FOOT_SIZE sizeof(FootDataMem_t)
/////////////////////////////////////////////////////////////////////////
#define FS_L_SIZE  sizeof(FlexibleSensorLMem_t)
#define FS_R_SIZE  sizeof(FlexibleSensorRMem_t)
/////////////////////////////////////////////////////////////////////////
#define HA_RECV_SIZE  sizeof(HydraulicActuatorRevcMem_t)
#define HA_SEND_SIZE  sizeof(HydraulicActuatorSendMem_t)
/////////////////////////////////////////////////////////////////////////
#define SIZE_UL_110 sizeof(upperL_recv_110_mem_t)
#define SIZE_UL_120 sizeof(upperL_recv_120_mem_t)
#define SIZE_UL_130 sizeof(upperL_recv_130_mem_t)
#define SIZE_UL_140 sizeof(upperL_recv_140_mem_t)
#define SIZE_UL_150 sizeof(upperL_recv_150_mem_t)
/////////////////////////////////////////////////////////////////////////
#define SIZE_UR_110 sizeof(upperR_recv_110_mem_t)
#define SIZE_UR_120 sizeof(upperR_recv_120_mem_t)
#define SIZE_UR_130 sizeof(upperR_recv_130_mem_t)
#define SIZE_UR_140 sizeof(upperR_recv_140_mem_t)
#define SIZE_UR_150 sizeof(upperR_recv_150_mem_t)
/////////////////////////////////////////////////////////////////////////
#define SIZE_SEND_UL_99  sizeof(upperL_send_99_mem_t)
#define SIZE_SEND_UR_99  sizeof(upperR_send_99_mem_t)
#define SIZE_SEND_UL_100 sizeof(upperL_send_100_mem_t)
#define SIZE_SEND_UR_100 sizeof(upperR_send_100_mem_t)
/////////////////////////////////////////////////////////////////////////
#define SIZE_SEND_UDP sizeof(UdpData_t)
#define SIZE_PAYLOAD_UDP 64
///////////////////////////test code/////////////////////////////////////
#define SIZE_MEM_TIME_COUNT sizeof(TimeCount_t)
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////ID//////////////////////////////////////////////


#define ID_UL_110		0x110
#define ID_UL_120		0x120
#define ID_UL_130		0x130
#define ID_UL_140		0x140
#define ID_UL_150		0x150
#define ID_UL_SEND_99	0x99
#define ID_UL_SEND_100	0x100

#define ID_UR_110		0x110
#define ID_UR_120		0x120
#define ID_UR_130		0x130
#define ID_UR_140		0x140
#define ID_UL_150		0x150
#define ID_UR_SEND_99	0x99
#define ID_UR_SEND_100	0x100

#define TIMER_1MS		(1000000) /* 1ms */
#define TIMER_10MS		(10000000) /* 100ms */
#define TIMER_100MS		(100000000) /* 100ms */
#define TIMER_1000MS	(1000000000) /* 100ms */

/*****************************************************************************/
/**
*
* @brief    Perform a 16-bit endian conversion.
*
* @param	Data: 16 bit value to be converted
*
* @return	16 bit Data with converted endianness
* */
static inline uint16_t Xil_EndianSwap16(uint16_t Data)
{
	return (uint16_t) (((Data & 0xFF00U) >> 8U) | ((Data & 0x00FFU) << 8U));
}


/*****************************************************************************/
/**
*
* @brief    Perform a 32-bit endian conversion.
*
* @param	Data: 32 bit value to be converted
*
* @return	32 bit data with converted endianness
*
******************************************************************************/
static inline uint16_t Xil_EndianSwap32(uint32_t Data)
{
	uint16_t LoWord;
	uint16_t HiWord;

	/* get each of the half words from the 32 bit word */

	LoWord = (uint16_t) (Data & 0x0000FFFFU);
	HiWord = (uint16_t) ((Data & 0xFFFF0000U) >> 16U);

	/* byte swap each of the 16 bit half words */

	LoWord = (uint16_t)(((LoWord & 0xFF00U) >> 8U) | ((LoWord & 0x00FFU) << 8U));
	HiWord = (uint16_t)(((HiWord & 0xFF00U) >> 8U) | ((HiWord & 0x00FFU) << 8U));

	/* swap the half words before returning the value */

	return ((((uint32_t)LoWord) << (uint32_t)16U) | (uint32_t)HiWord);
}

#define LPRINTF(format, ...) \
  printf("Linux> " format, ##__VA_ARGS__)

#define LPERROR(format, ...) LPRINTF("ERROR: " format, ##__VA_ARGS__)

void DataInit();
//////////////////////////////////////////////////////////////////////////////////////////////
#endif /* __COMMON_H__ */
