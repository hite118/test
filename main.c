/* ---------------------------------------------------------------
 * Copyright(C) 2024, BLUTEK Co., LTD. All Right Reserved.
 * ---------------------------------------------------------------
 * Author     : ChongHan Park (E-mail: chpark@blutek.co.kr)
 * Filename   : main.c
 * Created on : 2025. 07-18.
 * Description:
 * ---------------------------------------------------------------
 */


#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "common.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <sys/poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <termios.h>

#include "task.h"



#include <string.h>

#include "axi_pwm.h"

#include <inttypes.h>

#include <errno.h>
//////////////////////////////////////////////////////////////////////////////////////////
char ch;
int getch(void);
int prompt(char *s);

#define NONE	 					0
#define DOMU_SEND_MODE 				1
#define DOMU_RECV_MODE 				2
#define DOMU_SEND_RECV_MODE			3

#define RPU0_SEND 					4
#define RPU0_RECV 					5
#define RPU0_SEND_RECV_MODE 		6

#define RPU1_SEND 					7
#define RPU1_RECV 					8
#define RPU1_SEND_RECV_MODE 		9

#define TEST_MSG "park World - libmetal shared memory demo"


#define AXI_PWM_ADDR (0x80000000)
PWM_DEVICE *PWM;// = (PWM_DEVICE *)AXI_PWM_ADDR;

uint8_t can_data_0_rpu0 = 0;
uint8_t can_data_0_rpu1 = 0;

void *mapped_shmem_addr;
void *mapped_pwm_addr;

int fd;

uint32_t val_change = 0;
////////////////////////////////////////////////////2024-0401 : �ٸ�/�� ���� ���� ///////////////////////////////////////////////////////
LegDataMemL_t *shared_Leg_L;
LegDataMemR_t *shared_Leg_R;

FootDataMemL_t *shared_Foot_L;
FootDataMemR_t *shared_Foot_R;
////////////////////////////////////////////////////2024-0401 : �������� ���� ���� ///////////////////////////////////////////////////////
FlexibleSensorLMem_t *FlexibleSensorLData;
FlexibleSensorRMem_t *FlexibleSensorRData;


HydraulicActuatorRevcMem_t *HydraulicActuatorRecvData;
HydraulicActuatorSendMem_t *HydraulicActuatorSendData;

FlexibleSensorLMem_t FlexibleSensorLDataRead;
FlexibleSensorRMem_t FlexibleSensorRDataRead;

HydraulicActuatorRevcMem_t HydraulicActuatorRecvDataRead;
HydraulicActuatorSendMem_t HydraulicActuatorSendDataWrite;

uint8_t printf_view = 0;
uint8_t send_run = 0;


uint16_t seq_upper_99_l = 0;
uint16_t seq_upper_99_r = 0;
uint16_t seq_upper_100_l = 0;
uint16_t seq_upper_100_r = 0;
///////////////////////////////////////////////////////////CAN2.0 RPU-1/////////////////////////////////////////////////////////////////////

upperL_recv_110_mem_t 		*UpperRecvData110L;
upperL_recv_120_mem_t 		*UpperRecvData120L;
upperL_recv_130_mem_t 		*UpperRecvData130L;
upperL_recv_140_mem_t 		*UpperRecvData140L;
upperL_recv_150_mem_t 		*UpperRecvData150L;

upperR_recv_110_mem_t 		*UpperRecvData110R;
upperR_recv_120_mem_t 		*UpperRecvData120R;
upperR_recv_130_mem_t 		*UpperRecvData130R;
upperR_recv_140_mem_t 		*UpperRecvData140R;
upperR_recv_150_mem_t 		*UpperRecvData150R;

upperL_send_99_mem_t 			*UpperSendData99L;
upperR_send_99_mem_t 			*UpperSendData99R;

upperL_send_100_mem_t 			*UpperSendData100L;
upperR_send_100_mem_t 			*UpperSendData100R;


upperL_send_99_t UperTestSendData99L = {.drive_mode= 0x01, .drive_sub_mode = 0x02, .reserved1=0x0A, .reserved2 = 0x0B, .reserved3 = 0x0C};
upperR_send_99_t UperTestSendData99R = {.drive_mode= 0x03, .drive_sub_mode = 0x04, .reserved1=0x0A, .reserved2 = 0x0B, .reserved3 = 0x0C};

upperL_send_100_t UperTestSendData100L = {.shoulder_roll_cmd = -10000, .shoulder_pitch_cmd = -10000, .elbow_pitch_cmd=-10000, .elbow_yaw_cmd = 0};
upperR_send_100_t UperTestSendData100R = {.shoulder_roll_cmd = -10000, .shoulder_pitch_cmd = -10000, .elbow_pitch_cmd=-10000, .elbow_yaw_cmd = 0};


uint16_t Upper_L_seq = 0;
uint16_t Upper_R_seq = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static unsigned short crc_table[256] =
{
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

u16 ssCRC16_ccitt(uint8_t *buf, u8 len)
{
  u8 counter;
  u16 crc = 0;

  for( counter = 0; counter < len; counter++)
  {
    crc = (crc<<8) ^ crc_table[((crc>>8) ^ *(char *) buf++)&0x00FF];
  }
  return crc;
}

////////////////////////////////////////////////////2024-0416///////////////////////////////////////////////////////
MtiDataMem_t 				*sharedImu;
ConDataMem_t 				*sharedCon;
BmsDataMem_t 				*sharedBms;
ConSendDataMem_t  			*sharedConSend;
ConSendDataMem_t			write_ConSend;

uint16_t seq_con_send = 0;

MemoryData_t  			*sharedCount;


/////////////////////////////////////////////////////////////////
UdpFlexibleSensorL_t UdpFlexibleSensorLData;
UdpFlexibleSensorR_t UdpFlexibleSensorRData;
UdpHydraulicActuatorRecv_t UdpHydraulicActuatorRecvData;

UdpLensL_t UdpLeg_L;
UdpLensR_t UdpLeg_R;
UdpFootL_t UdpFoot_L;
UdpFootR_t UdpFoot_R;
UdpData_t SendData;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int period = 1;
unsigned int s_period = 0;
status_t * pdata;

//static task_t __send_task__;
static task_t __print__;
static task_t __send_task__;
static timer_t timer = NULL;

void* print_task(void *a);
void* send_task(void *a);

void send_timer(sigval_t sig);
//void* timer_thread(void* arg);

uint8_t run_mode = 0;

uint32_t tx_count = 0;
uint32_t rx_count = 0;

uint32_t rpu0_tx_cnt = 0;
uint32_t rpu0_rx_cnt = 0;

uint32_t rpu1_tx_cnt = 0;
uint32_t rpu1_rx_cnt = 0;

uint8_t cnt = 0;

#define NUM_TEST_DATA	4
#define SHM_SIZE		4096

int shmem_open();

void LEG_FOOT_L_PRINT(void);
void LEG_FOOT_R_PRINT(void);

void FLEXIBLE_PRINT(void);
void HYDRAULIC_PRINT(void);
void HYDRAULIC_SEND_PRINT(void);

void UP_L_PRINT(void);
void UP_R_PRINT(void);
void UP_L_SEND_PRINT(void);
void UP_R_SEND_PRINT(void);
void IMU_PRINT(void);
void CON_PRINT(void);
void BMS_PRINT(void);
void Period_PRINT(void);
void UHCB_L_SEND_PRINT(void);
void UHCB_R_SEND_PRINT(void);
int fd_mem, fd_pwm;
///////////////////////////////////////
#define BUF_SIZE 20
//char buf[BUF_SIZE] = {0xaa, 0x55, 0x70, 0x0d, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  0x00, 0x28, 0x88, 0xcc};

int n;
#define STX1 			0xAA
#define STX2 			0x55
#define MSG_ID 			0x70
#define LEN_VAL 		0x0D
#define ETX 			0xCC
#define MSG_ID_SEND 	0xF0
#define MSG_ID_RECV 	0x70
#define SEND_LEN 		0x0D

#define STX1_INDEX    0
#define STX2_INDEX    1
#define MSG_ID_INDEX  2
#define MSG_LEN_INDEX 3
#define PAYLOAD_INDEX 4
#define CRC1_INDEX    17
#define CRC2_INDEX    18
#define ETX_INDEX     19


#define BATTERY_LEVEL_INDEX 	4
#define POWER_LEN_INDEX 		5
#define ERROR_LEN_INDEX 		6
#define SEND_SPARE_INDEX 		7

#define PACKET_SIZE   20


//////////////////////////////////////
/*Update(2024-08-26) */
int sockfd;
struct sockaddr_in serv_addr;
socklen_t addrlen;
uint16_t PORT = 50001;

//static s32 WriteNumByte = 0;

uint8_t HydraulicActuator_send_flag = 0;
uint8_t upper_send_flag = 0;

uint16_t test_data_16 = 0x1234;
uint32_t test_data_32 = 0x12345678;
uint16_t swapped_data_16;
uint32_t swapped_data_32;
/*****************************************************************************/
/**
*
* Main program for Linux applications :
* 1. Open shared memory
* 2. Sockat Settings
* 3. Create a task
* 4. Key action on keyboard
* @return	int.
*
******************************************************************************/
int main(void)
{
	int ret =0;

/////////////////////////////////////////////////////////////
	if(shmem_open() < 0)
	{
		LPERROR("Failed to initialize system.\n");
	}
	else
	{
		LPRINTF("shmem_domu_open OK \r\n");
	}
/////////////////////////////////////////////////////////////


	memset(&UdpFlexibleSensorLData, 0x01, SIZE_PAYLOAD_UDP);
	memset(&UdpFlexibleSensorRData, 0x01, SIZE_PAYLOAD_UDP);
	memset(&UdpHydraulicActuatorRecvData, 0x01, SIZE_PAYLOAD_UDP);
	memset(&UdpLeg_L, 0x01, SIZE_PAYLOAD_UDP);
	memset(&UdpLeg_R, 0x01, SIZE_PAYLOAD_UDP);
	memset(&UdpFoot_L, 0x01, SIZE_PAYLOAD_UDP);
	memset(&UdpFoot_R, 0x01, SIZE_PAYLOAD_UDP);

	task_create(&__print__,  		print_task,		&pdata);  //
	task_create(&__send_task__,    	send_task,    	&pdata);  // print.

	 /*Update(2024-08-26)  */
    // 16
    swapped_data_16 = Xil_EndianSwap16(test_data_16);


    // 32
    swapped_data_32 = Xil_EndianSwap32(test_data_32);


    /*Update(2024-08-26) */
	if (task_is_running(&__print__))
	{
		LPRINTF("Task __print__ is running.\n");
	}
	else
	{
		LPRINTF("Task __print__ is not running.\n");
	}


	if (task_is_running(&__send_task__))
	{
		LPRINTF("Task __send_task__ is running.\n");
	}
	else
	{
		LPRINTF("Task __send_task__ is not running.\n");
	}


/*
	if(period >= 1000)
	{
		s_period = period / 1000;
		period = (period % 1000);
	}
*/

//	timer_create2(&timer, send_timer, pdata, s_period, (period * TIMER_100MS));
    while (ch != 'q')
    {
//      ch = prompt("\r\n\r\n 1:L-LEN,FOOT-L, 2:R-LEN,FOOT, 3:Flexible, 4:Recv-Hydraulic, 5:Send-Hydraulic, 6:L-Upper 7:R-Upper 8:Send Upper, 9: IMU  a: CON, b: PWM s:STOP(QUIT:q): \n");
    	ch = prompt("\r\n\r\n 1:L-LEG/FOOT, 2:R-LEG/FOOT, 3:L/R-Flexible, 4:Recv-Hydraulic, 5:Send-Hydraulic, 6:L-Upper 7:R-Upper, 8:Send L-Upper, 9:Send R-Upper a:IMU  b:CON, c:BMS, d:PWM e:Period f:UCBH_L g:UHCB_R s:STOP(QUIT:q): \n");
      switch (ch)
      {
        case '1' :
        	printf_view = 1;
            break;
        case '2' :
        	printf_view = 2;
            break;
        case '3' :
        	printf_view = 3;
            break;
        case '4':
        	printf_view = 4;
        	break;
        case '5':
        	printf_view = 5;
			break;
        case '6':
        	printf_view = 6;
			break;
        case '7':
        	printf_view = 7;
			break;
        case '8':
        	printf_view = 8;
			break;
        case '9':
        	printf_view = 9;
			break;
        case 'a':
        	printf_view = 10;
			break;
        case 'b':
          	printf_view = 11;
			break;
        case 'c':
          	printf_view = 12;
			break;
        case 'd':
          	printf_view = 13;
			break;
        case 's':
        	HydraulicActuator_send_flag = 0;
        	upper_send_flag = 0;
        	printf_view = 100;
        	val_change = 0;
        	break;
        case 'e':
        	printf_view = 14;
        	break;
        case 'f':
        	printf_view = 15;
        	break;
        case 'g':
        	printf_view = 16;
        	break;
        default :
        	break;
      }
      usleep(1000);
    }


	if (timer != NULL)
	{
		timer_delete(timer);
	}

	munmap(mapped_shmem_addr, SHM_SIZE);
	munmap(mapped_pwm_addr, 30UL);
	close(fd_mem);
	close(fd_pwm);
    close(sockfd);

    /*Update(2024-08-26) �ŷڼ��˻� : task_is_quit ���*/
    // �ŷڼ� �˻�: task_is_quit ���
	return ret;
}

/*****************************************************************************/
/**
*
* Keyboard Key Input Function
*
* @return	int.
******************************************************************************/
int getch(void)
{
	struct termios buf, save;
	char ch_val;

	tcgetattr (STDIN_FILENO, &save);
	buf = save;
	buf.c_lflag &= ~(unsigned int)(ICANON | ECHO | ECHOE | ISIG);
	buf.c_iflag &= ~(unsigned int)INLCR;
	buf.c_iflag &= ~(unsigned int)ICRNL;
	buf.c_cflag |= (CLOCAL | CREAD);
	buf.c_cflag |= (IXON | IXOFF);
//	buf.c_lflag  = ECHO;
	buf.c_cc[VMIN] = 1;
	buf.c_cc[VTIME] = 0;

	tcflush (STDIN_FILENO, TCIFLUSH);
	tcsetattr (STDIN_FILENO, TCSANOW, &buf);

	read (STDIN_FILENO, &ch_val, 1);

	tcflush (STDIN_FILENO, TCIFLUSH);
	tcsetattr (STDIN_FILENO, TCSANOW, &save);

	return (int)ch_val;
}

/*****************************************************************************/
/**
*
* Return the key entered
*
* @return	int.
*
******************************************************************************/
int prompt(char *s)
{
	int c;
	printf (s);
	fflush (stdout);
	c = getch();
	printf ("\n");
	return c;
}

/*****************************************************************************/
/**
*
* open shared memory on Linux
*
* @return	int.
******************************************************************************/
int shmem_open()
{
	int ret = 0;
	/*open with dev/mem device. Read and write permission OK*/
	fd_mem = open("/dev/mem", O_RDWR | O_CREAT , S_IREAD|S_IWUSR);
    if (fd_mem < 0)
	{
    	LPERROR("open");
    	ret = -1;
    }

    /*open with dev/mem device. Read and write permission OK*/
	fd_pwm = open("/dev/mem", O_RDWR | O_CREAT , S_IREAD|S_IWUSR);
    if (fd_pwm < 0)
	{
    	LPERROR("open");
    	ret = -1;
    }
    /* address mapping */
	mapped_shmem_addr = mmap(NULL, SHM_SIZE, PROT_WRITE | PROT_READ , MAP_SHARED, fd_mem, SHARED_MEM_UPPER_L_SEND_99);
    if (mapped_shmem_addr == MAP_FAILED)
	{
    	LPERROR("mmap");
        close(fd_mem);
        ret = -1;
    }

    /* address mapping */
	mapped_pwm_addr = mmap(NULL, 30UL, PROT_WRITE | PROT_READ , MAP_SHARED, fd_pwm, AXI_PWM_ADDR);
    if (mapped_pwm_addr == MAP_FAILED)
	{
    	LPERROR("mmap");
        close(fd_pwm);
        ret = -1;
    }

    LPRINTF("Shared memory is read from DomU(FreeRTOS). \r\n");

    /* Data-specific shared memory mapping */
    UpperSendData99L	 		=	(upperL_send_99_mem_t		*)	mapped_shmem_addr;
    UpperSendData100L	 		=	(upperL_send_100_mem_t		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_L_SEND_100);
	UpperRecvData110L			=	(upperL_recv_110_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_L_RECV_110);
	UpperRecvData120L			=	(upperL_recv_120_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_L_RECV_120);
	UpperRecvData130L			=	(upperL_recv_130_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_L_RECV_130);
	UpperRecvData140L			=	(upperL_recv_140_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_L_RECV_140);
	UpperRecvData150L			=	(upperL_recv_150_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_L_RECV_150);

	UpperSendData99R			=	(upperR_send_99_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_R_SEND_99);
	UpperSendData100R			=	(upperR_send_100_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_R_SEND_100);
	UpperRecvData110R			=	(upperR_recv_110_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_R_RECV_110);
	UpperRecvData120R			=	(upperR_recv_120_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_R_RECV_120);
	UpperRecvData130R			=	(upperR_recv_130_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_R_RECV_130);
	UpperRecvData140R			=	(upperR_recv_140_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_R_RECV_140);
	UpperRecvData150R			=	(upperR_recv_150_mem_t 		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_UPPER_R_RECV_150);

	shared_Leg_L				= 	(LegDataMemL_t 				*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_LEG_L);
	shared_Foot_L				= 	(FootDataMemL_t 			*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_FOOT_L);
	shared_Leg_R				= 	(LegDataMemR_t 				*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_LEG_R);
	shared_Foot_R				= 	(FootDataMemR_t 			*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_FOOT_R);

	FlexibleSensorLData			=	(FlexibleSensorLMem_t		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_FS_L);
	FlexibleSensorRData 		=	(FlexibleSensorRMem_t		*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_FS_R);

	HydraulicActuatorRecvData	=	(HydraulicActuatorRevcMem_t *)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_HA_RECV);
	HydraulicActuatorSendData	=	(HydraulicActuatorSendMem_t *)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_HA_SEND);

	sharedImu 					=	(MtiDataMem_t				*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_IMU);
	sharedCon					=	(ConDataMem_t 				*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_CON);
	sharedBms					=	(BmsDataMem_t 				*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_BMS);
	sharedConSend				=	(ConSendDataMem_t 			*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_CON_SEND);

	sharedCount					= 	(MemoryData_t	 			*)	((uint8_t *)mapped_shmem_addr + SHARED_MEM_COUNT);

	PWM 						= 	(PWM_DEVICE                 *) mapped_pwm_addr;

    return ret;
}


/*****************************************************************************/
/**
*
* Output function according to key input:
* It is displayed on the screen.
*
******************************************************************************/
void* print_task(void *a)
{
	/* Update(2024-08-26) */
    task_t *task = (task_t *)a;
	while (true)
	{
		switch(printf_view)
		{
		 	 case 0:
		 		 break;
		 	 case 1:
				LEG_FOOT_L_PRINT();
				break;
		 	 case 2:
				LEG_FOOT_R_PRINT();
				break;
		 	 case 3:
				FLEXIBLE_PRINT();
				break;
		 	 case 4:
				HYDRAULIC_PRINT();
				break;
		 	case 5:
				HYDRAULIC_SEND_PRINT();
		 		break;
		 	case 6:
		 		UP_L_PRINT();
		 		break;
		 	case 7:
		 		UP_R_PRINT();
		 		break;
		 	 case 8:
		 		UP_L_SEND_PRINT();
		 		break;
		 	 case 9:
		 		UP_R_SEND_PRINT();
		 		break;
		 	 case 10:
		 		IMU_PRINT();
		 		break;
		 	 case 11:
		 		CON_PRINT();
		 		break;
		 	 case 12:
		 		BMS_PRINT();
		 		break;
		 	 case 13:
				pwm_init(PWM);
				pwm_setup(PWM, 1, 0, 1, 20000);
				pwm_duty(PWM, 50.0F);
		 		break;
		 	 case 14:
		 		Period_PRINT();
		 		break;
		 	 case 15:
		 		UHCB_L_SEND_PRINT();
		 		break;
		 	 case 16:
		 		UHCB_R_SEND_PRINT();
		 		break;
		 	 default :
		 		 break;
		}
		usleep(1000000);
	}
}

/*****************************************************************************/
/**
*
* Ethernet transfer to GUI as transfer task
*
******************************************************************************/
void* send_task(void *a)
{

/*
    task_t *task = (task_t *)a;
    status_t *pdata_send = (status_t *)task_argument(task);
    printf("Task status: 0x%08x\n", pdata_send->status);
*/

#if 0
	sockfd = socket (AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0)
	{
		perror ("ERROR opening socket");
	}
	bzero((char *)&serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	if (inet_pton (AF_INET, "192.168.11.62", &serv_addr.sin_addr) <= 0)
	{
		perror ("ERROR in inet_pton");
		close(sockfd);  // ���� �ݱ�
	}
#endif


 	while (1)
	{
#if 0
 		if(HydraulicActuator_send_flag == 1)
 		{
 			HYDRAULIC_SEND_PRINT();
 		}

 		SendData.MSG_ID=0x1310;
 		memcpy(&SendData.UdpDataPayload, &UdpLens_L, SIZE_PAYLOAD_UDP);
 		WriteNumByte = sendto(sockfd, &SendData, SIZE_SEND_UDP, 0 ,(struct sockaddr*)&serv_addr, sizeof(serv_addr));

 		if (WriteNumByte < 0)
 		{
 			perror("send failed\r\n");
 		}
 		else
 		{

 		}
 		if(HydraulicActuator_send_flag == 1)
 		{
 			HYDRAULIC_SEND_PRINT();
 		}
 		if(upper_send_flag == 1)
 		{
 			UP_SEND_PRINT();
 		}
		usleep(1000);
#endif
	}
}

/*****************************************************************************/
/**
*
* Left leg/foot data output function
*
******************************************************************************/
void LEG_FOOT_L_PRINT(void)
{
	LPRINTF("============================================================Leg_L=================================================== \r\n");
	LPRINTF("shared_Leg_L->id  : 0x%X\r\n",  shared_Leg_L->id);
	LPRINTF("shared_Leg_L->seq : %d\r\n",  shared_Leg_L->seq);
	LPRINTF("shared_Leg_L->leg_data.IMU_roll[deg]   	  : %0.1f\r\n",   			(float)shared_Leg_L->leg_data.IMU_roll * 0.01f);
	LPRINTF("shared_Leg_L->leg_data.IMU_pitch[deg] 	  	  : %0.1f\r\n",   			(float)shared_Leg_L->leg_data.IMU_pitch * 0.01f);
	LPRINTF("shared_Leg_L->leg_data.IMU_yaw[deg] 		  : %0.1f\r\n",   			(float)shared_Leg_L->leg_data.IMU_yaw * 0.01f);
	LPRINTF("shared_Leg_L->leg_data.IMU_pitch_vel[deg]    : %0.1f\r\n",		    (float)shared_Leg_L->leg_data.IMU_pitch_vel * 0.1f);
	LPRINTF("shared_Leg_L->leg_data.IMU_acc_x[mm/s^2] 	  : %d\r\n",		   	shared_Leg_L->leg_data.IMU_acc_x);
	LPRINTF("shared_Leg_L->leg_data.IMU_acc_y[mm/s^2] 	  : %d\r\n",   			shared_Leg_L->leg_data.IMU_acc_y);
	LPRINTF("shared_Leg_L->leg_data.IMU_acc_z[mm/s^2] 	  : %d\r\n",   			shared_Leg_L->leg_data.IMU_acc_z);
	LPRINTF("shared_Leg_L->leg_data.hip_joint_loadcell[V] 		: %f\r\n",   	(float)shared_Leg_L->leg_data.hip_joint_loadcell * 0.0001f);
	LPRINTF("shared_Leg_L->leg_data.Knee_loadcell[V] 			: %f\r\n",   	(float)shared_Leg_L->leg_data.Knee_loadcell * 0.0001f);
	LPRINTF("shared_Leg_L->leg_data.hip_joint_potentiometer[V] : %f\r\n",   	(float)shared_Leg_L->leg_data.hip_joint_potentiometer * 0.0001f);
	LPRINTF("shared_Leg_L->leg_data.Knee_potentiometer[V] 		: %f\r\n",   	(float)shared_Leg_L->leg_data.Knee_potentiometer * 0.0001f);
	LPRINTF("shared_Leg_L->leg_data.thigh_flexible_sensor1[V]  : %0.4f\r\n",   	(float)shared_Leg_L->leg_data.thigh_flexible_sensor1 * 0.0001f);
	LPRINTF("shared_Leg_L->leg_data.thigh_flexible_sensor2[V]  : %0.4f\r\n",   	(float)shared_Leg_L->leg_data.thigh_flexible_sensor2 * 0.0001f);
	LPRINTF("shared_Leg_L->leg_data.thigh_flexible_sensor3[V]  : %0.4f\r\n",   	(float)shared_Leg_L->leg_data.thigh_flexible_sensor3 * 0.0001f);
	LPRINTF("shared_Leg_L->leg_data.reserved1 : %X\r\n",   							shared_Leg_L->leg_data.reserved1);
	LPRINTF("shared_Leg_L->leg_data.reserved2 : %X\r\n",   							shared_Leg_L->leg_data.reserved2);
	LPRINTF("============================================================Foot_L=================================================== \r\n");
	LPRINTF("shared_Foot_L->id :  0x%X\r\n", shared_Foot_L->id);
	LPRINTF("shared_Foot_L->seq : %d\r\n",  shared_Foot_L->seq);
	LPRINTF("shared_Foot_L->foot_data.IMU_roll[deg]  	    : %0.1f\r\n",	(float)shared_Foot_L->foot_data.IMU_roll * 0.01f);
	LPRINTF("shared_Foot_L->foot_data.IMU_pitch[deg] 	    : %0.1f\r\n",	(float)shared_Foot_L->foot_data.IMU_pitch * 0.01f);
	LPRINTF("shared_Foot_L->foot_data.IMU_yaw[deg]	    	: %0.1f\r\n",	(float)shared_Foot_L->foot_data.IMU_yaw * 0.01f);
	LPRINTF("shared_Foot_L->foot_data.IMU_pitch_vel[deg/s]  : %0.1f\r\n",  (float)shared_Foot_L->foot_data.IMU_pitch_vel * 0.1f);
	LPRINTF("shared_Foot_L->foot_data.IMU_acc_x[mm/s^2] 	: %d\r\n", 	shared_Foot_L->foot_data.IMU_acc_x);
	LPRINTF("shared_Foot_L->foot_data.IMU_acc_y[mm/s^2] 	: %d\r\n", 	shared_Foot_L->foot_data.IMU_acc_y);
	LPRINTF("shared_Foot_L->foot_data.IMU_acc_z[mm/s^2] 	: %d\r\n",	shared_Foot_L->foot_data.IMU_acc_z);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor1[V] : %0.4f\r\n",	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor1 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor2[V] : %0.4f\r\n",  	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor2 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor3[V] : %0.4f\r\n",	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor3 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor4[V] : %0.4f\r\n",  	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor4 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor5[V] : %0.4f\r\n",  	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor5 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor6[V] : %0.4f\r\n",  	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor6 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor7[V] : %0.4f\r\n",  	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor7 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.foot_floor_flexible_sensor8[V] : %0.4f\r\n",  	(float)shared_Foot_L->foot_data.foot_floor_flexible_sensor8 * 0.0001f);
	LPRINTF("shared_Foot_L->foot_data.reserved : %X\r\n",  shared_Foot_L->foot_data.reserved);

}
/*****************************************************************************/
/**
*
* Right leg/foot data output function
*
******************************************************************************/
void LEG_FOOT_R_PRINT(void)
{
		LPRINTF("============================================================Leg_R=================================================== \r\n");
		LPRINTF("shared_Leg_R->id  : 0x%X\r\n",  shared_Leg_R->id);
		LPRINTF("shared_Leg_R->seq : %d\r\n",  shared_Leg_R->seq);
		LPRINTF("shared_Leg_R->leg_data.IMU_roll[deg]   		: %0.1f\r\n",   			(float)shared_Leg_R->leg_data.IMU_roll * 0.01f);
		LPRINTF("shared_Leg_R->leg_data.IMU_pitch[deg] 	  		: %0.1f\r\n",   			(float)shared_Leg_R->leg_data.IMU_pitch * 0.01f);
		LPRINTF("shared_Leg_R->leg_data.IMU_yaw[deg] 	    	: %0.1f\r\n",   			(float)shared_Leg_R->leg_data.IMU_yaw * 0.01f);
		LPRINTF("shared_Leg_R->leg_data.IMU_pitch_vel[deg/s]    : %0.1f\r\n",		   	(float)shared_Leg_R->leg_data.IMU_pitch_vel * 0.1f);
		LPRINTF("shared_Leg_R->leg_data.IMU_acc_x[mm/s^2] 		: %d\r\n",		   		shared_Leg_R->leg_data.IMU_acc_x);
		LPRINTF("shared_Leg_R->leg_data.IMU_acc_y[mm/s^2] 		: %d\r\n",   			shared_Leg_R->leg_data.IMU_acc_y);
		LPRINTF("shared_Leg_R->leg_data.IMU_acc_z[mm/s^2] 		: %d\r\n",   			shared_Leg_R->leg_data.IMU_acc_z);
		LPRINTF("shared_Leg_R->leg_data.hip_joint_loadcell[V] 		: %0.4f\r\n",   	(float)shared_Leg_R->leg_data.hip_joint_loadcell * 0.0001f);
		LPRINTF("shared_Leg_R->leg_data.Knee_loadcell[V] 			: %0.4f\r\n",   	(float)shared_Leg_R->leg_data.Knee_loadcell * 0.0001f);
		LPRINTF("shared_Leg_R->leg_data.hip_joint_potentiometer[V]  : %0.4f\r\n",   	(float)shared_Leg_R->leg_data.hip_joint_potentiometer * 0.0001f);
		LPRINTF("shared_Leg_R->leg_data.Knee_potentiometer[V] 		: %0.4f\r\n",   	(float)shared_Leg_R->leg_data.Knee_potentiometer * 0.0001f);
		LPRINTF("shared_Leg_R->leg_data.thigh_flexible_sensor1[V]   : 0.4%f\r\n",   	(float)shared_Leg_R->leg_data.thigh_flexible_sensor1 * 0.0001f);
		LPRINTF("shared_Leg_R->leg_data.thigh_flexible_sensor2[V]   : 0.4%f\r\n",   	(float)shared_Leg_R->leg_data.thigh_flexible_sensor2 * 0.0001f);
		LPRINTF("shared_Leg_R->leg_data.thigh_flexible_sensor3[V]   : 0.4%f\r\n",   	(float)shared_Leg_R->leg_data.thigh_flexible_sensor3 * 0.0001f);
		LPRINTF("shared_Leg_R->leg_data.reserved1 : %X\r\n",   							shared_Leg_R->leg_data.reserved1);
		LPRINTF("shared_Leg_R->leg_data.reserved2 : %X\r\n",   							shared_Leg_R->leg_data.reserved2);
		LPRINTF("============================================================Foot_R=================================================== \r\n");
		LPRINTF("shared_Foot_R->id :  0x%X\r\n", shared_Foot_R->id);
		LPRINTF("shared_Foot_R->seq : %d\r\n",  shared_Foot_R->seq);
		LPRINTF("shared_Foot_R->foot_data.IMU_roll[deg]  	    : %0.1f\r\n",		(float)shared_Foot_R->foot_data.IMU_roll * 0.01f);
		LPRINTF("shared_Foot_R->foot_data.IMU_pitch[deg] 	    : %0.1f\r\n",	(float)shared_Foot_R->foot_data.IMU_pitch  * 0.01f);
		LPRINTF("shared_Foot_R->foot_data.IMU_yaw[deg]	        : %0.1f\r\n",	(float)shared_Foot_R->foot_data.IMU_yaw  * 0.01f);
		LPRINTF("shared_Foot_R->foot_data.IMU_pitch_vel[deg/s]  : %0.1f\r\n",  	(float)shared_Foot_R->foot_data.IMU_pitch_vel * 0.1f);
		LPRINTF("shared_Foot_R->foot_data.IMU_acc_x[mm/s^2] 		: %d\r\n", 	shared_Foot_R->foot_data.IMU_acc_x);
		LPRINTF("shared_Foot_R->foot_data.IMU_acc_y[mm/s^2] 		: %d\r\n", 	shared_Foot_R->foot_data.IMU_acc_y);
		LPRINTF("shared_Foot_R->foot_data.IMU_acc_z[mm/s^2] 		: %d\r\n",	shared_Foot_R->foot_data.IMU_acc_z);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor1[V] : %0.4f\r\n",	(float)shared_Foot_R->foot_data.foot_floor_flexible_sensor1 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor2[V] : %0.4f\r\n",  (float)shared_Foot_R->foot_data.foot_floor_flexible_sensor2 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor3[V] : %0.4f\r\n",	(float)shared_Foot_R->foot_data.foot_floor_flexible_sensor3 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor4[V] : %0.4f\r\n",  (float)shared_Foot_R->foot_data.foot_floor_flexible_sensor4 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor5[V] : %0.4f\r\n",  (float)shared_Foot_R->foot_data.foot_floor_flexible_sensor5 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor6[V] : %0.4f\r\n",  (float)shared_Foot_R->foot_data.foot_floor_flexible_sensor6 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor7[V] : %0.4f\r\n",  (float)shared_Foot_R->foot_data.foot_floor_flexible_sensor7 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.foot_floor_flexible_sensor8[V] : %0.4f\r\n",  (float)shared_Foot_R->foot_data.foot_floor_flexible_sensor8 * 0.0001f);
		LPRINTF("shared_Foot_R->foot_data.reserved : %X\r\n",  shared_Foot_L->foot_data.reserved);
}


/*****************************************************************************/
/**
*
* FLEXIBLE data output function
*
******************************************************************************/
void FLEXIBLE_PRINT(void)
{
	LPRINTF("============================================================FlexibleSensorL=================================================== \r\n");
	LPRINTF("FlexibleSensorLData->id  : 0x%X\r\n",  FlexibleSensorLData->id);
	LPRINTF("FlexibleSensorLData->seq : %d\r\n",  	FlexibleSensorLData->seq);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.IMU_roll[deg]             : %0.1f\r\n",     (float)FlexibleSensorLData->flexible_sensor_data.IMU_roll * 0.01f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.IMU_pitch[deg]            : %0.1f\r\n",  (float)FlexibleSensorLData->flexible_sensor_data.IMU_pitch * 0.01f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.IMU_yaw[deg]		         : %0.1f\r\n",  (float)FlexibleSensorLData->flexible_sensor_data.IMU_yaw * 0.01f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.IMU_pitch_vel[deg/s]	     : %0.1f\r\n",  (float)FlexibleSensorLData->flexible_sensor_data.IMU_pitch_vel * 0.1f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.IMU_acc_x[mm/s^2]	     : %d\r\n",  	FlexibleSensorLData->flexible_sensor_data.IMU_acc_x);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.IMU_acc_y[mm/s^2]	     : %d\r\n",  	FlexibleSensorLData->flexible_sensor_data.IMU_acc_y);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.IMU_acc_z[mm/s^2]         : %d\r\n",  	FlexibleSensorLData->flexible_sensor_data.IMU_acc_z);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_1[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_1 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_2[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_2 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_3[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_3 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_4[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_4 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_5[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_5 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_6[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_6 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_7[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_7 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_8[V] : %0.4f\r\n",  	(float)FlexibleSensorLData->flexible_sensor_data.upper_flexible_sensor_8 * 0.0001f);
	LPRINTF("FlexibleSensorLData->flexible_sensor_data.reserved : %X\r\n",  					FlexibleSensorLData->flexible_sensor_data.reserved);

	LPRINTF("============================================================ FlexibleSensorR=================================================== \r\n");
	LPRINTF("FlexibleSensorRData->id  : 0x%X\r\n",  FlexibleSensorRData->id);
	LPRINTF("FlexibleSensorRData->seq : %d\r\n",  FlexibleSensorRData->seq);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.IMU_roll[deg]             : %0.1f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.IMU_roll * 0.1f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.IMU_pitch[deg] 	         : %0.1f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.IMU_pitch * 0.1f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.IMU_yaw[deg] 		     : %0.1f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.IMU_yaw * 0.1f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.IMU_pitch_vel[deg/s] 	 : %0.1f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.IMU_pitch_vel);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.IMU_acc_x[mm/s^2] 		 : %d\r\n",  		FlexibleSensorRData->flexible_sensor_data.IMU_acc_x);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.IMU_acc_y[mm/s^2] 		 : %d\r\n",  		FlexibleSensorRData->flexible_sensor_data.IMU_acc_y);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.IMU_acc_z[mm/s^2] 		 : %d\r\n",  		FlexibleSensorRData->flexible_sensor_data.IMU_acc_z);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_1[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_1 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_2[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_2 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_3[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_3 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_4[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_4 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_5[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_5 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_6[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_6 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_7[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_7 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_8[V] : %0.4f\r\n",  	(float)FlexibleSensorRData->flexible_sensor_data.upper_flexible_sensor_8 * 0.0001f);
	LPRINTF("FlexibleSensorRData->flexible_sensor_data.reserved[-] : %X\r\n",  						FlexibleSensorRData->flexible_sensor_data.reserved);
}

/*****************************************************************************/
/**
*
* HYDRAULIC data output function
*
******************************************************************************/
void HYDRAULIC_PRINT(void)
{

	LPRINTF("============================================================HydraulicActuator/RECV=================================================== \r\n");
	LPRINTF("HydraulicActuatorRecvData->id  : 0x%X\r\n ",  HydraulicActuatorRecvData->id);
	LPRINTF("HydraulicActuatorRecvData->seq : %d\r\n"	,  HydraulicActuatorRecvData->seq);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.pressure_sensor1[V]    : %0.4f\r\n",       	(float)HydraulicActuatorRecvData->hydraulic_actuator_data.pressure_sensor1  * 0.0001f);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.pressure_sensor2[V]    : %0.4f\r\n",  		(float)HydraulicActuatorRecvData->hydraulic_actuator_data.pressure_sensor2  * 0.0001f);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.temperature_sensor1[V] : %0.4f\r\n",  	    (float)HydraulicActuatorRecvData->hydraulic_actuator_data.temperature_sensor1  * 0.0001f);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.temperature_sensor2[V] : %0.4f\r\n",  		(float)HydraulicActuatorRecvData->hydraulic_actuator_data.temperature_sensor2  * 0.0001f);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.hydraulic_pump_rpm[1/4 RPM] : %d\r\n",  		HydraulicActuatorRecvData->hydraulic_actuator_data.hydraulic_pump_rpm);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.swash_plate_angle1[count]   : %u\r\n",  		HydraulicActuatorRecvData->hydraulic_actuator_data.swash_plate_angle1);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.swash_plate_angle2[count]   : %u\r\n",  		HydraulicActuatorRecvData->hydraulic_actuator_data.swash_plate_angle2);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.sol_valve1_feedback[-] : %d\r\n",  				HydraulicActuatorRecvData->hydraulic_actuator_data.sol_valve1_feedback);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.sol_valve2_feedback[-] : %d\r\n",  				HydraulicActuatorRecvData->hydraulic_actuator_data.sol_valve2_feedback);
	LPRINTF("HydraulicActuatorRecvData->hydraulic_actuator_data.sol_valve3_feedback[-] : %d\r\n",  				HydraulicActuatorRecvData->hydraulic_actuator_data.sol_valve3_feedback);
}

/*****************************************************************************/
/**
*
* HYDRAULIC Send data output function
*
******************************************************************************/
void HYDRAULIC_SEND_PRINT(void)
{
#if 0
	hip_joint_l_drv_cmd = 10000;
	Knee_l_drv_cmd = 10000;
	hip_joint_r_drv_cmd = 10000;
	Knee_r_drv_cmd = 10000;
	pump_l_vane_drv_cmd = 10000;
	pump_r_vane_drv_cmd = 10000;
#endif



	LPRINTF("================================== HydraulicActuator/SEND================ \r\n");
	LPRINTF("HydraulicActuatorSendData->id        : 0x%X\r\n",  	HydraulicActuatorSendData->id);
	LPRINTF("HydraulicActuatorSendData->seq       : %d\r\n",  		HydraulicActuatorSendData->seq);
	LPRINTF("hip_joint_l_drv_cmd[V]        : %d\r\n",      	HydraulicActuatorSendData->hydraulic_actuator_data.hip_joint_l_drv_cmd);
	LPRINTF("Knee_l_drv_cmd[V]             : %d\r\n",  		HydraulicActuatorSendData->hydraulic_actuator_data.Knee_l_drv_cmd);
	LPRINTF("hip_joint_r_drv_cmd[V]        : %d\r\n",      	HydraulicActuatorSendData->hydraulic_actuator_data.hip_joint_r_drv_cmd);
	LPRINTF("Knee_r_drv_cmd[V]             : %d\r\n",      	HydraulicActuatorSendData->hydraulic_actuator_data.Knee_r_drv_cmd);
	LPRINTF("pump_motor_velocity_cmd[1/4 RPM]     : %d\r\n",    	HydraulicActuatorSendData->hydraulic_actuator_data.pump_motor_velocity_cmd);
	LPRINTF("pump_motor_on_off_cmd[-]             : %d\r\n",  		HydraulicActuatorSendData->hydraulic_actuator_data.pump_motor_on_off_cmd);
	LPRINTF("pump_l_swash_plate_run_cmd[1/1000 V] : %d\r\n",      	HydraulicActuatorSendData->hydraulic_actuator_data.pump_l_swash_plate_run_cmd);
	LPRINTF("pump_r_swash_plate_run_cmd[1/1000 V] : %d\r\n",     	HydraulicActuatorSendData->hydraulic_actuator_data.pump_r_swash_plate_run_cmd);
	LPRINTF("fan_1_velocity_cmd[%%]				  : %d\r\n",      	HydraulicActuatorSendData->hydraulic_actuator_data.fan_1_velocity_cmd );
	LPRINTF("fan_2_velocity_cmd[%%] 				  : %d\r\n",    HydraulicActuatorSendData->hydraulic_actuator_data.fan_2_velocity_cmd );
	LPRINTF("fan_3_velocity_cmd[%%] 				  : %d\r\n",  	HydraulicActuatorSendData->hydraulic_actuator_data.fan_3_velocity_cmd );
	LPRINTF("fan_4_velocity_cmd[%%] 				  : %d\r\n",    HydraulicActuatorSendData->hydraulic_actuator_data.fan_4_velocity_cmd );
	LPRINTF("sol_valve1_drv_cmd[1/500 %%] : %d\r\n",     			HydraulicActuatorSendData->hydraulic_actuator_data.sol_valve1_drv_cmd);
	LPRINTF("sol_valve2_drv_cmd[1/500 %%] : %d\r\n",      			HydraulicActuatorSendData->hydraulic_actuator_data.sol_valve2_drv_cmd);
	LPRINTF("sol_valve3_drv_cmd[1/500 %%] : %d\r\n",      			HydraulicActuatorSendData->hydraulic_actuator_data.sol_valve3_drv_cmd);
	LPRINTF("reserved1[-] : %d\r\n",  								HydraulicActuatorSendData->hydraulic_actuator_data.reserved1);
	LPRINTF("reserved2[-] : %d\r\n",      							HydraulicActuatorSendData->hydraulic_actuator_data.reserved2);
	LPRINTF("reserved3[-] : %d\r\n",     							HydraulicActuatorSendData->hydraulic_actuator_data.reserved3);
}
/*****************************************************************************/
/**
*
* UP_L data output function
*
******************************************************************************/
void UP_L_PRINT(void)
{
    LPRINTF("============================================================Upper_L===================================================\r\n");
	LPRINTF("============================================================0x110=================================================\r\n");
	LPRINTF("upperL_recv_110->id  : 0x%X\r\n",  									UpperRecvData110L->id);
	LPRINTF("upperL_recv_110->seq : %d\r\n",  										UpperRecvData110L->seq);
	LPRINTF("upperL_recv_110->data.loadcell_1[0.1 mV] : %u\r\n", 					UpperRecvData110L->data.loadcell_1);
	LPRINTF("upperL_recv_110->data.loadcell_2[0.1 mV] : %u\r\n", 					UpperRecvData110L->data.loadcell_2);
	LPRINTF("upperL_recv_110->data.loadcell_3[0.1 mV] : %u\r\n", 					UpperRecvData110L->data.loadcell_3);
	LPRINTF("upperL_recv_110->data.shoulder_yaw_position[0.01 deg] : %d\r\n", 		UpperRecvData110L->data.shoulder_yaw_position);

	LPRINTF("============================================================0x120=================================================\r\n");
	LPRINTF("upperL_recv_120->id  : 0x%X\r\n",  									UpperRecvData120L->id);
	LPRINTF("upperL_recv_120->seq : %d\r\n",  										UpperRecvData120L->seq);
	LPRINTF("upperL_recv_120->data.Potentiometer_1[0.1 mV] : %u\r\n", 				UpperRecvData120L->data.Potentiometer_1);
	LPRINTF("upperL_recv_120->data.Potentiometer_2[0.1 mV] : %u\r\n", 				UpperRecvData120L->data.Potentiometer_2);
	LPRINTF("upperL_recv_120->data.Potentiometer_3[0.1 mV] : %u\r\n", 				UpperRecvData120L->data.Potentiometer_3);
	LPRINTF("upperL_recv_120->data.elbow_yaw_position[0.01 deg] : %d\r\n", 			UpperRecvData120L->data.elbow_yaw_position);
	LPRINTF("=============================================================0x130=================================================\r\n");
	LPRINTF("upperL_recv_130->id  : 0x%X\r\n",  						UpperRecvData130L->id);
	LPRINTF("upperL_recv_130->seq : %d\r\n",  							UpperRecvData130L->seq);
	LPRINTF("upperL_recv_130->data.FT_sensor_Fx[1/180 N] : %d\r\n",		UpperRecvData130L->data.FT_sensor_Fx);
	LPRINTF("upperL_recv_130->data.FT_sensor_Fy[1/180 N] : %d\r\n", 	UpperRecvData130L->data.FT_sensor_Fy);
	LPRINTF("upperL_recv_130->data.FT_sensor_Fz[1/180 N] : %d\r\n", 	UpperRecvData130L->data.FT_sensor_Fz);
	LPRINTF("upperL_recv_130->data.switchs_on_off[bit] 	 : 0x%X\r\n", 	UpperRecvData130L->data.switch_on_off);
	LPRINTF("upperL_recv_130->data.reserved[-] : %d\r\n", 				UpperRecvData130L->data.reserved);
	LPRINTF("=============================================================0x140=================================================\r\n");
	LPRINTF("upperL_recv_140->id  : 0x%X\r\n",  							UpperRecvData140L->id);
	LPRINTF("upperL_recv_140->seq : %d\r\n",  								UpperRecvData140L->seq);
	LPRINTF("upperL_recv_140->data.FT_sensor_Fx[1/1600 Nm] : %d\r\n", 		UpperRecvData140L->data.FT_sensor_Fx);
	LPRINTF("upperL_recv_140->data.FT_sensor_Fy[1/1600 Nm] : %d\r\n", 		UpperRecvData140L->data.FT_sensor_Ty);
	LPRINTF("upperL_recv_140->data.FT_sensor_Tz[1/1600 Nm] : %d\r\n", 		UpperRecvData140L->data.FT_sensor_Tz);
	LPRINTF("upperL_recv_140->data.mode_echo[bit] 	  : %X\r\n", 			UpperRecvData140L->data.mode_echo);
	LPRINTF("upperL_recv_140->data.sub_mode_echo[bit] : %X\r\n", 			UpperRecvData140L->data.sub_mode_echo);

	LPRINTF("=============================================================0x150=================================================\r\n");
	printf("upperL_recv_150->id  : 0x%X\r\n",  											UpperRecvData150L->id);
	LPRINTF("upperL_recv_150->seq : %d\r\n",  												UpperRecvData150L->seq);
	LPRINTF("upperL_recv_150->data.Potentiometer1_velocity[25/320 mm/s] : %d\r\n", 		    UpperRecvData150L->data.Potentiometer1_velocity);
	LPRINTF("upperL_recv_150->data.Potentiometer2_velocity[25/320 mm/s] : %d\r\n", 		    UpperRecvData150L->data.Potentiometer2_velocity);
	LPRINTF("upperL_recv_150->data.Potentiometer3_velocity[25/320 mm/s] : %d\r\n", 			UpperRecvData150L->data.Potentiometer3_velocity);
	LPRINTF("upperL_recv_150->data.reserved[-] : %d\r\n", 									UpperRecvData150L->data.reserved);
}


/*****************************************************************************/
/**
*
* UP_R data output function
*
******************************************************************************/
void UP_R_PRINT(void)
{

	LPRINTF("============================================================Upper_R===================================================\r\n");
	LPRINTF("============================================================0x110===================================================\r\n");
	LPRINTF("upperR_recv_110->id  : 0x%X\r\n",  								UpperRecvData110R->id);
	LPRINTF("upperR_recv_110->seq : %d\r\n",  									UpperRecvData110R->seq);
	LPRINTF("upperR_recv_110->data.loadcell_1[0.1 mV] : %u\r\n", 					UpperRecvData110R->data.loadcell_1);
	LPRINTF("upperR_recv_110->data.loadcell_2[0.1 mV] : %u\r\n", 					UpperRecvData110R->data.loadcell_2);
	LPRINTF("upperR_recv_110->data.loadcell_3[0.1 mV] : %u\r\n", 					UpperRecvData110R->data.loadcell_3);
	LPRINTF("upperR_recv_110->data.shoulder_yaw_position[0.01 deg] : %d\r\n", 		UpperRecvData110R->data.shoulder_yaw_position);
	LPRINTF("============================================================0x120=================================================\r\n");
	LPRINTF("upperR_recv_120->id  : 0x%X\r\n",  								UpperRecvData120R->id);
	LPRINTF("upperR_recv_120->seq : %d\r\n",  									UpperRecvData120R->seq);
	LPRINTF("upperR_recv_120->data.Potentiometer_1[0.1 mV] : %u\r\n", 				UpperRecvData120R->data.Potentiometer_1);
	LPRINTF("upperR_recv_120->data.Potentiometer_2[0.1 mV] : %u\r\n", 				UpperRecvData120R->data.Potentiometer_2);
	LPRINTF("upperR_recv_120->data.Potentiometer_3[0.1 mV] : %u\r\n", 				UpperRecvData120R->data.Potentiometer_3);
	LPRINTF("upperR_recv_120->data.elbow_yaw_position[0.01 deg] : %d\r\n", 			UpperRecvData120R->data.elbow_yaw_position);
	LPRINTF("=============================================================0x130=================================================\r\n");
	LPRINTF("upperR_recv_130->id  : 0x%X\r\n",  					UpperRecvData130R->id);
	LPRINTF("upperR_recv_130->seq : %d\r\n",  						UpperRecvData130R->seq);
	LPRINTF("upperR_recv_130->data.FT_sensor_Fx[1/180 N] : %d\r\n",	UpperRecvData130R->data.FT_sensor_Fx);
	LPRINTF("upperR_recv_130->data.FT_sensor_Fy[1/180 N] : %d\r\n", UpperRecvData130R->data.FT_sensor_Fy);
	LPRINTF("upperR_recv_130->data.FT_sensor_Fz[1/180 N] : %d\r\n", UpperRecvData130R->data.FT_sensor_Fz);
	LPRINTF("upperR_recv_130->data.switchs_on_off[bit] 	 : 0x%X\r\n", UpperRecvData130R->data.switch_on_off);
	LPRINTF("upperR_recv_130->data.reserved[-] : %d\r\n", 			UpperRecvData130R->data.reserved);

	LPRINTF("=============================================================0x140=================================================\r\n");
	LPRINTF("upperR_recv_140->id  : 0x%X\r\n",  				UpperRecvData140R->id);
	LPRINTF("upperR_recv_140->seq : %d\r\n",  					UpperRecvData140R->seq);
	LPRINTF("upperR_recv_140->data.FT_sensor_Fx[1/1600 Nm] : %d\r\n", 		UpperRecvData140R->data.FT_sensor_Fx);
	LPRINTF("upperR_recv_140->data.FT_sensor_Fy[1/1600 Nm] : %d\r\n", 		UpperRecvData140R->data.FT_sensor_Ty);
	LPRINTF("upperR_recv_140->data.FT_sensor_Tz[1/1600 Nm] : %d\r\n", 		UpperRecvData140R->data.FT_sensor_Tz);
	LPRINTF("upperR_recv_140->data.mode_echo[bit] 	  : %X\r\n", 			UpperRecvData140R->data.mode_echo);
	LPRINTF("upperR_recv_140->data.sub_mode_echo[bit] : %X\r\n", 			UpperRecvData140R->data.sub_mode_echo);
	LPRINTF("=============================================================0x150=================================================\r\n");
	LPRINTF("upperR_recv_150->id  : 0x%X\r\n",  				UpperRecvData150R->id);
	LPRINTF("upperR_recv_150->seq : %d\r\n",  					UpperRecvData150R->seq);
	LPRINTF("upperR_recv_150->data.Potentiometer1_velocity[25/320 mm/s] : %d\r\n", 		    UpperRecvData150R->data.Potentiometer1_velocity);
	LPRINTF("upperR_recv_150->data.Potentiometer2_velocity[25/320 mm/s] : %d\r\n", 		    UpperRecvData150R->data.Potentiometer2_velocity);
	LPRINTF("upperR_recv_150->data.Potentiometer3_velocity[25/320 mm/s] : %d\r\n", 			UpperRecvData150R->data.Potentiometer3_velocity);
	LPRINTF("upperR_recv_150->data.reserved[-] : %d\r\n", 									UpperRecvData150R->data.reserved);
}

/*****************************************************************************/
/**
*
* UP_SEND L data output function
*
******************************************************************************/
void UP_L_SEND_PRINT(void)
{
	LPRINTF("============================================================Upper_Send_99 L=================================================== \r\n");
	LPRINTF("UpperSendData99L->id  : 0x%X\r\n",  								UpperSendData99L->id);
	LPRINTF("UpperSendData99L->seq : %d\r\n",  									UpperSendData99L->seq);
	LPRINTF("UpperSendData99L->data.FT_sensor_Fx[bit(hex)] : 0x%X\r\n", 		UpperSendData99L->data.drive_mode);
	LPRINTF("UpperSendData99L->data.FT_sensor_Fy[bit(hex)] : 0x%X\r\n", 		UpperSendData99L->data.drive_sub_mode);
	LPRINTF("UpperSendData99L->data.reserved1[-] : 0x%X\r\n", 			UpperSendData99L->data.reserved1);
	LPRINTF("UpperSendData99L->data.reserved2[-] : 0x%X\r\n", 			UpperSendData99L->data.reserved2);
	LPRINTF("UpperSendData99L->data.reserved3[-] : 0x%X\r\n", 			UpperSendData99L->data.reserved2);
}
/*****************************************************************************/
/**
*
* UP_SEND R data output function
*
******************************************************************************/
void UP_R_SEND_PRINT(void)
{
	LPRINTF("============================================================Upper_Send_99 R=================================================== \r\n");
	LPRINTF("UpperSendData99R->id  : 0x%X\r\n",  					UpperSendData99R->id);
	LPRINTF("UpperSendData99R->seq : %d\r\n",  						UpperSendData99R->seq);
	LPRINTF("UpperSendData99R->data.FT_sensor_Fx[bit(hex)] : 0x%X\r\n", 		UpperSendData99R->data.drive_mode);
	LPRINTF("UpperSendData99R->data.FT_sensor_Fy[bit(hex)] : 0x%X\r\n", 		UpperSendData99R->data.drive_sub_mode);
	LPRINTF("UpperSendData99R->data.reserved1[-] : 0x%X\r\n", 			UpperSendData99R->data.reserved1);
	LPRINTF("UpperSendData99R->data.reserved2[-] : 0x%X\r\n", 			UpperSendData99R->data.reserved2);
	LPRINTF("UpperSendData99R->data.reserved3[-] : 0x%X\r\n", 			UpperSendData99R->data.reserved2);
}

/*****************************************************************************/
/**
*
* UP_SEND R data output function
*
******************************************************************************/
void UHCB_L_SEND_PRINT(void)
{

	LPRINTF("============================================================UHCB_L_SEND_100 L=================================================== \r\n");
	LPRINTF("UpperSendData100L->id  : 0x%X\r\n",  						UpperSendData100L->id);
	LPRINTF("UpperSendData100L->seq : %d\r\n",  						UpperSendData100L->seq);
	LPRINTF("UpperSendData100L->data : ");
	LPRINTF("UpperSendData100L->data.shoulder_roll_cmd[1/1000 V]  : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100L->data.shoulder_roll_cmd));
	LPRINTF("UpperSendData100L->data.shoulder_pitch_cmd[1/1000 V] : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100L->data.shoulder_pitch_cmd));
	LPRINTF("UpperSendData100L->data.elbow_pitch_cmd[1/1000 V]    : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100L->data.elbow_pitch_cmd));
	LPRINTF("UpperSendData100L->data.elbow_yaw_cmd[7/300 mNm]     : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100L->data.elbow_yaw_cmd));
}

/*****************************************************************************/
/**
*
* UP_SEND R data output function
*
******************************************************************************/
void UHCB_R_SEND_PRINT(void)
{



	LPRINTF("============================================================UHCB_R_SEND_100 R=================================================== \r\n");
	LPRINTF("UpperSendData100R->id  : 0x%X\r\n",  						UpperSendData100R->id);
	LPRINTF("UpperSendData100R->seq : %d\r\n",  						UpperSendData100R->seq);
	LPRINTF("UpperSendData100R->data.shoulder_roll_cmd[1/1000 V]  : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100R->data.shoulder_roll_cmd));
	LPRINTF("UpperSendData100R->data.shoulder_pitch_cmd[1/1000 V] : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100R->data.shoulder_pitch_cmd));
	LPRINTF("UpperSendData100R->data.elbow_pitch_cmd[1/1000 V]    : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100R->data.elbow_pitch_cmd));
	LPRINTF("UpperSendData100R->data.elbow_yaw_cmd[7/300 mNm]     : %d\r\n", 	(int16_t)Xil_EndianSwap16((uint16_t)UpperSendData100R->data.elbow_yaw_cmd));
}
/*****************************************************************************/
/**
*
* IMU data output function
*
******************************************************************************/
void IMU_PRINT(void)
{
	LPRINTF("============================================================IMU=================================================== \r\n");
	LPRINTF("sharedImu->seq : %d\r\n",  			sharedImu->seq);
	LPRINTF("X_ROLL 		: %f\r\n",  			sharedImu->mti_data.X_ROLL);
	LPRINTF("Y_PITCH 		: %f\r\n",  			sharedImu->mti_data.Y_PITCH);
	LPRINTF("Z_YAW 			: %f\r\n",  			sharedImu->mti_data.Z_YAW);
	LPRINTF("X_ACCEL 		: %f\r\n",  			sharedImu->mti_data.X_ACCEL);
	LPRINTF("Y_ACCEL 		: %f\r\n",  			sharedImu->mti_data.Y_ACCEL);
	LPRINTF("Z_ACCEL 		: %f\r\n",  			sharedImu->mti_data.Z_ACCEL);
	LPRINTF("X_ROLL_VEL 	: %f\r\n",  			sharedImu->mti_data.X_ROLL_VEL);
	LPRINTF("Y_PITCH_VEL 	: %f\r\n",  			sharedImu->mti_data.Y_PITCH_VEL);
	LPRINTF("Z_YAW_VEL 		: %f\r\n",  			sharedImu->mti_data.Z_YAW_VEL);
}

/*****************************************************************************/
/**
*
* Controller data output function
*
******************************************************************************/
void CON_PRINT(void)
{
	LPRINTF("============================================================CON=================================================== \r\n");
	LPRINTF("sharedCon->seq : %d\r\n",  							sharedCon->seq);
	LPRINTF("sharedCon->con_data.active : %d\r\n",  					sharedCon->con_data.LED1);
	LPRINTF("sharedCon->con_data.mode1 : %d\r\n",  					sharedCon->con_data.LED2);
	LPRINTF("sharedCon->con_data.reset : %d\r\n",  					sharedCon->con_data.LED3);
	LPRINTF("sharedCon->con_data.mode2 : %d\r\n",  					sharedCon->con_data.LED4);

	LPRINTF("============================================================CON_SEND=================================================== \r\n");

	if (val_change == 0)
	{
		sharedConSend->con_data.battery_level_status = 10;
		sharedConSend->con_data.PowerLed = 0x01;
		sharedConSend->con_data.errorLed = 0x01;
		for (int i = 0; i < 10 ; i++)
		{
			sharedConSend->con_data.Spare[i] = 0x00;
		}

		val_change++;
	}
	else if (val_change == 2 )
	{
		sharedConSend->con_data.battery_level_status = 20;
		sharedConSend->con_data.PowerLed = 0x02;
		sharedConSend->con_data.errorLed = 0x02;
		for (int i = 0; i < 10 ; i++)
		{
			sharedConSend->con_data.Spare[i] = 0x00;
		}

		val_change++;
	}
	else if (val_change == 4 )
	{
		sharedConSend->con_data.battery_level_status = 30;
		sharedConSend->con_data.PowerLed = 0x03;
		sharedConSend->con_data.errorLed = 0x03;
		for (int i = 0; i < 10 ; i++)
		{
			sharedConSend->con_data.Spare[i] = 0x00;
		}
		val_change++;
	}
	else
	{
		val_change++;
	}

	if(val_change >= 6)
	{
		val_change = 0;
	}
	LPRINTF("sharedCon->seq : %d\r\n",  								sharedConSend->seq);
	LPRINTF("sharedConSend->con_data.battery_level_status : %d\r\n",	sharedConSend->con_data.battery_level_status);
	LPRINTF("sharedConSend->con_data.PowerLed : %d\r\n" , sharedConSend->con_data.PowerLed);
	LPRINTF("sharedConSend->con_data.errorLed : %d\r\n" , sharedConSend->con_data.errorLed);
}

/*****************************************************************************/
/**
*
* BMS data output function
*
******************************************************************************/
void BMS_PRINT(void)
{
	LPRINTF("============================================================BMS=================================================== \r\n");
	LPRINTF("sharedBms->seq                     : %d\r\n",						sharedBms->seq);
	LPRINTF("sharedBms->bms_data.battery_status : 0x%02X\r\n",  					sharedBms->bms_data.battery_status);
	LPRINTF("sharedBms->bms_data.error_code 	: 0x%02X\r\n",  					sharedBms->bms_data.error_code);
	LPRINTF("sharedBms->bms_data.pack_current 	: %d\r\n",  					sharedBms->bms_data.pack_current);
	LPRINTF("sharedBms->bms_data.pack_voltage   : %d\r\n",  					sharedBms->bms_data.pack_voltage);
	LPRINTF("sharedBms->bms_data.SOC 			: %d\r\n",  					sharedBms->bms_data.SOC);
	LPRINTF("sharedBms->bms_data.reserved1 			: 0x%02X\r\n",  					sharedBms->bms_data.reserved1);
	LPRINTF("sharedBms->bms_data.reserved2 			: 0x%02X\r\n",  					sharedBms->bms_data.reserved2);
}

void Period_PRINT(void)
{
	LPRINTF("============================================================Period=================================================== \r\n");
	LPRINTF("start_count 	: %"PRIu64" \r\n",  					sharedCount->start_count);
	LPRINTF("end_count 		: %"PRIu64" \r\n", 						sharedCount->end_count);
	LPRINTF("mesuer_count 	: %"PRIu64" \r\n",  					sharedCount->mesuer_count);
}

