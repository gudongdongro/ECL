/**
 *****************************************************************************************
 * 이 파일은 2단 도립진자를 2-DOF 제어로 Transition control 하는 파일이다.                           *
 * 값은 MATLAB에서 SINGLE 형태로 저장한 값으로 이 값을 저장하기 위해서는 압축형식을 사용하면 안된다.            *
 * 이를 위해서 저장할 mat 파일을 저장할 때, 다음의 형태로 만들어야한다. (m-file로 미리 만들어 놓으면 좋다.)      *
 * 																						 *
 * control_mat = single(control_mat);													 *
 * K_LQ_mat = single(K_LQ_mat);															 *
 * K_mat = single(K_mat);																 *
 * state_mat = single(state_mat);														 *
 * time_mat = single(time_mat);															 *
 * save 'DIP_data.mat' control_mat K_LQ_mat K_mat state_mat time_mat '-nocompression'    *
 * 																						 *
 * 다음의 저장값을 SD카드에 넣고 이 파일을 업로드 후 DIP 시스템에 연결하면 동작한다.						 	 *
 * 이후 시퀀스는 다음과 같이 만들었다.															 *
 * 1. USB나 5V 전원을 통해 STM407_IP_SD board에 전원을 인가한다.									 *
 * 2. 이후 SD카드에서 정상적으로 정보를 수신 받았다면, LED2(PD12)에 불이 들어온다.						 *
 * 		2.1 불이 들어오지 않는다면 SD카드가 없는지 확인해봐야한다.										 *
 * 		2.2 시작 단계에서 LED3, LED4에 불이 들어온다면, 이는 setup 오류로 무한루프에 빠진 상태이다.			 *
 * 3. 이후 SW를 1부터 8까지 눌러 EP0~EP3과 랜덤 설정으로 변경할 수 있다.(1 -> EP0, ..., 5 -> Random1)  *
 * 		3.1 천이중에는 다른 버튼이 눌리지 않는다.													 *
 * 		3.2 천이 이후 선형제어 구간으로 넘어가면 원하는 상태로 천이할 수 있다.							 *
 * 		3.3 각 넘버에 따라 LED 불이 들어오는데, 이는 1~4까지 각각의 LED가 불로 들어온다.(Ex) EP2 -> LED3) *
 * 																						 *
 * 																		-made by Jeong.  *
 *****************************************************************************************
 * @file           : main.c
 * @brief          : DIP_transition_all
 **************************************
 * STM32F407			Motor driver  *
 **************************************
 *	5V					 5V 	  *
 * 	GND					 GND	  *
 * 	PE8					 dir	  *
 * 	PE9	-> PA2			 PWM	  *
 * 	PA15				 enc A	  *
 * 	PB3					 enc B	  *
 **************************************
 * STM32F407		 Pendulum driver  *
 **************************************
 *  3.3V				3.3V	  *
 * 	GND					GND		  *
 * 	PB4					P1A		  *
 * 	PB5					P1B		  *
 * 	PB6					P2A		  *
 * 	PB7					P2B		  *
 * 	PA0					P3A		  * (사용 x)
 * 	PA1					P3B		  * (사용 x)
 **************************************
 * STM32F407		 STM32F407 Board  *
 **************************************
 * 	PE0					SW1		  *		EP0
 * 	PE1					SW2		  *		EP1
 * 	PE2					SW3		  *		EP2
 * 	PE3					SW4		  *		EP3
 * 	PE4					SW5		  *		Random pattern1
 * 	PE5					SW6		  *		Random pattern2
 * 	PE6					SW7		  *		Random pattern3
 * 	PE7					SW8		  *		Random pattern4
 * 	PD12				LED2	  *
 * 	PD13				LED3	  *
 * 	PD14				LED4	  *
 * 	PD15				LED5	  *
 **************************************
 * STM32F407		 Peripheral Board *
 **************************************
 * 	PA4					CS		  *
 * 	PA5			       	SCK		  *
 * 	PA6			     	MISO	  *
 * 	PA7			    	MOSI	  *
 *************************************/
// PC0, PC2,


// ------------------------------ Header -----------------------------------------
#include "stm32f4xx.h"   // Compiler symbols에 STM32F407xx를 선언하면 stm32f407xx.h를 include 해준다.
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "monitor.h"  // Full speed USB 기반의 고속 monitoring을 위한 header file

// ------------------------------SD define-----------------------------------------
#define CMD0		0			// GO_IDLE_STATE
#define CMD8		8			// SEND_IF_COND

#define CMD17		17			// READ_SINGLE_BLOCK
#define CMD55		55			// APP_CMD
#define CMD58		58			// READ_OCR
#define ACMD41		41			// SD_SEND_OP_COND

#define VER1X		0			// Ver1.X SD Memory Card
#define VER2X		1			// Ver2.X SD Memory Card
#define VER2X_SC	0			// Ver2.X Standard Capacity SD Memory Card
#define VER2X_HC	1			// Ver2.X High Capacity SD Memory Card(SDHC)

uint8_t SD_type;				// SD card version(Ver1.X or Ver2.0)

void SD_Init();
void SD_command(uint8_t command, uint32_t sector);
void SD_read_sector(uint32_t sector, uint8_t *buffer);// read a sector of SD card

// ------------------------------SD_FAT32 define-----------------------------------------
void FAT32_Init(void);								// initialize FAT32
uint8_t fatGetDirEntry(uint32_t cluster);			// get directory entry
uint8_t* fatDir(uint32_t cluster, uint32_t offset);	// get directory entry sector
uint32_t fatNextCluster(uint32_t cluster);			// get next cluster
uint32_t fatClustToSect(uint32_t cluster);			// convert cluster to sector

uint8_t Get_long_filename(uint8_t file_number); 	// check long file name
void Save_short_filename();							// UART3으로 짧은 제목 출력
void Save_long_filename();							// UART3으로 긴 제목 출력
void Filename_arrange(uint16_t new, uint16_t old, uint8_t *FileNameBuffer); // arrange file name

/* ---------------------------------------------------------------------------- */

#define MAX_FILE			200			// maximum file number
#define BYTES_PER_SECTOR	512			// bytes per sector
#define CLUST_FIRST			2			// first legal cluster number
#define CLUST_EOFE			0xFFFFFFFF	// end of eof cluster range
#define FAT16_MASK			0x0000FFFF	// mask for 16 bit cluster numbers
#define FAT32_MASK			0x0FFFFFFF	// mask for FAT32 cluster numbers

uint32_t file_start_cluster[MAX_FILE];	// file start cluster
uint32_t file_start_sector[MAX_FILE];	// file start sector number
uint8_t file_start_entry[MAX_FILE];		// file directory entry
uint32_t file_size[MAX_FILE];			// file size

uint8_t SectorBuffer[BYTES_PER_SECTOR];	// sector buffer
uint8_t EntryBuffer[BYTES_PER_SECTOR];	// directory entry buffer

uint32_t sector_cluster = 0;
uint32_t extension = 0;					// filename extension string(3 character)

uint8_t Fat32Enabled;					// indicate if is FAT32 or FAT16
uint32_t FirstDataSector;				// first data sector address
uint16_t SectorsPerCluster;				// number of sectors per cluster
uint32_t FirstFATSector;				// first FAT sector address
uint32_t FirstFAT2Sector;				// first FAT2 sector address
uint32_t FirstDirCluster;				// first directory(data) cluster address

/* ---------------------------------------------------------------------------- */

// 가장 중요. 메모리 들이 제대로 정렬 될 수 있게 만들어줌.
// uint8_t, uint32_t 를 포함하는 구조체를 선언하게 되면, 각각 1byte, 4byte 공간만큼 할당되어 5byte일 것 같지만
// 사실은 4byte 공간을 매번 할당해서 8byte 공간의 할당되고, 예상과 맞지 않게된다.
// #pragma pack(1)을 써주게 되면 5byte가 나오는데, 이는 사이즈에 맞춰서 차곡차곡 쌓아주는 역할을 한다.
// https://she11.tistory.com/109
#pragma pack(1)

struct MBR /* MBR(Master Boot Record) */
{
	uint8_t mbrBootCode[512 - 64 - 2];		// boot code
	uint8_t mbrPartition[64];		// four partition records (64 = 16*4)
	uint8_t mbrSignature0;			// signature byte 0 (0x55)
	uint8_t mbrSignature1;			// signature byte 1 (0xAA)
};

struct partition /* partition table(16 byte) of MBR */
{
	uint8_t partBootable;			// 0x80 indicates active bootable partition
	uint8_t partStartHead;			// starting head for partition
	uint16_t partStartCylSect;		// starting cylinder and sector
	uint8_t partPartType;			// partition type
	uint8_t partEndHead;			// ending head for this partition
	uint16_t partEndCylSect;		// ending cylinder and sector
	uint32_t partStartLBA;			// first LBA sector for this partition
	uint32_t partSize;				// size of this partition(bytes or sectors)
};

#define PART_TYPE_FAT16		0x04		// partition type
#define PART_TYPE_FAT16BIG	0x06
#define PART_TYPE_FAT32		0x0B
#define PART_TYPE_FAT32LBA	0x0C
#define PART_TYPE_FAT16LBA	0x0E

struct bootrecord /* PBR(Partition Boot Record) */
{
	uint8_t brJumpBoot[3];			// jump instruction E9xxxx or EBxx90
	uint8_t brOEMName[8];			// OEM name and version
	uint8_t brBPB[53];				// volume ID(= BIOS parameter block)
	uint8_t brExt[26];				// Bootsector Extension
	uint8_t brBootCode[418];		// pad so structure is 512 bytes
	uint8_t brSignature2;			// 2 & 3 are only defined for FAT32
	uint8_t brSignature3;
	uint8_t brSignature0;			// boot sector signature byte 0(0x55)
	uint8_t brSignature1;			// boot sector signature byte 1(0xAA)
};

struct volumeID /* BPB for FAT16 and FAT32 */
{
	// same at FAT16 and FAT32
	uint16_t bpbBytesPerSec;		// bytes per sector
	uint8_t bpbSecPerClust;			// sectors per cluster
	uint16_t bpbResSectors;			// number of reserved sectors
	uint8_t bpbFATs;				// number of FATs
	uint16_t bpbRootDirEnts;	// number of root directory entries(0 = FAT32)
	uint16_t bpbSectors;			// total number of sectors(0 = FAT32)
	uint8_t bpbMedia;				// media descriptor
	uint16_t bpbFATsecs;			// number of sectors per FAT16(0 = FAT32)
	uint16_t bpbSecPerTrack;		// sectors per track
	uint16_t bpbHeads;				// number of heads
	uint32_t bpbHiddenSecs;			// number of hidden sectors
	uint32_t bpbHugeSectors;		// number of sectors for FAT32
	// only at FAT32
	uint32_t bpbBigFATsecs;			// number of sectors per FAT32
	uint16_t bpbExtFlags;			// extended flags
	uint16_t bpbFSVers;				// file system version
	uint32_t bpbRootClust;			// start cluster for root directory
	uint16_t bpbFSInfo;				// file system info structure sector
	uint16_t bpbBackup;				// backup boot sector
	uint8_t bpbReserved[12];
	// same at FAT16 and FAT32
	uint8_t bpbDriveNum;			// INT 0x13 drive number
	uint8_t bpbReserved1;
	uint8_t bpbBootSig;				// extended boot signature(0x29)
	uint8_t bpbVolID[4];			// extended
	uint8_t bpbVolLabel[11];		// extended
	uint8_t bpbFileSystemType[8];	// extended
};

typedef struct Dir_entry /* structure of DOS directory entry */
{
	uint8_t dirName[8];      		// filename, blank filled
	uint8_t dirExtension[3]; 		// extension, blank filled
	uint8_t dirAttributes;   		// file attributes
	uint8_t dirLowerCase;    		// NT VFAT lower case flags
	uint8_t dirCHundredth;			// hundredth of seconds in CTime
	uint8_t dirCTime[2];			// create time
	uint8_t dirCDate[2];			// create date
	uint8_t dirADate[2];			// access date
	uint16_t dirHighClust;			// high bytes of cluster number
	uint8_t dirMTime[2];			// last update time
	uint8_t dirMDate[2];			// last update date
	uint16_t dirStartCluster;		// starting cluster of file
	uint32_t dirFileSize;			// size of file in bytes
} Dir_entry;

#define 	SLOT_EMPTY      	0x00	// slot has never been used
#define 	SLOT_DELETED    	0xE5	// file in this slot deleted

#define 	ATTR_NORMAL     	0x00	// normal file
#define 	ATTR_READONLY   	0x01	// file is read-only
#define 	ATTR_HIDDEN     	0x02	// file is hidden
#define 	ATTR_SYSTEM     	0x04	// file is a system file
#define 	ATTR_VOLUME     	0x08	// entry is a volume label
#define 	ATTR_LONG_FILENAME	0x0F	// this is a long filename entry
#define 	ATTR_DIRECTORY  	0x10	// entry is a directory name
#define 	ATTR_ARCHIVE    	0x20	// file is new or modified

#define 	LCASE_BASE      	0x08	// filename base in lower case
#define 	LCASE_EXT       	0x10	// filename extension in lower case

typedef struct Long_dir_entry /* structure of long directory entry */
{
	uint8_t Longdir_Ord;			// 00(0x00)
	uint8_t Longdir_Name1[10];		// 01(0x01)
	uint8_t Longdir_Attr;			// 11(0x0B)
	uint8_t Longdir_Type;			// 12(0x0C)
	uint8_t Longdir_Chksum;			// 13(0x0D)
	uint8_t Longdir_Name2[12];		// 14(0x0E)
	uint16_t Longdir_FstClusLO;		// 26(0x1A)
	uint8_t Longdir_Name3[4];		// 28(0x1C)
} Long_dir_entry;

// ------------------------------MAT-file define-----------------------------------------

// MAT-File Data Types
#define	miINT8			0x01
#define	miUINT8			0x02
#define	miINT16			0x03
#define	miUINT16		0x04
#define	miINT32			0x05
#define	miUINT32		0x06
#define	miSINGLE		0x07
#define	miDOUBLE		0x09
#define	miINT64			0x0C
#define	miUINT64		0x0D
#define	miMATRIX		0x0E
#define	miCOMPRESSED	0x0F
#define	miUTF8			0x10
#define	miUTF16			0x11

// Classes
#define mxCELL			0x01
#define mxSTRUCT		0x02
#define mxOBJECT		0x03
#define mxCHAR			0x04
#define mxSPARSE		0x05
#define mxDOUBLE		0x06
#define mxSINGLE		0x07
#define mxINT8			0x08
#define mxUINT8			0x09
#define mxINT16			0x0A
#define mxUINT16		0x0B
#define mxINT32			0x0C
#define mxUINT32		0x0D

#define MatFile_name	"DIP_data.mat"	// 파일명을 선택한다. (이름 설정은 미리 맞추자!)
#define name_max		0xFF			// 이름 최대 길이
#define MF_index		0x28			//**** MatFile의 고정된 변수 공간 길이
#define DIP_EP_cnt		4				//**** EP 개수
#define DIP_point_cnt	161				//**** DIP의 node값 개수 (이 부분은 내가 저장한 mat 파일의 수에 맞춰야한다.)
#define DIP_state_cnt	7				// 상태변수 개수
#define DIP_trans_cnt	DIP_EP_cnt*(DIP_EP_cnt-1) // 천이 가능한 총 궤적 수

typedef struct /* MatFile의 변수 저장 형식 */
{
	uint32_t mf_sector;			// 변수가 저장된 섹터 저장
	uint16_t mf_sector_index;	// 해당 섹터에서 읽어야하는 첫 위치

	uint32_t mf_len;			// 변수의 총 데이터 크기
	uint16_t mf_classType;		// 변수의 저장형식 Classes 0x07 -> SIGNLE, 0x06 -> DOUBLE

	uint32_t mf_rows;			// 변수값의 행 개수
	uint32_t mf_cols;			// 변수값의 열 개수

	uint32_t mf_name_len;		// 변수명의 총 크기
	char mf_name[name_max];		// 변수명 저장

	uint32_t mf_data_len;		// 변수 데이터 크기
	uint32_t mf_data_index;		// 변수 데이터가 있는 첫 위치
} MatFile_format;

uint8_t Mat_Info[128]; 		// MatFile 파일 정보 저장
uint8_t Mat_Format[40]; 	// 변수 고정 형식
char file_name[name_max];	// SD카드에 저장되어있는 파일명
uint8_t file_cnt = 0;		// 파일 번호 저장
char tmp_name[] = MatFile_name;

// MatFile 5개 변수. 아스키 코드값이 빠른 순으로 정렬되는 것 같다.
// K_LQ_mat 	-> 시불변 LQ gain이 들어있는 변수로 1xn의 값을 갖는다. (여기서 n은 상태변수와 EP수의 곱)
// K_mat		-> feedforward의 궤적을 보상하는 시변 LQ gain값으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 상태변수와 천이가능한 궤적수의 곱)
// control_mat	-> feedforward의 입력 궤적으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 천이가능한 궤적수의 수)
// state_mat	-> feedforward의 상태 궤적으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 상태변수와 천이가능한 궤적수의 곱)
// time_mat		-> feedforward의 궤적 시간으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 천이가능한 궤적수의 수)
MatFile_format MF[5] = { };

volatile float EPnK[DIP_EP_cnt * DIP_state_cnt] = { }; 		// 시불변 LQ gain은 처음에 다 저장 받는다.
volatile float TRnnt[DIP_point_cnt] = { };					// 선택된 feedforward 시간
volatile float TRnnu[DIP_point_cnt] = { };					// 선택된 feedforward 가속도
volatile float TRnnx[DIP_point_cnt][DIP_state_cnt] = { };	// 선택된 feedforward 상태변수
volatile float TRnnK[DIP_point_cnt][DIP_state_cnt] = { };	// 선택된 feedforward 시변 LQ gain
volatile float EPnx_end[DIP_state_cnt] = { };				// 선택된 feedforward의 종단 지점 상태변수값 미리 저장.

void Get_EPnK_param(float K[DIP_state_cnt * DIP_EP_cnt]);	// 시불변 LQ gain을 얻어오는 함수.
void Get_Transition_param(float t[DIP_point_cnt],			// 천이 종류에 맞는 천이궤적을 얻어오는 함수.
						  float u[DIP_point_cnt],
						  float x[DIP_point_cnt][DIP_state_cnt],
						  float K[DIP_point_cnt][DIP_state_cnt],
						  uint8_t Transition_num);

// ------------------------------USER define-----------------------------------------

// define
#define sample_time 	0.001		// 샘플링 시간 [s]
#define EPSILON 		1e-6 		// float 형의 유효 자리수 소숫점이하 6자리
#define teeth 			38			// timing pully teeth 갯수 (2 [mm] 피치)
#define motor_cpr 		20000		// motor encoder cpr
#define pendulum1_cpr	8192		// pendulum1 encoder cpr
#define pendulum2_cpr	8192		// pendulum2 encoder cpr -> 3단에서 분리해서 할때 8192로 바꿔야함
#define MaxV 			24      	// 인가 전압 맥스값 24 [V]
#define KP 				16.0080	 	// 모터제어에 사용되는 Kp값 -> 3단에서 분리해서 할때 16.0080로 바꿔야함
#define KI 				647.1190	// 모터제어에 사용되는 Ki값 -> 3단에서 분리해서 할때 647.1190로 바꿔야함
#define fc 				10			// 차단 주파수 [Hz]
#define PI 				3.14159265 	// arm_math.h 헤더파일에도 정의 되어 있음.

// 엔코더 값 변환식
#define volt_to_duty 	4200.0 / MaxV				// 전압을 듀티로 전환
#define enc1_to_pos     2*PI*0.0095 / motor_cpr    // 엔코더 1번의 값을 cart_position [m] 으로 변환 -> 3단에서 분리해서 할때 바꿔야함
#define enc2_to_rad 	2 * PI / pendulum1_cpr		// 엔코더 2번의 값을 pendulum 1 angle [rad] 으로 변환
#define enc3_to_rad 	2 * PI / pendulum2_cpr 		// 엔코더 3번의 값을 pendulum 2 angle [rad] 으로 변환

// 구조체 선언부
// PIcontrol Structure
typedef struct {
	float Ref;              // 위치값
	float V_Fdb;            // 속도 피드백
	float Err;              // 오차값
	float samT;  			// 샘플링 타임
	float Kp;           	// P 계수
	float Ki;           	// I 계수
	float OutMax;       	// 최댓값
	float OutMin;      		// 최솟값
	float Intg;         	// 적분
	float Out_tmp;      	// 아웃풋 임시
	float Out;          	// 아웃풋
	float SatErr;       	// 포화 에러
	float Kc;           	// 와인드업 계수
	void (*calc)(void*);
} con_PI;
void con_PI_calc(con_PI *str);	// con_PI 계산 함수
con_PI con_PI1; // 하나의 PI구조 사용

// 정의된 함수 선언부
void SystemClock_Config(void);
void Error_Handler(void);
void RCC_Configuration();
void DWT_Init();             	// DWT initialization 함수
void DWT_us_Delay(uint32_t us); // 인자의 단위는 microseconds
void DWT_ms_Delay(uint32_t ms); // 인자의 단위는 microseconds
void UsbPutString(char *s);

void Timer8_PWM_dir_Init(); 	// Timer1(Advanced-control timer)의 Encoder counter 기능을 학습하는 함수
void Timer2_Encoder_Init(); 	// Timer2(32-bit General-purpose timer)의 Encoder counter 기능을 학습하는 함수
void Timer3_Encoder_Init(); 	// Timer3(16-bit General-purpose timer)의 Encoder counter 기능을 학습하는 함수
void Timer4_Encoder_Init(); 	// Timer4(16-bit General-purpose timer)의 Encoder counter 기능을 학습하는 함수
void Timer6_Interrupt_Init();	// 1ms 주기로 인터럽트를 걸어주는 interrupt timer

void Timer1_MT_Interrupt_Init();// 1단부 엔코더 값 input capture 후 interrupt 후 속도 계산하는 함수
void Timer12_MT_Interrupt_Init();// 2단부 엔코더 값 input capture 후 interrupt 후 속도 계산하는 함수
void Timer7_UPCounter_Init();	// 단순 UpCounter. M/T method에서 cnt의 변화를 보기 위한 함수의 init
void Timer5_UPCounter_Init();	// 단순 UpCounter. M/T method에서 cnt의 변화를 보기 위한 함수의 init
void Timer9_Interrupt_Init();	// 1ms 주기로 인터럽트를 걸어주는 interrupt timer - M/T method 1단부
void Timer11_Interrupt_Init();	// 1ms 주기로 인터럽트를 걸어주는 interrupt timer - M/T method 2단부
void PIO_Configure_Init();

void LED_SW_Init();				// 도립진자 제어와 상태를 표시하기 위한 LED 및 switch 활성화
void con_PI1_Init();			// con_PI1 초기설정
float modulo(float alpha);		// 원하는 각도로 값을 만들어 줄 modulo 함수

// 선형 시간에 맞는 선형 보간을 하기 위한 방법 변수가 7개인것과 1개인 것을 구분했다.
void calculate_interpolation7(float *t, float (*x)[DIP_state_cnt], int num_points, float Rtime, float *result);
void calculate_interpolation1(float *t, float *x, int num_points, float Rtime, float *result);

// 값 확인을 위해 UART3 통신 관련 설정.
void USART3_Init();  // USART3를 사용하기 위한 설정(configuration) 함수
void TX3_Char(char);         // USART3를 이용하여 char 하나를 전송하는 함수
void TX3_PutString(char*);   // USART3를 이용하여 string data를 전송하는 함수

// SD 통신을 위해 SPI를 설정하는 함수
void SPI1_Init();               		// SPI 통신을 학습할 수 있는 함수
uint8_t SPI1_Transfer(uint8_t data); 	// 특정 문자형 데이터를 SPI로 보내는 함수

// Simulink High speed USB 통신을 위한 변수선언
extern uint8_t UserTxBufferFS[];  		// USB Transmit Buffer
extern uint8_t UserRxBufferFS[];  		// USB Receive Buffer
extern int RxPosition, RxSize;    		// USB Receive에 사용되는 변수들
extern char ReadDone;
extern USBD_HandleTypeDef hUsbDeviceFS; // usb_device.c에 정의되어 있음
extern ToHost toSimulink;

char str[64]; // 문자열을 전송하기 위한 문자열 버퍼

// 엔코더1(motor), 2(pendulum1), 3(pendulum2) 값을 수신할 변수
volatile int32_t enc1, enc1_p;
volatile int16_t enc2, enc2_p;
volatile int16_t enc3, enc3_p;

// 상태 변수 7개와 입력변수 1개
volatile float cart_pos = 0;
volatile float cart_vel = 0;
volatile float cart_acc = 0;
volatile float cart_int = 0;
volatile float theta1 = 0;
volatile float theta2 = 0;
volatile float d_theta1 = 0;
volatile float d_theta2 = 0;


// 가속도 적분,(속도 레퍼런스)값
volatile float acc_int = 0;

// 실제 모터에 인가할 전압값
volatile float volt;

// 실제 시뮬레이션 시간을 저장하는 변수의 경우 신뢰도를 위해 double형으로 저장함.
volatile double time_r = 0.0;  	// simulation time
volatile double time_ff = 0.0;  // feedforward time
volatile double time_L = 0.0;  	// Linear time

// 스위치 플래그
volatile bool sd_flag = false;		// shut down flag 이상감지시 전원 정지 플레그
volatile bool tr_flag = false;		// transition flag 천이 상태인지 확인하는 플래그
volatile bool reset_flag = false;	// 적분값을 초기화 하기 위한 플래그.

bool auto_flag1 = false;	// Ramdom pattern1 플래그.
bool auto_flag2 = false;	// Ramdom pattern2 플래그.
bool auto_flag3 = false;	// Ramdom pattern3 플래그.
bool auto_flag4 = false;	// Ramdom pattern4 플래그.

// EP 현제 모드(mode), 과거 모드(mode_p), 천이 궤적 number(m)
volatile uint8_t mode = 0;
volatile uint8_t mode_p = 0;
volatile uint8_t m = 0;
volatile uint8_t m_p = 0;

// 보간으로 구해진 선행 궤적을 저장할 변수와 가속도 보상 계산에 필요한 변수들
volatile float FF_x[DIP_state_cnt] = { };
volatile float FF_K[DIP_state_cnt] = { };
volatile float FF_u = 0;
volatile float err[DIP_state_cnt] = { };

// M/T Method 에서 사용되는 변수
volatile int16_t enc1_cnt;
volatile int16_t enc2_cnt;
volatile int16_t enc1_cnt_prev;
volatile int16_t enc2_cnt_prev;
volatile int16_t enc1_diff;
volatile int16_t enc2_diff;
volatile uint32_t clock1_cnt;
volatile uint32_t clock2_cnt;
volatile uint32_t clock1_cnt_prev;
volatile uint32_t clock2_cnt_prev;
volatile uint32_t clock1_diff;
volatile uint32_t clock2_diff;
volatile float d_theta1_mt = 0;
volatile float d_theta2_mt = 0;
volatile float temp_d_theta1_mt;
volatile float temp_d_theta2_mt;
float temp_d_theta1;
float temp_d_theta2;

int main(void) {

	uint8_t total_file = 0;					// SD카드에 들어있는 총 파일 수 확인한다.
	uint8_t file_number = 0;				// 선택할 파일의 번호를 저장한다.
	uint32_t SD_start_sector[MAX_FILE];		// 파일에 수에 맞는 초기값 저장
	uint32_t SD_backup_sector[MAX_FILE];	// 파일에 수에 맞는 초기값 백업
	uint32_t SD_end_sector[MAX_FILE];		// 파일에 수에 맞는 종단값 저장
	uint8_t SDbuffer[BYTES_PER_SECTOR];		// 512byte의 값을 저장하는 SD카드 buffer
	uint8_t MFbuffer[2 * BYTES_PER_SECTOR];	// MatFile의 변수별 형식을 저장할 때 1024byte 정도의 크기면 충분하다.
	const uint8_t *pbuffer;					// 포인터 변수의 주솟값을 받아온다.

	uint32_t nextMF;			// 다음 MatFile의 해당하는 오리지널값
	uint16_t nextMF_index;		// 다음 sector의 시작 위치값
	uint32_t nextMF_sector;		// 다음 sector 번호 저장값

	uint8_t pattern1[12] = {1, 2, 3, 2, 1, 3, 1, 0, 2, 0, 3, 0};
	uint8_t pattern2[12] = {1, 2, 3, 2, 1, 0};
	uint8_t pattern3[12] = {3, 0, 2, 0, 1, 0};
	uint8_t pattern4[12] = {3, 0};
	uint8_t p_i = 0;


	// 주기적 시간계산에 관련한 변수들
	uint32_t SampleTimeCycle;  	// sample time에 해당하는 instruction cycle의 수
	uint32_t update_time;		// 샘플링 타임의 맞춰 start 시간과 update 시간을 지정한다.
	float dt = sample_time;   	// sample time

	RCC_Configuration();		// clock 설정
	SystemCoreClockUpdate(); 	// clock update 설정
	DWT_Init();            		// DWT initialization

//	SDIO_Init();				// SDIO1 통신 초기설정
	MX_USB_DEVICE_Init();  		// USB를 사용하려면 이 line을 활성화 해야 한다.
	Timer8_PWM_dir_Init(); 		// TIM8_CH3N : PB1 --> PWM 		// PIOE     : PE8 --> dir
	Timer2_Encoder_Init(); 		// TIM2_CH1 : PA15 	--> A상 연결 	// TIM2_CH2 : PB3 --> B상 연결 (BLDC모터 풀업저항 구성해야 함.)
	Timer3_Encoder_Init(); 		// TIM3_CH1 : PB4 	--> A상 연결 	// TIM3_CH2 : PB5 --> B상 연결
	Timer4_Encoder_Init(); 		// TIM4_CH1 : PB6 	--> A상 연결 	// TIM4_CH2 : PB7 --> B상 연결
	Timer6_Interrupt_Init();	// 1ms 주기로 인터럽트를 걸저주는 초기화
	Timer1_MT_Interrupt_Init();	// 1단부 ENC 값을 받아와서 속도 계산하는 핸들러의 init
	Timer12_MT_Interrupt_Init();	// 2단부 ENC 값을 받아와서 속도 계산하는 핸들러의 init
	Timer5_UPCounter_Init();	// 단순 UpCounter. M/T method에서 cnt의 변화를 보기 위한 함수의 init
	Timer9_Interrupt_Init();  	// 1단부 1ms 주기로 인터럽트를 걸어주는 interrupt timer 초기화 M/T method 용
	Timer11_Interrupt_Init();  	// 2단부 1ms 주기로 인터럽트를 걸어주는 interrupt timer 초기화 M/T method 용
	PIO_Configure_Init();
	LED_SW_Init();				// LED, SW 설정
	con_PI1_Init(); 			// PI 속도 제어기
	USART3_Init();				// UART3 사용 선언 초기화

	SPI1_Init();
	SD_Init();
	FAT32_Init();

	// encoder 과거값 변수의 초기값을 0으로 설정
	enc1_p = 0;
	enc2_p = 0;
	enc3_p = 0;

	// 돌입 전류를 막고자 시작 전 1초간 여유를 주기 위함.
	DWT_ms_Delay(1000);
	total_file = fatGetDirEntry(FirstDirCluster); 			// 파일 수 확인

	// SD카드 파일들의 시작 sector값을 저장한다.
	for (int i = 0; i < total_file; i++) SD_start_sector[i] = fatClustToSect(file_start_cluster[i]);
	// 백업파일에 초기값 저장.
	memcpy(SD_backup_sector, SD_start_sector, sizeof(SD_start_sector));

	// SD카드 제목 출력
	for (int i = 0; i < total_file; i++) {
		uint8_t file_flag;		// 파일 이름에 따라 포맷형식 확인.
		bool name_flag = false;	// 이름 일치 여부 확인.

		// 파일명을 저장할 변수값을 초기화 시켜줌
		for (int j = 0; j < name_max; j++)
			file_name[j] = 0;

		// 파일을 하나 선택한다!
		file_flag = Get_long_filename(i);	// check file name

		// 그 이름이 짧으면 0, 길면 1
		if (file_flag == 0)					// short file name(8.3 format)
			Save_short_filename();

		// 이름이 길면 이 방식을 이용한다.
		else if (file_flag == 1)			// long file name
			Save_long_filename();

		else if (file_flag == 2) {	// file name is longer than 195 characters
		}
		else {			// file name error
		}

		for (int j = 0; j < sizeof(tmp_name); j++){
			name_flag = true;
			if(file_name[j] != tmp_name[j]) // 파일명이 다르면 끝낸다.
				{
					name_flag = false;
					break;
				}
		}

		if (name_flag) file_number = i;

		DWT_us_Delay(100); // 약간의 기다림이 필요한 것 같다.
	}

	// 파일의 마지막 섹터를 저장하기.
	for (int i = 0; i < total_file; i++) SD_end_sector[i] = (file_size[i] >> 9) + SD_start_sector[i];

	// mat 파일의 기본 파일 데이터 가져오기
	SD_read_sector(SD_start_sector[file_number], SDbuffer); // MatFile의 가장 첫 데이터 512Byte 읽기
	memcpy(Mat_Info, SDbuffer, sizeof(Mat_Info));			// MatFile의 기본 정보를 저장한다.
	memcpy(MFbuffer, SDbuffer, sizeof(SDbuffer));			// MFbuffer에도 초기 값을 저장.

	pbuffer = SDbuffer + sizeof(Mat_Info); 					// pbuffer 버퍼는 MatFile 기본정보 이후부터 시작한다.
	memcpy(Mat_Format, pbuffer, sizeof(Mat_Format));		// 첫 번째 변수 Format을 저장한다.

	for (int i = 0; i < 5; i++) {
		uint32_t DataType[5] = { }; // 올바른 데이터인지 확인하기 위한 5개의 DataType
		uint32_t name_len;			// name 길이값
		uint32_t data_len;			// data 길이값

		// MF의 해당위치에 반드시 들어가야하는 dataTye 수신
		DataType[0] = *((uint32_t*) (Mat_Format));		// MF의 가장 첫 부분에는 miMATRIX값이 있어야 한다. 없다면 mat 파일을 저장할 때 압축형식으로 되었을 확률이 크다.
		DataType[1] = *((uint32_t*) (Mat_Format + 8));	// MF의 해당 부분은 Array Flags로 반드시 miUINT32값이 있어야 한다.
		DataType[2] = *((uint32_t*) (Mat_Format + 24));	// MF의 해당 부분은 Dimensions Array로 반드시 miINT32값이 있어야 한다.

		// 모든 데이터가 정상일 때 동작.
		if (DataType[0] == miMATRIX && DataType[1] == miUINT32 && DataType[2] == miINT32) {}
		// 데이터가 잘못된 값이면 모든 동작을 멈춘다.(무한 루프)
		else {
			GPIOD->BSRR |= GPIO_BSRR_BS13;
			while (1) ;
		}

		// 가장 첫번째로 수신한 경우 이미 값을 받아왔기 때문에, 예외처리를 한다.
		if (i == 0) {
			MF[i].mf_sector = SD_start_sector[file_number]; // MF의 초기값을 그대로 섹터값으로 저장한다.
			MF[i].mf_sector_index = 0x80;					// MF의 시작 위치를 저장한다.
		}
		// 첫 번쨰가 아니라면, 마지막 단계에서 값을 업데이트한다. 따라서 그 값을 그대로 사용한다.
		else {
			MF[i].mf_sector = nextMF_sector;				// MF의 섹터값을 저장한다.
			MF[i].mf_sector_index = nextMF_index;			// MF의 시작 위치를 저장한다.
		}

		// 이후 MF에서 필요한 데이터를 알맞게 저장한다.
		MF[i].mf_len = *((uint32_t*) (Mat_Format + 4)) + 8; 	// 해당하는 변수값의 총 데이터를 포함한 크기를 저장한다. 그리고 시작부분도 포함하기 위해 8 byte를 더해준다.
		MF[i].mf_classType = *((uint32_t*) (Mat_Format + 16));	// 데이터 저장 형식이 SINGLE인지, DOUBLE인지 확인 (Classes 0x07 -> SIGNLE, 0x06 -> DOUBLE)
		MF[i].mf_rows = *((uint32_t*) (Mat_Format + 32));		// 변수의 행 크기값 저장
		MF[i].mf_cols = *((uint32_t*) (Mat_Format + 36));		// 변수의 열 크기값 저장

		if(MF[i].mf_classType == mxSINGLE){}
		// 데이터가 잘못된 값이면 모든 동작을 멈춘다.(무한 루프)
		else {
			GPIOD->BSRR |= GPIO_BSRR_BS13;
			while (1) ;
		}

		pbuffer = MFbuffer + MF[i].mf_sector_index + MF_index;	// 포인터값을 이용해서 이후 값을 저장한다. (시작 인덱스 값과 정해진 MF값이 끝나는 값으로 초기 위치를 정함.)

		name_len = *((uint16_t*) (pbuffer + 0x02));		// 변수 길이가 16bit라고 가정한다.

		// 만약에 변수 길이가 잡히지 않는다면, 변수 크기가 32bit일 것으로 (변수명의 길이가 큰 경우 32bit로 저장됨.) 가정한다.
		if (name_len == 0) {
			DataType[3] = *((uint32_t*) pbuffer);		// MF의 해당 부분은 Array Name으로 반드시 miINT8값이 있어야 한다.
			name_len = *((uint32_t*) (pbuffer + 0x04));	// 변수 길이가 32bit일 것으로 값을 다시 저장한다.
			pbuffer += 0x08;							// pbuffer에 변수 길이와 DataType 이후 값으로 값을 업데이트 해준다.
		}
		// 변수명 길이가 16bit가 맞다면, 그대로 진행시킨다.
		else {
			DataType[3] = *((uint16_t*) pbuffer);		// 이 때는 MF의 해당 부분은 Array Name으로 반드시 miINT8값이 있어야 하며, 16bit의 공간을 사용한다.
			pbuffer += 0x04;							// pbuffer에 변수 길이와 DataType 이후 값으로 값을 업데이트 해준다.
		}

		// DataType이 제대로 된 값이라면, 다음 일을 수행한다.
		if (DataType[3] == miINT8) {
			MF[i].mf_name_len = name_len;				// 최종 변수명 크기를 저장한다.
			memcpy(MF[i].mf_name, pbuffer, name_len);	// 변수명을 저장한다.
		}
		// DataType이 값이 잘못된 값이면 모든 동작을 멈춘다.(무한 루프)
		else {
			GPIOD->BSRR |= GPIO_BSRR_BS13;
			while (1) ;
		}

		// 변수명을 읽을 때, 이름의 길이에 따라 생기는 빈칸의 수가 다르다. 이 값을 보상해주기 위해 이 작업이 필요하다.
		// 변수명이 4byte 단위로 끊어진다면, pbuffer에 변수명 크기 그대로 값을 밀면 된다.
		if (name_len % 4 == 0)
			pbuffer += name_len;
		// 변수명이 4byte 단위가 아니라면, 빈칸을 채워야한다.
		else {
			// 문자가 8byte 이하라면, 다음의 조건을 이용하여 pbuffer에 값을 업데이트한다.
			if (name_len < 8)
				pbuffer += name_len + (4 - name_len % 4);
			// 문자가 8byte 이상이라면, 다음의 조건을 이용하여 pbuffer에 값을 업데이트한다.
			else
				pbuffer += name_len + (8 - name_len % 8);
		}


		data_len = *((uint16_t*) (pbuffer + 0x02));		// 변수 길이가 16bit라고 가정한다.

		// 만약에 변수 길이가 잡히지 않는다면, 변수 크기가 32bit일 것으로 (변수명의 길이가 큰 경우 32bit로 저장됨.) 가정한다.
		if (data_len == 0) {
			DataType[4] = *((uint32_t*) pbuffer);		// MF의 해당 부분은 변수의 Real part으로 저장형식을 맞췄다면 반드시 miSINGLE값이 있어야 한다.
			data_len = *((uint32_t*) (pbuffer + 0x04)); // 변수 길이가 32bit일 것으로 값을 다시 저장한다.
			pbuffer += 0x08;							// pbuffer에 변수 길이와 DataType 이후 값으로 값을 업데이트 해준다.
		}
		// 변수명 길이가 16bit가 맞다면, 그대로 진행시킨다.
		else {
			DataType[4] = *((uint16_t*) pbuffer);		// 이 때는 MF의 해당 부분은 변수의 Real part으로 저장형식을 맞췄다면 반드시 miSINGLE값이 있어야 하며, 16bit의 공간을 사용한다.
			pbuffer += 0x04;							// pbuffer에 변수 길이와 DataType 이후 값으로 값을 업데이트 해준다.
		}

		// DataType이 제대로 된 값이라면, 다음 일을 수행한다.
		if (DataType[4] == miSINGLE) {
			MF[i].mf_data_len = data_len;				// 최종 변수명 크기를 저장한다.
		}
		// DataType이 값이 잘못된 값이면 모든 동작을 멈춘다.(무한 루프)
		else {
			GPIOD->BSRR |= GPIO_BSRR_BS13;
			while (1) ;
		}

		MF[i].mf_data_index = pbuffer - MFbuffer; 		// 마지막으로 데이터가 시작되는 순서값을 저장한다.

		nextMF = MF[i].mf_sector_index + MF[i].mf_len;	// 다음 변수가 있는 곳은 현재 데이터가 있는 index에서 데이터의 총 길이를 더한값을 이용한다.
		nextMF_index = nextMF % BYTES_PER_SECTOR;		// 512의 나머지로 시작 지점의 인덱스를 찾아 저장한다.
		nextMF_sector = MF[i].mf_sector + nextMF / BYTES_PER_SECTOR; // 현재 sector에서 512로 나는 몫을 더해 다음 변수의 위치를 저장한다.

		// 다음 MFbuffer를 채워준다.
		SD_read_sector(nextMF_sector, SDbuffer);
		memcpy(MFbuffer, SDbuffer, sizeof(SDbuffer));
		SD_read_sector(nextMF_sector + 1, SDbuffer);
		memcpy(MFbuffer + BYTES_PER_SECTOR, SDbuffer, sizeof(SDbuffer));

		// 다음 변수의 고정된 40byte의 형식을 미리 저장한다.
		pbuffer = MFbuffer + nextMF_index;
		memcpy(Mat_Format, pbuffer, sizeof(Mat_Format));
	}// 이 과정이 끝나면, 모든 변수를 읽어온 상태이다.


	// 변수명이 맞는지 확인하기
	// ASCII code의 빠른 순으로 정렬되는 듯 함.
	// 이름순을 정렬되기 때문에, MF 순서에 맞는 mat 변수가 고정되어있음.
	// 제어 설계자가 필요에 의해서 다른 변수명을 사용했다면, 그것에 맞게 바꿔주어야한다.
	// 그리고 바꾸는기준은 ASCII code의 이름순이라는 것을 명심해야 한다.
	// 혹시나 mat 변수의 수가 늘었다면 MF의 개수 자체도 늘려야한다.
	if (strcmp(MF[0].mf_name, "K_LQ_mat")
			|| strcmp(MF[1].mf_name, "K_mat")
			|| strcmp(MF[2].mf_name, "control_mat")
			|| strcmp(MF[3].mf_name, "state_mat")
			|| strcmp(MF[4].mf_name, "time_mat")) {
		GPIOD->BSRR |= GPIO_BSRR_BS13;
		while (1) ; // 미리 선정한 이름과 다르다면 무한 루프에 빠진다.
	}

	// 각각에 변수의 크기가 맞는지 확인한다.
	// K_LQ_mat 	-> 시불변 LQ gain이 들어있는 변수로 1xn의 값을 갖는다. (여기서 n은 상태변수와 EP수의 곱)
	// K_mat		-> feedforward의 궤적을 보상하는 시변 LQ gain값으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 상태변수와 천이가능한 궤적수의 곱)
	// control_mat	-> feedforward의 입력 궤적으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 천이가능한 궤적수의 수)
	// state_mat	-> feedforward의 상태 궤적으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 상태변수와 천이가능한 궤적수의 곱)
	// time_mat		-> feedforward의 궤적 시간으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 천이가능한 궤적수의 수)
	if (!(MF[0].mf_rows == 1 && MF[0].mf_cols == DIP_state_cnt * DIP_EP_cnt
			&& MF[1].mf_rows == DIP_point_cnt && MF[1].mf_cols == DIP_trans_cnt * DIP_state_cnt
			&& MF[2].mf_rows == DIP_point_cnt && MF[2].mf_cols == DIP_trans_cnt
			&& MF[3].mf_rows == DIP_point_cnt && MF[3].mf_cols == DIP_trans_cnt * DIP_state_cnt
			&& MF[4].mf_rows == DIP_point_cnt && MF[4].mf_cols == DIP_trans_cnt)) {
		GPIOD->BSRR |= GPIO_BSRR_BS13;
		while (1) ;// 미리 선정한 깂과 다르다면 무한 루프에 빠진다.
	}

	// 시불변 K_LQ_mat의 값을 읽어 EPnK값에 저장한다.
	// 시변 값들은 선택된 궤적에 따라 바뀌지만, 이 값은 처음부터 받아온 값을 계속 사용한다.
	Get_EPnK_param(EPnK);

	// loop 시간에 관한 설정으로 천이 제어 자체는 타이머 인터럽트로 진행중이기 때문에,
	// 메인 loop의 시간을 너무 빠르게 할 필요는 없다.
	SampleTimeCycle = (uint32_t) (SystemCoreClock * dt);	// system core clock = 168e6
	update_time = DWT->CYCCNT;								// 현재 시간 저장

	// 여기서 encoder 값을 초기화하지 않으면 timer/counter 4번에 쓰레기값이 자꾸 생기는 증상이 있음.
	TIM2->CNT = 0;  // Counter를 0으로 clear
	TIM3->CNT = 0;  // Counter를 0으로 clear
	TIM4->CNT = 0;  // Counter를 0으로 clear

	// LED 출력에 관한 핀들을 세팅함
	// 모든 준비가 setup 단계에서 마무리 되었다면 EP0의 상태와 같으므로 LED2번의 빛만 ON 시킨다.
	// 혹시 시작단계에서 LED3, 4번에 ON이 되어있다면, 문제가 생겨 무한 루프에 빠진 상태이다.
	GPIOD->BSRR = GPIO_BSRR_BS12;	// EP0 상태
	GPIOD->BSRR = GPIO_BSRR_BR13;	// EP1 상태
	GPIOD->BSRR = GPIO_BSRR_BR14;	// EP2 상태
	GPIOD->BSRR = GPIO_BSRR_BR15;	// EP3 상태
	//===========================================================
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	//============================================================

	enc1_cnt_prev = TIM3->CNT;
	clock1_cnt_prev = TIM5->CNT;
	enc2_cnt_prev = TIM4->CNT;
	clock2_cnt_prev = TIM5->CNT;
	// 해당 부분부터는 메인 loop 부분으로 이 영역에서는 스위치 신호에 맞춰 천이 궤적을 SD카드에서 가져오고,
	// 모든 값을 가져온 이후에 tr_flag를 True로 바꾸는 순간 천이 시작된다.
	// 2-DOF 제어는 천이 과정 중에 다음 상태로 바꿀 수 없으므로, 천이 과정중 다음 궤적 명령은 무시된다.
	while (1) {
		GPIOC->BSRR |= GPIO_BSRR_BS2;   // PC2 HIGH;
		temp_d_theta1_mt = d_theta1_mt;
		temp_d_theta1 = d_theta1;
		temp_d_theta2_mt = d_theta2_mt;
		temp_d_theta2 = d_theta2;
		// 천이 궤적 선택부로, 기본적으로 스위치가 8개인 상황임.
		// 각각 EP0, EP1, EP2, EP3, Random1, Random2, ... 과 대응되며,
		// 또한 천이 중이 아닌경우에만 실행. (!tr_flag)
		// mode는 EP의 상태를 지정하는 변수, auto_flag는 Random의 종류를 선택하는 변수.
//		//
//		if (!tr_flag) {
//
//			if (!(GPIOE->IDR & 0x01)) {			// SW1
//				mode = 0;						// 원하는 다음 상태의 EP 번호
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				GPIOD->ODR |= 0x00001000;		// EP 번호에 맞는 LED ON
//				auto_flag1 = false;
//				auto_flag2 = false;
//				auto_flag3 = false;
//				auto_flag4 = false;
//
//			} else if (!(GPIOE->IDR & 0x02)) {	// SW2
//				mode = 1;						// 원하는 다음 상태의 EP 번호
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				GPIOD->ODR |= 0x00002000;		// EP 번호에 맞는 LED ON
//				auto_flag1 = false;
//				auto_flag2 = false;
//				auto_flag3 = false;
//				auto_flag4 = false;
//
//			} else if (!(GPIOE->IDR & 0x04)) {	// SW3
//				mode = 2;						// 원하는 다음 상태의 EP 번호
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				GPIOD->ODR |= 0x00004000;		// EP 번호에 맞는 LED ON
//				auto_flag1 = false;
//				auto_flag2 = false;
//				auto_flag3 = false;
//				auto_flag4 = false;
//
//			} else if (!(GPIOE->IDR & 0x08)) {	// SW4
//				mode = 3;						// 원하는 다음 상태의 EP 번호
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				GPIOD->ODR |= 0x00008000;		// EP 번호에 맞는 LED ON
//				auto_flag1 = false;
//				auto_flag2 = false;
//				auto_flag3 = false;
//				auto_flag4 = false;
//
//			} else if (!(GPIOE->IDR & 0x10)) {	// SW5
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				auto_flag1 = true;				// Random pattern1 ON
//				auto_flag2 = false;
//				auto_flag3 = false;
//				auto_flag4 = false;
//
//			} else if (!(GPIOE->IDR & 0x20)) {	// SW6
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				auto_flag1 = false;
//				auto_flag2 = true;				// Random pattern2 ON
//				auto_flag3 = false;
//				auto_flag4 = false;
//
//			} else if (!(GPIOE->IDR & 0x40)) {	// SW7
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				auto_flag1 = false;
//				auto_flag2 = false;
//				auto_flag3 = true;				// Random pattern3 ON
//				auto_flag4 = false;
//
//			} else if (!(GPIOE->IDR & 0x80)) {	// SW8
//				GPIOD->ODR &= ~0x0000F000;		// LED 초기화
//				auto_flag1 = false;
//				auto_flag2 = false;
//				auto_flag3 = false;
//				auto_flag4 = true;				// Random pattern4 ON
//			}
//
//			// Random pattern 선택시 다음의 조건문을 들어온다.
//			if (auto_flag1 || auto_flag2 || auto_flag3 || auto_flag4) {
//
//				// 선형 구간에서 4초 이상 안정적인 상태일 때 동작하도록 설계
//				if (time_L > 4) {
//
//					// Random pattern1
//					// 12가지 천이궤적을 모두 천이하는 궤적
//					// 천이순서는 다음과 같음.
//					// 1->2->3->2->1->3->1->0->2->0->3->0-> ...
//					if (auto_flag1) {
//						// 12가지 패턴이 끝나면 0으로 초기화
//						if (p_i >= 12)
//							p_i = 0;
//						// 현재 모드에 맞는 값으로 들어감.
//						mode = pattern1[p_i];
//					}
//
//					// Random pattern2
//					// 6가지 천이궤적을 모두 천이하는 궤적
//					// 천이순서는 다음과 같음.
//					// 1->2->3->2->1->0->...
//					if (auto_flag2) {
//						// 6가지 패턴이 끝나면 0으로 초기화
//						if (p_i >= 6)
//							p_i = 0;
//						// 현재 모드에 맞는 값으로 들어감.
//						mode = pattern2[p_i];
//					}
//
//					// Random pattern3
//					// 6가지 천이궤적을 모두 천이하는 궤적
//					// 천이순서는 다음과 같음.
//					// 3->0->2->0->1->0-> ...
//					if (auto_flag3) {
//						// 6가지 패턴이 끝나면 0으로 초기화
//						if (p_i >= 6)
//							p_i = 0;
//						// 현재 모드에 맞는 값으로 들어감.
//						mode = pattern3[p_i];
//					}
//
//					// Random pattern4
//					// 2가지 천이궤적을 모두 천이하는 궤적
//					// 천이순서는 다음과 같음.
//					// 3->0-> ...
//					if (auto_flag4) {
//						// 2가지 패턴이 끝나면 0으로 초기화
//						if (p_i >= 2)
//							p_i = 0;
//						// 현재 모드에 맞는 값으로 들어감.
//						mode = pattern4[p_i];
//					}
//
//					GPIOD->ODR &= ~0x0000F000;			// LED 초기화
//					GPIOD->ODR |= (0x00001000 << mode); // 현재 모드에 맞는 LED에 불이 켜짐.
//					p_i++;								// 패턴 index의 값을 증가시킴.
//				}
//			}
//			// Random pattern이 아닐 경우,
//			else
//				p_i = 0;			// Pattern index 초기화
//
//			DWT_us_Delay(100);
//
//			// 2-DOF m값 연산
//			//   \   0   1   2   3   (start)
//			//       ---------------
//			//   0|  x   4   7  10
//			//   1|  1   x   8  11
//			//   2|  2   5   x  12
//			//   3|  3   6   9   x
//			// (end)
//			//
//			// 위 표를 참고하여, 다음과 같은 식을 이용하면,
//			// 현재 EP(mode)와 과거 EP(mode_p)를 이용해서 궤적의 종류(m)을 구할 수 있음.
//			// 이는 현재 EP와 과거 EP가 다른 순간에만 진행되며, 그 때 천이 상태가 아니어야 함.
//			// 현재 상태가 과거 상태와 다를 떄만 동작, 가장 초기단계에서 mode와 mode_p가 0으로 초기화 되어있다.
//			if (mode != mode_p) {
//				// 이 연산 과정은 EP의 과거값과 현재값을 이용하면, 다음과 같은 식으로 m값을 쉽게 구할 수 있다.
//				// 다음의 과정은 위의 표를 참고하면 쉽게 이해할 수 있다.
//				if (mode_p < mode)
//					m = 3 * mode_p + mode;
//				else
//					m = 3 * mode_p + mode + 1;
//
//				// SD카드에서 m값에 맞는 feedforward 천이궤적을 가져옴.
//				Get_Transition_param(TRnnt, TRnnu, TRnnx, TRnnK, m);
//
//				// feedforward 상태 궤적의 가장 마지막 수렴점을 미리 받는다.
//				// 이는 선형제어 시 수렴값을 선정하기 위함.
//				for (int i = 0; i < DIP_state_cnt; i++)
//					EPnx_end[i] = TRnnx[DIP_point_cnt-1][i];
//
//				// 이후 과거값을 저장한다.
//				mode_p = mode;
//				m_p = m;
//
//				time_ff = time_r; 	// 궤적을 가져온 직후의 simulation time을 가져와서 feedforwawrd 시작 지점으로 선정한다.
//				tr_flag = true;		// 모든 값을 수정했다면, 천이를 진행한다.(시간 설정 이후에 바로 천이를 시작해야 오류가 나지 않는다. 이 사이에는 절대 아무 코드도 넣으면 안된다.)
//			}
//		}

//		sprintf(str, "%.2f\n",d_theta1_mt);
//		sprintf(str, "%.2f %.2f\n", temp_d_theta1_mt, temp_d_theta1);
		sprintf(str, "%.2f %.2f\n", temp_d_theta2_mt, temp_d_theta2);
		UsbPutString(str);
//		TX3_PutString(str);
		GPIOC->BSRR |= GPIO_BSRR_BR2;   // PC2 HIGH;
		while (!((update_time - DWT->CYCCNT) & 0x80000000));
		update_time += SampleTimeCycle;
	}
}

// 1ms의 주기마다 interrupt가 발생되는 TIM6 Handeler
// 이 곳에서는 샘플링 시간에 맞춰, 값을 수신받아 처리하고, 천이 연산을 수행하며, 최종적으로 선형 구간을 유지한다.
// 제한 값을 벗어났을 때의 예외처리가 중요하다.
//----------------------------------------------------------------
// Timer6 interrupt test의 설정에 의해 enable 된 interrupt handler.
//----------------------------------------------------------------
void TIM6_DAC_IRQHandler()  // IRQ Handler의 이름은 startup_stm32f407xx.s 에서 찾아볼 수 있다.
{
	if (TIM6->SR & TIM_SR_UIF) {
		// encoder 값을 수신받아, 상태 변수값으로 변환하는 단계, 우리가 사용할 parameter로 변환
		enc1 = TIM2->CNT;
		enc2 = TIM3->CNT;
		enc3 = TIM4->CNT;
//		temp_d_theta1_mt = d_theta1_mt;
		// encoder 값을 이용해서 상태변수값 생성
		cart_pos = (float) enc1 * enc1_to_pos;
		theta1 = (float) enc2 * enc2_to_rad - PI;
		theta2 = (float) enc3 * enc3_to_rad;
		cart_vel = (float) (enc1 - enc1_p) * enc1_to_pos / sample_time;	// 속도값 계산
		d_theta1 = (float) (enc2 - enc2_p) * enc2_to_rad / sample_time;	// 각속도값 계산
//		d_theta1 = temp_d_theta1_mt;
		d_theta2 = (float) (enc3 - enc3_p) * enc3_to_rad / sample_time;	// 각속도값 계산
//		d_theta2 = temp_d_theta2_mt;
		cart_int += cart_pos * sample_time;

		// 미분 계산을 위해 과거값 저장
		enc1_p = enc1;
		enc2_p = enc2;
		enc3_p = enc3;

//		// 천이 신호가 있을 때, 천이 제어 시작하는 부분
		if (tr_flag) {

			// 현재 시간이, 천이 시간보다 작다면 다음의 천이 부분을 수행한다.
			if (time_r <= TRnnt[DIP_point_cnt-1] + time_ff) {
				// 선형 보간을 통해 스윙업 궤적을 저장하는 부분으로, 각각 feedforward 상태변수, LQ보상, 입력변수를 업데이트한다.
				calculate_interpolation7(TRnnt, TRnnx, DIP_point_cnt, time_r - time_ff, FF_x);
				calculate_interpolation7(TRnnt, TRnnK, DIP_point_cnt, time_r - time_ff, FF_K);
				calculate_interpolation1(TRnnt, TRnnu, DIP_point_cnt, time_r - time_ff, &FF_u);
			}
			// 천이 시간이 넘어서면, tr_flag를 false시켜 천이를 종료한다.
			else
				tr_flag = false;

			time_L = 0; // 선형 구간이 아니면, 값을 초기화 시킨다.
		}
		// 천이 신호가 없을 때, 선형 제어를 유지하는 부분이다.
		else {
			// 가장 처음에, 아무런 입럭을 주고 있지 않다면, EP0을 유지하는 선형 제어 부분이 필요하다.
			// m값을 그대로 사용하면, SD카드에서 값을 읽어올 때 m값을 사용하는데, 아직 SD카드의 값을 읽어오는 과정이므로, 문제가 생긴다.
			// 따라서 SD카드가 값을 다 읽기 전까지는 바닥상태를 유지하기 위해 과거값을 이용한다.
			if (m_p == 0) {
				// EP0은 theta1 부분만 -pi이므로 종단 값을 다음과 같이 표기했다.
				FF_x[1] = -PI;
				memcpy(FF_K, EPnK, sizeof(FF_K));
			}
			// 가장 초기값이 아니라면, 이 부분으로 넘어간다.
			else {
				// 종단값은 상태 변수의 마지막 값을 넣어준다.
				memcpy(FF_x, EPnx_end, sizeof(FF_x));
				// 시불변 LQ 게인값은 미리 저장해둔 값을 사용하며, EP값에 맞춰 따라서 사용한다.
				memcpy(FF_K, EPnK + DIP_state_cnt * mode_p, sizeof(FF_K));
			}

			FF_u = 0;				// 선형 제어 부분에서 입력이 0으로 고정이다.
			time_L += sample_time; 	// 선형 구간으로 들어섰으니 시간을 체크한다.
		}

		// feedforward 궤적과 실제값의 오차값을 미리 계산해주고, 진자각도는 -pi~pi 사이로 변환시켜 저장한다.
		err[0] = cart_pos - FF_x[0];
		err[1] = modulo(theta1 - FF_x[1]);
		err[2] = modulo(theta2 - FF_x[2]);
		err[3] = cart_vel - FF_x[3];
		err[4] = d_theta1 - FF_x[4];
		err[5] = d_theta2 - FF_x[5];
		err[6] = cart_int - FF_x[6];


		// 카트에 인가될 가속도값 계산
		cart_acc = FF_u; // 선행 입력 가속도를 미리 저장한다.
		// 이후 반복문을 통해 보상값을 합해서 식을 간단하게 만들어준다.
		// 반복문은 간단하다. 상태변수 개수만큼 반복하며, 각각의 오차값에 보상값을 곱해주는 것을 더한다.
		for (int i = 0; i < DIP_state_cnt; i++)
			cart_acc += FF_K[i] * err[i];

		// 가속도 제한기
		if (cart_acc > 100.0)
			cart_acc = 100.0;
		else if (cart_acc < -100)
			cart_acc = -100.0;

		// 가속도 값을 적분하여, 속도 지령치를 만들어준다.
		acc_int += cart_acc * sample_time;

		// shut down 조건
		// 모든 연산과정이 끝난 이후 갑을 입력해주기 전에 경계값을 넘어서는 지령치가 존재한다면, 모든 과정을 멈춘다.
		// 이 과정에서 엔코더 값이 나오지 않는다면 큰 충돌이 나올 수 있으니 반드시 카트 엔코더가 이상이 없는지 먼저 확인해야한다.
		// 사용하는 DIP 플랜트에서 제약은 다음과 같다.
		// 1. cart 위치의 절댓값이 0.4m를 벗어났는지 확인
		// 2. cart 속도 지령치가 3m/s 이상의 빠른 속도를 요구하는지 확인
		if (fabs(cart_pos) > 0.4 || fabs(acc_int) > 3) {
			sd_flag = true;
			reset_flag = true;
		}

		// 혹시 reset_flag가 활성화 되었다면, 속도 지령과 카트의 적분값을 초기화 시켜라.
		if (reset_flag) {
			acc_int = 0;
			cart_int = 0;
			// 초기화를 했다면, 다시 flag를 비활성화 시킨다.
			reset_flag = false;
		}

		// shut down 상태가 되었을 때 모든 PWM 입력을 끄는 코드.
		if (sd_flag) {

			GPIOD->ODR &= ~0x0000F000;
			TIM8->CCR3 = 0;
		}
		// 정상이라면, PWM을 입력하라.
		else{
			// PI 속도 제어부 (LPF는 사용하지 않음.)
			con_PI1.Ref = acc_int;		// 모터 레퍼런스 값에 우리가 추종해야 할 계산된 속도 지령치를 인가한다.
			con_PI1.V_Fdb = cart_vel;	// 연산에 필요한 현재 cart 속도를 피드백으로 넣어줌
			con_PI1.calc(&con_PI1);		// PI 제어를 수행함.
			volt = con_PI1.Out;			// PI 제어기를 통해 계산된 전압값을 입력 받음.

			// 전압을 PWM 형태로 인가해주기위해 변환과정이 필요하며, 양의 방향일때와 음의 방향일 경우 Dir방향이 다름.
			if (volt * volt_to_duty > 0) {
				GPIOE->BSRR = GPIO_BSRR_BR8;		// Dir 정방향 설정
				TIM8->CCR3 = volt * volt_to_duty;	// 전압을 duty 형태로 변환해서 CCR값으로 입력
			}
			else {
				GPIOE->BSRR = GPIO_BSRR_BS8;		// Dir 역방향 설정
				TIM8->CCR3 = -volt * volt_to_duty;	// 전압을 duty 형태로 변환해서 CCR값으로 입력
			}
		}

		time_r += sample_time; // 시간 누적

		TIM6->SR &= ~TIM_SR_UIF;		// Clear pending bit of TIM8 interrupt.
	}
}

// 7개의 변수를 보간하는 함수
void calculate_interpolation7(float *t, float (*x)[DIP_state_cnt], int num_points, float Rtime, float *result) {
	int i; // 시간에 따라 현재 point가 어디인지 확인하기 위한 변수선언.
	// 현재 시간이 시간의 point 중 어디에 해당하는지 확인한다.
	// 알맞은 위치에 point의 i값을 찾는다.
	// 시간의 길이가 길어질 수 록 반복문의 반복횟수가 늘어난다. (점차 cost가 많이 소요된다.)
	for (i = 1; i < num_points; i++) {
		if (t[i] >= Rtime) break;
	}

	// 현재 시간, point 즉 i값을 찾았다면, 그 값이 배열의 총 갯수보다 작을 떄 동작한다.
	if (i < num_points) {
		// 선형 보간을 하기위해, 시간의 앞단과 뒷단을 가져온다.
		float t0 = t[i - 1], t1 = t[i];
		// 7번 반복해 각각의 변수에 맞춰 각각의 보간값을 결과값에 넣어준다.
		for (int k = 0; k < DIP_state_cnt; k++) {
			// 각각의 변수값의 앞단과 뒷단을 가져와 시간에 비율에 맞춰 값을 가져온다.
			float x0 = x[i - 1][k], x1 = x[i][k];
			// 계산결과 선형 보간의 결과값을 저장한다.
			result[k] = x0 + ((x1 - x0) * (Rtime - t0) / (t1 - t0));
		}
	}
	// 예외의 경우라면, 현재 시간이 총 갯수보다 크다는 의미로 문제가 있는 상태이다.
	else {
//		sprintf(str, "Target time is out of range\n");
//		UsbPutString(str);
	}
}

// 1개의 변수를 보간하는 함수
void calculate_interpolation1(float *t, float *x, int num_points, float Rtime, float *result) {
	int i; // 시간에 따라 현재 point가 어디인지 확인하기 위한 변수선언.
	// 현재 시간이 시간의 point 중 어디에 해당하는지 확인한다.
	// 알맞은 위치에 point의 i값을 찾는다.
	// 시간의 길이가 길어질 수 록 반복문의 반복횟수가 늘어난다. (점차 cost가 많이 소요된다.)
	for (i = 1; i < num_points; i++) {
		if (t[i] >= Rtime) break;
	}

	// 현재 시간, point 즉 i값을 찾았다면, 그 값이 배열의 총 갯수보다 작을 떄 동작한다.
	if (i < num_points) {
		// 선형 보간을 하기위해, 시간의 앞단과 뒷단을 가져온다.
		float t0 = t[i - 1], t1 = t[i];
		// 변수값의 앞단과 뒷단을 가져와 시간에 비율에 맞춰 값을 가져온다.
		float x0 = x[i - 1], x1 = x[i];
		// 계산결과 선형 보간의 결과값을 저장한다.
		*result = x0 + ((x1 - x0) * (Rtime - t0) / (t1 - t0));
	} else {
//		sprintf(str, "Target time is out of range\n");
//		UsbPutString(str);
	}
}

void RCC_Configuration() {
	// Flash memory Prefetch buffer, wait state 설정
	FLASH->ACR |= (uint32_t) FLASH_ACR_ICEN;    // Instruction cache enable
	FLASH->ACR |= (uint32_t) FLASH_ACR_DCEN;    // Data cache enable
	FLASH->ACR |= (uint32_t) FLASH_ACR_PRFTEN;  // Prefetch enable
	FLASH->ACR &= ~((uint32_t) FLASH_ACR_LATENCY);
	FLASH->ACR |= (0x5U << FLASH_ACR_LATENCY_Pos); // (0x5)가   five wait states에 해당

	//----------------------------------------------------------
	// HSEBYP의 사용에 대해
	// HSE crystal oscillator bypassed with external clock
	// X-TAL이 아닌 그냥 oscillator를 사용할 경우 이 option을 사용한다. 하지만
	// Core407V의 경우 X-TAL을 사용하므로 이 option을 사용하면 안된다.
	//----------------------------------------------------------
	//RCC->CR |= (uint32_t)RCC_CR_HSEBYP;  //
	RCC->CR |= (uint32_t) RCC_CR_HSEON;   // Enable HSE
	while ((RCC->CR & (uint32_t) RCC_CR_HSERDY) == 0) ; // Wait till HSE is ready

	// SYSCLK, HCLK, PCLK2, PCLK1 configuration
	RCC->CFGR &= ~((uint32_t) RCC_CFGR_HPRE); // AHB Prescaler=1. 즉 SYSCLK not divided -> 168 MHz
	RCC->CFGR &= ~((uint32_t) RCC_CFGR_PPRE2); // APB2 Prescaler=2. AHB clock divided by 2 -> 84 MHz (PCLK2)
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	RCC->CFGR &= ~((uint32_t) RCC_CFGR_PPRE1); // APB1 Prescaler=4. AHB clock divided by 4 -> 42 MHz (PCLK1)
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	//-------------------------------------------------------------------------
	// STM32F303K에서는 PLL을 설정하기 위해 RCC_CFGR을 사용하였다.
	// 하지만 STM32F407에서는 PLL을 설정하기 위한 Register인 RCC_PLLCFGR이 별도로 있다.
	// PLL을 설정하기 위해 PLL의 source 설정, HSE의 분주비 설정, PLL multiplication 설정
	// 등은 모드 PLL을 off 시킨 후에야 설정할 수 있다.
	// PLL의 source로 HSE를 선택할 것이다. 이것은 PLLCFGR의 22번 bit를 1로 설정하면된다.
	//
	// SYSCLK = HSE*PLLN/PLLM/PLLP = 8*168/4/2 = 168 MHz
	// PLL48CLK = HSE*PLLN/PLLM/PLLQ = 8*168/4/7 = 48 MHz
	//--------------------------------------------------------------------
	RCC->CR &= ~((uint32_t) RCC_CR_PLLON);     // 일단 PLL을 Off시킨다.
	while ((RCC->CR & RCC_CR_PLLRDY) == 1) ;  // PLL이 off 될때까지 기다린다.

	// PLL Source 설정
	RCC->PLLCFGR &= ~((uint32_t) RCC_PLLCFGR_PLLSRC);
	RCC->PLLCFGR |= ((uint32_t) 0x1 << RCC_PLLCFGR_PLLSRC_Pos); // PLL Source = HSE clock

	// PLLN 설정
	RCC->PLLCFGR &= ~((uint32_t) RCC_PLLCFGR_PLLN);  // 168
	RCC->PLLCFGR |= ((uint32_t) 168 << RCC_PLLCFGR_PLLN_Pos);  // PLLN = 168

	// PLLM 설정
	RCC->PLLCFGR &= ~((uint32_t) RCC_PLLCFGR_PLLM);
	RCC->PLLCFGR |= ((uint32_t) 0x4 << RCC_PLLCFGR_PLLM_Pos); // PLLM = 4 [주] stm32f407xx.h에 이 부분 오류가 있다.

	// PLLP 설정
	RCC->PLLCFGR &= ~((uint32_t) RCC_PLLCFGR_PLLP);
	RCC->PLLCFGR |= ((uint32_t) 0x0 << RCC_PLLCFGR_PLLP_Pos); // PLLP = 2로 설정하려면 해당 부분에 0b00을 써주어야 한다.

	// PLLQ 설정
	RCC->PLLCFGR &= ~((uint32_t) RCC_PLLCFGR_PLLQ);
	RCC->PLLCFGR |= ((uint32_t) 0x7 << RCC_PLLCFGR_PLLQ_Pos);  // PLLQ = 7

	RCC->CFGR &= ~((uint32_t) RCC_CFGR_MCO1);
	RCC->CFGR |= ((uint32_t) 0x3 << RCC_CFGR_MCO1_Pos); // MCO=0b11으로 설정해서 PLL clock이 MCO에 출력되도록 한다.

	RCC->CFGR &= ~((uint32_t) RCC_CFGR_MCO1PRE);
	RCC->CFGR |= ((uint32_t) 0x7 << RCC_CFGR_MCO1PRE_Pos); // 111을 써 주어야 division by 5가 된다.
														   // MCO prescaler = division by 5, PLL일 경우 168/5=33.6 MHz를 출력하게 된다.

	RCC->CR |= RCC_CR_PLLON;  // Enable PLL
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) ;   // Wait till PLL is ready

	RCC->CFGR &= ~((uint32_t) RCC_CFGR_SW);
	RCC->CFGR |= (uint32_t) RCC_CFGR_SW_PLL; // Select PLL as system clock source

	/* Wait till PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t) RCC_CFGR_SWS) != (uint32_t) RCC_CFGR_SWS_PLL) ;
}

//---------------------------------------------------------
// CubeMx로 할 때는 SystemClock_Config() 함수에 잔뜩 내용이 있지만
// 우리의 경우는 그 일을 RCC_Configuration()에서 다 하므로 필요없음
// compile 오류가 나는 것을 막기 위해 텅빈 함수로 만들어 버렸음.
//---------------------------------------------------------
void SystemClock_Config(void) {
}

//--------------------------------------------------------------------
// The definitive guide to ARM Cortex-M3 and Cortex-M4 processors 라는
// 책의 page 464에 보면 CoreDebug->DEMCR에 대한 설명을 찾아볼 수 있다.
// DEMCR : Debug exception and monitor control register
// DEMCR의 24번 bit가 TRCENA 인데 DWT, ETM, ITM, TPIU를 사용하려면 TRCENA bit가
// 1로 set 되어야만 한다.
//--------------------------------------------------------------------
void DWT_Init(void) {
	// DWT를 사용하기 위해서는 먼저 CoreDebug->DEMCR의 24번 bit인 TRCENA를 1로
	// set 한다. 그 다음 DWT의 CYCCNT를 0으로 clear 시키고 counter를 enable
	// 시키면 cycle counter를 동작시킬 수 있다.
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;  // Reset the counter
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // Enable the counter
	}
}

void DWT_us_Delay(uint32_t us) // microseconds
{
	uint32_t tp = DWT->CYCCNT + us * (SystemCoreClock / 1000000) - 1; // SystemCoreClock 계산이 잘못 되고 있다. Bug 수정 필요
	while (!((tp - DWT->CYCCNT) & 0x80000000));
}

void DWT_ms_Delay(uint32_t ms) {
	uint32_t tp = DWT->CYCCNT + ms * 1000 * (SystemCoreClock / 1000000) - 1;
	while (!((tp - DWT->CYCCNT) & 0x80000000));
}

//-----------------------------------------------------------
// UsbPutString : USB를 통해 string data를 전송하는 함수
//-----------------------------------------------------------
void UsbPutString(char *s) {
	uint32_t length;
	length = strlen(s);
	memcpy(UserTxBufferFS, s, length);
	CDC_Transmit_FS(UserTxBufferFS, length);
}

//--------------------------------------------------------------SDIO------
// 모터 PWM/dir 설정하기 위한 타이머1 설정 dir의 경우 GPIO를 이용한다.
//------------------------------
// STM32F407      Motor driver
//------------------------------
// PE9 -> PB1 (AF3)     PWM
// PE8 		            dir
// GND             Encoder GND (Green)
//----------------------------------------
// 50us로 PWM 생성
//-----------------------------------------------------------------------
void Timer8_PWM_dir_Init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // GPIOE clock enable, DIR
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB clock enable, PWM
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;   // TIM8 clock enable

	// direction 구현을 위해 PIO설정을 한다.
	GPIOE->MODER &= ~((uint32_t) GPIO_MODER_MODER8);
	GPIOE->MODER |= (0x1U << GPIO_MODER_MODER8_Pos); // Output의 경우 01 (=0x1)로 설정해야 함

	GPIOE->OSPEEDR &= ~((uint32_t) GPIO_OSPEEDER_OSPEEDR8);
	GPIOE->OSPEEDR |= (0x3U << GPIO_OSPEEDR_OSPEED8_Pos); // Very high speed일 경우 11 (=0x3)로 설정

	// PE9을 alternate function으로 설정하자. -> PB1 (AF3)
	GPIOB->MODER &= ~GPIO_MODER_MODER1;
	GPIOB->MODER |= 0x2 << GPIO_MODER_MODER1_Pos; // 10 (=0x2) : alternate function
	//----------------------------------------------------------
	// PE9은 AFR[1]에서 설정해야 한다. -> PB1는 AFR[0]
	// AFR[1]에서는 1번에 해당한다.
	//----------------------------------------------------------
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL1;
	GPIOB->AFR[0] |= 0x3 << GPIO_AFRL_AFSEL1_Pos;  //  AF3 할당, AF3는 0011 (=0x3)

	// 여기서부터 TIM8을 설정해보자. APB2 - 168MHz임
	TIM8->PSC = 0x0; // Prescale 설정 PSC=0, The counter clock frequency (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1). 즉 no division에 해당함.
	TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
	TIM8->CR1 &= ~TIM_CR1_CMS;
	TIM8->CR1 |= (0x3 << TIM_CR1_CMS_Pos); // 11 (=0x3) : Center-aligned mode 3. Output compare interrupt flag이 counter가 up, down일 때 모두 set 되는 방식
	TIM8->ARR = 4200;   // PWM frequency는 168 MHz/(2*4200) = 20 KHz

	// Channel 3N를 PWM mode로 설정한다. 여기부터 쭉 손봐야할듯
	TIM8->CCER &= ~TIM_CCER_CC3E;
	TIM8->CCER |= TIM_CCER_CC3NE;    // Capture/Compare 3 complementary output enable
	TIM8->CCER &= ~TIM_CCER_CC3NP; // CC3NP=0, Capture/compare complementary output 3을 active high로 설정

	TIM8->CCMR2 &= ~TIM_CCMR2_CC3S; // CC3S=0 : CC3 channel is configured as output

	TIM8->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM8->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos);  // OC3M=0110 (=0x6) : PWM mode 1

	//--------------------------------------------------------------------------------
	// Idle state 값을 설정하자. idle state라는 것은 MOE (Master Output Enable)이 0 이 되었을 때
	// PWM pin의 출력값을 뜻한다. 이 값들은 MOE=0 이 되고 dead-time 만큼 시간이 지난후에 출력된다.
	//--------------------------------------------------------------------------------
	TIM8->CR2 &= ~TIM_CR2_OIS3N;       // OC3N의 idle state는 Lwo로 설정

	TIM8->CR1 &= ~TIM_CR1_CKD;
	TIM8->CR1 |= (0x0 << TIM_CR1_CKD_Pos); // 00: t_DTS = tck_INT, 01:t_DTS=2*tck_INT, 10:t_DTS=4*tck_INT

	TIM8->BDTR &= ~TIM_BDTR_OSSI; // OSSI=0 : MOE=0일 경우 OC/OCN outputs are disabled-> Hi-Z state가 됨.
								  // 만약 OSSI=1 이면 MOE=0 일 때 deadtime 만큼 지난 후 idle level로 지정한 값으로 forced 된다.
	TIM8->BDTR &= ~TIM_BDTR_OSSR;

	TIM8->BDTR &= ~TIM_BDTR_DTG;
	TIM8->BDTR |= (10 << TIM_BDTR_DTG_Pos);  // dead-time=10*dts=59.52 [ns]

	TIM8->CNT = 0;   // Counter를 0으로 clear

	TIM8->CCR3 = 0;  // 2100은 4200의 절반에 해당하는 값

	TIM8->CR1 |= TIM_CR1_CEN;   // TIM1 enable.
	TIM8->BDTR |= TIM_BDTR_MOE; // MOE=1 : Main output enable
}

//void Timer1_PWM_dir_Init() {
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // GPIOE clock enable
//	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   // TIM1 clock enable
//
//	// direction 구현을 위해 PIO설정을 한다.
//	GPIOE->MODER &= ~((uint32_t) GPIO_MODER_MODER8);
//	GPIOE->MODER |= (0x1U << GPIO_MODER_MODER8_Pos); // Output의 경우 01 (=0x1)로 설정해야 함
//
//	GPIOE->OSPEEDR &= ~((uint32_t) GPIO_OSPEEDER_OSPEEDR8);
//	GPIOE->OSPEEDR |= (0x3U << GPIO_OSPEEDR_OSPEED8_Pos); // Very high speed일 경우 11 (=0x3)로 설정
//
//	// PE9을 alternate function으로 설정하자.
//	GPIOE->MODER &= ~GPIO_MODER_MODER9;
//	GPIOE->MODER |= 0x2 << GPIO_MODER_MODER9_Pos; // 10 (=0x2) : alternate function
//
//	// GPIO 속도를 High speed로 설정하자. 사실 이 부분이 필요한지 안한지는 잘 모르겠다. comment 처리해도 PWM은 잘 생성되더라...
//	GPIOE->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9;
//	GPIOE->OSPEEDR |= 0x3U << GPIO_OSPEEDR_OSPEED9_Pos; // High speed일 경우 11 (=0x3)로 설정
//
//	//----------------------------------------------------------
//	// PE9은 AFR[1]에서 설정해야 한다.
//	// AFR[1]에서는 1번에 해당한다.
//	// PWM과 관련해서는 모두 AF1에 해당한다.
//	//----------------------------------------------------------
//	GPIOE->AFR[1] &= ~GPIO_AFRH_AFSEL9;
//	GPIOE->AFR[1] |= 0x1 << GPIO_AFRH_AFSEL9_Pos;  //  AF1 할당, AF1는 0001 (=0x1)
//
//	// 여기서부터 TIM1을 설정해보자.
//	TIM1->PSC = 0x0; // Prescale 설정 PSC=0, The counter clock frequency (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1). 즉 no division에 해당함.
//	TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
//	TIM1->CR1 &= ~TIM_CR1_CMS;
//	TIM1->CR1 |= (0x3 << TIM_CR1_CMS_Pos); // 11 (=0x3) : Center-aligned mode 3. Output compare interrupt flag이 counter가 up, down일 때 모두 set 되는 방식
//	TIM1->ARR = 4200;   // PWM frequency는 168 MHz/(2*4200) = 20 KHz
//
//	// Channel 1를 PWM mode로 설정한다.
//	TIM1->CCER |= TIM_CCER_CC1E;    // Capture/compare output 1를 enable
//	TIM1->CCER &= ~TIM_CCER_CC1P; // CC1P=0 : Capture/compare output 1을 active high로 설정
////	TIM1->CCER |= TIM_CCER_CC1P; // CC1P=0 : Capture/compare output 1을 active Low로 설정
//
//	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S; // CC1S=0, CC1 channel is configured as output
//
//	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
//	TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);  // OC1M=0110 (=0x6) PWM mode 1
//
//	//--------------------------------------------------------------------------------
//	// Idle state 값을 설정하자. idle state라는 것은 MOE (Master Output Enable)이 0 이 되었을 때
//	// PWM pin의 출력값을 뜻한다. 이 값들은 MOE=0 이 되고 dead-time 만큼 시간이 지난후에 출력된다.
//	//--------------------------------------------------------------------------------
////	TIM1->CR2 |= TIM_CR2_OIS1;       // OC1의 idle state는 High로 설정
//	TIM1->CR2 &= ~TIM_CR2_OIS1;       // OC1의 idle state는 Low로 설정
//
//	TIM1->CR1 &= ~TIM_CR1_CKD;
//	TIM1->CR1 |= (0x0 << TIM_CR1_CKD_Pos); // 00: t_DTS = tck_INT, 01:t_DTS=2*tck_INT, 10:t_DTS=4*tck_INT
//
//	TIM1->BDTR &= ~TIM_BDTR_OSSI; // OSSI=0 : MOE=0일 경우 OC/OCN outputs are disabled-> Hi-Z state가 됨.
//								  // 만약 OSSI=1 이면 MOE=0 일 때 deadtime 만큼 지난 후 idle level로 지정한 값으로 forced 된다.
//	TIM1->BDTR &= ~TIM_BDTR_DTG;
//	TIM1->BDTR |= (10 << TIM_BDTR_DTG_Pos);  // dead-time=10*dts=59.52 [ns]
//
//	TIM1->CNT = 0;   // Counter를 0으로 clear
//
//	TIM1->CCR1 = 0;  // 2100은 4200의 절반에 해당하는 값
//
//	TIM1->CR1 |= TIM_CR1_CEN;   // TIM1 enable.
//	TIM1->BDTR |= TIM_BDTR_MOE; // MOE=1 : Main output enable
//}

//--------------------------------------------------------------------------
// TIM2는 General purpose timer이고 32 bit counter 이다. TIM2는 APB1에 포함된
// Peripheral 이다. Encoder counter를 처리하기 위한 Pin 은 다음과 같이 설정하기로 한다.
//
// TIM2_CH1 : PA15 --> A상 연결
// TIM2_CH2 : PB3  --> B상 연결
//
// Timer2는 APB1의 clock인 PCLK1을 사용하는데 만약에 APB1의 prescaler가 1보다 크면
// PCLK1 x 2에 해당하는 clock을 받게 된다. RCC_Configuration에서 APB1 prescaler를 4로
// 설정했으므로 PCLK1은 42 MHz가 되고 그것의 2배인 84 MHz의 clock이 Timer2에 사용되게 된다.
//--------------------------------------------------------------------------
// STM32F407      Pololu motor
//--------------------------------------------------------------------------
// PA15            A (Yellow)
// PB3             B (White)
// 5V              3.3V (Blue)
// GND             Encoder GND (Green)
//--------------------------------------------------------------------------
void Timer2_Encoder_Init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // TIM2 clock enable

	// PA15를 Alternate function으로 설정한다.
	GPIOA->MODER &= ~GPIO_MODER_MODER15;
	GPIOA->MODER |= 0x2 << GPIO_MODER_MODER15_Pos; // 10 (=0x2) : alternate function

	//----------------------------------------------------------
	// PA15는 AFR[0]에서 설정해야 한다. AFR[0]에서는 0번과 1번에 해당한다.
	// TIM2_CH1과 TIM2_CH2는 AF1에 해당한다.
	//----------------------------------------------------------
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL15; // AF1 할당, AF1는 0001 (=0x1)
	GPIOA->AFR[1] |= 0x1 << GPIO_AFRH_AFSEL15_Pos;  //  AF1 할당, AF1는 0001 (=0x1)

	// PB3를 Alternate function으로 설정한다.
	GPIOB->MODER &= ~GPIO_MODER_MODER3;
	GPIOB->MODER |= 0x2 << GPIO_MODER_MODER3_Pos; // 10 (=0x2) : alternate function

	//----------------------------------------------------------
	// PB3는 AFR[0]에서 설정해야 한다. AFR[0]에서는 0번과 1번에 해당한다.
	// TIM2_CH1과 TIM2_CH2는 AF1에 해당한다.
	//----------------------------------------------------------
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL3; // AF1 할당, AF1는 0001 (=0x1)
	GPIOB->AFR[0] |= 0x1 << GPIO_AFRL_AFSEL3_Pos;  //  AF1 할당, AF1는 0001 (=0x1)

	//------------------------------------------------------------------------
	// TIM2을 Encoder counter 기능을 수행하도록 설정해보자.
	// 먼저 Encoder counter는 slave mode로 동작할 때 얻는 기능이다. 따라서 SMCR을 조작해야 한다.
	//------------------------------------------------------------------------
	TIM2->SMCR &= ~TIM_SMCR_SMS;  // SMS는 모두 3-bit로 구성
	TIM2->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // 011 (=0x3) : Encoder mode 3

	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM2->CCMR1 |= (0x1 << TIM_CCMR1_CC1S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM2->CCMR1 |= (0x1 << TIM_CCMR1_CC2S_Pos); // 01 (=0x1) : CC2 channel is configured as input, IC2 is mapped on TI2

	TIM2->CCMR1 &= ~TIM_CCMR1_IC1F;
	TIM2->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
	TIM2->CCMR1 &= ~TIM_CCMR1_IC2F;
	TIM2->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	TIM2->CCER &= ~TIM_CCER_CC1P; // CC1P=0, non-inverted/rising edge. The circuit is sensitive to TIxFP1 rising edge
	TIM2->CCER &= ~TIM_CCER_CC2P;  // CC2P=0, non-inverted/rising edge.

	TIM2->CNT = 0;  // Counter를 0으로 clear

	TIM2->CR1 |= TIM_CR1_CEN; // TIM2을 enable 시킨다.
}


//--------------------------------------------------------------------------
// ****************FOR M/T Method****************** ==> datasheet 보니까 slave mode에서 encoder mode 3으로 쓰려면 채널 1과 2로 두 상을 받아와야하네 ㅅㅂ
// 6) 하다가 발견함.. 진짜 살자마렵네 일단 그러면 물리적으로 연결해서 핀을 뽑아야겠다.
// TIM3_CH1 => PB4 = A상
// TIM3_CH2 => PB5
// TIM3_CH3 => PB0 = B상
// TIM3_CH4 => PB1
// 0) TIM3, GPIOB clock 인가
// 1) GPIO 4개 모두 alternate function으로 설정
// 2) 각각 AFR[0] 설정 AF2
// 3) SMCR 조작. slave mode - encoder mode 3
// 4) 연결 : CCMR1_CC1S IC1를 TI1에, CCMR1_CC2S IC2를 TI1에, CCMR2_CC3S IC3를 TI3에, CCMR2_CC4S IC4를 TI3에 mapping
// 5) input filter : CCMR1에서 IC1F, CCMR2에서 IC3F
// 6) 원래대로 rising edge CCER_CC1P, CCER_CC3P. both edge detection 설정하기. CCER_CC2P, CCER_CC4P
// 7) input capture enable시키기 CCER_CC2E, CCER_CC4E
// 8) TIM3->CNT = 0; 초기화
// 9) TIM3 enable 시키고
// 10) TIM3->DIER |= TIM_DIER_CC2IE | TIM_DIER_CC4IE; capture 인터럽트 enable
// 11) NVIC_EnableIRQ(TIM3_IRQn);
// 12) TIM3_IRQHandler() 제작
//--------------------------------------------------------------------------
//void TIM3_IRQHandler() {
//
//}
//void Timer3_Encoder_Init() {
//	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   // TIM3 clock enable
//
//	// PB0, PB1, PB4, PB5를 Alternate function으로 설정한다.
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB clock enable
//	GPIOB->MODER &= (~GPIO_MODER_MODER0 & ~GPIO_MODER_MODER1 & ~GPIO_MODER_MODER4 & ~GPIO_MODER_MODER5);
//	GPIOB->MODER |= ((0x2 << GPIO_MODER_MODER0_Pos)	| (0x2 << GPIO_MODER_MODER1_Pos) | (0x2 << GPIO_MODER_MODER4_Pos) | (0x2 << GPIO_MODER_MODER5_Pos)); // 10 (=0x2) : alternate function
//
//	GPIOB->AFR[0] &= (~GPIO_AFRL_AFSEL0 & ~GPIO_AFRL_AFSEL1 & ~GPIO_AFRL_AFSEL4 & ~GPIO_AFRL_AFSEL5);
//	GPIOB->AFR[0] |= ((0x2 << GPIO_AFRL_AFSEL0_Pos)	| (0x2 << GPIO_AFRL_AFSEL1_Pos) | (0x2 << GPIO_AFRL_AFSEL4_Pos)	| (0x2 << GPIO_AFRL_AFSEL5_Pos));  //  AF2 할당, AF2는 0010 (=0x2)
//
//	//------------------------------------------------------------------------
//	// TIM3을 Encoder counter 기능을 수행하도록 설정해보자.
//	// 먼저 Encoder counter는 slave mode로 동작할 때 얻는 기능이다. 따라서 SMCR을 조작해야 한다.
//	//------------------------------------------------------------------------
//	TIM3->SMCR &= ~TIM_SMCR_SMS;  // SMS는 모두 3-bit로
//	TIM3->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // 011 (=0x3) : Encoder mode 3
//
//	//------------------------------------------------------------------------
//	// 내부에서 연결을 진행해야함.
//	// PB4 (TIM3_CH1)----------IC1
//	//                   |
//	// PB5 (TIM3_CH2)    ----- IC2
//	// PB0 (TIM3_CH3)----------IC3
//	//                   |
//	// PB1 (TIM3_CH4)    ----- IC4
//	//------------------------------------------------------------------------
//	TIM3->CCMR1 &= (~TIM_CCMR1_CC1S & ~TIM_CCMR1_CC2S);
//	TIM3->CCMR1 |= (0x1 << TIM_CCMR1_CC1S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1
//	TIM3->CCMR1 |= (0x2 << TIM_CCMR1_CC2S_Pos); // 10 (=0x2) : CC2 channel is configured as input, IC2 is mapped on TI1
//	TIM3->CCMR2 &= (~TIM_CCMR2_CC3S & ~TIM_CCMR2_CC4S);
//	TIM3->CCMR2 |= (0x1 << TIM_CCMR2_CC3S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC3 is mapped on TI3
//	TIM3->CCMR2 |= (0x2 << TIM_CCMR2_CC4S_Pos); // 10 (=0x2) : CC2 channel is configured as input, IC4 is mapped on TI3
//
//	// input filter CH1과 CH3으로 받기에, 두 채널에 필터. 4번의 연속된 이벤트가 있어야 안정된 신호로 인식
//	TIM3->CCMR1 &= ~TIM_CCMR1_IC1F;
//	TIM3->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
//	TIM3->CCMR2 &= ~TIM_CCMR2_IC3F;
//	TIM3->CCMR2 |= (0x2 << TIM_CCMR2_IC3F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
//
//	TIM3->CCER &= (~TIM_CCER_CC1P & ~TIM_CCER_CC3P); // CC1P=0,CC3P=0 non-inverted/rising edge. The circuit is sensitive to TIxFP1 rising edge
//	TIM3->CCER &= (~TIM_CCER_CC2P & ~TIM_CCER_CC4P);
//	TIM3->CCER |= ((0x3 << TIM_CCER_CC2P_Pos) | (0x3 << TIM_CCER_CC4P_Pos));  // CC2P=11(=0x3),CC4P=11(=0x3) noninverted/both edges
//	TIM3->CCER &= ~TIM_CCER_CC3P;
//
//	TIM3->CNT = 0;  // Counter를 0으로 clear
//
//	TIM3->CR1 |= TIM_CR1_CEN; // TIM3을 enable 시킨다.
//}
//----------------------------------------------------------------------------
// TIM3는 General purpose timer이고 16 bit counter 이다. Encoder counter를 처리하기
// 위한 Pin 은 다음고 같다.
//
// TIM3_CH1 : PB4 --> A상 연결 (Datasheet의  page 59 참조)
// TIM3_CH2 : PB5 --> B상 연결
//
// Timer3는 APB1의 clock인 PCLK1을 사용하는데 만약에 APB1의 prescaler가 1보다 크면
// PCLK1 x 2에 해당하는 clock을 받게 된다. RCC_Configuration에서 APB1 prescaler를 4로
// 설정했으므로 PCLK1은 42 MHz가 되고 그것의 2배인 84 MHz의 clock이 Timer3에 사용되게 된다.
//--------------------------------------------------------------------------
void Timer3_Encoder_Init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   // TIM3 clock enable

	// PB4, PB5를 Alternate function으로 설정한다.
	GPIOB->MODER &= (~GPIO_MODER_MODER4 & ~GPIO_MODER_MODER5);
	GPIOB->MODER |= ((0x2 << GPIO_MODER_MODER4_Pos)
			| (0x2 << GPIO_MODER_MODER5_Pos)); // 10 (=0x2) : alternate function

	//----------------------------------------------------------
	// PB4, PB5는 AFR[0]에서 설정해야 한다. AFR[0]에서는 4번과 5번에 해당한다.
	// TIM3_CH1과 TIM3_CH2는 AF2에 해당한다.
	//----------------------------------------------------------
	GPIOB->AFR[0] &= (~GPIO_AFRL_AFSEL4 & ~GPIO_AFRL_AFSEL5); // AF2 할당, AF2는 0010 (=0x2)
	GPIOB->AFR[0] |= ((0x2 << GPIO_AFRL_AFSEL4_Pos)
			| (0x2 << GPIO_AFRL_AFSEL5_Pos));  //  AF2 할당, AF2는 0010 (=0x2)

	//------------------------------------------------------------------------
	// TIM3을 Encoder counter 기능을 수행하도록 설정해보자.
	// 먼저 Encoder counter는 slave mode로 동작할 때 얻는 기능이다. 따라서 SMCR을 조작해야 한다.
	//------------------------------------------------------------------------
	TIM3->SMCR &= ~TIM_SMCR_SMS;  // SMS는 모두 3-bit로
	TIM3->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // 011 (=0x3) : Encoder mode 3

	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3->CCMR1 |= (0x1 << TIM_CCMR1_CC1S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1
	TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM3->CCMR1 |= (0x1 << TIM_CCMR1_CC2S_Pos); // 01 (=0x1) : CC2 channel is configured as input, IC2 is mapped on TI2

	TIM3->CCMR1 &= ~TIM_CCMR1_IC1F;
	TIM3->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
	TIM3->CCMR1 &= ~TIM_CCMR1_IC2F;
	TIM3->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	TIM3->CCER &= ~TIM_CCER_CC1P; // CC1P=0, non-inverted/rising edge. The circuit is sensitive to TIxFP1 rising edge
	TIM3->CCER &= ~TIM_CCER_CC2P;  // CC2P=0, non-inverted/rising edge.

	TIM3->CNT = 0;  // Counter를 0으로 clear

	TIM3->CR1 |= TIM_CR1_CEN; // TIM3을 enable 시킨다.
}

//--------------------------------------------------------------------------
// TIM4는 General purpose timer이고 16 bit counter 이다. Encoder counter를 처리하기
// 위한 Pin 은 다음고 같다.
//
// TIM4_CH1 : PB6 --> A상 연결 (Datasheet의  page 59 참조)
// TIM4_CH2 : PB7 --> B상 연결
//
// Timer3는 APB1의 clock인 PCLK1을 사용하는데 만약에 APB1의 prescaler가 1보다 크면
// PCLK1 x 2에 해당하는 clock을 받게 된다. RCC_Configuration에서 APB1 prescaler를 4로
// 설정했으므로 PCLK1은 42 MHz가 되고 그것의 2배인 84 MHz의 clock이 Timer3에 사용되게 된다.
//--------------------------------------------------------------------------
// STM32F407      Pololu motor
//--------------------------------------------------------------------------
// PB6             A (Yellow)
// PB7             B (White)
// 3.3V            3.3V (Blue)
// GND             Encoder GND (Green)
//--------------------------------------------------------------------------
void Timer4_Encoder_Init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   // TIM4 clock enable

	// PB6, PB7를 Alternate function으로 설정한다.
	GPIOB->MODER &= (~GPIO_MODER_MODER6 & ~GPIO_MODER_MODER7);
	GPIOB->MODER |= ((0x2 << GPIO_MODER_MODER6_Pos)
			| (0x2 << GPIO_MODER_MODER7_Pos)); // 10 (=0x2) : alternate function

	//----------------------------------------------------------
	// PB6, PB7는 AFR[0]에서 설정해야 한다. AFR[0]에서는 6번과 7번에 해당한다.
	// TIM4_CH1과 TIM4_CH2는 AF2에 해당한다.
	//----------------------------------------------------------
	GPIOB->AFR[0] &= (~GPIO_AFRL_AFSEL6 & ~GPIO_AFRL_AFSEL7); // AF2 할당, AF2는 0010 (=0x2)
	GPIOB->AFR[0] |= ((0x2 << GPIO_AFRL_AFSEL6_Pos)
			| (0x2 << GPIO_AFRL_AFSEL7_Pos));  //  AF2 할당, AF2는 0010 (=0x2)

	//------------------------------------------------------------------------
	// TIM4을 Encoder counter 기능을 수행하도록 설정해보자.
	// 먼저 Encoder counter는 slave mode로 동작할 때 얻는 기능이다. 따라서 SMCR을 조작해야 한다.
	//------------------------------------------------------------------------
	TIM4->SMCR &= ~TIM_SMCR_SMS;  // SMS는 모두 3-bit로
	TIM4->SMCR |= (0x3 << TIM_SMCR_SMS_Pos); // 011 (=0x3) : Encoder mode 3

	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM4->CCMR1 |= (0x1 << TIM_CCMR1_CC1S_Pos); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1
	TIM4->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM4->CCMR1 |= (0x1 << TIM_CCMR1_CC2S_Pos); // 01 (=0x1) : CC2 channel is configured as input, IC2 is mapped on TI2

	TIM4->CCMR1 &= ~TIM_CCMR1_IC1F;
	TIM4->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4
	TIM4->CCMR1 &= ~TIM_CCMR1_IC2F;
	TIM4->CCMR1 |= (0x2 << TIM_CCMR1_IC2F_Pos); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	TIM4->CCER &= ~TIM_CCER_CC1P; // CC1P=0, non-inverted/rising edge. The circuit is sensitive to TIxFP1 rising edge
	TIM4->CCER &= ~TIM_CCER_CC2P;  // CC2P=0, non-inverted/rising edge.

	TIM4->CNT = 0;  // Counter를 0으로 clear

	TIM4->CR1 |= TIM_CR1_CEN; // TIM4을 enable 시킨다.
}

//--------------------------------------------------------------------------
// 메인으로 1ms 마다 제어를 수행하도록 하는 인터럽트에 대한 초기설정
//--------------------------------------------------------------------------
void Timer6_Interrupt_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // TIM6 clock enable
	//--------------------------------------------------------------------------------------
	// TIM6에 대한 설정을 수행하자. 설정해주어야 할 것은 Prescaler 값, ARR(Auto-reload register)의 값
	// 참고로 TIM6는 APB1에 속하는데, Timer의 경우 APB1 peripheral과 달리 APB1의 prescaler 값이
	// 1보다 큰 경우 PCLK의 x2에 해당하는 clock을 입력받는다. RCC_Configuration()에서
	// APB1 Prescaler=4으로 설정했기 때문에 HCLK divided by 4가 되어 42MHz이다. 그런데
	// Timer의 경우 prescaler value=4는 1보다 큰 값이므로 PCLK의 x2가 되어 42MHz x 2 = 84MHz의 clock을
	// 사용하게 되는 것에 유의하자.
	//---------------------------------------------------------------------------------------
	TIM6->PSC = 839;   //  84 MHz/(839+1) = 100 KHz
	TIM6->ARR = 99;  // ARR의 16-bit register 이다. 100000 -> 1 KHz
	TIM6->CNT = 0;   // counter vale = 0 으로 초기화
	TIM6->DIER |= TIM_DIER_UIE; // Update interrupt enable.
	TIM6->CR1 |= TIM_CR1_ARPE;   // ARPE=1: TIMx_ARR register is buffered.
	TIM6->CR1 |= TIM_CR1_URS;   // URS=1: Only counter overflow/underflow generates an update interrupt or DMA request if enabled.

	//-----------------------------------------------------------------------------
	// NVIC 설정을 수행하자. TIM6_DAC1_IRQn=54 이다. 이것을 확인하려면 STM32F407의 reference manual
	// 의 page374를 보면 된다.
	//-----------------------------------------------------------------------------
	NVIC_DisableIRQ(TIM6_DAC_IRQn); // 위의 code 대신 이렇게 처리할 수도 있다. TIM6_DAC_IRQn는 stm32f407xx.h에 정의
	NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); // pending data를 clear
	NVIC_SetPriority(TIM6_DAC_IRQn,1); // 우선순위를 1번으로

	TIM6->CR1 |= TIM_CR1_CEN;     // TIM6 enable.
}
//=================For M/T Method=====================
//====TIM1====
//	A상 = TIM3_CH1(PB4) = TIM1_CH1(PE9)
//	B상 = TIM3_CH2(PB5) = TIM1_CH2(PE11)
//	0) GPIO 클럭 인가, TIM1 클럭 인가
//	1) GPIO 2개 alternate function으로 설정
//	2) AFR 설정
//	3) 연결 IC1 - TI1, IC2 - TI2
//	4) input filter
//	5) CCER에서 both edge detection
//	6) input capture enable 시키기. CCER_CC1E, CCER_CC2E
//	7) TIM1 enable
//	8) TIM1->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE; capture 인터럽트 enable
//	11) NVIC_EnableIRQ(TIM1_CC_IRQn);
//	12) TIM1_CC_IRQHandler() 제작

void Timer1_MT_Interrupt_Init() {
	// 1단은 PE9(A상), PE11(B상)을 Alternate function으로 설정한다.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // GPIOE clock enable
	GPIOE->MODER &= (~GPIO_MODER_MODER9 & ~GPIO_MODER_MODER11);
	GPIOE->MODER |= ((0x2 << GPIO_MODER_MODER9_Pos)	| (0x2 << GPIO_MODER_MODER11_Pos)); // 10 (=0x2) : alternate function

	GPIOE->AFR[1] &= (~GPIO_AFRH_AFSEL9 & ~GPIO_AFRH_AFSEL11);
	GPIOE->AFR[1] |= ((0x1 << GPIO_AFRH_AFSEL9_Pos)	| (0x1 << GPIO_AFRH_AFSEL11_Pos));  //  AF1 할당, AF1는 0001 (=0x1)

	// TIMER1 설정 input capture. 1단 A상 CC1 채널, B상 CC2 채널
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   // TIM1 clock enable
	TIM1->CCMR1 &= (~TIM_CCMR1_CC1S & ~TIM_CCMR1_CC2S);
	TIM1->CCMR1 |= ((0x1 << TIM_CCMR1_CC1S_Pos) |(0x1 << TIM_CCMR1_CC2S_Pos)); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1,CC2 channel is configured as input, IC2 is mapped on TI2
	// input filter CH1과 CH2으로 받기에, 두 채널에 필터. 4번의 연속된 이벤트가 있어야 안정된 신호로 인식
	TIM1->CCMR1 &= (~TIM_CCMR1_IC1F & ~TIM_CCMR1_IC2F);
	TIM1->CCMR1 |= ((0x2 << TIM_CCMR1_IC1F_Pos) | (0x2 << TIM_CCMR1_IC2F_Pos)); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	// both edge detection
	TIM1->CCER &= (~TIM_CCER_CC1P & ~TIM_CCER_CC2P);
	TIM1->CCER |= ((0x3 << TIM_CCER_CC1P_Pos) | (0x3 << TIM_CCER_CC2P_Pos));  // CC1P=11(=0x3),CC2P=11(=0x3) non inverted/both edges

	// input capture enable
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E); // 1 : Capture enabled

	TIM1->CR1 |= TIM_CR1_CEN; // TIM1을 enable 시킨다.

	TIM1->DIER &= (~TIM_DIER_CC1IE & ~TIM_DIER_CC2IE); // Capture/Compare 1,2 interrupt disable -> TIM9 Handler에서 enable

	NVIC_DisableIRQ(TIM1_CC_IRQn);
	NVIC_ClearPendingIRQ(TIM1_CC_IRQn); // pending data를 clear
	NVIC_SetPriority(TIM1_CC_IRQn,2);
	NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void TIM1_CC_IRQHandler() { // IRQ Handler의 이름은 startup_stm32f407xx.s 에서 찾아볼 수 있다.

	static int flag1=0;

	TIM1->SR &= (~TIM_SR_CC1IF & ~TIM_SR_CC2IF);
	TIM1->DIER &= (~TIM_DIER_CC1IE & ~TIM_DIER_CC2IE); // Capture/Compare 1,2 interrupt disable

	TIM9->SR &= ~TIM_SR_UIF;
	TIM9->CNT = 0;						// 1ms interrupt 켜기 전 카운트 초기화
	TIM9->CR1 |= TIM_CR1_CEN;   		// TIM9 counter enable.
	TIM9->DIER |= TIM_DIER_UIE;			// TIM9 1ms interrupt enable

	enc1_cnt = TIM3->CNT; 					// Encoder의 값을 받아옴
	clock1_cnt = TIM5->CNT;

	enc1_diff = enc1_cnt - enc1_cnt_prev;		// 논문에서 m1에 해당
	enc1_cnt_prev = enc1_cnt; // 값 저장

	clock1_diff = clock1_cnt - clock1_cnt_prev;		// 논문에서 m2에 해당
	clock1_cnt_prev = clock1_cnt;

	// 논문을 보면 RPM 기준 속도가 (60*fc*m1)/(P*m2)[RPM]. (2*pi*fc*m1)/(P*m2)[rad/sec]
	// fc는 m2가 생성되는 주파수 즉, 단순 Upcounter의 주파수가 들어가는데, prescale = 1로 가져서 FCK_PSC = 84MHz이다. 즉 42MHz
	// P는 CPR로 생각한다. 2PI/P의 값을 enc2_to_rad라는 이름의 변수로 설정하였다.
	d_theta1_mt = (enc2_to_rad*42000000*(float)enc1_diff)/(float)clock1_diff;

//	if(flag1==0)
//	{
//		GPIOB->BSRR |= GPIO_BSRR_BS0;   // PB0 HIGH;
//		flag1 = 1;
//	}
//	else
//	{
//		GPIOB->BSRR |= GPIO_BSRR_BR0;   // PB0 LOW;
//		flag1 = 0;
//	}
}
//=========================================================
//================2단용 MT 메소드======================
// A상 PB14 : TIM12_CH1
// A상 PB15 : TIM12_CH2
//=========================================================
void Timer12_MT_Interrupt_Init() {
	// 2단은 PB14(A상), PB15(B상)을 Alternate function으로 설정한다. 1001: AF9
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB clock enable
	GPIOB->MODER &= (~GPIO_MODER_MODER14 & ~GPIO_MODER_MODER15);
	GPIOB->MODER |= ((0x2 << GPIO_MODER_MODER14_Pos) | (0x2 << GPIO_MODER_MODER15_Pos)); // 10 (=0x2) : alternate function

	GPIOB->AFR[1] &= (~GPIO_AFRH_AFSEL14 & ~GPIO_AFRH_AFSEL15);
	GPIOB->AFR[1] |= ((0x9 << GPIO_AFRH_AFSEL14_Pos) | (0x9 << GPIO_AFRH_AFSEL15_Pos));  //  AF9 할당, AF9는 1001 (=0x9)

	// TIMER12 설정 input capture. A상 CC1 채널, B상 CC2 채널
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;   // TIM12 clock enable
	TIM12->CCMR1 &= (~TIM_CCMR1_CC1S & ~TIM_CCMR1_CC2S);
	TIM12->CCMR1 |= ((0x1 << TIM_CCMR1_CC1S_Pos) |(0x1 << TIM_CCMR1_CC2S_Pos)); // 01 (=0x1) : CC1 channel is configured as input, IC1 is mapped on TI1,CC2 channel is configured as input, IC2 is mapped on TI2

	// input filter CH1과 CH2으로 받기에, 두 채널에 필터. 4번의 연속된 이벤트가 있어야 안정된 신호로 인식
	TIM12->CCMR1 &= (~TIM_CCMR1_IC1F & ~TIM_CCMR1_IC2F);
	TIM12->CCMR1 |= ((0x2 << TIM_CCMR1_IC1F_Pos) | (0x2 << TIM_CCMR1_IC2F_Pos)); // input filter 설정 0010 (=0x2) : 0010: fSAMPLING=fCK_INT, N=4

	// both edge detection
	TIM12->CCER &= (~TIM_CCER_CC1P & ~TIM_CCER_CC2P);
	TIM12->CCER |= ((0x3 << TIM_CCER_CC1P_Pos) | (0x3 << TIM_CCER_CC2P_Pos));  // CC1P=11(=0x3),CC2P=11(=0x3) non inverted/both edges

	// input capture enable
	TIM12->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E); // 1 : Capture enabled

	TIM12->CR1 |= TIM_CR1_CEN; // TIM1을 enable 시킨다.

	TIM12->DIER &= (~TIM_DIER_CC1IE & ~TIM_DIER_CC2IE); // Capture/Compare 1,2 interrupt disable -> TIM11 Handler에서 enable

	NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
	NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn); // pending data를 clear
	NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,3);
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
}

void TIM8_BRK_TIM12_IRQHandler() { // IRQ Handler의 이름은 startup_stm32f407xx.s 에서 찾아볼 수 있다.

	static int flag1=0;

	TIM12->SR &= (~TIM_SR_CC1IF & ~TIM_SR_CC2IF);
	TIM12->DIER &= (~TIM_DIER_CC1IE & ~TIM_DIER_CC2IE); // Capture/Compare 1,2 interrupt disable

	TIM11->SR &= ~TIM_SR_UIF;
	TIM11->CNT = 0;						// 1ms interrupt 켜기 전 카운트 초기화
	TIM11->CR1 |= TIM_CR1_CEN;   		// TIM11 counter enable.
	TIM11->DIER |= TIM_DIER_UIE;			// TIM11 1ms interrupt enable

	enc2_cnt = TIM4->CNT; 					// Encoder의 값을 받아옴
	clock2_cnt = TIM5->CNT;

	enc2_diff = enc2_cnt - enc2_cnt_prev;		// 논문에서 m1에 해당
	enc2_cnt_prev = enc2_cnt; // 값 저장

	clock2_diff = clock2_cnt - clock2_cnt_prev;		// 논문에서 m2에 해당
	clock2_cnt_prev = clock2_cnt;

	// 논문을 보면 RPM 기준 속도가 (60*fc*m1)/(P*m2)[RPM]. (2*pi*fc*m1)/(P*m2)[rad/sec]
	// fc는 m2가 생성되는 주파수 즉, 단순 Upcounter의 주파수가 들어가는데, prescale = 1로 가져서 FCK_PSC = 84MHz이다. 즉 42MHz
	// P는 CPR로 생각한다. 2PI/P의 값을 enc2_to_rad라는 이름의 변수로 설정하였다.
	d_theta2_mt = (enc3_to_rad*42000000*(float)enc2_diff)/(float)clock2_diff;

	if(flag1==0)
	{
		GPIOB->BSRR |= GPIO_BSRR_BS0;   // PB0 HIGH;
		flag1 = 1;
	}
	else
	{
		GPIOB->BSRR |= GPIO_BSRR_BR0;   // PB0 LOW;
		flag1 = 0;
	}
}

void Timer5_UPCounter_Init() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;   // TIM5 clock enable

    TIM5->PSC = 0x1; // Prescale 설정 PSC=1, fCK_PSC / (PSC[15:0] + 1). 즉 나누기 2. fCK_PSC = 84MHz
    TIM5->ARR = 0xFFFFFFFF;  // Set the auto-reload register to the maximum value
    TIM5->CNT = 0;		// count 초기화

    TIM5->CR1 |= TIM_CR1_CEN;  // Enable TIM7,
}

//================================================================
// 1단부 엔코더값에 M/T Method를 적용하기 위한 1ms timer interrupt 설정
// TIM9 사용
//================================================================
void Timer9_Interrupt_Init() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;   // TIM9 clock enable
	// 여기서부터 TIM9을 설정해보자.
	TIM9->PSC = 1679; 						// Prescale 설정 PSC=1679, The counter clock frequency (CK_CNT) is equal to 168MHz/(1679+1) = 100KHz.
	TIM9->ARR = 99;   						// interrupt의 freq는 1ms 168MHz/1680*(99+1) = 1KHz
	TIM9->CNT = 0;   						// Counter를 0으로 clear
	// Interrupt enable
	TIM9->DIER |= TIM_DIER_UIE; 			// Update interrupt enable.
	TIM9->CR1 |= TIM_CR1_ARPE; 				// Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
	TIM9->CR1 |= TIM_CR1_URS; 				// URS=1: Only counter overflow generates an update interrupt or DMA request if enabled.

	NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn); 	// 위의 code 대신 이렇게 처리할 수도 있다. TIM1_BRK_TIM9_IRQn는 stm32f407xx.h에 정의
	NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn); // pending data를 clear
	NVIC_SetPriority(TIM1_BRK_TIM9_IRQn,4);
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	TIM9->CR1 |= TIM_CR1_CEN;   			// TIM9 enable.
}

//----------------------------------------------------------------
// Timer9 interrupt init의 설정에 의해 enable 된 interrupt handler. 1ms에 한번
//----------------------------------------------------------------
void TIM1_BRK_TIM9_IRQHandler()  // IRQ Handler의 이름은 startup_stm32f407xx.s 에서 찾아볼 수 있다.
{
	static int flag=0;
	TIM9->SR &= ~TIM_SR_UIF;				// TIM9 켜기 전에 인터럽트 status 초기화
	TIM9->CR1 &= ~TIM_CR1_CEN;   			// TIM9 counter disable.
	TIM9->DIER &= ~TIM_DIER_UIE;			// TIM9 1ms interrupt disable
	//=====================================================================================
	TIM1->SR &= (~TIM_SR_CC1IF & ~TIM_SR_CC2IF);	// TIM1 켜기 전에 인터럽트 status 초기화
	TIM1->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE); // Capture/Compare 1,2 interrupt enable
	//=====================================================================================

//	if(flag==0)
//	{
//		GPIOC->BSRR |= GPIO_BSRR_BS0;   // PC0 HIGH;
//		flag = 1;
//	}
//	else
//	{
//		GPIOC->BSRR |= GPIO_BSRR_BR0;   // PC0 LOW;
//		flag = 0;
//	}
}
//================================================================
// 2단부 엔코더값에 M/T Method를 적용하기 위한 1ms timer interrupt 설정
// TIM11 사용
//================================================================
void Timer11_Interrupt_Init() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;   // TIM11 clock enable
	// 여기서부터 TIM9을 설정해보자.
	TIM11->PSC = 1679; 						// Prescale 설정 PSC=1679, The counter clock frequency (CK_CNT) is equal to 168MHz/(1679+1) = 100KHz.
	TIM11->ARR = 99;   						// interrupt의 freq는 1ms 168MHz/1680*(99+1) = 1KHz
	TIM11->CNT = 0;   						// Counter를 0으로 clear
	// Interrupt enable
	TIM11->DIER |= TIM_DIER_UIE; 			// Update interrupt enable.
	TIM11->CR1 |= TIM_CR1_ARPE; 				// Auto-reload preload enable, ARPE=1 : TIMx_ARR register is buffered
	TIM11->CR1 |= TIM_CR1_URS; 				// URS=1: Only counter overflow generates an update interrupt or DMA request if enabled.

	NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn); 	// 위의 code 대신 이렇게 처리할 수도 있다. TIM1_BRK_TIM9_IRQn는 stm32f407xx.h에 정의
	NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn); // pending data를 clear
	NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn,5);
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

	TIM11->CR1 |= TIM_CR1_CEN;   			// TIM11 enable.
}
//----------------------------------------------------------------
// Timer11 interrupt init의 설정에 의해 enable 된 interrupt handler. 1ms에 한번
//----------------------------------------------------------------
void TIM1_TRG_COM_TIM11_IRQHandler()  // IRQ Handler의 이름은 startup_stm32f407xx.s 에서 찾아볼 수 있다.
{
	static int flag=0;
	TIM11->SR &= ~TIM_SR_UIF;				// TIM11 켜기 전에 인터럽트 status 초기화
	TIM11->CR1 &= ~TIM_CR1_CEN;   			// TIM11 counter disable.
	TIM11->DIER &= ~TIM_DIER_UIE;			// TIM11 1ms interrupt disable
	//=====================================================================================
	TIM12->SR &= (~TIM_SR_CC1IF & ~TIM_SR_CC2IF);	// TIM12 켜기 전에 인터럽트 status 초기화
	TIM12->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE); // Capture/Compare 1,2 interrupt enable
	//=====================================================================================

	if(flag==0)
	{
		GPIOC->BSRR |= GPIO_BSRR_BS0;   // PC0 HIGH;
		flag = 1;
	}
	else
	{
		GPIOC->BSRR |= GPIO_BSRR_BR0;   // PC0 LOW;
		flag = 0;
	}
}
// 각 interrupt들이 잘 동작하는지 확인하기 위한 핀들에 대한 초기설정
void PIO_Configure_Init(){
	RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOCEN;    // Port C에 clock 공급
	RCC->AHB1ENR |= (uint32_t) RCC_AHB1ENR_GPIOBEN;   // GPIOB clock enable
	// PC2 - main loop
	GPIOC->MODER &= ~((uint32_t)GPIO_MODER_MODER2);
	GPIOC->MODER |= (0x1U << GPIO_MODER_MODER2_Pos);  // Output의 경우 01 (=0x1)로 설정해야 함
	GPIOC->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDR_OSPEED2);
	GPIOC->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED2_Pos);  // Very high speed일 경우 11 (=0x3)로 설정
	// PC0 - TIM9 interrupt (1ms)
	GPIOC->MODER &= ~((uint32_t)GPIO_MODER_MODER0);
	GPIOC->MODER |= (0x1U << GPIO_MODER_MODER0_Pos);  // Output의 경우 01 (=0x1)로 설정해야 함
	GPIOC->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDR_OSPEED0);
	GPIOC->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED0_Pos);  // Very high speed일 경우 11 (=0x3)로 설정
	// PB0 - TIM1 interrupt (mt method)
	GPIOB->MODER &= ~((uint32_t)GPIO_MODER_MODER0);
	GPIOB->MODER |= (0x1U << GPIO_MODER_MODER0_Pos);  // Output의 경우 01 (=0x1)로 설정해야 함
	GPIOB->OSPEEDR &= ~((uint32_t)GPIO_OSPEEDR_OSPEED0);
	GPIOB->OSPEEDR |= (0x3U <<GPIO_OSPEEDR_OSPEED0_Pos);  // Very high speed일 경우 11 (=0x3)로 설정
}
void LED_SW_Init() {
	RCC->AHB1ENR |= (uint32_t) RCC_AHB1ENR_GPIOEEN; // Port E에 clock 공급
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;   	// GPIOD clock enable

	// PD12~15를 출력으로 설정 (즉 01로 설정)
	GPIOD->MODER &= ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13
					| GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
	// Output의 경우 01 (=0x1)로 설정해야 함
	GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0
				  | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	// GPIO speed를 High speed (즉, 11로 설정) 사실 LED 속도가 빠를 필요는 없으니 주석처리해도 괜찮다고 생각.
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13
					| GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15;

	GPIOD->BSRR |= GPIO_BSRR_BR12 | GPIO_BSRR_BR13 | GPIO_BSRR_BR14
				 | GPIO_BSRR_BR15; // LED OFF
}

// PI 속도제어기 초기설정
void con_PI1_Init() {
	con_PI1.Ref = 0;			// 위치값
	con_PI1.V_Fdb = 0;			// 속도 피드백
	con_PI1.Err = 0;			// 오차값
	con_PI1.samT = sample_time;	// 샘플링 타임
	con_PI1.Kp = KP;         	// P 계수
	con_PI1.Ki = KI;         	// I 계수
	con_PI1.OutMax = MaxV;		// 최댓값
	con_PI1.OutMin = -MaxV;		// 최솟값
	con_PI1.Intg = 0;			// 적분
	con_PI1.Out_tmp = 0;		// 아웃풋 임시
	con_PI1.Out = 0;			// 아웃풋
	con_PI1.SatErr = 0;			// 포화 에러
	con_PI1.Kc = 1 / KP;			// 와인드업 계수
	con_PI1.calc = (void (*)(void*)) con_PI_calc;
}

// PI 속도제어 연산
void con_PI_calc(con_PI *str) {
	str->Err = (str->Ref - str->V_Fdb);					// Err 계산
	str->Out_tmp = str->Err * str->Kp + str->Intg;    	// Out_tmp 계산
	if (str->Out_tmp > str->OutMax)      	// Out_tmp이 최대 전압값보다 크면 맥스값을 Out에 저장
		str->Out = str->OutMax;
	else if (str->Out_tmp < str->OutMin) // Out_tmp이 최소 전압값보다 작으면 미니멈값 Out에 저장
		str->Out = str->OutMin;
	else
		str->Out = str->Out_tmp;              // Out_tmp이 최대 최소 사이라면 그대로 Out에 저장
	str->SatErr = str->Out - str->Out_tmp;        // SatErr에 Out_tmp과 Out 차이를 계산
	str->Intg += str->samT * (str->Ki * str->Err + str->Kc * str->SatErr); // SatErr과 변수들을 이용해서 적분값 계산
}

// -pi ~ pi 영역으로 제한하기 위한 함수
// EPSILON은 실수상의 오차 성분을 고려하는 텀
float modulo(float alpha) {
	alpha = fmod(alpha, 2 * PI);
	if (alpha > PI - EPSILON)
		return alpha - 2 * PI;
	else if (alpha < -PI + EPSILON)
		return alpha + 2 * PI;

	return alpha;
}

void SPI1_Init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN; // GPIOA, GPIOB clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;    // SPI clock enable

	// PA4, PA5, PA6, PA7를 SPI 기능을 수행하도록 설정하자. 여기서 SPI 기능은 AF5에 해당한다.
	GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6
			| GPIO_MODER_MODER7);
	GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1
			| GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

	// AF5 설정 (SPI AF)
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6
			| GPIO_AFRL_AFRL7);
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos)
			| (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);

	// 여기서부터 SPI 관련 설정이 시작된다.
	SPI1->CR1 &= ~SPI_CR1_CPHA;   	// CPHA=1
	SPI1->CR1 &= ~SPI_CR1_CPOL; 	// CPOL=1
	SPI1->CR1 |= SPI_CR1_MSTR;   	// Master Mode로 설정
	SPI1->CR1 &= ~SPI_CR1_BR;    	// Baud rate를 설정해보자.
	SPI1->CR1 |= (0b111 << SPI_CR1_BR_Pos); // Baud rate를 f_PCLK/256로 분주설정. 84MHz/256 = 328.125 KHz.
	SPI1->CR1 &= ~SPI_CR1_DFF; 		// 0: 8-bit data frame format, 1:16-bit data frame format

	SPI1->CR1 |= SPI_CR1_SSM;   	// bit SSM=1
	SPI1->CR1 |= SPI_CR1_SSI;   	// bit SSI=1
//	SPI1->CR2 &= ~SPI_CR2_SSOE; 	// bit SSOE=0, 즉 SS output을 disable 시킨다.
	SPI1->CR2 |= SPI_CR2_SSOE; 		// bit SSOE=1, 즉 SS output을 enable 시킨다.

	SPI1->CR1 |= SPI_CR1_SPE;   	// SPI를 enable 시킨다.
}

uint8_t SPI1_Transfer(uint8_t data) {
    SPI1->DR = data; // 데이터 전송 (이런 쉬운 코드가 있는데, 위의 코드가 더 빠른건가?)
	while (!(SPI1->SR & SPI_SR_RXNE)) ; // 수신 버퍼가 차있는지 확인
	return SPI1->DR; // 수신한 데이터 반환
}

void SD_Init() {
	uint8_t i, j;
	uint8_t CMD_flag, R1, R3[5], R7[5];

	// SD카드의 SPI 모드를 설정하기 위해 낮은 속도로 진행해야 한다.
	// SPI 400KHz 보다 낮게 설정.
	SPI1->CR1 |= SPI_CR1_BR; // Baud rate를 f_PCLK/256로 분주설정. 84MHz/256 = 328.125 KHz.

	// set SPI 80 clocks
	for (i = 0; i < 10; i++)
		SPI1_Transfer(0xFF);

	DWT_ms_Delay(1);

	// 이 flag는 CMD에 값을 보냈을 때 정상적인 반응이 나오는지 확인하는 flag이다.
	CMD_flag = 0;

	// CMD0 명령 전달 (IDLE 상태에 진입하는 코드)
	// 이 때 두 번 전달하는 이유는, 이 코드의 원작자의 의도로 안정성 향상이 목적이다.
	for(j = 0; j < 2 ; j++){
		SD_command(CMD0, 0);
		for (i = 0; i < 5; i++) {
			R1 = SPI1_Transfer(0xFF);
			if (R1 == 0x01)
				CMD_flag = 1;
		}
		DWT_ms_Delay(1);
	}

	// CMD가 제대로 인식되었다면, 지나가고 그렇지 않다면 무한루프에 빠진다.
	// 위 설정이 끝났으면, SD카드를 SPI 모드가 가능해진다.
	// 이후에는 SPI의 BR를 변경해 빠른 속도에서 SPI가 진행 될 수 있도록 세팅한다.
	if (CMD_flag == 1);
	else {
		GPIOD->BSRR |= GPIO_BSRR_BS14;
		while (1) ;
	}

	// SPI BR 설정
	// 000(0): 84MHz/2 = 42MHz
	// 001(1): 84MHz/4 = 21MHz
	// 010(2): 84MHz/8 = 10.5MHz
	// 011(3): 84MHz/16 = 5.25MHz
	// 100(4): 84MHz/32 = 2.625MHz
	// 101(5): 84MHz/64 = 1.3125MHz
	// 110(6): 84MHz/128 = 656.25KHz
	// 111(7): 84MHz/256 = 328.125KHz

	// 실험결과 21MHz까지 통신 가능한 것이 확인됨.
	SPI1->CR1 &= ~SPI_CR1_BR;
	SPI1->CR1 |= (0b001 << SPI_CR1_BR_Pos);

	// 이 flag는 CMD에 값을 보냈을 때 정상적인 반응이 나오는지 확인하는 flag이다.
	CMD_flag = 0;

	// 다음 CMD8 명령 전달(Sends SD Memory카드 interface condition)
	// SD 카드의 정보를 출력하는 명령이다.
	SD_command(CMD8, 0x000001AA);
	for (i = 0; i < 5; i++) {
		R7[0] = SPI1_Transfer(0xFF);
		// R7값이 어떤 값이 나오는지에 따라서 규격이 정해진다.
		// R7값이 5가 나왔을 때.
		if (R7[0] == 0x05) {			// if R1 = 0x05, Ver 1.X
			SD_type = VER1X;
			CMD_flag = 1;
			break;
		}
		// R7값이 1이 나왔을 때.
		else if (R7[0] == 0x01) {		// if R1 = 0x01, Ver 2.X
			// 그 다음 값을 수신 받는다.
			for (j = 1; j <= 4; j++) {
				R7[j] = SPI1_Transfer(0xFF);
			}
			// 수신 받은 성분이 아래의 조건과 같을 때 반복 조건을 빠져나간다.
			if ((R7[3] == 0x01) && (R7[4] == 0xAA)) {
				SD_type = VER2X;
				CMD_flag = 1;
				break;
			}
		}
	}

	// 이후 더미값을 보내 입출력을 초기화 시킨다.
	SPI1_Transfer(0xFF);

	// CMD가 제대로 규격을 인식되었다면, 지나가고 그렇지 않다면 무한루프에 빠진다.
	if (CMD_flag == 1);
	else {
		GPIOD->BSRR |= GPIO_BSRR_BS14;
		while (1) ;
	}

	// 이 flag는 CMD에 값을 보냈을 때 정상적인 반응이 나오는지 확인하는 flag이다.
	CMD_flag = 0;

	// 초기화 과정에서 횟수가 많은 이유는, 보드레이트를 바꾸게 되면 반복횟수가 증가하는데, 1000번 이전에
	// break로 빠져나가기 때문에 큰 코스트 걱정은 안 해도 괜찮다. 저 만큼 해도 안되는거면 SD카드 문제.
	// 윤덕용 저 책에서는 5회만 반복했다. (stm32f051 책에서는 100번 반복했다. 이게 맞나보다.)
	// SPI의 속도가 빠를 수록 인식을 늦게 한다.
	for (i = 0; i < 1000; i++) {			// send ACMD41(start initialization)
		// CMD55를 보내 Application specific command로 보내는 명령임을 전달한다.
		SD_command(CMD55, 0);
		for (j = 0; j < 5; j++) {		// receive R1 = 0x01
			R1 = SPI1_Transfer(0xFF);
		}

		// ACMD41을 보내카드 초기화 프로세스를 시작하며, 0의 값을 반환하여야 한다.
		SD_command(ACMD41, 0x40000000);		// HCS = 1
		for (j = 0; j < 5; j++) {		// receive R1 = 0x00
			R1 = SPI1_Transfer(0xFF);
			if (R1 == 0x00)
				CMD_flag = 1;
		}
		// 1000 회 이상의 많은 반복횟수가 있으므로, 그 전에 정상 응답이 들어왔다면 반복문을 탈출하라는 의미.
		if (CMD_flag)
			break;
	}

	// CMD가 제대로 규격을 인식되었다면, 지나가고 그렇지 않다면 무한루프에 빠진다.
	if (CMD_flag == 1);
	else {
		GPIOD->BSRR |= GPIO_BSRR_BS14;
		while (1) ;
	}

	// SD카드의 종류에 따라 초기화 과정이 조금 다르지만,
	// 우리가 사용할 VER2X 버전의 경우 다음의 프로세스를 거쳐 세부 버전을 확인한다.
	if (SD_type == VER2X) {
		// CMD58을 통해 OCR 레지스터를 읽는다.
		// CCS는 OCR[30]에 할당되는데, 이 비트를 읽어내서 SD카드의 type을 알아낸다.
		SD_command(CMD58, 0);
		for (i = 0; i < 5; i++) {
			R3[0] = SPI1_Transfer(0xFF);
			if (R3[0] == 0x00) {
				for (j = 1; j <= 4; j++) {
					R3[j] = SPI1_Transfer(0xFF);
				}
				break;
			}
		}

		if ((R3[1] & 0xC0) == 0xC0)		// if CCS = 1, High Capacity(SDHC)
			SD_type = VER2X_HC;
		else // if CCS = 0, Standard Capacity
			SD_type = VER2X_SC;

		SPI1_Transfer(0xFF);				// send dummy clocks
	}
	// 이 과정까지 끝났다면, SD카드를 사용할 준비는 끝났다.
}

// SD카드 커맨드를 입력하는 함수
// SPI를 통해 여러 커맨드를 전송하여, SD카드에 원하는 명령을 전달하는 역할을 수행한다.
void SD_command(uint8_t command, uint32_t sector) /* send SD카드 command */
{
	uint8_t SPI_CRC;

	// 데이터는 8bit씩 보낼 수 있기 때문에 5번을 끊어 보낸다.
	SPI1_Transfer(command | 0x40);
	SPI1_Transfer(sector >> 24);
	SPI1_Transfer(sector >> 16);
	SPI1_Transfer(sector >> 8);
	SPI1_Transfer(sector);

	// 각 CMD에 따라 CRC 값이 정해져있는데, SPI 모드의 경우에는 초기 설정때만 CRC 값이 필요하다.
	if (command == CMD0)
		SPI_CRC = 0x95;		// CRC for CMD0
	else if (command == CMD8)
		SPI_CRC = 0x87;		// CRC for CMD8(0x000001AA)
	else
		SPI_CRC = 0xFF;		// no CRC for other CMD

	SPI1_Transfer(SPI_CRC);				// send CRC
}

// SD카드에 특정 sector 갖고 있는 512byte의 정보를 가져오는 함수.
// sector 번호를 입력하면, buffer의 값에 저장된다.
void SD_read_sector(uint32_t sector, uint8_t *buffer) /* read a sector of SD카드 */
{
	// SD카드의 type에 맞춰 sector값을 수정한다.
	if ((SD_type == VER1X) || (SD_type == VER2X_SC)) // if not SDHC, sector = sector*512
		sector <<= 9;

	// CMD17을 이용해 한 block(512byte)의 데이터를 불러오는 명령을 준다.
	SD_command(CMD17, sector);			// send CMD17(read a block)

	// 명령을 보낸 후 response가 오는데, 마지막 두 값이 0x00, 0xFE가 나온다.
	while (SPI1_Transfer(0xFF) != 0x00) ;		// wait for R1 = 0x00
	while (SPI1_Transfer(0xFF) != 0xFE) ;		// wait for Start Block Token = 0xFE

	// response 이후에 데이터가 연속적으로 들어온다.
	for (uint16_t i = 0; i < 512; i++)			// receive 512-byte data
		buffer[i] = SPI1_Transfer(0xFF);

	// 데이터가 다 들어온 후 CRC 2번 보내고 더미값을 마지막에 보내서 연속신호를 수신하는 오류를 방지한다.
	SPI1_Transfer(0xFF);				// receive CRC(2 bytes)
	SPI1_Transfer(0xFF);
	SPI1_Transfer(0xFF);				// send dummy clocks
}


// SD카드를 읽게되면 FAT32형식으로 파일을 저장한다.
// 따라서 FAT32의 형식에 맞춰 파일을 읽어야 한다.
void FAT32_Init(void) /* initialize FAT32 */
{
	struct volumeID *BPB;
	struct partition PartInfo;

	// SD카드의 0번 Sector는 SD카드의 정보를 담고있다.
	// 이 값을 이용해서 SD카드의 저장형식과 다양한 정보를 알 수 있다.
	// 따라서, 0번 Sector를 가져온다.
	SD_read_sector(0, SectorBuffer);		// read partition table

	// 가져온 buffer에서 Master Boot Record로 구분하고, 그중 첫 Partition을 저장한다.
	// 우리가 사용할 SD카드의 경우는 대부분 Partition을 하나만 이용한다.
	PartInfo = *((struct partition*) ((struct MBR*) SectorBuffer)->mbrPartition);

	// Partition에서 first LBA sector의 값
	FirstDataSector = PartInfo.partStartLBA;	// setup global disk constants

	// 가져온 Partition에서 first LBA sector의 값으로 SD카드를 읽어온다.
	SD_read_sector(FirstDataSector, SectorBuffer);// read start LBA sector

	// 이 값에서는 Partition Boot Record를 확인할 수 있다.
	// 이중에서 volume ID(= BIOS parameter block)의 파트를 따로 저장한다.
	BPB = (struct volumeID*) ((struct bootrecord*) SectorBuffer)->brBPB;

	// 이제 이 volume ID를 읽어 Data Sector를 저장한다.(FAT16이라면 1이고, FAT32라면 0)
	if (BPB->bpbFATsecs)
		FirstDataSector += BPB->bpbResSectors + BPB->bpbFATs * BPB->bpbFATsecs;
	else
		FirstDataSector += BPB->bpbResSectors + BPB->bpbFATs * BPB->bpbBigFATsecs;

	// sectors per cluster를 저장한다.
	SectorsPerCluster = BPB->bpbSecPerClust;

	// 첫 FAT sector의 주솟값 값 저장.
	FirstFATSector = BPB->bpbResSectors + PartInfo.partStartLBA;

	// FAT Type이 FAT32인지 확인한다.
	switch (PartInfo.partPartType) {
	// 우리가 사용할 FAT32 Type이 맞는지 확인한다.
	case PART_TYPE_FAT32:			// FAT32
	case PART_TYPE_FAT32LBA:
		FirstDirCluster = BPB->bpbRootClust;
		Fat32Enabled = 1;
		break;
	// 우리가 사용하지 않는 Type의 경우 무한루프에 빠진다.
	case PART_TYPE_FAT16:			// FAT16
	case PART_TYPE_FAT16BIG:
	case PART_TYPE_FAT16LBA:
		FirstDirCluster = CLUST_FIRST;
		Fat32Enabled = 0;
		while (1) ;
		break;
	// 예외 Type의 경우 무한루프에 빠진다.
	default:
		while (1) ;
		break;
	}

	// 우리가 사용할 FAT32 형식이 맞다면, 1, 아니라면 0,
	// 사실 이 코드는 윤교수님의 책을 기반으로 만들어 졌으나, 해당 부분은
	// FAT16을 사용하고 있다면, 이미 이전 코드들에서 무한 루프가 걸릴 것이기 때문에
	// 하나만 써도 괜찮을 것 같아보인다.
	if (Fat32Enabled == 1)
		FirstFAT2Sector = FirstFATSector + BPB->bpbBigFATsecs;
	else
		FirstFAT2Sector = FirstFATSector + BPB->bpbFATsecs;
}

/* ---------------------------------------------------------------------------- */
/*	각 디렉토리 엔트리에서 클러스터 넘버를 추출하고 총 파일수 파악 하는 함수.						*/
/*  함수의 결과값으로 총 파일 수를 반환한다.												*/
/* ---------------------------------------------------------------------------- */
uint8_t fatGetDirEntry(uint32_t cluster) /* get directory entry */
{
	uint8_t index, files = 0;
	uint32_t cluster_number, offset = 0;

	Dir_entry *pDirentry;
	Long_dir_entry *pLDirentry;

	for (offset = 0;; offset++) {
		pDirentry = (Dir_entry*) fatDir(cluster, offset);

		if (pDirentry == 0)
			return files;
		for (index = 0; index < 16; index++) {	// 16 direntries
			if (*pDirentry->dirName == SLOT_EMPTY)
				break;
			else {
				if ((*pDirentry->dirName != SLOT_DELETED)
						&& (pDirentry->dirAttributes == ATTR_LONG_FILENAME)) {
					pLDirentry = (Long_dir_entry*) pDirentry;
					if (((pLDirentry->Longdir_Ord & 0x40) == 0x40)
							|| ((pLDirentry->Longdir_Ord & 0x50) == 0x50)) {
						file_start_sector[files] = sector_cluster;
						file_start_entry[files] = index;
					}
				}

				if ((*pDirentry->dirName != SLOT_DELETED)
						&& (pDirentry->dirAttributes != ATTR_LONG_FILENAME)
						&& (pDirentry->dirAttributes != ATTR_VOLUME)
						&& (pDirentry->dirAttributes != ATTR_DIRECTORY)) {
					if (((pDirentry->dirLowerCase & 0x18) == 0x18)
							|| ((pDirentry->dirLowerCase & 0x18) == 0x10)) {
						file_start_sector[files] = sector_cluster;
						file_start_entry[files] = index;
					}

					cluster_number = (uint32_t) (pDirentry->dirHighClust);
					cluster_number <<= 16;
					cluster_number |= (uint32_t) (pDirentry->dirStartCluster);

					file_size[files] = pDirentry->dirFileSize;

					file_start_cluster[files++] = cluster_number;

					if (files > MAX_FILE) {
						while (1) ;
					}
				}
				pDirentry++;
			}
		}	// end of sector
	}		// end of cluster
	return files;
}

/* ---------------------------------------------------------------------------- */
/*	클러스터에서 디렉토리 엔트리 정보와 오프셋을 가진 섹터를 추출									*/
/* ---------------------------------------------------------------------------- */

uint8_t* fatDir(uint32_t cluster, uint32_t offset) /* get directory entry sector */
{
	uint32_t index;

	for (index = 0; index < offset / SectorsPerCluster; index++)
		cluster = fatNextCluster(cluster);

	if (cluster == 0)
		return 0;

	sector_cluster = fatClustToSect(cluster) + offset % SectorsPerCluster;
	SD_read_sector(sector_cluster, SectorBuffer);

	return SectorBuffer;
}

/* ---------------------------------------------------------------------------- */
/*		FAT 체인으로 다음 클러스터를 찾음												*/
/* ---------------------------------------------------------------------------- */

uint32_t fatNextCluster(uint32_t cluster) /* get next cluster */
{
	uint32_t nextCluster, fatOffset, fatMask, sector;
	uint16_t offset;
	uint8_t FAT_cache[512];

	if (Fat32Enabled == 1) {
		fatOffset = cluster << 2;	// four FAT bytes(32 bits) for every cluster
		fatMask = FAT32_MASK;
	} else {
		fatOffset = cluster << 1;	// two FAT bytes(16 bits) for every cluster
		fatMask = FAT16_MASK;
	}

	sector = FirstFATSector + (fatOffset / BYTES_PER_SECTOR); // calculate the FAT sector

	offset = fatOffset % BYTES_PER_SECTOR; // calculate offset of entry in FAT sector

	SD_read_sector(sector, (uint8_t*) FAT_cache);

	nextCluster = (*((uint32_t*) &((uint8_t*) FAT_cache)[offset])) & fatMask;

	if (nextCluster == (CLUST_EOFE & fatMask))	// check the end of the chain
		nextCluster = 0;

	return nextCluster;
}

/* ---------------------------------------------------------------------------- */
/*							클러스터 번호를 섹터 번호로 변환								*/
/* ---------------------------------------------------------------------------- */

uint32_t fatClustToSect(uint32_t cluster) /* convert cluster to sector */
{
	if (cluster == 0)
		cluster = 2;

	return ((cluster - 2) * SectorsPerCluster) + FirstDataSector;
}

/* ---------------------------------------------------------------------------- */
/*						긴 파일명을 추출(13*15=195자까지 사용)							*/
/* ---------------------------------------------------------------------------- */

uint8_t Get_long_filename(uint8_t file_number) /* check long file name */
{
	uint16_t i, n = 0;
	uint16_t dir_address, dir_number = 0;
	uint32_t dir_sector = 0;

	uint8_t ShortFileName[11];		// short file name buffer

	SD_read_sector(file_start_sector[file_number], SectorBuffer);// read directory entry
	dir_address = file_start_entry[file_number] * 32;

	if (SectorBuffer[dir_address + 11] == 0x0F) {	// *** long file name entry
		dir_number = (SectorBuffer[dir_address] & 0x1F) + 1; // total number of directory entry

		if (dir_number > 16)		// if file name character > 13*15, return 2
			return 2;

		for (; dir_address < 512; dir_address++)
			EntryBuffer[n++] = SectorBuffer[dir_address];

		if ((dir_number * 32) >= n) {			// directory entry use 2 sectors
			dir_sector = file_start_sector[file_number];
			SD_read_sector(++dir_sector, SectorBuffer);

			i = 0;
			for (; n < 512; n++)
				EntryBuffer[n] = SectorBuffer[i++];
		}

		dir_number = (EntryBuffer[0] & 0x1F) * 32;// last entry is short file name entry
		for (n = 0; n < 11; n++)
			ShortFileName[n] = EntryBuffer[dir_number++];

		extension = ShortFileName[8];		// get filename extension character
		extension <<= 8;
		extension += ShortFileName[9];
		extension <<= 8;
		extension += ShortFileName[10];
		return 1;					// if long file name, return 1
	}

	else if (SectorBuffer[dir_address + 11] == 0x20) {// *** short file name entry
		for (; dir_address < 512; dir_address++)
			EntryBuffer[n++] = SectorBuffer[dir_address];

		for (n = 0; n < 11; n++)
			ShortFileName[n] = EntryBuffer[dir_number++];

		extension = ShortFileName[8];		// get filename extension character
		extension <<= 8;
		extension += ShortFileName[9];
		extension <<= 8;
		extension += ShortFileName[10];
		return 0;					// if short file name, return 0
	}

	else
		// if file name error, return 3
		return 3;
}

/* ---------------------------------------------------------------------------- */
/*		짧은 파일명 출력																*/
/* ---------------------------------------------------------------------------- */

void Save_short_filename() /* display short filename */
{
	uint8_t i, ch1;
	uint16_t ch2;
	uint8_t utf8String[4]; // 충분한 공간 할당

	// 소문자로 변환한다.
	for (i = 0; i < 11; i++) {			// convert upper case to lower case
		if ((EntryBuffer[i] >= 'A') && (EntryBuffer[i] <= 'Z'))
			EntryBuffer[i] += 0x20;
	}

	for (i = 0; i < 11; i++) {
		if (i == 8)
			file_name[i] = '.';

		ch1 = EntryBuffer[i];

		// 값이 영어인지 확인.
		if (ch1 < 0x80) {			// English ASCII character
			if (ch1 != 0x20)
				file_name[i] = EntryBuffer[i];
		}
		// 값이 한국어면 다음의 출력 방식을 이용한다. (한글은 배부분 long filename으로 나온다.)
		else					// Korean character
		{
			ch2 = (ch1 << 8) + EntryBuffer[++i];
			UTF16CharToUTF8(ch2, utf8String);
		}
		DWT_us_Delay(100);
	}
}

/* ---------------------------------------------------------------------------- */
/*		긴 파일명 출력(52자까지 표시)													*/
/* ---------------------------------------------------------------------------- */

void Save_long_filename() /* display long filename */
{
	uint16_t i, entrynumber, charnumber, newindex, oldindex, ch2;
	uint8_t FileNameBuffer[512];
	uint8_t ch1;
	uint8_t utf8String[4]; // 충분한 공간 할당

	entrynumber = EntryBuffer[0] & 0x1F;		// long entry number
	charnumber = (EntryBuffer[0] & 0x1F) * 13;	// character number of file name
	oldindex = ((EntryBuffer[0] & 0x1F) - 0x01) * 32;
	newindex = 0;

	for (i = 0; i < entrynumber; i++) {		// arrange file name
		Filename_arrange(newindex, oldindex, FileNameBuffer);
		newindex += 26;
		oldindex -= 32;
	}

	for (i = 0; i < charnumber * 2; i++) {	// display long file name format
		if (i >= 40 * 2) {				// limit length(40 character)
			return;
		}
		ch1 = FileNameBuffer[i];

		if (ch1 == 0xFF) {
			return;
		}
		if (ch1 < 0x80) {
			DWT_us_Delay(100); // 약간의 기다림이 필요한 것 같다.
			file_name[i/2] = FileNameBuffer[i];
		} else {
			ch2 = (ch1 << 8) + FileNameBuffer[++i];
			UTF16CharToUTF8(ch2, utf8String);
		}
		DWT_us_Delay(100);
	}
}

/* ---------------------------------------------------------------------------- */
/*		파일명 문자들을 순서대로 정리하여 재배열											*/
/* ---------------------------------------------------------------------------- */

void Filename_arrange(uint16_t newx, uint16_t oldx, uint8_t *FileNameBuffer) /* arrange file name */
{
	FileNameBuffer[newx + 0] = EntryBuffer[oldx + 2];  // 1
	FileNameBuffer[newx + 1] = EntryBuffer[oldx + 1];
	FileNameBuffer[newx + 2] = EntryBuffer[oldx + 4];  // 2
	FileNameBuffer[newx + 3] = EntryBuffer[oldx + 3];
	FileNameBuffer[newx + 4] = EntryBuffer[oldx + 6];  // 3
	FileNameBuffer[newx + 5] = EntryBuffer[oldx + 5];
	FileNameBuffer[newx + 6] = EntryBuffer[oldx + 8];  // 4
	FileNameBuffer[newx + 7] = EntryBuffer[oldx + 7];
	FileNameBuffer[newx + 8] = EntryBuffer[oldx + 10]; // 5
	FileNameBuffer[newx + 9] = EntryBuffer[oldx + 9];
	FileNameBuffer[newx + 10] = EntryBuffer[oldx + 15]; // 6
	FileNameBuffer[newx + 11] = EntryBuffer[oldx + 14];
	FileNameBuffer[newx + 12] = EntryBuffer[oldx + 17]; // 7
	FileNameBuffer[newx + 13] = EntryBuffer[oldx + 16];
	FileNameBuffer[newx + 14] = EntryBuffer[oldx + 19]; // 8
	FileNameBuffer[newx + 15] = EntryBuffer[oldx + 18];
	FileNameBuffer[newx + 16] = EntryBuffer[oldx + 21]; // 9
	FileNameBuffer[newx + 17] = EntryBuffer[oldx + 20];
	FileNameBuffer[newx + 18] = EntryBuffer[oldx + 23]; // 10
	FileNameBuffer[newx + 19] = EntryBuffer[oldx + 22];
	FileNameBuffer[newx + 20] = EntryBuffer[oldx + 25]; // 11
	FileNameBuffer[newx + 21] = EntryBuffer[oldx + 24];
	FileNameBuffer[newx + 22] = EntryBuffer[oldx + 29]; // 12
	FileNameBuffer[newx + 23] = EntryBuffer[oldx + 28];
	FileNameBuffer[newx + 24] = EntryBuffer[oldx + 31]; // 13
	FileNameBuffer[newx + 25] = EntryBuffer[oldx + 30];
}


/* ---------------------------------------------------------------------------- */
/*		한글 유니코드 UTF16을 UTF8로 바꾸는 코드  										*/
/* ---------------------------------------------------------------------------- */
void UTF16CharToUTF8(uint16_t utf16Char, char *utf8String) {
	if (utf16Char <= 0x7F) {
		// 1바이트로 인코딩
		utf8String[0] = (char) utf16Char;
		utf8String[1] = '\0'; // NULL 종료
	} else if (utf16Char <= 0x7FF) {
		// 2바이트로 인코딩
		utf8String[0] = (char) (0xC0 | ((utf16Char >> 6) & 0x1F));
		utf8String[1] = (char) (0x80 | (utf16Char & 0x3F));
		utf8String[2] = '\0'; // NULL 종료
	} else {
		// 3바이트로 인코딩
		utf8String[0] = (char) (0xE0 | ((utf16Char >> 12) & 0x0F));
		utf8String[1] = (char) (0x80 | ((utf16Char >> 6) & 0x3F));
		utf8String[2] = (char) (0x80 | (utf16Char & 0x3F));
		utf8String[3] = '\0'; // NULL 종료
	}
}

//------------------------------------------------------
// USART3를 Configure 하기 위해 필요한 절차는 다음과 같다.
// 먼저 TX, RX에 해당하는 GPIO pin을 USART 기능으로 설정해준다.
// 여기서는 USART3_TX로는 PC10 (AF7)을, USART3_RX로는 PC11 (AF7)을
// 사용한다. USART기능은 Alternate function중 AF7에 해당한다.
// PC10과 PC11를 각각 Tx와 Rx로 사용해야 한다. 필요한 기능이 USART이지만 GPIO pin을
// 이용해야 하므로 GPIOC에 clock을 enable 시켜야 한다. USART를 사용한다고
// USART clock만 enable 시키면 동작하지 않는다.
// USART3로 공급되는 clock의 source를 정해줄 수 있다.
// STM32F407에서는 USART3의 clock으로 APB1 clock(즉 PCLK1)을 사용하도록 설정한다.
// RCC_Configuration에서 APB1 clock을 42 MHz로 설정해 놓았던 것을 기억하자.
// USART3의 clock을 enable 시킨다.
// Baudrate, data bit length 등과 같은 설정을 해준다.
// 이를 위해 USART3의 CR1, CR2, CR3, BRR 등의 register를 조작한다.
// 이것들을 조작하기 위해서 항상 CR1의 UE(USART Enable)을 0으로 만든다음
// 필요한 설정을 하고 모든 것이 완료되면 다시 UE를 1로 만들면 USART가 동작한다.
// USART1, USART7는 PCLK2를 clock source로 사용
// 나머지 USART는 PCLK1을 clock source로 사용 (Reference manual page 985)
//------------------------------------------------------
void USART3_Init() {
	// PC10는 USART3_TX로, PC11는 USART_RX로 설정하자.
	// USART로 사용하겠지만 GPIOC의 pin을 사용하므로 일단 GPIOC로 가는 clock을 Enable
	// 시켜야 한다. 이거 안했다가 하루 정도 날렸다.

	RCC->AHB1ENR |= (uint32_t) RCC_AHB1ENR_GPIOBEN;   // GPIOB clock enable

	//----------------------------------------------------------------------
	// PC10과 PC11를 alternate function AF7 (USART3_TX와 USART3_RX)로 설정하자.
	// 참고로 PC10과 PC11는 GPIOx_AFRH에서 설정하는데, 이것은 ARF[1]에 해당한다.
	//----------------------------------------------------------------------
	GPIOB->MODER &= ~((uint32_t) GPIO_MODER_MODER10); // PC10을 Alternate function으로 설정.
	GPIOB->MODER |= (0x2U << GPIO_MODER_MODER10_Pos); // Alternate function의 경우 10 (=0x2)로 설정해야 함
	GPIOB->AFR[1] &= ~((uint32_t) GPIO_AFRH_AFRH2);  // AF7 할당, 즉 USART3_TX로 할당.
	GPIOB->AFR[1] |= (0x7U << GPIO_AFRH_AFSEL10_Pos);  // AF7은 0111 (=0x7)

	GPIOB->MODER &= ~((uint32_t) GPIO_MODER_MODER11); // PC11을 Alternate function으로 설정.
	GPIOB->MODER |= (0x2U << GPIO_MODER_MODER11_Pos); // Alternate function의 경우 10 (=0x2)로 설정해야 함
	GPIOB->AFR[1] &= ~((uint32_t) GPIO_AFRH_AFRH3);  // AF7 할당, 즉 USART3_RX로 할당.
	GPIOB->AFR[1] |= (0x7U << GPIO_AFRH_AFSEL11_Pos);  // AF7은 0111 (=0x7)

	// STM32F407에서는 USART3의 clock source로 PCLK1을 사용
	RCC->APB1ENR |= ((uint32_t) RCC_APB1ENR_USART3EN);   // USART3 clock enable

	//------------------------------------------------------------------------------------
	// USART3의 CR1, CR2, CR3, BRR 등을 이용하여 USART3의 parameter를 설정해준다.
	// CR1을 통해서는 start bit, data bits를 설정할 수 있는데 reset value로 1 start bit, 8 data bits로
	// 설정되어 있다. stop bit의 설정은 CR2에서 하도록 되어 있다.
	//------------------------------------------------------------------------------------
	USART3->CR1 = 0x00000000; // USART3->CR1의 UE를 0으로 설정하면 USART3가 disable 됨. 일단 UE=0으로 만든다.
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE; // TE, RE (Transmit Enable, Receive Enable)
	USART3->CR1 &= ~USART_CR1_OVER8; // oversampling by 16. 이 bit는 USART가 disabled 되어 있을 때만 write가 가능하다.
									 // reset value에 의해 1 start bit, 8 data bits 설정이 되어 있다.
	USART3->CR2 = 0x00000000;    // 1 stop bit 로 설정.
	USART3->CR3 = 0x00000000;
	//----------------------------------------------------------
	// USART3->BRR 설정에 주의해야 한다.
	// 만약 115200 bps로 baud rate를 설정하려 한다면
	// 42000000/115200 = 364.5833 이 된다. 이 값에 해당하는 BRR을 만들어주어야
	// 하는데 BRR의 하위 16 bit가 USARTDIV를 구성하는데 이것은 12-bit Mantisa와
	// 4-bit Fractional bit로 구성된다.
	// Baud rate = fck/(8*(2-OVER8)*USARTDIV)
	// 위의 설정에서 OVER8=0으로 설정했으므로 USARTDIV = fck/(8*(2-OVER8)*baudrate)
	// 이므로 USARTDIV = 42000000/(8*(2-0)*115200) = 22.7865 가 된다.
	// Mantisa = 22 (=0x16), Fraction = round(0.7865*16)=12 (=0xC)
	// 가 되므로 USARTDIV = 0x16C로 설정해야 한다.

	//----------------------------------------------------------
	USART3->BRR = 0x16C; //
	USART3->CR1 |= USART_CR1_UE; // UE=1, 즉 USART를 Enable 시킨다.

	sprintf(str, "\n\n\n... USART3 init...\n"); UsbPutString(str);
}

// USART3를 이용하여 char 한개를 전송하는 함수. polling 기반의 전송
void TX3_Char(char data) {
	while ((USART3->SR & USART_SR_TXE) == 0) ; // Transmit data register가 empty 될때까지 기다린다.
	USART3->DR = data; // Transmit data
}

// USART3를 이용하여 string data를 전송하는 함수
void TX3_PutString(char *s) {
	while (*s != '\0') {
		TX3_Char(*s);
		s++;
	}
}

// 시불변 LQ gain 값을 저장하는 함수.
void Get_EPnK_param(float K[DIP_state_cnt * DIP_EP_cnt]) {
	uint8_t SDbuffer[BYTES_PER_SECTOR];
	float *pbuffer;
	uint32_t index = MF[0].mf_data_index;

	// SINGLE 형의 경우 숫자하나는 4Byte에 저장된다.
	// 시불변 LQ gain의 경우 4 * 7 * 4Byte = 112Byte 정도면 충분하기 때문에,
	// 한 개의 버퍼만 있어도 충분하다.
	SD_read_sector(MF[0].mf_sector, SDbuffer); // 데이터 512Byte 읽기

	// 해당 버퍼에서 데이터가 있는 영역으로 이동해서 float형으로 형변환한다.
	pbuffer = ((float*) (SDbuffer + index));

	// 그 값을 32개 저장한다.
	memcpy(K, pbuffer, sizeof(float) * DIP_state_cnt * DIP_EP_cnt);
}

// 이것은 천이 제어에 필요한 feedforward 궤적을 원하는 m값에 맞춰 저장값에 넣어주는 함수로.
// m 값에 맞춰 t, x, u, K 값을 얻어온다.
void Get_Transition_param(float t[DIP_point_cnt],
		float u[DIP_point_cnt],
		float x[DIP_point_cnt][DIP_state_cnt],
		float K[DIP_point_cnt][DIP_state_cnt],
		uint8_t Transition_num) {
	uint8_t SDbuffer[BYTES_PER_SECTOR];
	uint32_t addsize = (sizeof(float) * DIP_point_cnt * DIP_state_cnt) / BYTES_PER_SECTOR + 2;
	uint8_t Allbuffer[BYTES_PER_SECTOR * addsize];
	uint32_t index;
	float *pbuffer;

	// 여기서의 핵심전략은 다음과 같다.
	// 각각의 변수 순서는 다음과 같으니.
	// MF[1]. K_mat			-> feedforward의 궤적을 보상하는 시변 LQ gain값으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 상태변수와 천이가능한 궤적수의 곱)
	// MF[2]. control_mat	-> feedforward의 입력 궤적으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 천이가능한 궤적수의 수)
	// MF[3]. state_mat		-> feedforward의 상태 궤적으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 상태변수와 천이가능한 궤적수의 곱)
	// MF[4]. time_mat		-> feedforward의 궤적 시간으로 nxm의 값을 갖는다. (여기서 n은 저장된 node 개수, m은 천이가능한 궤적수의 수)
	// 여기서, MF[1]과 MF[3]은 nx7 행렬이고, MF[2], MF[4]는 nx1 행렬이므로 비슷한 두 개끼리 얻어오는 방식이 비슷하다.

	// MF[1] 데이터부터 처리한다.
	// 데이터가 시작되는 지점을 기준으로,
	// m값에 맞춰 필요한 데이터가 있는 위치의 위치값을 찾는다.
	index = MF[1].mf_data_index;
	index += sizeof(float) * DIP_point_cnt * DIP_state_cnt * (Transition_num - 1);

	// addsize만큼 반복하라는 의미는 이 변수가 갖고 있을 수 최대 크기기를 계산하여 512Byte로 변환했을 때,
	// SD카드를 몇 번 읽어야하는지를 나타내는 값이다.
	// SIGNLE형 4byte기준
	// (4byte x 161 x 7 / 512byte) + 2
	// 2 번을 더한 것은 기본적으로 setup에서 sector를 읽어올 때부터 2개를 합친 값을 이용하기 때문에, 적어도 2번은 읽어와야 한다.
	for (int i = 0; i < addsize; i++) {
		// 여기서 읽어오는 부분의 시작은 데이터가 있는 sector의 영역부터 읽어오며,
		// 나누기 연산을 하면 몫만 반환된다는 점을 이용해서 sector가 읽을 시작 위치를 지정한다.
		SD_read_sector(MF[1].mf_sector + (index / BYTES_PER_SECTOR) + i, SDbuffer); // 데이터 512Byte 읽기
		memcpy(Allbuffer + BYTES_PER_SECTOR * i, SDbuffer, sizeof(SDbuffer)); 		// 저장한다.
	}

	// 값을 float형태로 변환헀을 때, 초기 위치를 지정한다. 이 때는 나머지 연산을 통해서 위치를 저장한다.
	pbuffer = ((float*) (Allbuffer + index % BYTES_PER_SECTOR));

	// 이제 순서대로 값을 가져온다.
	// mat 파일의 경우 행렬은 열벡터 형태로 저장되고, 변수 저장을 161개의 열로 했기 때문에 161개의 값이 모두 한 열에 있는 값이다.
	// 따라서 161개의 값을 읽어오고, 7개의 상태 변수를 따로 저장해야한다.
	for (int i = 0; i < DIP_state_cnt; i++) {
		for (int j = 0; j < DIP_point_cnt; j++)
			K[j][i] = pbuffer[DIP_point_cnt * i + j];
	}

	// 위와 방식이 같다.
	index = MF[3].mf_data_index;
	index += sizeof(float) * DIP_point_cnt * DIP_state_cnt * (Transition_num - 1);

	for (int i = 0; i < addsize; i++) {
		SD_read_sector(MF[3].mf_sector + (index / BYTES_PER_SECTOR) + i, SDbuffer); // 데이터 512Byte 읽기
		memcpy(Allbuffer + BYTES_PER_SECTOR * i, SDbuffer, sizeof(SDbuffer));
	}

	pbuffer = ((float*) (Allbuffer + index % BYTES_PER_SECTOR));

	for (int i = 0; i < DIP_state_cnt; i++) {
		for (int j = 0; j < DIP_point_cnt; j++)
			x[j][i] = pbuffer[DIP_point_cnt * i + j];

	}

	// 여기서는 MF[2], MF[4]처럼 한개의 열만 필요한 경우이다.
	// 이제 큰 크기에 sector는 필요 없으므로 addsize를 작게 바꾼다.
	// 이 값이 작으면 SD카드를 읽어올 때 적은 반복수만 필요로하므로 SD카드의 접근이 적다.
	addsize = (sizeof(float) * DIP_point_cnt) / BYTES_PER_SECTOR + 2;

	// 이 이후과정은 위와 비슷하지만 상태변수값만큼 곱하는 양만 뺀다.
	index = MF[2].mf_data_index;
	index += sizeof(float) * DIP_point_cnt * (Transition_num - 1);

	for (int i = 0; i < addsize; i++) {
		SD_read_sector(MF[2].mf_sector + (index / BYTES_PER_SECTOR) + i, SDbuffer); // 데이터 512Byte 읽기
		memcpy(Allbuffer + BYTES_PER_SECTOR * i, SDbuffer, sizeof(SDbuffer));
	}

	pbuffer = ((float*) (Allbuffer + index % BYTES_PER_SECTOR));
	memcpy(u, pbuffer, sizeof(float) * DIP_point_cnt);

	//이 값도 위와 같다.
	index = MF[4].mf_data_index;
	index += sizeof(float) * DIP_point_cnt * (Transition_num - 1);

	for (int i = 0; i < addsize; i++) {
		SD_read_sector(MF[4].mf_sector + (index / BYTES_PER_SECTOR) + i, SDbuffer); // 데이터 512Byte 읽기
		memcpy(Allbuffer + BYTES_PER_SECTOR * i, SDbuffer, sizeof(SDbuffer));
	}

	pbuffer = ((float*) (Allbuffer + index % BYTES_PER_SECTOR));
	memcpy(t, pbuffer, sizeof(float) * DIP_point_cnt);
}

void Error_Handler(void) {
	while (1) ;
}
