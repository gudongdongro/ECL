//-------------------------------------------------------------------------
// File name : monitor.c
// <Abstract>
//
// STM32F407의 Full speed USB를 이용한 monitoring을 수행하기 위해 필요한 함수 library
//
// Programmed by Prof. Lee on 7 April 2023
//-------------------------------------------------------------------------
#include "monitor.h"
#include "usbd_cdc_if.h"

ToHost toSimulink; //
extern uint8_t UserTxBufferFS[]; // usbd_cdc_if.c에 선언되어 있다.
extern USBD_HandleTypeDef hUsbDeviceFS; // usb_device.c에 선언

//-----------------------------------------------------------
// PC로 전송할 data를 ToHost structure에 연결해준다.
// data pointer 연결하는 것과 data의 수, channel number 등등을
// 갱신해 준다.
//------------------------------------------------------------
void connectData(ToHost *toHostVar, char type, char channel, void *ptr)
{
	toHostVar->NumOfToHostBlock++;  // ToHostBlock의 수를 하나씩 증가 
	if(type==SS_DOUBLE) // 만약 double형 data 이면
	{
		toHostVar->NumOfDoubleData++;  // Double형 data의 수를 1씩 증가
	}
	toHostVar->TypeAndChannel[channel] = (type<<4)|(channel &0x0F);
	toHostVar->DataPtr[channel] = ptr;  // pointer 연결 
	//toHostVar->ModNumber = 510/(10*(toHostVar->NumOfToHostBlock-toHostVar->NumOfDoubleData) + 20*toHostVar->NumOfDoubleData + 2);
}


void putcToBuffer(ToHost *toHostVar, char c)
{
	toHostVar->data[toHostVar->indx++] = c;
	return;
}


void outhex8_Buffer(ToHost *toHostVar, char n)
{
	char c;
	c = (n >> 4) & 0x0000000f;
	if (c > 9)  toHostVar->data[toHostVar->indx++] = c - 10 + 'a';
	else toHostVar->data[toHostVar->indx++] = c+'0';

	c = n & 0x0000000f;
	if (c > 9) toHostVar->data[toHostVar->indx++] = c - 10 + 'a';
	else toHostVar->data[toHostVar->indx++] = c+'0';

	return;
}

void outhex32_Buffer(ToHost *toHostVar, unsigned long n)
{
	outhex8_Buffer(toHostVar, (char) (n >> 24) );
	outhex8_Buffer(toHostVar, (char) (n >> 16) );	
	outhex8_Buffer(toHostVar, (char) (n >> 8) );	
	outhex8_Buffer(toHostVar, (char) n);
}

//------------------------------------------------------------------
// EZUSB를 사용했을 때에는 polling을 사용하지 않고 그냥 data를 EZUSB에다가
// write 할 수 있었는데 STM32F407의 Full speed USB를 Virtual COM으로 사용할
// 경우에는 무작정 write 할 수는 없다. 만약 쌍이 되어서 돌아가는 PC 측
// application이 돌아가지 않을 경우에는 lock 걸리므로 USB buffer에 바로
// 쓰지 말고 별도의 software buffer에다가 전송할 data를 담아만 놓자. 그런다음
// 전송해야 할 시점이 되면 USB의 tx_buffer가 확보되어 있는지를 확인하고
// 만약 확보가 되었다면 tx_buffer에다가 write 하면 되고 만약 tx_buffer가
// 확보가 되지 않았다면 그냥 아무일도 안하고 넘어가면 된다. 그리고 
// tx_buffer가 확보될때까지 software buffer도 더이상 채워넣지 안는다.
// 아래의 함수는 PC쪽으로 전송해야 될 data가 만약 갱신되었을 경우 
// 전송해야 될 형태의 data를 만들어 놓은 다음 software buffer에다가
// 채워넣고 전송해야 될 시점이 되었을 때 USB의 tx_buffer에다가 
// write 하는 역할을 하는 함수이다.
// Data는 hex encoding을 사용하고 특정 샘플타임때의 data와 다음
// 샘플타임때의 data를 구분하기 위한 구분자로써 'T'와 'Q'를 사용한다.
// 그리고 전송해야 될 data의 마지막에는 'Z'를 사용한다. 따라서 
// 보내야 될 data가 2개, Modulus number가 2인 경우 data의 형태는 
// 다음과 같다.
// T_XXXXXXXXXX_XXXXXXXXXX_Q_T_XXXXXXXXXX_XXXXXXXXXX_Q_Z
// 따라서 전송 data의 수를 n, modulus number를 mod 라고 했을 때
// 한번에 전송하는 data에 포함된 byte의 수는 다음과 같이 계산된다.
// 
// # of bytes = (n*10+2)*mod+1
//
// 여기서 10은 하나의 data가 hex encoding 되고 난후의 byte 수
// 2는 'T'와 'Q'에 대한 byte 수
// 마지막의 1은 'Z'에 대한 byte 수
//
// 따라서 Full speed USB의 경우 Buffer size를 512로 잡았으므로
//
// 위의 계산식에 의해 나온 총 byte 수가 512보다 1적은 511보다 작은
// 값이 나오게끔 mod를 계산해야 한다. 따라서 이것을 공식으로 만들어
// 보면 
// 
//  mod = 510/(n*10+2) <== 정수연산이므로 값은 정수가 나온다.
//
// 예를 들어 n=2일 경우 
// 
// mod = 510/(2*10+2) = 23.18 => mod = 23
//------------------------------------------------------------------
void ProcessToHostBlock()
{
	unsigned int i, n;
	signed long temp_int32;
	char type;
	static int flag = 0;
	
	n = toSimulink.NumOfToHostBlock;

	if(toSimulink.UpdateDoneFlag)
	{
		if(toSimulink.SendHoldFlag==0)
		{
			putcToBuffer(&toSimulink, 'T');  // 
			for(i=0;i<n;i++)
			{
				//---------------------------------------------------------
				// For-loop를 돌면서 해당 block이 입력이 갱신이 되어
				// 있는 상태라면 Host쪽으로 전송한다.
				// 갱신이 되었는지의 여부는 UpdateDoneFlag[]를 검사한다.
				//---------------------------------------------------------
				//TypeAndChannel[i]의 구조
				// 하위 4-bit는 channel number, 상위 4-bit는 data type.
				// data type에 대한 정보는 아래에 주어져 있다.
				//---------------------------------------------------------

				if(GetBit(toSimulink.UpdateDoneFlag,i))  // 갱신이 된 경우
				{
					outhex8_Buffer(&toSimulink, toSimulink.TypeAndChannel[i]); // channel number 전송

					type = (toSimulink.TypeAndChannel[i] >> 4) & 0x0F;  // data type을 알아낸다.

					//--------------------------------------------
					// data type
					//--------------------------------------------
					// SS_DOUBLE  =  0,    64-bit double 형 data 
					// SS_SINGLE  =  1,    /* real32_T  */
					// SS_INT8    =  2,    /* int8_T    */
					// SS_UINT8   =  3,    /* uint8_T   */
					// SS_INT16   =  4,    /* int16_T   */
					// SS_UINT16  =  5,    /* uint16_T  */
					// SS_INT32   =  6,    /* int32_T   */
					// SS_UINT32  =  7,    /* uint32_T  */
					// SS_BOOLEAN =  8     /* boolean_T */
					//--------------------------------------------
					switch(type)
					{
						case 0:  // double
							outhex32_Buffer(&toSimulink, *(unsigned long*)(toSimulink.DataPtr[i])); // double 형 data의 하위 4-byte 먼저 전송
							
							//---------------------------------------------------------------------------
							// 하위 4 byte를 보내고 나서 상위 4 byte를 보내기 위해 다시 type과 channel 
							// 정보를 조립한다. 여기서 type은 double형 data를 나타내는 0이 아니라 9를 
							// 사용하기로 한다. 여기서 9가 의미하는 것은 double형 data의 두번째 data라는 
							// 것을 의미한다. 수신측에서는 이 정보로부터 data를 이어붙여 써야 되는 것을 
							// 알 수 있거나 혹여라도 type 이 0 이였는데 그 다음에 9가 나오지 않으면 
							// 오류 처리할 수 있도록 한다.
							//-------------------------------------------------------------------------
							type = toSimulink.TypeAndChannel[i];
							type = type & 0x0F;  // 상위 4 bit를 clear 시키고 하위 4 bit(channel 정보)는 그대로 쓴다. 
							type = type | 0x90;  // 상위 4 bit를 9로 대치시킨다. double 형 data는 10 byte 2번에 나누어 보내야
							                     // 하므로 상위 4 byte가 9라는 것은 double 형 data의 2번째 data를 의미
							outhex8_Buffer(&toSimulink, type);  // 2번째 data를 의미
							outhex32_Buffer(&toSimulink, *((unsigned long*)(toSimulink.DataPtr[i])+1)); // double 형 data의 상위 4-byte 전송
							break;
						case 1:  // signle data
						case 6:  // int 32
						case 7:  // uinit 32
							outhex32_Buffer(&toSimulink, *(unsigned long*)(toSimulink.DataPtr[i])); // channel data 전송
							break;
						case 2:  // int 8
						case 4:  // int 16
							temp_int32 = (signed long)(*(signed int *)(toSimulink.DataPtr[i]));
							outhex32_Buffer(&toSimulink,*(unsigned long*)&temp_int32);
							break;
						case 3:  // uint 8
						case 5:  // uint 16
						case 8:  // bool
							temp_int32 = (signed long)(*(unsigned int *)(toSimulink.DataPtr[i]));
							outhex32_Buffer(&toSimulink, *(unsigned long*)&temp_int32);
							break;
						default:
							break;
					}

					//--------------------------------------------
     				// 전송하고 나서 Flag resetting. 이렇게 해야지
					// 다음에 또 갱신하고 나서 Flag=1로 설정해서
					// 갱신여부를 판단해 또 전송하게 되는 것이다.
					//--------------------------------------------
					ClearBit(toSimulink.UpdateDoneFlag,i);
				}
			}

			putcToBuffer(&toSimulink,'Q');  // Frame End
		
			toSimulink.ModCntr++;
			toSimulink.ModCntr%=toSimulink.ModNumber;
			if(toSimulink.ModCntr==0)
			{
				putcToBuffer(&toSimulink,'Z');
			}
		}
		
		//--------------------------------------------------------------------------
		// ModCntr=0 이면 data를 전송해야 할 시점을 의미한다. 만약 이 시점에
		// tx_buffer가 확보가 안되어있다면 data를 보내지 않고 SendHoldFlag을 true로
		// 설정해서 buffer가 확보될때까지는 더이상 toSimulink에 채워넣는
		// 일을 하지 않도록 한다. PC측에 Simulink가 구동되지 않으면 PC 측에서
		// data를 읽어가지 않으므로 여기서 설명하는 현상이 발생한다. 
		// PC측 Simulink가 구동되지 않더라도 이로 인해 u-controller 쪽의
		// application이 Lock 되는 경우가 발생하면 안되므로 이를 해결하기 위해
		// 요기 아래 부분이 추가된다. 일단 SendHoldFlag=true가 되면 더이상 
		// ModCntr을 갱신하지 않으므로 계속해서 0으로 남아 있게 된다. 따라서
		// 다음 iteration 에도 여기로 들어오게 되고 만약 tx_buffer가 확보되면
		// data를 보내면 된다. 그렇게 되면 자연스럽게 SendHoldFlag 이 다시 false
		// 가 되므로 정상적인 전송동작을 계속할 수 있다. 
		// SendHoldFlag 이 true라는 것은 Sending 동작을 Hold 하고 있는 상태를
		// 의미한다
		//--------------------------------------------------------------------------
		if(toSimulink.ModCntr==0)  // <-- 전송해야 할 시점을 의미 
		{

			USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
			if (hcdc->TxState != 0){
				toSimulink.SendHoldFlag = 1; // 준비가 안된 SendHoldFlag을 1로 설정
			}
			else
			{

				memcpy(UserTxBufferFS, toSimulink.data, toSimulink.indx);
				CDC_Transmit_FS(UserTxBufferFS, toSimulink.indx);
				toSimulink.indx = 0;  // 전송한 셈이므로 toSimulink에는 data가 0개인 것으로 해 놓는다.
				toSimulink.SendHoldFlag = 0; // 역시 전송한 셈이므로 SendHoldFlag은 0로 설정
			}
		}
	}
	return;
}
