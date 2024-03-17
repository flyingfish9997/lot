#ifndef  _BSP_UART_H
#define  _BSP_UART_H 

//#include "stm32f10x.h"
//#include "circle_buffer.h"
//#include "bsp_dma.h"
//#include <string.h>
//#include "./driver/platform/nvic/nvic.h"
//#include "./driver/modules/wifi/wifi.h"

#include "includes.h"
#define 	BUFFER_SIZE			1024




//串口模式
//	串口空闲+DMA
typedef enum 
{
	Uart1Object=0,//232
	Uart2Object,//ble
	Uart3Object,//WIFI
	UartObjectMax,
}eUartObj_t;



typedef enum _RxState
{
		eReciveIdle = 0,
		eWaitRx,//等待接收
		eReciveIng,
		eReciveFinish,//接收完成等待处理
		eReciveDone,//test
}RxState;

typedef enum _TxState
{
	  eWaitTx,//等待发送
	  eTxFinish,//接收完成等待处理
}TxState;


typedef struct utcb
{	
	USART_TypeDef 							*USART_x;
	uint32_t 								BaudRate;
	uint32_t                                DMA_x;
	DMA_Config 						  		DMA_P2MConfigStr;
	DMA_Config 								DMA_M2PConfigStr;
	DMA_Channel_TypeDef 					*DMA_TX_CHx;	
	DMA_Channel_TypeDef 					*DMA_RX_CHx;
	uint32_t 								DMA_PeripheralDataSize;
	uint32_t 								DMA_MemoryDataSize;
	RxState   	          					ReciveSta;
	uint32_t               					RxNewByte;
	TxState									SendSta;
	uint32_t								rx_len;
	uint32_t								tx_len;
	uint8_t                 				*rx_Buff;	
	uint8_t 								*tx_Buff;
	Struct_CircleBuffer     				circlebuffer;
	RxState									ReciveStatus;//test
	
}UTCB;


void USARTx_Init(void);
uint16_t DataInput(eUartObj_t eUart,uint8_t *Buff ,uint16_t *Len);
//void CachePoor_Init(uint8_t dev,uint8_t *tx_Buff,uint8_t *rx_buff,uint32_t tx_size,uint32_t rx_size);
void USART_DMASend(uint8_t dev, void *Buff,unsigned char Len);
//void USARTx_Config(void);
//usart_obj *Get_USART_Table(void);



//int MX_UART_Init(void);
//unsigned char USART_Send( eUartObj_t eUart, const void * Buff , unsigned char Len );
//uint16_t DataInputBlock(eUartObj_t eUart,uint8_t *Buff ,uint16_t *Len);
//uint16_t DataInput(eUartObj_t eUart,uint8_t *Buff ,uint16_t *Len);

//void usr_printf(char *str);
//uint8_t *Get_Buffer_Addr(void);

// void UART_CallBack(void *Pdata);
#endif
