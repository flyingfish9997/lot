#include "includes.h"

uint8_t U1_TXBuff[BUFFER_SIZE];
uint8_t U1_RXBuff[BUFFER_SIZE];

uint8_t U2_TXBuff[BUFFER_SIZE];
uint8_t U2_RXBuff[BUFFER_SIZE];

uint8_t U3_TXBuff[BUFFER_SIZE];
uint8_t U3_RXBuff[BUFFER_SIZE];


UTCB UART_CB[UartObjectMax]=
{
	{USART1, 115200, DMA_1, MYDMA_P2M_Config, MYDMA_M2P_Config, DMA1_Channel4, DMA1_Channel5, DMA_PeripheralDataSize_Byte, 
	 DMA_MemoryDataSize_Byte , .rx_Buff=NULL, .tx_Buff=NULL},
	
	{USART2, 115200, DMA_1, MYDMA_P2M_Config, MYDMA_M2P_Config, DMA1_Channel7, DMA1_Channel6, DMA_PeripheralDataSize_Byte, 
 	 DMA_MemoryDataSize_Byte , .rx_Buff=NULL,.tx_Buff=NULL},		
		
	{USART3, 115200, DMA_1, MYDMA_P2M_Config, MYDMA_M2P_Config, DMA1_Channel2, DMA1_Channel3, DMA_PeripheralDataSize_Byte, 
	 DMA_MemoryDataSize_Byte , .rx_Buff=NULL, .tx_Buff=NULL }
};

/*************************************************************************** 
  * @ժ Ҫ �� ���ڳ�ʼ������
  * @�� �� �� usart_port_table�����ڱ�ָ��
  * @����ֵ�� ��
****************************************************************************/
void USARTx_Config(uint8_t dev)
{
	USART_InitTypeDef USART_InitStructure;
	
	if(UART_CB[dev].USART_x==USART1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //��������ʱ�ӣ�����2���ܴ�
		NVIC_Config(2,2,USART1_IRQn);
	}
	else if(UART_CB[dev].USART_x==USART2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		NVIC_Config(2,3,USART2_IRQn);
	}
	else if(UART_CB[dev].USART_x==USART3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		NVIC_Config(2,4,USART3_IRQn);
	}
	else
	{
		return;
	}
	USART_DeInit(UART_CB[dev].USART_x);
	USART_InitStructure.USART_BaudRate 										= UART_CB[dev].BaudRate;
	USART_InitStructure.USART_WordLength 									= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 										= USART_StopBits_1;
	USART_InitStructure.USART_Parity 											= USART_Parity_No;
	USART_InitStructure.USART_Mode 												= USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl 				= USART_HardwareFlowControl_None;
	USART_Init(UART_CB[dev].USART_x, &USART_InitStructure);	
	
	//�����ж�+DMA����
	USART_ITConfig(UART_CB[dev].USART_x, USART_IT_IDLE, ENABLE);
	USART_ITConfig(UART_CB[dev].USART_x, USART_IT_TC, ENABLE);
	USART_Cmd(UART_CB[dev].USART_x, ENABLE);
	
	USART_ClearITPendingBit(UART_CB[dev].USART_x, USART_IT_TC);
	USART_ClearITPendingBit(UART_CB[dev].USART_x, USART_IT_IDLE);
	USART_ClearITPendingBit(UART_CB[dev].USART_x, USART_IT_RXNE);
	USART_ClearITPendingBit(UART_CB[dev].USART_x, USART_IT_TXE);
	USART_ClearITPendingBit(UART_CB[dev].USART_x, USART_IT_TC);
	
	UART_CB[dev].DMA_P2MConfigStr(0,UART_CB[dev].DMA_RX_CHx, DMA_PeripheralDataSize_Byte , DMA_MemoryDataSize_Byte,(uint32_t)&UART_CB[dev].USART_x->DR,(uint32_t)UART_CB[dev].circlebuffer.Buffer,BUFFER_SIZE);
	UART_CB[dev].DMA_M2PConfigStr(0,UART_CB[dev].DMA_TX_CHx, DMA_PeripheralDataSize_Byte ,DMA_MemoryDataSize_Byte,(uint32_t)&UART_CB[dev].USART_x->DR,(uint32_t)UART_CB[dev].tx_Buff,0 );
	
	USART_DMACmd(UART_CB[dev].USART_x,USART_DMAReq_Rx,ENABLE);
	DMA_Cmd(UART_CB[dev].DMA_RX_CHx,ENABLE);
	USART_DMACmd(UART_CB[dev].USART_x,USART_DMAReq_Tx,ENABLE);
	DMA_Cmd(UART_CB[dev].DMA_TX_CHx,DISABLE);
}

void CachePoor_Init(uint8_t dev,uint8_t *tx_buff,uint8_t *rx_buff,uint32_t tx_size,uint32_t rx_size)
{
	if(tx_buff==NULL||rx_buff==NULL)
	{
		return;
	}
	UART_CB[dev].ReciveSta	=0;
	UART_CB[dev].SendSta	=0;	
	UART_CB[dev].RxNewByte	=0;
	UART_CB[dev].rx_len		=0;
	UART_CB[dev].tx_len		=0;
	UART_CB[dev].rx_Buff	=rx_buff;
	UART_CB[dev].tx_Buff	=tx_buff;
	
	memset(UART_CB[dev].rx_Buff,0,rx_size);
	memset(UART_CB[dev].tx_Buff,0,tx_size);
	CircleBuffer_Init(&UART_CB[dev].circlebuffer,UART_CB[dev].rx_Buff,rx_size);
	UART_CB[dev].ReciveStatus=0;
}

void USART_DMASend(uint8_t dev, void *Buff,unsigned char Len)
{
	UART_CB[dev].SendSta=eWaitTx;
	memset(UART_CB[dev].tx_Buff,0,Len);
	memcpy(UART_CB[dev].tx_Buff,Buff,Len);
	MYDMA_Enable(UART_CB[dev].DMA_TX_CHx,Len);
}

#define	DMA_PRINTF			0
#define	NORMAL_PRINTF		1
#define	USR_PRINTF			DMA_PRINTF

#if (USR_PRINTF==DMA_PRINTF)
int fputc(int ch, FILE *f)
{
	DMA_Cmd(DMA1_Channel4, DISABLE ); 
	DMA1_Channel4->CMAR=(uint32_t)&ch;
	MYDMA_Enable(UART_CB[0].DMA_TX_CHx,1);
	while(DMA1_Channel4->CNDTR!=0){};
	return ch;
}
#else
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	
	return ch;
}
#endif

/*************************************************************************** 
  * @ժ Ҫ �� �Ӵ��ڻ������ȡ����
  * @�� �� �� eUart�����ںš�ö�ٱ�����
  * @�� �� �� Buff�����ݴ��ָ��
  * @����ֵ�� ��
****************************************************************************/
static uint32_t uart_get_byte(eUartObj_t eUart,uint8_t *Buff)
{
	if(Buff == NULL) return ERROR;
	if(SUCCESS == CircleBuffer_Read(&UART_CB[eUart].circlebuffer,Buff))
	{
		return SUCCESS;
	}
	else
	{
	 return ERROR;
	}
}

/*************************************************************************** 
  * @ժ Ҫ �� �����ڽ��������ݺ󣬴��ڴ���л�ȡ���ݡ���ʽ2��
  * @�� �� �� �ޣ�
  * @����ֵ�� ��
****************************************************************************/
uint16_t DataInputBlock(eUartObj_t eUart,uint8_t *Buff ,uint16_t *Len)
{
	int i = 0;
	*Len = 0;
	if(Buff == NULL || Len == NULL)                                //����ж�
	{
		return ERROR;
	}
	if(UART_CB[eUart].ReciveStatus == eReciveIng)                  //�ȴ�����״̬��Ϊ����̬
	{ 
		OSTimeDlyHMSM(0,0,0,20);                                               //������ʱ
		*Len = CircleBuffer_size(&UART_CB[eUart].circlebuffer);   //��ȡ����������ݵĸ���
		for( i = 0; i < *Len ;i++)
		{
			uart_get_byte(eUart,Buff+i);                               //��ȡ������е�����
		}
		UART_CB[eUart].ReciveStatus = eReciveIdle;                    //״̬����Ϊ����̬
		return 1;
	}
	else
	{
		return ERROR;
	}
}


/*************************************************************************** 
  * @ժ Ҫ �� �����ڽ��������ݺ󣬴��ڴ���л�ȡ���ݡ���ʽ1��
  * @�� �� �� �ޣ�
  * @����ֵ�� ��
****************************************************************************/
uint16_t DataInput(eUartObj_t eUart,uint8_t *Buff ,uint16_t *Len)
{
	int i = 0;
	*Len = 0;
	if(Buff == NULL || Len == NULL)                              //����ж�
	{
		return ERROR;
	}
	if(UART_CB[eUart].ReciveStatus == eReciveFinish)             //�ȴ�����״̬��Ϊ�������̬
	{
		*Len = CircleBuffer_size(&UART_CB[eUart].circlebuffer); //��ȡ����������ݵĸ���
		for( i = 0; i < *Len ;i++)
		{
			uart_get_byte(eUart,Buff+i);                             //��ȡ������е�����
		}
		UART_CB[eUart].ReciveStatus = eReciveIdle;   							 //״̬����Ϊ����̬
		return 1;
	}
	else
	{
		return ERROR;
	}
}

/*************************************************************************** 
  * @ժ Ҫ �� ͨ�����ڷ�����
  * @�� �� �� eUart�����ںš�ö�ٱ�����
  * @�� �� �� Buff�����������ݵĻ������׵�ַ
  * @�� �� �� Len�����������ݵĳ���
  * @����ֵ�� ��
****************************************************************************/
//unsigned char USART_Send( eUartObj_t eUart, const void * Buff , unsigned char Len )
//{
//	u8 i = 0, b1 = 0;
//	if(Buff == NULL || Len <= 0 ) 
//	{	
//		return 1;
//	}	
//	for (i = 0; i < Len ; i++)
//	{
//  	b1 = *((uint8_t *)Buff + i);                                                 //�ӷ��ͻ�����������ȡ����                                
//		USART_SendData(UART_CB[eUart].USART_x, b1);
//		while(USART_GetFlagStatus(UART_CB[eUart].USART_x,USART_FLAG_TC)==RESET){};   //�ȴ��������
//	  USART_ClearFlag(UART_CB[eUart].USART_x,USART_FLAG_TC);	                                     //���������ɱ�־
//	}	
//	return 0;
//}


 void UART_CallBack(void *Pdata)
{
	int i = 0;
	uint16_t Len = 0;
	for( i = 0 ; i < UartObjectMax  ; i++)
	{
		if(UART_CB[i].ReciveStatus == eReciveIng)
		{
			if(UART_CB[i].RxNewByte)
			{
				UART_CB[i].RxNewByte = 0;
			}
			else
			{
				UART_CB[i].ReciveStatus = eReciveFinish;
				if(Uart1Object == i)
				{//����1�����ݣ�����Դ���Դ���3���յ�������
					memset(UART_CB[Uart3Object].tx_Buff,0,BUFFER_SIZE);
					DataInput(Uart1Object,UART_CB[Uart3Object].tx_Buff ,&Len);
					MYDMA_Enable(UART_CB[Uart3Object].DMA_TX_CHx,Len);//DMA���ĵ��Ǵ���3��TX
				}
				if(Uart2Object == i)
				{
					memset(UART_CB[i].rx_Buff,0,BUFFER_SIZE);
					DataInput(Uart1Object,UART_CB[i].rx_Buff ,&Len);
					MYDMA_Enable(UART_CB[1].DMA_TX_CHx,Len);
				}
				if(Uart3Object == i)
				{//����3�����ݣ�����Դ���Դ���1���յ�������
					memset(UART_CB[Uart1Object].tx_Buff,0,BUFFER_SIZE);
					DataInput(Uart3Object,UART_CB[Uart1Object].tx_Buff ,&Len);
					MYDMA_Enable(UART_CB[Uart1Object].DMA_TX_CHx,Len);
				}				
			}
		}
	}
}

void USARTx_Init(void)        
{
	CachePoor_Init(Uart1Object,U1_TXBuff,U1_RXBuff,BUFFER_SIZE,BUFFER_SIZE);
	CachePoor_Init(Uart2Object,U2_TXBuff,U2_RXBuff,BUFFER_SIZE,BUFFER_SIZE);
//	CachePoor_Init(Uart3Object,U3_TXBuff,U3_RXBuff,BUFFER_SIZE,BUFFER_SIZE);
	USARTx_Config(Uart1Object);
	USARTx_Config(Uart2Object);
//	USARTx_Config(Uart3Object);
	//task_create(&UART_callback_task,UART_CallBack,NULL,15);
}

//�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}



