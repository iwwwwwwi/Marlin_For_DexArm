/*
 * front_rotation.c
 *
 *  Created on: Jun 15, 2020
 *      Author: fbc
 */
#include "HAL.h"

#include "stm32f4xx_hal.h"
#include "dexarm_front_rotation.h"
#include "interrupt.h"
#include "../../module/stepper/indirection.h"

#define KEY_Pin GPIO_PIN_10
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI15_10_IRQn

extern uint8_t usart_rev_ch;
extern uint8_t front_rotation_init_flag;
#define BUF_LEN 50
extern uint8_t usart_rev_buf[BUF_LEN];

#define DATA_BUF_LEN	30
typedef struct _r_data
{
	uint8_t buf[DATA_BUF_LEN];
	uint8_t len;
}data_typedef;

//协议头
uint8_t cmd_head[2]={0xFF,0xFF};

uint8_t cmd_buf[CMD_MAX_LEN]={};

UART_HandleTypeDef huart1;


/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
	//1000000
	//115200
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 117647;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

uint8_t front_button_flag = 0;
void front_button_callback()
{	
	if(front_button_flag)
	{
		MYSERIAL0.println("Front button is pressed!\r\n");

		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET)
		{
			MYSERIAL0.println(" level down!\r\n");
			DISABLE_AXIS_X();
			DISABLE_AXIS_Y();
			DISABLE_AXIS_Z();

		}
		else if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_SET)
		{
			MYSERIAL0.println(" level up!\r\n");
			ENABLE_AXIS_X();
			ENABLE_AXIS_Y();
			ENABLE_AXIS_Z();		
		}		
	}
	
}

void front_rotation_init(void){
	stm32_interrupt_enable(KEY_GPIO_Port,KEY_Pin,front_button_callback,GPIO_MODE_IT_RISING_FALLING);
	MX_USART1_UART_Init();
	HAL_Delay(100);
	front_button_flag = 1;
}

//除去协议头计算 ~校检和
uint16_t check_sum(uint8_t buf[],uint8_t len)
{
	uint16_t check_val=0;
	int offset=2;
	for(int i=offset;i<len+offset+1;i++)
	{
		check_val += buf[i];
	}

	check_val = ~check_val & 0x00FF;

	return check_val;
}

//FF FF 	01 05 03 1E F4 01 	E3 EE
uint8_t set_servo_pos(uint8_t id,uint16_t pos)
{
	int data_len=0;
	uint8_t cmd = WRITE_CMD;
	uint8_t pos_hex[2];
	uint16_t check_val=0;
	uint8_t addr = 0x1E;	//地址

	cmd_buf[0] = cmd_head[0];
	cmd_buf[1] = cmd_head[1];
	cmd_buf[2] = id;			data_len++;
	cmd_buf[4] = cmd;			data_len++;
	//字节转换
	pos_hex[0] = pos>>8;
	pos_hex[1] = pos&0xFF;

	cmd_buf[5] = addr;			data_len++;
	cmd_buf[6] = pos_hex[1];	data_len++;
	cmd_buf[7] = pos_hex[0];	data_len++;


	cmd_buf[3] = data_len;		data_len++;

	check_val = check_sum(cmd_buf,data_len);

	cmd_buf[8] = check_val;

	return data_len+2;
}

data_typedef w_data_buf(uint8_t id,uint8_t cmd,uint8_t addr,uint16_t info)
{

	data_typedef data;
	memset(&data,0,sizeof(data_typedef));
	uint8_t index = 4;
	data.buf[0] = cmd_head[0];
	data.buf[1] = cmd_head[1];
	data.buf[2] = id;
	data.len ++;
	data.buf[index++] = cmd;
	data.len++;
	data.buf[index++] = addr;
	data.len++;
	data.buf[index++] = info&0xFF;
	data.len++;
	data.buf[index++] = info>>8;
	data.len++;

	data.buf[3] = data.len;
	data.len++;
	data.buf[index++] = check_sum(data.buf,data.len);

	return data;

}

void usart_get_flag_wait()
{
	int outtime = 1000*20;
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET)
	{
		 outtime--;
		 if(outtime<0)
		 {
		 	MYSERIAL0.println("receive time out......\r\n");
			front_rotation_init_flag = 0;
		 	return ;
		 }
	}
}

bool write_info(int id,uint8_t reg,int val)
{

	data_typedef temp;
	uint8_t rev_buf[DATA_BUF_LEN];
	memset(&temp,0,sizeof(data_typedef));
	memset(rev_buf,0,sizeof(uint8_t)*DATA_BUF_LEN);
	temp = w_data_buf(id,WRITE_CMD,reg,val);
	rev_buf[4] = 1;//status flag
	do
	{
		HAL_HalfDuplex_EnableTransmitter(&huart1);
		//HAL_UART_Transmit(&huart1, temp.buf, temp.len+3, 1000);
		HAL_UART_Transmit(&huart1, temp.buf, 10, 1000);

		while( __HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET);	//等待发送结束
//		usart_get_flag_wait();

		uint8_t rev_len = 0;

		//先接收4 个字节
		//两个协议头 一个ID 一个长度
		HAL_HalfDuplex_EnableReceiver(&huart1);
//		while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
		usart_get_flag_wait();
		HAL_UART_Receive(&huart1, rev_buf, 4, 1000);

		if(rev_buf[0]==0xFF&&rev_buf[1]==0xFF)
		{
			rev_len = rev_buf[3];//获取长度
		}

		HAL_HalfDuplex_EnableReceiver(&huart1);
//		while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
		usart_get_flag_wait();
		HAL_UART_Receive(&huart1, rev_buf+4, rev_len, 1000);

	}while (rev_buf[4]);//发送失败,重发
	return true;

}


data_typedef r_data_buf(uint8_t id,uint8_t cmd,uint8_t addr,int addr_len)
{
	data_typedef data;
	memset(&data,0,sizeof(data_typedef));
	data.buf[0] = cmd_head[0];
	data.buf[1] = cmd_head[1];
	data.buf[2] = id;
	data.len ++;
	data.buf[4] = cmd;
	data.len++;
	data.buf[5] = addr;
	data.len++;
	data.buf[6] = addr_len;
	data.len++;
	data.buf[3] = data.len;

	data.buf[7] = check_sum(data.buf,data.len);

	return data;
}



uint8_t ret_reg_count(uint8_t reg)
{
	uint8_t ret_val=0;
	switch (reg)
	{
		case TORQUE_REG:		ret_val = 2;break;
		case POS_REG:			ret_val = 2;break;
		case ANGLE_SPEED_REG:	ret_val = 2;break;
		case BURDEN_REG:		ret_val = 2;break;
		case MOTION_SPEED_REG:	ret_val = 2;break;
		case MIN_ANGLE_REG:		ret_val = 2;break;
		case MAX_ANGLE_REG:		ret_val = 2;break;
		case VOLTAGE_REG:		ret_val = 1;break;
		case TEMP_REG:			ret_val = 1;break;
		case TORQUE_ENABLE_REG:	ret_val = 1;break;
		case BONED_SPEED:		ret_val = 1;break;

		default:
		break;
	}
	return ret_val;
}



uint16_t read_info(uint8_t id,uint8_t reg)
{
	uint8_t rev_buf[DATA_BUF_LEN]={0};
	data_typedef temp;
	temp = r_data_buf(id,READ_CMD,reg,ret_reg_count(reg));

	HAL_HalfDuplex_EnableTransmitter(&huart1);
	HAL_UART_Transmit(&huart1, temp.buf, temp.len+4, 1000);

	while( __HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET);	//等待发送结束

	uint8_t rev_len = 0;

	//先接收4 个字节
	//两个协议头 一个ID 一个长度
	HAL_HalfDuplex_EnableReceiver(&huart1);
//	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
	usart_get_flag_wait();
	HAL_UART_Receive(&huart1, rev_buf, 4, 1000);

	if(rev_buf[0]==0xFF&&rev_buf[1]==0xFF)
	{
		rev_len = rev_buf[3];//获取长度
	}

	//状态
	//rev_buf[4]
	//根据长度接收剩下的字符

	HAL_HalfDuplex_EnableReceiver(&huart1);
//	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET);
	usart_get_flag_wait();
	HAL_UART_Receive(&huart1, rev_buf+4, rev_len, 1000);

	//结果

	volatile uint16_t info_val = 0;
	if(ret_reg_count(reg) == 2)
		info_val = rev_buf[6]<<8|rev_buf[5];
	else if(ret_reg_count(reg) == 1)
		info_val = rev_buf[5];

	return info_val;
}

int scope_limit(int min,int val,int max)
{
	if (val <= min) 
	{val = min;}
	else if(val >= max)
	{val = max;}
	return val;
}

//同步写入
void sync_write()
{

}


/*
//temp
//目前硬件未支持,读出的数值为0
uint16_t read_temp(uint8_t id)
{
	return read_info(id,TEMP_REG);
}

//vola
//目前硬件未支持,读出的数值为0
uint16_t read_vola(uint8_t id)
{
	return read_info(id,VOLTAGE_REG);
}

*/

// 读取位置
uint16_t read_pos(uint8_t id)
{
	return read_info(id,POS_REG);
}

//读取扭矩是否使能
uint16_t read_enable(uint8_t id)
{
	return read_info(id,TORQUE_ENABLE_REG);
}



//设置扭矩限制
uint16_t set_torque_limt(uint8_t id,int val)
{
	return write_info(id,TORQUE_REG,val);
}

//读取扭矩限制
bool read_torque_limt(uint8_t id)
{
	return read_info(id,TORQUE_REG);
}


// 读取最小位置
uint16_t read_min_pos(uint8_t id)
{
	return read_info(id,MIN_ANGLE_REG);
}

// 读取最大位置
uint16_t read_max_pos(uint8_t id)
{
	return read_info(id,MAX_ANGLE_REG);
}

// 设置最小位置
bool set_min_pos(int id,int val)
{
	return write_info(id,MIN_ANGLE_REG,val);
}

// 设置最大位置
bool set_max_pos(int id,int val)
{
	return write_info(id,MAX_ANGLE_REG,val);
}
//读速度
uint16_t read_motion_speed(uint8_t id)
{
	return read_info(id,MOTION_SPEED_REG);
}
//设置速度
uint16_t set_motion_speed(uint8_t id,int val)
{
	return write_info(id,MOTION_SPEED_REG,val);
}

//读速度
uint16_t read_bps(uint8_t id)
{
	return read_info(id,BONED_SPEED);
}
//设置速度
uint16_t set_bps(uint8_t id,int val)
{
	return write_info(id,BONED_SPEED,val);
}

void motion_speed_demo()
{
	int val =0;
	val=read_motion_speed(SERO_1);
	HAL_Delay(1000);
	set_motion_speed(SERO_1,200);
	HAL_Delay(1000);
	val=read_motion_speed(SERO_1);
	HAL_Delay(1000);

}

// 设置位置
bool set_pos(int id,int val)
{
	return write_info(id,TARGET_POS_REG,val);
}

// 设置使能
bool set_enable(int id,int val)
{
	return write_info(id,TORQUE_ENABLE_REG,val);
}

//设置最大位置最小位置
//0-1023
void max_min_pos_demo_test()
{
	int min_val =0;
	int max_val =0;
	// 读取
	min_val = read_max_pos(SERO_1);
	max_val = read_min_pos(SERO_1);
	HAL_Delay(200);
	// 设置
	set_min_pos(SERO_1,0);
	HAL_Delay(200);//延时不可省略
	set_max_pos(SERO_1,1000);
	// 读取
	HAL_Delay(200);
	min_val = read_max_pos(SERO_1);
	max_val = read_min_pos(SERO_1);

}

// 使能读写测试
void enable_demo_test()
{
	static uint8_t test_val[100]={0};
	static int index = 0;
	set_enable(SERO_1,1);
	test_val[index++] = read_enable(SERO_1); // 0  1

	HAL_Delay(1000);

	set_enable(SERO_1,0);
	test_val[index++] = read_enable(SERO_1);

	HAL_Delay(1000);
	if(index>98)
	{
		index=0;
		memset(test_val,0,sizeof(int)*100);
	}
}


// 读写位置测试
void pos_demo_test()
{
	char str[30];
	memset(&str,0,30);

	int i=0;
	for(i=0;i<=2;i++)
	{
		set_pos(SERO_1,100*i);
		HAL_Delay(1000);
	}

	for(;i>0;i--)
	{
		set_pos(SERO_1,i*100);
		HAL_Delay(1000);
		sprintf(str,"surrent positon = %d",read_pos(SERO_1));
		HAL_Delay(100);
		sprintf(str,"surrent speed = %d",read_motion_speed(SERO_1));
		HAL_Delay(100);		
		MYSERIAL0.println(str);
	}
}


//扭矩测试
void torque_demo_test()
{
	int val=0;
	val = read_torque_limt(SERO_1);
	HAL_Delay(500);
	set_torque_limt(SERO_1, 1023);	//0-1023
	HAL_Delay(1000);
	val = read_torque_limt(SERO_1);
	HAL_Delay(500);
}
//16 : 115200
//207:	9600
//读写波特率
void bps_demo()
{
	int val=0;
	val = read_bps(SERO_1);
	HAL_Delay(500);
	set_bps(SERO_1,16);//117647
//	HAL_Delay(500);
//	val = read_bps(SERO_1);
	HAL_Delay(500);


}

