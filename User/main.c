
/*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "SysTick.h"
#include "usb_hw.h"			 
#include "sw_delay.h"
#include "bsec_integration.h"
#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include "bsec_serialized_configurations_iaq.h"

#define Link_Off						GPIOC->BSRR = GPIO_Pin_13
#define Link_On 						GPIOC->BRR  = GPIO_Pin_13
#define BME_CS_H						GPIOA->BSRR = GPIO_Pin_4
#define BME_CS_L						GPIOA->BRR  = GPIO_Pin_4

ErrorStatus HSEStartUpStatus;
uint64_t sys_run_time=0;
char usb_buff[200]={0};
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);


void spi1_gpio_config(void)	
{
		GPIO_InitTypeDef GPIO_InitStructure;

		SPI_InitTypeDef SPI_InitStructure;
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);  //RCC_APB2_SPI1使能
		
		 /* Configure SPI1 pins:  SCK(PA5), MISO(PA6) and MOSI(PA7) CS(PA4) */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	/* GPIO配置为上拉输入模式 */
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_SetBits(GPIOA, GPIO_Pin_4); // aCS default high
		
		
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;  //主模式
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  //波特率 72M/64
		//SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx ;
		SPI_InitStructure.SPI_Direction =SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_CRCPolynomial = 0x0007;  //CRC多项式
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;  //时钟极性高
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;  //第2个时钟沿采样
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //帧格式：MSB
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;  //数据大小：16位
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;  //NSS信号管理：
		SPI_Init(SPI1, &SPI_InitStructure);

		SPI_Cmd(SPI1, ENABLE);  //SPI使能
		
		//config led
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
} 

 

u8 SPI1_Send_Receive_Byte(u8 dat)
{
		u8 ReceivedDat=0;
		SPI1->DR = dat;  //SPI_SendData(SPI1, 0xA5);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); //wait until get the data
		ReceivedDat = SPI_I2S_ReceiveData(SPI1);
		return ReceivedDat;
	
}

/*!
 * @brief           Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    // ...
    // Please insert system specific function to write to the bus where BME680 is connected
    // ...
	int dat_num=0;
	BME_CS_L;
	SPI1_Send_Receive_Byte(0x7F & reg_addr);
	for(dat_num=0; dat_num<data_len; dat_num++)
	{
		SPI1_Send_Receive_Byte(reg_data_ptr[dat_num]);
	}
	BME_CS_H;
    
	return 0;
}

/*!
 * @brief           Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    // ...
    // Please insert system specific function to read from bus where BME680 is connected
    // ...
	int dat_num=0;
	BME_CS_L;
	SPI1_Send_Receive_Byte(0x80|reg_addr);
	for(dat_num=0; dat_num<data_len; dat_num++)
	{
		reg_data_ptr[dat_num]=SPI1_Send_Receive_Byte(0x00);
	}
	BME_CS_H;
	
    return 0;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void sleep(uint32_t t_ms) //must be accuracy, otherwise the next measure start time is missmatch with the systicket,To ensure proper operation, a maximum jitter of 6.25% is allowed.
{
    // ...
    // Please insert system specific function sleep or delay for t_ms milliseconds
    // ...
	uint64_t sys_time_go = sys_run_time + t_ms ;
	while(sys_run_time <= sys_time_go);
}
 

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    int64_t system_current_time = 0;
    // ...
    // Please insert system specific function to retrieve a timestamp (in microseconds)
    // ...
		system_current_time=sys_run_time * 1000; //
	
    return system_current_time;
}

 

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
	
	int64_t time = 0;
	char i=0;
	if(i == 0 ) { Link_On; i=1; }
	else {Link_Off; i=0;}
		

	time = timestamp/1000000; 
	sprintf ( usb_buff, "Time: %d, IQA=%.2f, Accur=%d, T=%.2f, H=%.2f%%, P=%.2f, R=%.2f, STIAQ=%.2f, CO2=%.2f, BVoc=%.2f \r\n", (int)time, iaq, iaq_accuracy, temperature, humidity, pressure, gas, static_iaq, co2_equivalent, breath_voc_equivalent);
	usb_print(usb_buff);
	
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}
 
/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
	memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));
	return sizeof(bsec_config_iaq);
	//return 0;
    
}

void EXTI_PB0_Init(void)
{
  
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOB,&GPIO_InitStructure);  
  
 
  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);  
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure); 
  
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


int main(void)
{
  
	int ret;
	//return_values_init st;
 
	RCC_Configuration();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//open the clock for AFIO,should put before Remap, otherwise not work
																												//RCC->APB2ENR |= 0x00000001;	//open the clock for AFIO
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); 
	
	bsp_InitUsb();
	EXTI_PB0_Init();
	delayms(1000);
	spi1_gpio_config();
	 
 
	BME_CS_L;
	//change to page 0, must do, maybe caused by the csb initial
		 SPI1_Send_Receive_Byte(0x73);
		 SPI1_Send_Receive_Byte(0x01);
	BME_CS_H;
	delayms(1);
	BME_CS_L;
	//reset the DUT
		 SPI1_Send_Receive_Byte(0x60);
		 SPI1_Send_Receive_Byte(0xB6);
	BME_CS_H;
	 
	delayms(1);
 
	sys_run_time=0;
	SysTick_Init();
	usb_SendDataToHost("Ready\r\n", 7);
	
	/* Call to the function which initializes the BSEC library
* Switch on low-power mode and provide no temperature offset */
	ret = bsec_iot_init(BSEC_SAMPLE_RATE_ULP, 0.0f, bus_write, bus_read, sleep, state_load, config_load).bme680_status;
	if (ret)
	{
		/* Could not intialize BME680 or BSEC library */
		return ret;
	}
	/* Call to endless loop function which reads and processes data based on sensor settings */
	/* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes */
		bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
	
#ifdef DEBUG
	  debug();
	#endif
	
	
  
}


void RCC_Configuration(void)
{   
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
	
}

 

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif






