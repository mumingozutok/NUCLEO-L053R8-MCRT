/*
 * adaptor.c
 *
 *  Created on: Nov 8, 2021
 *      Author: mg
 */

#include "stm32l0xx_hal.h"

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

uint8_t uart_rx_data;

typedef struct S_Digital_Channel{
	uint32_t port;
	uint32_t pin;
} Digital_Channel;

static Digital_Channel inputChannel[4];
static Digital_Channel outputChannel[6];

void initiate_input_channels(){
	inputChannel[0].port = GPIOB;
	inputChannel[0].pin = GPIO_PIN_1; //EXT1

	inputChannel[1].port = GPIOB;
	inputChannel[1].pin = GPIO_PIN_4; //EXT4

	inputChannel[2].port = GPIOB;
	inputChannel[2].pin = GPIO_PIN_5; //EXT5

	inputChannel[3].port = GPIOA;
	inputChannel[3].pin = GPIO_PIN_8; //EXT8
}

void initiate_output_channels(){
	outputChannel[0].port = GPIOB;
	outputChannel[0].pin = GPIO_PIN_12;

	outputChannel[1].port = GPIOB;
	outputChannel[1].pin = GPIO_PIN_8;

	outputChannel[2].port = GPIOB;
	outputChannel[2].pin = GPIO_PIN_9;

	outputChannel[3].port = GPIOB;
	outputChannel[3].pin = GPIO_PIN_13;

	outputChannel[4].port = GPIOB;
	outputChannel[4].pin = GPIO_PIN_14;

	outputChannel[5].port = GPIOB;
	outputChannel[5].pin = GPIO_PIN_15;
}

//Please write down GPIO output function in your hardware
void hal_gpio_write_pin(uint16_t chNum, uint8_t value){
	HAL_GPIO_WritePin(outputChannel[chNum].port, outputChannel[chNum].pin, value);
}

//Please write down "get system tick" function in your hardware
uint32_t hal_get_tick(){
	return HAL_GetTick();
}

void /*__attribute__((weak))*/ hal_init_tick(){
	HAL_InitTick(0);
}

//Communication Channel Adaptation

//Please write down functions for your communication channel
//And put this function right after your initialisations
void init_comm_data_service(){
	HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
}

//Please write down functions for communication timing services
//And put this function right after your initialisations
void init_comm_timing_service(){
	HAL_TIM_Base_Start_IT(&htim6);
	stop_comm_timer(&htim6);
}

void start_comm_timer(TIM_HandleTypeDef* htim){
	htim->Instance->CR1 &= ~0x01; //Stop Timer
	htim->Instance->CNT = 0; //Reset Counter
	htim->Instance->CR1 |= 0x01; //Start Timer
}

void stop_comm_timer(TIM_HandleTypeDef* htim){
	htim->Instance->CR1 &= ~0x01; //Stop Timer
	htim->Instance->CNT = 0; //Reset Counter
}

//Callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart, &uart_rx_data, 1);
	start_comm_timer(&htim6);

	//When new data received copy this data to the runtime buffers
	Runtime_CommDataService_NewData_Received(0, &uart_rx_data, 1);
/*
	//Call user callback
	User_Callback_Function* ucf = get_ucf();
	if(ucf[MODBUS_UART_CALLBACK_FUNCTION_SLOT].f != 0){
		ucf[MODBUS_UART_CALLBACK_FUNCTION_SLOT].f(&uart_rx_data);
	}
	*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim6){
		stop_comm_timer(&htim6);

		//External trigger makes runtime to process data
		//This trigging is needed for Modbus (3.5 Char)
		Runtime_CommDataService_Process_DataBuffer(0);
	}
}

//Modbus UART Transmit Functions
void /*__attribute__((weak))*/ hal_modbus_uart_tx(uint8_t* pData, uint16_t Size){
	HAL_UART_Transmit_IT(&huart1, pData, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
}

//-----------------------UNIQUE ID-----------------------------------------
#define UID_BASE              (0x1FF80050UL)        /*!< Unique device ID register base address  */
void get_uniqueid(uint8_t* id, uint16_t len){
	uint32_t* buf = id;
	buf[0] 	= (uint32_t) READ_REG(*((uint32_t *)UID_BASE));
	buf[1] = (uint32_t) READ_REG(*((uint32_t *)(UID_BASE + 0x04U)));
	buf[2] = (uint32_t) READ_REG(*((uint32_t *)(UID_BASE + 0x14U)));
}

//---------------------Flash functions---------------------------------------
#define NUM_OF_ALLOCATED_PAGE_FOR_PERSISTENT_STORAGE 2
#define FLASH_MEMORY_SIZE (NUM_OF_ALLOCATED_PAGE_FOR_PERSISTENT_STORAGE*FLASH_PAGE_SIZE)

void get_flash_memory_info(uint32_t* start_addr, uint32_t* size){
	*start_addr = FLASH_END-FLASH_MEMORY_SIZE+1;
	*size = FLASH_MEMORY_SIZE;
}

uint8_t write_to_flash(uint8_t* p, uint32_t start_addr, uint16_t size)
{
	uint8_t ret = 0;
	uint16_t i;
	uint32_t data;

	HAL_FLASH_Unlock();

	for (i = 0; i < size; i+=4) {
                data = *(uint32_t*)(p+i);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr + i, data) == HAL_OK) ret = 1;
		else {
			ret = 0;
			break;
		}
	}

	HAL_FLASH_Lock();
}

uint8_t erase_flash(uint32_t start_addr)
{
	uint8_t ret = 0;
	uint16_t page_number, page_start_address = 0;
	uint32_t PageError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;

	//Find flash start address of page that will be erased
	page_number = (uint32_t) (start_addr / FLASH_PAGE_SIZE);
	page_start_address = page_number * FLASH_PAGE_SIZE;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = page_start_address;
	EraseInitStruct.NbPages = NUM_OF_ALLOCATED_PAGE_FOR_PERSISTENT_STORAGE;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		ret = 1;
	}

	else ret = 0;

	return ret;
}
