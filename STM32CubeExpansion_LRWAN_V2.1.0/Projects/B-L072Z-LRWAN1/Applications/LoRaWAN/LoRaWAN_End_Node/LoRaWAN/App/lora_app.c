/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "sys_conf.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"

/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <time.h>

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

typedef struct agg_value {
	uint32_t sum;
	uint32_t sqSum;
	uint16_t count;
	uint16_t min;
	uint16_t max;
} agg_value;

typedef enum cond_type
{
  GREATER,
  GREATER_THAN_EQUAL,
  LESS,
  LESS_THAN_EQUAL
} agg_type;

typedef enum param_type
{
	TEMPERATURE,
    PRESSURE,
    HUMIDITY
} param_type;

typedef enum data_agg_type
{
    DATA_AGG_OFF,
	DATA_AGG_ON,
} data_agg_type;

typedef enum cond_sel_type
{
    OR,
	AND,
} cond_sel_type;

typedef struct agg_condition {
	uint16_t value;
	param_type parameter;
	agg_type condition;
} agg_condition;

typedef struct agg_storage {
	uint8_t measurementId;
	uint8_t AggStateOn;
	uint8_t AggMaxOn;
	uint8_t AggMinOn;
	uint8_t AggCountOn;
	uint8_t AggAvgOn;
	uint8_t AggSumOn;
	uint8_t AggSqSumOn;
	uint8_t epDuration;
	agg_value collectedData;
	uint8_t condSize;
	uint8_t condSel; // condition selector: "OR" or "AND"
	agg_condition aggConditions[6];
} agg_storage;

typedef struct children { // children of OBJECT or ARRAY
	int length;
	struct nx_json *first;
	struct nx_json *last;
} children;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LORAWAN_RULE_MESS_PORT 5

#define LORAWAN_USER_RULE_PORT 5

/* USER CODE END PM */


/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  Data measure timer callback function
  * @param  context ptr of timer context
  */
static void OnMeasureDataTimerEvent(void *context);

static void OnMeasureTemperatureTimerEvent(void *context);

static void OnMeasurePressureTimerEvent(void *context);

static void OnMeasureHumidityTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRa application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/*!
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */
static void OnMacProcessNotify(void);

/* USER CODE BEGIN PFP */

static void CollectMeasurements(void);

static void SendTemperatureMeasurements(void);

static void SendPressureMeasurements(void);

static void SendHumidityMeasurements(void);


/**
  * @brief  LED Tx timer callback function
  * @param  context ptr of LED context
  */
static void OnTxTimerLedEvent(void *context);

/**
  * @brief  LED Rx timer callback function
  * @param  context ptr of LED context
  */
static void OnRxTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  context ptr of LED context
  */
static void OnJoinTimerLedEvent(void *context);

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =           GetBatteryLevel,
  .GetTemperature =            GetTemperatureLevel,
  .GetUniqueId =               GetUniqueId,
  .GetDevAddr =                GetDevAddr,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/* USER CODE BEGIN PV */

static UTIL_TIMER_Object_t DataMeasureTimer;

static UTIL_TIMER_Object_t TxTemperatureMeasureTimer;

static UTIL_TIMER_Object_t TxPressureMeasureTimer;

static UTIL_TIMER_Object_t TxHumidityMeasureTimer;

static uint8_t AggStateOn = 0;

static uint8_t PressDataOn = 0;

static uint8_t TempDataOn = 0;

static uint8_t HumDataOn = 0;

//

static agg_storage humidity_storage = { 0, 0, 0, 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0 } };

static agg_storage pressure_storage = { 0, 0, 0, 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0 } };

static agg_storage temperature_storage = { 0, 0, 0, 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0 } };

/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/**
  * @brief Specifies the state of the application LED
  */
static uint8_t AppLedStateOn = RESET;

//static uint8_t UpdatableAppTxDutycycle = 1000000;

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static UTIL_TIMER_Object_t TxLedTimer;

/**
  * @brief Timer to handle the application Rx Led to toggle
  */
static UTIL_TIMER_Object_t RxLedTimer;

/**
  * @brief Timer to handle the application Join Led to toggle
  */
static UTIL_TIMER_Object_t JoinLedTimer;

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_1 */

  LED_Init(LED_BLUE);
  LED_Init(LED_RED1);
  LED_Init(LED_RED2);

  /* Get LoRa APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION:        V%X.%X.%X\r\n",
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW LoraWAN info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:   V%X.%X.%X\r\n",
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB2_SHIFT));

  UTIL_TIMER_Create(&TxLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&RxLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnRxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&JoinLedTimer, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, OnJoinTimerLedEvent, NULL);
  UTIL_TIMER_SetPeriod(&TxLedTimer, 500);
  UTIL_TIMER_SetPeriod(&RxLedTimer, 500);
  UTIL_TIMER_SetPeriod(&JoinLedTimer, 500);

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_CollectMeasurmentsEvent), UTIL_SEQ_RFU, CollectMeasurements);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTemperatureTimerEvent), UTIL_SEQ_RFU, SendTemperatureMeasurements);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnPressureTimerEvent), UTIL_SEQ_RFU, SendPressureMeasurements);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnHumidityTimerEvent), UTIL_SEQ_RFU, SendHumidityMeasurements);
  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  UTIL_TIMER_Start(&JoinLedTimer);

  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
	APP_LOG(TS_ON, VLEVEL_L, "LORA SETUP\r\n");

    UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Create(&DataMeasureTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnMeasureDataTimerEvent, NULL);
    UTIL_TIMER_Create(&TxTemperatureMeasureTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnMeasureTemperatureTimerEvent, NULL);
    UTIL_TIMER_Create(&TxPressureMeasureTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnMeasurePressureTimerEvent, NULL);
    UTIL_TIMER_Create(&TxHumidityMeasureTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnMeasureHumidityTimerEvent, NULL);

    UTIL_TIMER_SetPeriod(&TxTimer, APP_TX_DUTYCYCLE);
    UTIL_TIMER_SetPeriod(&DataMeasureTimer, APP_TX_DUTYCYCLE);
    UTIL_TIMER_SetPeriod(&TxTemperatureMeasureTimer, APP_TX_DUTYCYCLE);
    UTIL_TIMER_SetPeriod(&TxPressureMeasureTimer, APP_TX_DUTYCYCLE);
    UTIL_TIMER_SetPeriod(&TxHumidityMeasureTimer, APP_TX_DUTYCYCLE);

    UTIL_TIMER_Start(&TxTimer);
    UTIL_TIMER_Start(&DataMeasureTimer);
    UTIL_TIMER_Start(&TxTemperatureMeasureTimer);
    UTIL_TIMER_Start(&TxPressureMeasureTimer);
    UTIL_TIMER_Start(&TxHumidityMeasureTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* send every time button is pushed */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
//    case  USER_BUTTON_PIN:
//      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
//      break;
    default:
      break;
  }
}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  APP_LOG(TS_ON, VLEVEL_L, "MESSAGE WITH DATA RECEIVED\r");
  if ((appData != NULL) || (params != NULL))
  {
    LED_On(LED_BLUE);

    UTIL_TIMER_Start(&RxLedTimer);

    static const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };

    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Indication ==========\r\n");
    APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\r\n",
            params->DownlinkCounter, slotStrings[params->RxSlot], appData->Port, params->Datarate, params->Rssi, params->Snr);
    APP_LOG(TS_ON, VLEVEL_L, "PORT:%d\r\n", appData->Port);

	uint8_t *buffer;
	uint8_t parameters_number;
	uint8_t epDur;
	uint8_t status;
	uint8_t l = 0;
	uint8_t i = 0;
	uint8_t j = 0;
	UTIL_TIMER_Time_t nextTxIn = 0;
	time_t start,end;

    switch (appData->Port)
    {
      case LORAWAN_SWITCH_CLASS_PORT:
        /*this port switches the class*/
        if (appData->BufferSize == 1)
        {
          switch (appData->Buffer[0])
          {
            case 0:
            {
              LmHandlerRequestClass(CLASS_A);
              break;
            }
            case 1:
            {
              LmHandlerRequestClass(CLASS_B);
              break;
            }
            case 2:
            {
              LmHandlerRequestClass(CLASS_C);
              break;
            }
           default:
              break;
          }
        }
        break;
      case LORAWAN_USER_APP_PORT:
        if (appData->BufferSize == 1)
        {
          AppLedStateOn = appData->Buffer[0] & 0x01;
          if (AppLedStateOn == RESET)
          {
            APP_LOG(TS_OFF, VLEVEL_H,   "LED OFF\r\n");

            LED_Off(LED_RED1);
          }
          else
          {
            APP_LOG(TS_OFF, VLEVEL_H, "LED ON\r\n");

            LED_On(LED_RED1);
          }
        }
        break;
      case LORAWAN_RULE_MESS_PORT:

    	  // Processing received data
    	  buffer = appData->Buffer;

    	  PressDataOn = 0;
    	  TempDataOn = 0;
    	  HumDataOn = 0;
    	  l = 0;

    	  parameters_number = buffer[l++];

    	  if (parameters_number != 0) {
    		  for (i = 0; i < parameters_number; i++) {
    			  switch (buffer[l++])
    			  {
    			  	  case TEMPERATURE:
    	    			  APP_LOG(TS_ON, VLEVEL_L, "READING PARAMETER\r\n");
    	    			  APP_LOG(TS_ON, VLEVEL_L, "ENABLING PARAMETER: TEMPERATURE\r\n");
    	    			  TempDataOn = 1;

    	    			  temperature_storage.AggMaxOn = 0;
    	    			  temperature_storage.AggMinOn = 0;
    	    			  temperature_storage.AggCountOn = 0;
    	    			  temperature_storage.AggAvgOn = 0;
    	    			  temperature_storage.AggSumOn = 0;
    	    			  temperature_storage.AggSqSumOn = 0;
    	    			  temperature_storage.condSize = 0;
    	    			  temperature_storage.measurementId = 0;

    	    			  // START AGGREGATOR

    	    			  APP_LOG(TS_ON, VLEVEL_L, "READING AGG FUNCTION\r\n");
    	    			  if (buffer[l++] == 1){
    	    			  	temperature_storage.AggMaxOn = 1;
    	    			  }
    	    			  if (buffer[l++] == 1){
    	    			  	temperature_storage.AggMinOn = 1;
    	    			  }
    	    			  if (buffer[l++] == 1){
    	    			    temperature_storage.AggCountOn = 1;
    	    			  }
    	    			  if (buffer[l++] == 1){
    	    			    temperature_storage.AggSumOn = 1;
    	    			  }
    	    			  if (buffer[l++] == 1){
    	    			    temperature_storage.AggSqSumOn = 1;
    	    			  }

    	    			  APP_LOG(TS_ON, VLEVEL_L, "END READING AGG FUNCTION\r\n");

     	    			  // END AGGREGATOR

    	    			  // START EPOCH DURATION

    	    			  epDur = (uint8_t)((buffer[l++] << 8) | buffer[l++]);

    	    			  APP_LOG(TS_ON, VLEVEL_L, "REPORTING PERIOD: %d [s]\r\n", epDur);
    	    			  temperature_storage.epDuration = epDur;

    	    			  UTIL_TIMER_Stop(&TxTimer);

    	    			  UTIL_TIMER_Create(&TxTemperatureMeasureTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnMeasureTemperatureTimerEvent, NULL);
    	    			  UTIL_TIMER_SetPeriod(&TxTemperatureMeasureTimer, epDur*1000);
    	    			  UTIL_TIMER_Start(&TxTemperatureMeasureTimer);

    	    			  // END EPOCH DURATION

    	    			  // START CONDITION

    	    			  // condition chain

    	    			  switch(buffer[l++])
    	    			  {
    	    			  	  case AND:
    	    				  	  temperature_storage.condSel = AND;
    	    	    			   APP_LOG(TS_ON, VLEVEL_L, "CONDITION CHAIN: AND\r\n");
    	    				  	  break;
    	    			  	  case OR:
    	    			      	  temperature_storage.condSel = OR;
    	    	    			   APP_LOG(TS_ON, VLEVEL_L, "CONDITION CHAIN: OR\r\n");
    	    			      	  break;
    	    			  	  default:
    	    			  		  break;
    	    			  }

    	    			  temperature_storage.condSize = buffer[l++];

    	    			  if (temperature_storage.condSize != 0) {
    	    			    APP_LOG(TS_ON, VLEVEL_L, "ENABLING CONDITION:\r\n");

    	    			    for (j = 0; j < temperature_storage.condSize; j++) {
    	    	    			APP_LOG(TS_ON, VLEVEL_L, "READING CONDITION\r\n");
    	    			    	// condition parameter
    	    			      	switch (buffer[l++])
    	    			      	{
    	    			      		case TEMPERATURE:
    	    			      	    	APP_LOG(TS_ON, VLEVEL_L, "parameter: temperature\r\n");
    	    			      	    	temperature_storage.aggConditions[j].parameter = TEMPERATURE;
    	    			      	    	break;
    	    			      	    case PRESSURE:
    	    			      	        APP_LOG(TS_ON, VLEVEL_L, "parameter: pressure\r\n");
    	    			      	      temperature_storage.aggConditions[j].parameter = PRESSURE;
    	    			      	        break;
    	    			      	    case HUMIDITY:
    	    			      	        APP_LOG(TS_ON, VLEVEL_L, "parameter: humidity\r\n");
    	    			      	      temperature_storage.aggConditions[j].parameter = HUMIDITY;
    	    			      	        break;
    	    			      	    default:
    	    			      	       break;
    	    			      	}

    	    			      	// condition type

    	    			    	switch (buffer[l++])
    	    			    	{
    	    			    		case GREATER:
										APP_LOG(TS_ON, VLEVEL_L, "code: GREATER\r\n");
										temperature_storage.aggConditions[j].condition = GREATER;
										break;
    	    			    		case GREATER_THAN_EQUAL:
										APP_LOG(TS_ON, VLEVEL_L, "code: GREATER_THAN_EQUAL\r\n");
										temperature_storage.aggConditions[j].condition = GREATER_THAN_EQUAL;
										break;
    	    			    		case LESS:
										APP_LOG(TS_ON, VLEVEL_L, "code: LESS\r\n");
										temperature_storage.aggConditions[j].condition = LESS;
										break;
    	    			    		case LESS_THAN_EQUAL:
										APP_LOG(TS_ON, VLEVEL_L, "code: LESS_THAN_EQUAL\r\n");
										temperature_storage.aggConditions[j].condition = LESS_THAN_EQUAL;
										break;
    	    			    		default:
    	    			    			break;
    	    			    	}

								temperature_storage.aggConditions[j].value = (uint16_t)((buffer[l++] << 8) | buffer[l++]);
    	    			    	APP_LOG(TS_ON, VLEVEL_L, "value: %d\r\n", temperature_storage.aggConditions[j].value);
    	    	    			APP_LOG(TS_ON, VLEVEL_L, "END READING CONDITION\r\n");
    	    			    }
    	    			  }

    	    			  // END CONDITION

    	    			  temperature_storage.AggStateOn = 1;
    	    			  APP_LOG(TS_ON, VLEVEL_L, "END READING PARAMETER: \r\n");

    			  		  break;
    			  	  case PRESSURE:
    	    			  APP_LOG(TS_ON, VLEVEL_L, "ENABLING PARAMETER: PRESSURE\r\n");
    	    			  PressDataOn = 1;

    	    			  pressure_storage.AggMaxOn = 0;
    	    			  pressure_storage.AggMinOn = 0;
    	    			  pressure_storage.AggCountOn = 0;
    	    			  pressure_storage.AggAvgOn = 0;
    	    			  pressure_storage.AggSumOn = 0;
    	    			  pressure_storage.AggSqSumOn = 0;
    	    			  pressure_storage.condSize = 0;
    	    			  pressure_storage.measurementId = 0;

    	    			  // START AGGREGATOR

						  if (buffer[l++] == 1){
							  pressure_storage.AggMaxOn = 1;
						  }
						  if (buffer[l++] == 1){
							  pressure_storage.AggMinOn = 1;
						  }
						  if (buffer[l++] == 1){
							  pressure_storage.AggCountOn = 1;
						  }
						  if (buffer[l++] == 1){
							  pressure_storage.AggSumOn = 1;
						  }
						  if (buffer[l++] == 1){
							  pressure_storage.AggSqSumOn = 1;
						  }

						  // END AGGREGATOR

    	    			  // START EPOCH DURATION

						  epDur = (uint8_t)((buffer[l++] << 8) | buffer[l++]);

    	    			  APP_LOG(TS_ON, VLEVEL_L, "REPORTING PERIOD: %d [s]\r\n", epDur);
    	    			  pressure_storage.epDuration = epDur;

    	    			  UTIL_TIMER_Stop(&TxTimer);

    	    			  UTIL_TIMER_Create(&TxPressureMeasureTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnMeasurePressureTimerEvent, NULL);
    	    			  UTIL_TIMER_SetPeriod(&TxPressureMeasureTimer, epDur*1000);
    	    			  UTIL_TIMER_Start(&TxPressureMeasureTimer);

    	    			  // END EPOCH DURATION

    	    			  // condition chain

						  switch(buffer[l++])
						  {
							  case AND:
								  pressure_storage.condSel = AND;
								   APP_LOG(TS_ON, VLEVEL_L, "CONDITION CHAIN: AND\r\n");
								  break;
							  case OR:
								  pressure_storage.condSel = OR;
								   APP_LOG(TS_ON, VLEVEL_L, "CONDITION CHAIN: OR\r\n");
								  break;
							  default:
								  break;
						  }

						  pressure_storage.condSize = buffer[l++];

						  if (pressure_storage.condSize != 0) {
							APP_LOG(TS_ON, VLEVEL_L, "ENABLING CONDITION:\r\n");

							for (j = 0; j < pressure_storage.condSize; j++) {
								// condition parameter
								switch (buffer[l++])
								{
									case TEMPERATURE:
										APP_LOG(TS_ON, VLEVEL_L, "parameter: temperature\r\n");
										pressure_storage.aggConditions[j].parameter = TEMPERATURE;
										break;
									case PRESSURE:
										APP_LOG(TS_ON, VLEVEL_L, "parameter: pressure\r\n");
										pressure_storage.aggConditions[j].parameter = PRESSURE;
										break;
									case HUMIDITY:
										APP_LOG(TS_ON, VLEVEL_L, "parameter: humidity\r\n");
										pressure_storage.aggConditions[j].parameter = HUMIDITY;
										break;
									default:
									   break;
								}

								// condition type

								switch (buffer[l++])
								{
									case GREATER:
										APP_LOG(TS_ON, VLEVEL_L, "code: GREATER\r\n");
										pressure_storage.aggConditions[j].condition = GREATER;
										break;
									case GREATER_THAN_EQUAL:
										APP_LOG(TS_ON, VLEVEL_L, "code: GREATER_THAN_EQUAL\r\n");
										pressure_storage.aggConditions[j].condition = GREATER_THAN_EQUAL;
										break;
									case LESS:
										APP_LOG(TS_ON, VLEVEL_L, "code: LESS\r\n");
										pressure_storage.aggConditions[j].condition = LESS;
										break;
									case LESS_THAN_EQUAL:
										APP_LOG(TS_ON, VLEVEL_L, "code: LESS_THAN_EQUAL\r\n");
										pressure_storage.aggConditions[j].condition = LESS_THAN_EQUAL;
										break;
									default:
										break;
								}

								pressure_storage.aggConditions[j].value = (uint16_t)((buffer[l++] << 8) | buffer[l++]);
								APP_LOG(TS_ON, VLEVEL_L, "value: %d\r\n", pressure_storage.aggConditions[j].value);
							}
						  }

						  // END CONDITION

    	    			  pressure_storage.AggStateOn = 1;
    			  		  break;
    			  	  case HUMIDITY:
    	    			  APP_LOG(TS_ON, VLEVEL_L, "ENABLING PARAMETER: HUMIDITY\r\n");
    	    			  HumDataOn = 1;

    	    			  humidity_storage.AggMaxOn = 0;
    	    			  humidity_storage.AggMinOn = 0;
    	    			  humidity_storage.AggCountOn = 0;
    	    			  humidity_storage.AggAvgOn = 0;
    	    			  humidity_storage.AggSumOn = 0;
    	    			  humidity_storage.AggSqSumOn = 0;
    	    			  humidity_storage.condSize = 0;
    	    			  humidity_storage.measurementId = 0;

    	    			  // START AGGREGATOR

						  if (buffer[l++] == 1){
							  humidity_storage.AggMaxOn = 1;
						  }
						  if (buffer[l++] == 1){
							  humidity_storage.AggMinOn = 1;
						  }
						  if (buffer[l++] == 1){
							  humidity_storage.AggCountOn = 1;
						  }
						  if (buffer[l++] == 1){
							  humidity_storage.AggSumOn = 1;
						  }
						  if (buffer[l++] == 1){
							  humidity_storage.AggSqSumOn = 1;
						  }

						  // END AGGREGATOR

    	    			  // START EPOCH DURATION

    	    			  epDur = (uint8_t)((buffer[l++] << 8) | buffer[l++]);

    	    			  APP_LOG(TS_ON, VLEVEL_L, "REPORTING PERIOD: %d [s]\r\n", epDur);
    	    			  humidity_storage.epDuration = epDur;

    	    			  UTIL_TIMER_Stop(&TxTimer);

    	    			  UTIL_TIMER_Create(&TxHumidityMeasureTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnMeasureHumidityTimerEvent, NULL);
    	    			  UTIL_TIMER_SetPeriod(&TxHumidityMeasureTimer, epDur*1000);
    	    			  UTIL_TIMER_Start(&TxHumidityMeasureTimer);

    	    			  // END EPOCH DURATION

    	    			  // START CONDITION

						  // condition chain

						  switch(buffer[l++])
						  {
							  case AND:
								  humidity_storage.condSel = AND;
								   APP_LOG(TS_ON, VLEVEL_L, "CONDITION CHAIN: AND\r\n");
								  break;
							  case OR:
								  humidity_storage.condSel = OR;
								   APP_LOG(TS_ON, VLEVEL_L, "CONDITION CHAIN: OR\r\n");
								  break;
							  default:
								  break;
						  }

						  humidity_storage.condSize = buffer[l++];

						  if (humidity_storage.condSize != 0) {
							APP_LOG(TS_ON, VLEVEL_L, "ENABLING CONDITION:\r\n");

							for (j = 0; j < temperature_storage.condSize; j++) {
								// condition humidity_storage
								switch (buffer[l++])
								{
									case TEMPERATURE:
										APP_LOG(TS_ON, VLEVEL_L, "parameter: temperature\r\n");
										humidity_storage.aggConditions[j].parameter = TEMPERATURE;
										break;
									case PRESSURE:
										APP_LOG(TS_ON, VLEVEL_L, "parameter: pressure\r\n");
										humidity_storage.aggConditions[j].parameter = PRESSURE;
										break;
									case HUMIDITY:
										APP_LOG(TS_ON, VLEVEL_L, "parameter: humidity\r\n");
										humidity_storage.aggConditions[j].parameter = HUMIDITY;
										break;
									default:
									   break;
								}

								// condition type

								switch (buffer[l++])
								{
									case GREATER:
										APP_LOG(TS_ON, VLEVEL_L, "code: GREATER\r\n");
										humidity_storage.aggConditions[j].condition = GREATER;
										break;
									case GREATER_THAN_EQUAL:
										APP_LOG(TS_ON, VLEVEL_L, "code: GREATER_THAN_EQUAL\r\n");
										humidity_storage.aggConditions[j].condition = GREATER_THAN_EQUAL;
										break;
									case LESS:
										APP_LOG(TS_ON, VLEVEL_L, "code: LESS\r\n");
										humidity_storage.aggConditions[j].condition = LESS;
										break;
									case LESS_THAN_EQUAL:
										APP_LOG(TS_ON, VLEVEL_L, "code: LESS_THAN_EQUAL\r\n");
										humidity_storage.aggConditions[j].condition = LESS_THAN_EQUAL;
										break;
									default:
										break;
								}

								humidity_storage.aggConditions[j].value = (uint16_t)((buffer[l++] << 8) | buffer[l++]);
								APP_LOG(TS_ON, VLEVEL_L, "value: %d\r\n", humidity_storage.aggConditions[j].value);
							}
						  }

						  // END CONDITION

						  humidity_storage.AggStateOn = 1;
    			  		  break;
    			  }
    		  }
    	      status = 1;

      	      APP_LOG(TS_ON, VLEVEL_L, "DATA PARSED CORRECTLY\r\n", (AppLedStateOn));
    	  }
    	  else
    	  {
    		  status = 0;
          	  APP_LOG(TS_ON, VLEVEL_L, "ERROR WHILE PARSING DATA\r\n", (AppLedStateOn));
    	  }
  		  start = clock();
    	  // Sending "Success" message via uplink
    	  AppData.Port = LORAWAN_USER_APP_PORT;
    	  AppData.Buffer[i++] = (uint8_t)(status & 0xFF);
    	  AppData.BufferSize = i;

    	  if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
    	  {
    	  	APP_LOG(TS_ON, VLEVEL_L, "%d\r\n", (AppData.Buffer));
    		end = clock();
    	    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST %d\r\n", (end-start)/CLOCKS_PER_SEC);
    	  }

    	  break;

      default:

        break;
    }
  }
  APP_LOG(TS_OFF, VLEVEL_L, "###### ======================================\r\n\r\n");
  /* USER CODE END OnRxData_1 */
}

static void CollectMeasurements(void)
{
	uint16_t pressure = 0;
	uint16_t temperature = 0;
	uint16_t humidity = 0;

	sensor_t sensor_data;
	int8_t i = 0;

	int8_t collectTemperature = 0;
	int8_t collectPressure = 0;
	int8_t collectHumidity = 0;

	APP_LOG(TS_ON, VLEVEL_L, "pressure state: %d\r\n", (PressDataOn));
	APP_LOG(TS_ON, VLEVEL_L, "temperature state: %d\r\n", (TempDataOn));
	APP_LOG(TS_ON, VLEVEL_L, "humidity state: %d\r\n", (HumDataOn));

	if (HumDataOn == 1 || TempDataOn == 1 || PressDataOn == 1)
	{
		APP_LOG(TS_OFF, VLEVEL_L, "\r\n###### ============ COLLECTING MEASUREMENS ============\r\n");
		EnvSensors_Read(&sensor_data);
		temperature = (SYS_GetTemperatureLevel() >> 8);
		pressure    = (uint16_t)(sensor_data.pressure);  /* in hPa */
		humidity    = (uint16_t)(sensor_data.humidity);            /* in %    */
		APP_LOG(TS_ON, VLEVEL_L, "pressure before condition: %d\r\n", (pressure));
		APP_LOG(TS_ON, VLEVEL_L, "temperature before condition: %d\r\n", (temperature));
		APP_LOG(TS_ON, VLEVEL_L, "humidity before condition: %d\r\n", (humidity));

		for (i = 0; i < temperature_storage.condSize; i++) {
			APP_LOG(TS_ON, VLEVEL_L, "READING CONDTION INSIDE COLLECTING MEASUREMENTS\r\n");
			switch (temperature_storage.aggConditions[i].condition){
				case GREATER:
					if (temperature_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature > temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == PRESSURE) {
						if (temperature  > temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity > temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					break;
				case GREATER_THAN_EQUAL:
					if (temperature_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature >= temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure >= temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity >= temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					break;
				case LESS:
					if (temperature_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature < temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure < temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity < temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					break;
				case LESS_THAN_EQUAL:
					if (temperature_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature <= temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure <= temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					else if (temperature_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity <= temperature_storage.aggConditions[i].value) {
							collectTemperature++;
						}
					}
					break;
			}
			APP_LOG(TS_ON, VLEVEL_L, "END READING CONDTION INSIDE COLLECTING MEASUREMENTS\r\n");
		}

		for (i = 0; i < pressure_storage.condSize; i++) {
			switch (pressure_storage.aggConditions[i].condition){
				case GREATER:
					if (pressure_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature > pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure > pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity > pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					break;
				case GREATER_THAN_EQUAL:
					if (pressure_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature >= pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure >= pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity >= pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					break;
				case LESS:
					if (pressure_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature < pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure < pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity < pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					break;
				case LESS_THAN_EQUAL:
					if (pressure_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature <= pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure <= pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					else if (pressure_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity <= pressure_storage.aggConditions[i].value) {
							collectPressure++;
						}
					}
					break;
			}
		}

		for (i = 0; i < humidity_storage.condSize; i++) {
			switch (humidity_storage.aggConditions[i].condition){
				case GREATER:
					if (humidity_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature > humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure > humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity > humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					break;
				case GREATER_THAN_EQUAL:
					if (humidity_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature >= humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure >= humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity >= humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					break;
				case LESS:
					if (humidity_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature < humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure < humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity < humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					break;
				case LESS_THAN_EQUAL:
					if (humidity_storage.aggConditions[i].parameter == TEMPERATURE) {
						if (temperature <= humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == PRESSURE) {
						if (pressure <= humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					else if (humidity_storage.aggConditions[i].parameter == HUMIDITY) {
						if (humidity <= humidity_storage.aggConditions[i].value) {
							collectHumidity++;
						}
					}
					break;
			}
		}
		APP_LOG(TS_ON, VLEVEL_L, "DATA GATHERING\r\n");

		if (TempDataOn == 1 && ((temperature_storage.condSel == AND && collectTemperature == temperature_storage.condSize) || (temperature_storage.condSel == OR && collectTemperature > 0) || (temperature_storage.condSize == 0)) )
		{
			APP_LOG(TS_ON, VLEVEL_L, "temperature: %d\r\n", (temperature));
			temperature_storage.collectedData.count = (temperature_storage.collectedData.count) + 1;
			temperature_storage.collectedData.sum = temperature_storage.collectedData.sum + temperature;
			temperature_storage.collectedData.sqSum = temperature_storage.collectedData.sqSum + (temperature * temperature);
			if (temperature_storage.collectedData.min > temperature || temperature_storage.collectedData.sum == temperature)
			{
				temperature_storage.collectedData.min = temperature;
			}
			else if (temperature_storage.collectedData.max < temperature)
			{
				temperature_storage.collectedData.max = temperature;
			}
		}
		APP_LOG(TS_ON, VLEVEL_L, "END DATA GATHERING\r\n");

		if (PressDataOn == 1 && ((pressure_storage.condSel == AND && collectPressure == pressure_storage.condSize) || (pressure_storage.condSel == OR && collectPressure > 0) || (pressure_storage.condSize == 0)))
		{
			APP_LOG(TS_ON, VLEVEL_L, "pressure: %d\r\n", (pressure));
			pressure_storage.collectedData.count = (pressure_storage.collectedData.count) + 1;
			pressure_storage.collectedData.sum = pressure_storage.collectedData.sum + pressure;
			pressure_storage.collectedData.sqSum = pressure_storage.collectedData.sqSum + (pressure * pressure);
			if (pressure_storage.collectedData.min > pressure || pressure_storage.collectedData.sum == pressure)
			{
				pressure_storage.collectedData.min = pressure;
			}
			else if (pressure_storage.collectedData.max < pressure)
			{
				pressure_storage.collectedData.max = pressure;
			}
		}

		if (HumDataOn == 1 && ((humidity_storage.condSel == AND && collectHumidity == humidity_storage.condSize) || (humidity_storage.condSel == OR && collectHumidity > 0)  || (humidity_storage.condSize == 0)))
		{
			APP_LOG(TS_ON, VLEVEL_L, "humidity: %d\r\n", (humidity));
			humidity_storage.collectedData.count = humidity_storage.collectedData.count + 1;
			humidity_storage.collectedData.sum = humidity_storage.collectedData.sum + humidity;
			humidity_storage.collectedData.sqSum = humidity_storage.collectedData.sqSum + (humidity * humidity);
			if (humidity_storage.collectedData.min > humidity || humidity_storage.collectedData.sum == humidity)
			{
				humidity_storage.collectedData.min = humidity;
			}
			else if (humidity_storage.collectedData.max < humidity)
			{
				humidity_storage.collectedData.max = humidity;
			}
		}

	}
	/* Collect measurements */
}

static void SendTemperatureMeasurements(void)
{
	int16_t temperature_max = 0;
	int16_t temperature_min = 0;
	int16_t temperature_avg = 0;
	int16_t temperature_count = 0;
	int16_t temperature_sum = 0;
	uint32_t temperature_sqSum = 0;

	sensor_t sensor_data;
	UTIL_TIMER_Time_t nextTxIn = 0;

	uint32_t i = 0;
	int32_t latitude = 0;
	int32_t longitude = 0;
	uint8_t battery = 0;
	uint16_t altitudeGps = 0;
	EnvSensors_Read(&sensor_data);

	if (temperature_storage.AggStateOn == 1 && TempDataOn == 1) {
		APP_LOG(TS_OFF, VLEVEL_L, "\r\n###### ============ SENDING TEMPERATURE TX DATA ============\r\n");

		if (temperature_storage.AggMaxOn == 1) {
			temperature_max = temperature_storage.collectedData.max;
		}
   	    if (temperature_storage.AggMinOn == 1) {
		   temperature_min = temperature_storage.collectedData.min;
		}
		if (temperature_storage.AggAvgOn == 1) {
		   temperature_avg = temperature_storage.collectedData.sum / temperature_storage.collectedData.count;
		}
		if (temperature_storage.AggSumOn == 1) {
		   temperature_sum = temperature_storage.collectedData.sum;
		}
		if (temperature_storage.AggCountOn == 1) {
		   temperature_count = temperature_storage.collectedData.count;
		}
		if (temperature_storage.AggSqSumOn == 1) {
		   temperature_sqSum = temperature_storage.collectedData.sqSum;
		}

		AppData.Port = LORAWAN_USER_APP_PORT;

		AppData.Buffer[i++] = DATA_AGG_ON;
		AppData.Buffer[i++] = TEMPERATURE;
		AppData.Buffer[i++] = temperature_storage.measurementId;
		AppData.Buffer[i++] = temperature_storage.AggMaxOn;
		AppData.Buffer[i++] = temperature_storage.AggMinOn;
		AppData.Buffer[i++] = temperature_storage.AggAvgOn;
		AppData.Buffer[i++] = temperature_storage.AggSumOn;
		AppData.Buffer[i++] = temperature_storage.AggCountOn;
		AppData.Buffer[i++] = temperature_storage.AggSqSumOn;

		if (temperature_storage.AggMaxOn == 1) {
			AppData.Buffer[i++] = (uint8_t)(temperature_max & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "temperature max: %d\r\n", (temperature_max));
		}
		if (temperature_storage.AggMinOn == 1) {
			AppData.Buffer[i++] = (uint8_t)(temperature_min & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "temperature min: %d\r\n", (temperature_min));
		}
		if (temperature_storage.AggAvgOn == 1) {
			AppData.Buffer[i++] = (uint8_t)(temperature_avg & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "temperature avg: %d\r\n", (temperature_avg));
		}
		if (temperature_storage.AggSumOn == 1) {
			AppData.Buffer[i++] = (uint8_t)(temperature_sum & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "temperature sum: %d\r\n", (temperature_sum));
		}
		if (temperature_storage.AggCountOn == 1) {
			AppData.Buffer[i++] = (uint8_t)(temperature_count & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "temperature count: %d\r\n", (temperature_count));
		}
		if (temperature_storage.AggSqSumOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((temperature_sqSum >> 24) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((temperature_sqSum >> 16) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((temperature_sqSum >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(temperature_sqSum & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "temperature sqsum: %d\r\n", (temperature_sqSum));
		}

		latitude = sensor_data.latitude;
		longitude = sensor_data.longitude;
		battery = GetBatteryLevel();

		AppData.Buffer[i++] = battery;      /* 1 (very low) to 254 (fully charged) */
		AppData.Buffer[i++] = (uint8_t)((latitude >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((latitude >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(latitude & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((longitude >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((longitude >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(longitude & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((altitudeGps >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(altitudeGps & 0xFF);
		APP_LOG(TS_ON, VLEVEL_L, "battery level: %d\r\n", (battery));
		APP_LOG(TS_ON, VLEVEL_L, "latitude: %d\r\n", (latitude));
		APP_LOG(TS_ON, VLEVEL_L, "longitude: %d\r\n", (longitude));
		APP_LOG(TS_ON, VLEVEL_L, "altitude: %d\r\n", (altitudeGps));
		//  }

		AppData.BufferSize = i;

		if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
		{
			APP_LOG(TS_ON, VLEVEL_L, "%d\r\n", (AppData.Buffer));
		    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
		}
		else if (nextTxIn > 0)
		{
			APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
		}

		// CLEAN UP

		temperature_storage.measurementId++;
		temperature_storage.collectedData.max = 0;
		temperature_storage.collectedData.min = 0;
		temperature_storage.collectedData.sum = 0;
		temperature_storage.collectedData.count = 0;
		temperature_storage.collectedData.sqSum = 0;
	}

}

static void SendPressureMeasurements(void)
{
	uint16_t pressure = 0;
	uint16_t pressure_max = 0;
	uint16_t pressure_min = 0;
	uint16_t pressure_sum = 0;
	uint32_t pressure_sqSum = 0;
	uint16_t pressure_avg = 0;
	int16_t pressure_count = 0;

	sensor_t sensor_data;
	UTIL_TIMER_Time_t nextTxIn = 0;

	uint32_t i = 0;
	int32_t latitude = 0;
	int32_t longitude = 0;
	uint8_t battery = 0;
	uint16_t altitudeGps = 0;
	EnvSensors_Read(&sensor_data);

	if (pressure_storage.AggStateOn == 1 && PressDataOn == 1) {
		APP_LOG(TS_OFF, VLEVEL_L, "\r\n###### ============ SENDING PRESSURE TX DATA ============\r\n");

		if (pressure_storage.AggMaxOn == 1) {
			pressure_max = pressure_storage.collectedData.max;
		}
	   	if (pressure_storage.AggMinOn == 1) {
	       	pressure_min = pressure_storage.collectedData.min;
		}
		if (pressure_storage.AggAvgOn == 1) {
			pressure_avg = pressure_storage.collectedData.sum / pressure_storage.collectedData.count;
		}
		if (pressure_storage.AggSumOn == 1) {
			pressure_sum = pressure_storage.collectedData.sum;
		}
		if (pressure_storage.AggCountOn == 1) {
			pressure_count = pressure_storage.collectedData.count;
		}
		if (pressure_storage.AggSqSumOn == 1) {
			pressure_sqSum = pressure_storage.collectedData.sqSum;
		}

		AppData.Port = LORAWAN_USER_APP_PORT;

		AppData.Buffer[i++] = DATA_AGG_ON;
		AppData.Buffer[i++] = PRESSURE;
		AppData.Buffer[i++] = pressure_storage.measurementId;
		AppData.Buffer[i++] = pressure_storage.AggMaxOn;
		AppData.Buffer[i++] = pressure_storage.AggMinOn;
		AppData.Buffer[i++] = pressure_storage.AggAvgOn;
		AppData.Buffer[i++] = pressure_storage.AggSumOn;
		AppData.Buffer[i++] = pressure_storage.AggCountOn;
		AppData.Buffer[i++] = pressure_storage.AggSqSumOn;

		if (pressure_storage.AggMaxOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((pressure_max >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(pressure_max & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "pressure max: %d\r\n", (pressure_max));
		}
		if (pressure_storage.AggMinOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((pressure_min >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(pressure_min & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "pressure min: %d\r\n", (pressure_min));
		}
		if (pressure_storage.AggAvgOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((pressure_avg >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(pressure_avg & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "pressure avg: %d\r\n", (pressure_avg));
		}
		if (pressure_storage.AggSumOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((pressure_sum >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(pressure_sum & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "pressure sum: %d\r\n", (pressure_sum));
		}
		if (pressure_storage.AggCountOn == 1) {
			AppData.Buffer[i++] = (uint8_t)(pressure_count & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "pressure count: %d\r\n", (pressure_count));
		}
		if (pressure_storage.AggSqSumOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((pressure_sqSum >> 24) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((pressure_sqSum >> 16) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((pressure_sqSum >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(pressure_sqSum & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "pressure sqsum: %d\r\n", (pressure_sqSum));
		}

		latitude = sensor_data.latitude;
		longitude = sensor_data.longitude;
		battery = GetBatteryLevel();

		AppData.Buffer[i++] = battery;      /* 1 (very low) to 254 (fully charged) */
		AppData.Buffer[i++] = (uint8_t)((latitude >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((latitude >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(latitude & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((longitude >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((longitude >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(longitude & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((altitudeGps >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(altitudeGps & 0xFF);
		APP_LOG(TS_ON, VLEVEL_L, "battery level: %d\r\n", (battery));
		APP_LOG(TS_ON, VLEVEL_L, "latitude: %d\r\n", (latitude));
		APP_LOG(TS_ON, VLEVEL_L, "longitude: %d\r\n", (longitude));
		APP_LOG(TS_ON, VLEVEL_L, "altitude: %d\r\n", (altitudeGps));
		//  }

		AppData.BufferSize = i;
		//#endif /* CAYENNE_LPP */

		if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
		{
			APP_LOG(TS_ON, VLEVEL_L, "%d\r\n", (AppData.Buffer));
		    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
		}
		else if (nextTxIn > 0)
		{
			APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
		}

		// CLEAN UP

		pressure_storage.measurementId++;
		pressure_storage.collectedData.max = 0;
		pressure_storage.collectedData.min = 0;
		pressure_storage.collectedData.sum = 0;
		pressure_storage.collectedData.count = 0;
		pressure_storage.collectedData.sqSum = 0;
	}
}

static void SendHumidityMeasurements(void)
{
	uint16_t humidity = 0;
	uint16_t humidity_max = 0;
	uint16_t humidity_min = 0;
	uint16_t humidity_sum = 0;
	uint32_t humidity_sqSum = 0;
	uint16_t humidity_avg = 0;
	int16_t humidity_count = 0;

	sensor_t sensor_data;
	UTIL_TIMER_Time_t nextTxIn = 0;

	uint32_t i = 0;
	int32_t latitude = 0;
	int32_t longitude = 0;
	uint8_t battery = 0;
	uint16_t altitudeGps = 0;
	EnvSensors_Read(&sensor_data);

	if (humidity_storage.AggStateOn == 1 && HumDataOn == 1) {
		APP_LOG(TS_OFF, VLEVEL_L, "\r\n###### ============ SENDING HUMIDITY TX DATA ============\r\n");

		if (humidity_storage.AggMaxOn == 1) {
			humidity_max = humidity_storage.collectedData.max;
		}
	   	if (humidity_storage.AggMinOn == 1) {
	   		humidity_min = humidity_storage.collectedData.min;
		}
		if (humidity_storage.AggAvgOn == 1) {
			humidity_avg = humidity_storage.collectedData.sum / humidity_storage.collectedData.count;
		}
		if (humidity_storage.AggSumOn == 1) {
			humidity_sum = humidity_storage.collectedData.sum;
		}
		if (humidity_storage.AggCountOn == 1) {
			humidity_count = humidity_storage.collectedData.count;
		}
		if (humidity_storage.AggSqSumOn == 1) {
			humidity_sqSum = humidity_storage.collectedData.sqSum;
		}

		AppData.Port = LORAWAN_USER_APP_PORT;

		AppData.Buffer[i++] = DATA_AGG_ON;
		AppData.Buffer[i++] = HUMIDITY;
		AppData.Buffer[i++] = humidity_storage.measurementId;
		AppData.Buffer[i++] = humidity_storage.AggMaxOn;
		AppData.Buffer[i++] = humidity_storage.AggMinOn;
		AppData.Buffer[i++] = humidity_storage.AggAvgOn;
		AppData.Buffer[i++] = humidity_storage.AggSumOn;
		AppData.Buffer[i++] = humidity_storage.AggCountOn;
		AppData.Buffer[i++] = humidity_storage.AggSqSumOn;

		if (humidity_storage.AggMaxOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((humidity_max >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(humidity_max & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "humidity max: %d\r\n", (humidity_max));
		}
		if (humidity_storage.AggMinOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((humidity_min >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(humidity_min & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "humidity min: %d\r\n", (humidity_min));
		}
		if (humidity_storage.AggAvgOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((humidity_avg >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(humidity_avg & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "humidity avg: %d\r\n", (humidity_avg));
		}
		if (humidity_storage.AggSumOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((humidity_sum >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(humidity_sum & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "humidity sum: %d\r\n", (humidity_sum));
		}
		if (humidity_storage.AggCountOn == 1) {
			AppData.Buffer[i++] = (uint8_t)(humidity_count & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "humidity count: %d\r\n", (humidity_count));
		}
		if (humidity_storage.AggSqSumOn == 1) {
			AppData.Buffer[i++] = (uint8_t)((humidity_sqSum >> 24) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((humidity_sqSum >> 16) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((humidity_sqSum >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(humidity_sqSum & 0xFF);
			APP_LOG(TS_ON, VLEVEL_L, "humidity sqsum: %d\r\n", (humidity_sqSum));
		}

		latitude = sensor_data.latitude;
		longitude = sensor_data.longitude;
		battery = GetBatteryLevel();

		AppData.Buffer[i++] = battery;      /* 1 (very low) to 254 (fully charged) */
		AppData.Buffer[i++] = (uint8_t)((latitude >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((latitude >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(latitude & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((longitude >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((longitude >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(longitude & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((altitudeGps >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)(altitudeGps & 0xFF);
		APP_LOG(TS_ON, VLEVEL_L, "battery level: %d\r\n", (battery));
		APP_LOG(TS_ON, VLEVEL_L, "latitude: %d\r\n", (latitude));
		APP_LOG(TS_ON, VLEVEL_L, "longitude: %d\r\n", (longitude));
		APP_LOG(TS_ON, VLEVEL_L, "altitude: %d\r\n", (altitudeGps));
		//  }

		AppData.BufferSize = i;
		//#endif /* CAYENNE_LPP */

		if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
		{
			APP_LOG(TS_ON, VLEVEL_L, "%d\r\n", (AppData.Buffer));
		    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
		}
		else if (nextTxIn > 0)
		{
			APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
		}

		// CLEAN UP

		humidity_storage.measurementId++;
		humidity_storage.collectedData.max = 0;
		humidity_storage.collectedData.min = 0;
		humidity_storage.collectedData.sum = 0;
		humidity_storage.collectedData.count = 0;
		humidity_storage.collectedData.sqSum = 0;
	}

}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */

  sensor_t sensor_data;
  UTIL_TIMER_Time_t nextTxIn = 0;

  uint32_t i = 0;
  int32_t latitude = 0;
  int32_t longitude = 0;
  uint8_t battery = 0;
  uint16_t altitudeGps = 0;

  EnvSensors_Read(&sensor_data);

  uint16_t temperature = (SYS_GetTemperatureLevel() >> 8);
  uint16_t pressure    = (uint16_t)(sensor_data.pressure);      /* in hPa / 10 */
  uint16_t humidity    = (uint16_t)(sensor_data.humidity);

  if (HumDataOn == 0 && TempDataOn == 0 && PressDataOn == 0) {

	  APP_LOG(TS_OFF, VLEVEL_L, "\r\n###### ============ SENDING TX DATA ============\r\n");

  	  AppData.Port = LORAWAN_USER_APP_PORT;

  	  AppData.Buffer[i++] = DATA_AGG_OFF;
  	  AppData.Buffer[i++] = (uint8_t)((pressure >> 8) & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)(pressure & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)(temperature & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)((humidity >> 8) & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)(humidity & 0xFF);
  	  APP_LOG(TS_ON, VLEVEL_L, "humidity: %d\r\n", (humidity));
  	  APP_LOG(TS_ON, VLEVEL_L, "temperature: %d\r\n", (temperature));
  	  APP_LOG(TS_ON, VLEVEL_L, "pressure: %d\r\n", (pressure));

  	  APP_LOG(TS_ON, VLEVEL_L, "AppLedStateOn: %d\r\n", (AppLedStateOn));

  	  latitude = sensor_data.latitude;
  	  longitude = sensor_data.longitude;
  	  battery = GetBatteryLevel();

  	  AppData.Buffer[i++] = battery;      /* 1 (very low) to 254 (fully charged) */
  	  AppData.Buffer[i++] = (uint8_t)((latitude >> 16) & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)((latitude >> 8) & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)(latitude & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)((longitude >> 16) & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)((longitude >> 8) & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)(longitude & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)((altitudeGps >> 8) & 0xFF);
  	  AppData.Buffer[i++] = (uint8_t)(altitudeGps & 0xFF);
  	  APP_LOG(TS_ON, VLEVEL_L, "battery level: %d\r\n", (battery));
  	  APP_LOG(TS_ON, VLEVEL_L, "latitude: %d\r\n", (latitude));
  	  APP_LOG(TS_ON, VLEVEL_L, "longitude: %d\r\n", (longitude));
  	  APP_LOG(TS_ON, VLEVEL_L, "altitude: %d\r\n", (altitudeGps));

  	  AppData.BufferSize = i;

  	  if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
  	  {
	  	  APP_LOG(TS_ON, VLEVEL_L, "%d\r\n", (AppData.Buffer));
	  	  APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
  	  }
  	  else if (nextTxIn > 0)
  	  {
	  	  APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
  	  }
  }

  /* USER CODE END SendTxData_1 */
}

static void OnMeasureDataTimerEvent(void *context)
{
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_CollectMeasurmentsEvent), CFG_SEQ_Prio_0);

	/*Wait for next tx slot*/
	UTIL_TIMER_Start(&DataMeasureTimer);
}

static void OnMeasureTemperatureTimerEvent(void *context)
{
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTemperatureTimerEvent), CFG_SEQ_Prio_0);

	/*Wait for next tx slot*/
	UTIL_TIMER_Start(&TxTemperatureMeasureTimer);
}

static void OnMeasurePressureTimerEvent(void *context)
{
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnPressureTimerEvent), CFG_SEQ_Prio_0);

	/*Wait for next tx slot*/
	UTIL_TIMER_Start(&TxPressureMeasureTimer);
}

static void OnMeasureHumidityTimerEvent(void *context)
{
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnHumidityTimerEvent), CFG_SEQ_Prio_0);

	/*Wait for next tx slot*/
	UTIL_TIMER_Start(&TxHumidityMeasureTimer);
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */
static void OnTxTimerLedEvent(void *context)
{
  LED_Off(LED_RED2);
}

static void OnRxTimerLedEvent(void *context)
{
  LED_Off(LED_BLUE) ;
}

static void OnJoinTimerLedEvent(void *context)
{
  LED_Toggle(LED_RED1) ;
}

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  if ((params != NULL))
  {
    /* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
    if (params->IsMcpsConfirm != 0)
    {
      LED_On(LED_RED2) ;
      UTIL_TIMER_Start(&TxLedTimer);

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
      APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
              params->AppData.Port, params->Datarate, params->TxPower);

      APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
      if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
      {
        APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
      }
    }
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
      UTIL_TIMER_Stop(&JoinLedTimer);

      LED_Off(LED_RED1) ;

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
      }
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }
  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
