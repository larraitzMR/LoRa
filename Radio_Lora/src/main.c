/*

 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "delay.h"
#include "low_power.h"
#include "vcom.h"
#include "lora.h"
#include "version.h"
#include "gps.h"
#include "display.h"

/** Lora **/
/*!
 * CAYENNE_LPP is myDevices Application server.
 */
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73

#define LPP_APP_PORT 99

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            10000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG                    DISABLE
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa will transmit a frame*/
static void LoraTxData(lora_AppData_t *AppData, FunctionalState* IsTxConfirmed);

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
		HW_GetUniqueId, HW_GetRandomSeed, LoraTxData, LoraRxData };

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent(void);
#endif
/* !
 *Initialises the Lora Parameters
 */
static LoRaParam_t LoRaParamInit = { TX_ON_TIMER,
APP_TX_DUTYCYCLE, CLASS_A,
LORAWAN_ADR_ON,
DR_0,
LORAWAN_PUBLIC_NETWORK };

/**
 * Main application entry point.
 */

extern UART_HandleTypeDef huart1;
__IO ITStatus UartReady = RESET;

#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#else
#error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
#error "Please define a modem in the compiler options."
#endif

typedef enum {
	LOWPOWER, RX, RX_TIMEOUT, RX_ERROR, TX, TX_TIMEOUT,
} States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here
#define LED_PERIOD_MS               200

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;


States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/* Led Timers objects*/
static TimerEvent_t timerLed;

/* Private function prototypes -----------------------------------------------*/
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

/*!
 * \brief Function executed on when led timer elapses
 */
static void OnledEvent(void);
/**
 * Main application entry point.
 */

/* SPI handler declaration */
extern SPI_HandleTypeDef hspi2;

/* Buffer used for transmission */
#define BUFFERSIZE                       39
//#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Buffer used for transmission */
uint8_t aTxBuffer[] =" ";

/* Buffer used for reception */
uint8_t buffLora[40];
uint8_t RxReady[5];
uint8_t parsingBuff[BUFFERSIZE];

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";
uint8_t ReadyMsg[] = "READY";
uint8_t OKMsg[2] = "OK";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];


extern UART_HandleTypeDef huart1;
//__IO ITStatus UartReady = RESET;

int recibidoMaster = 0;
int recibidoSlave = 0;
int recibidoReady = 0;
int enviadoReady = 0;
int errorReady = 0;
int transmite = 0;
int estado = 0;
int i = 0;

char hora[8];
char lat[10];
char latC[1];
char lon[10];
char lonC[1];
char buffGPS[40];


struct datosMicro {
	char datos[100];
};

struct datosMicro misDat[150];
int dat = 0;

int main(void) {

	HAL_Init();
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	SystemClock_Config();

	DBG_Init();
	HW_Init();

	LCD_Config();
	LCD_Init();
	LCD_Command(LCD_CLEAR_DISPLAY);

	SPI_Config();
	SPI_Init();

	/* Configure the Lora Stack*/
	lora_Init(&LoRaMainCallbacks, &LoRaParamInit);

	PRINTF("VERSION: %X\n\r", VERSION);

	/* Led Timers*/
	TimerInit(&timerLed, OnledEvent);
	TimerSetValue(&timerLed, LED_PERIOD_MS);

	TimerStart(&timerLed);

//	 Radio initialization
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;

	Radio.Init(&RadioEvents);

	Radio.SetChannel( RF_FREQUENCY);

#if defined( USE_MODEM_LORA )

	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
	LORA_SPREADING_FACTOR, LORA_CODINGRATE,
	LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
	true, 0, 0, LORA_IQ_INVERSION_ON, 3000000);

	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
	LORA_IQ_INVERSION_ON, true);
#endif

	//Establece la radio en modo de recepcion durante un tiempo
	Radio.Rx( RX_TIMEOUT_VALUE);

	/* Master */
	bool isMaster = true;
	/* Slave */
//	bool isMaster = false;
//	LCD_Cursor(1);
//	LCD_Print_String("abcdefghijklmnopqrstuvwyz");
//	LCD_Print_String("0123456789");

	while (1) {
		if (recibidoReady == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) ReadyMsg, (uint8_t *) RxReady, 5, 3000) == HAL_OK) {
				while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
				}
				PRINTF("%s\r\n", RxReady);
				if (strncmp((const char*) RxReady, (const char*) ReadyMsg, 5) == 0) {
//					Flush_Buffer(RxReady, 5);
					recibidoReady = 1;
					PRINTF("Recibido Ready\r\n");
				}
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		} else if (recibidoReady == 1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) OKMsg, (uint8_t *) parsingBuff, 40, 3000) == HAL_OK) {
				while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
				}
				PRINTF("%s\r\n", parsingBuff);
				strcpy(misDat[i].datos, parsingBuff);

				if (strncmp((const char*) parsingBuff, (const char*) "GPS", 3)	== 0) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
					if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) parsingBuff,(uint8_t *) OKMsg, 40, 3000) == HAL_OK) {
						while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
						}
					}
				}
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
		switch (State) {
		case RX:
			if (isMaster == true) {
				if (BufferSize > 0) {
//					PRINTF(" Master: %s\r\n", Buffer);
					if (strncmp((const char*) Buffer, (const char*) ReadyMsg, 5) == 0) {
						enviadoReady = 1;
						TimerStop(&timerLed);
						LED_Off(LED_BLUE);
						LED_Off(LED_GREEN);
						LED_Off(LED_RED1);
						// Indicates on a LED that the received frame is a PONG
						LED_Toggle(LED_RED2);

						DelayMs(1);
						Radio.Send(misDat[i].datos, BUFFERSIZE);
						Radio.Rx( RX_TIMEOUT_VALUE);
						recibidoMaster = 1;
						errorReady = 1;
					}
					if ((recibidoMaster == 1)
							&& (strncmp((const char*) Buffer,
									(const char*) OKMsg, 2) == 0)) {
						DelayMs(1);
						Radio.Send(misDat[i].datos, BUFFERSIZE);
						Radio.Rx( RX_TIMEOUT_VALUE);
//						PRINTF("Enviando LAR\r\n");
					}
					Radio.Rx( RX_TIMEOUT_VALUE);
					memset(Buffer, '\0', BUFFER_SIZE);
				}
			} else {
				if (BufferSize > 0) {
					PRINTF("%s\r\n", Buffer);
//					LCD_Cursor(1);
//					LCD_Print_String("abcdefghijklmnopqrstuvwyz");
//					LCD_Print_String("0123456789");
//					strcpy(buffGPS, Buffer);
//					LCD_Print(buffGPS);
					if (strncmp((const char*) Buffer, (const char*) ReadyMsg, 5)
							== 0) {
						// Indicates on a LED that the received frame is a PING
						LCD_Command(LCD_CLEAR_DISPLAY);
						TimerStop(&timerLed);
						LED_Off(LED_RED1);
						LED_Off(LED_RED2);
						LED_Off(LED_GREEN);
						LED_Toggle(LED_BLUE);
						Radio.Send(ReadyMsg, 5);
						PRINTF("Slave Ready\r\n");
						recibidoSlave = 1;
						Radio.Rx( RX_TIMEOUT_VALUE);
						LCD_Cursor(1);
						strcpy(buffGPS, Buffer);
						LCD_Print_String(buffGPS);
					}
					if ((recibidoSlave == 1) && (strncmp((const char*) Buffer,(const char*) "\nGPS", 4) == 0)) {
						LCD_Command(LCD_CLEAR_DISPLAY);
						Radio.Send(OKMsg, 2);
						memcpy(hora, &Buffer[5], 8 );
						memcpy(lat, &Buffer[14], 10 );
						memcpy(latC, &Buffer[25], 1 );
						memcpy(lon, &Buffer[27], 10 );
						memcpy(lonC, &Buffer[38], 1 );
						LCD_Cursor(1);
						LCD_Print_String(hora);
						LCD_Cursor(2);
						LCD_Print_String(lat);
						LCD_Print_String(latC);
						LCD_Cursor(3);
						LCD_Print_String(lon);
						LCD_Print_String(lonC);
						//						PRINTF("OK\r\n");
						Radio.Rx( RX_TIMEOUT_VALUE);
						//						strcpy(buffGPS, Buffer);
						//						memcpy(hora, &Buffer[5], 8 );
						//						LCD_Print(buffGPS);

					}
					Radio.Rx( RX_TIMEOUT_VALUE);
					memset(Buffer, '\0', BUFFER_SIZE);
				}
			}
			State = LOWPOWER;
			break;
		case TX:
			Radio.Rx( RX_TIMEOUT_VALUE);
			State = LOWPOWER;
			break;
		case RX_TIMEOUT:
		case RX_ERROR:
			if (isMaster == true) {
				if (enviadoReady == 0) {
					Radio.Send(ReadyMsg, 5);
					PRINTF("Master Ready\r\n");
				}
				if (errorReady == 1) {
					Radio.Send(misDat[i].datos, BUFFERSIZE);
				}
				// Send the next PING frame
				DelayMs(1);
				Radio.Rx( RX_TIMEOUT_VALUE);
			} else {
				Radio.Rx( RX_TIMEOUT_VALUE);
			}
			State = LOWPOWER;
			break;
		case TX_TIMEOUT:
			Radio.Rx( RX_TIMEOUT_VALUE);
			break;
		case LOWPOWER:
		default:
			// Set low power
			break;
		}
		i++;
		if (i == 149){
			i = 0;
		}

		DISABLE_IRQ( );
		/* if an interupt has occured after __disable_irq, it is kept pending
		 * and cortex will not enter low power anyway  */
		if (State == LOWPOWER) {
#ifndef LOW_POWER_DISABLE
			LowPower_Handler();
#endif
		}
		ENABLE_IRQ( );
	}
}

//La llamada  PRINTF("txDone\n\r"); esta en sx1276.c, funcion SX1276OnDio0Irq
void OnTxDone(void) {
	Radio.Sleep();
	State = TX;
}

//La llamada PRINTF("rxDone\n\r"); esta en sx1276.c, funcion SX1276OnDio0Irq
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
	Radio.Sleep();
	BufferSize = size;
	memcpy(Buffer, payload, BufferSize);
	RssiValue = rssi;
	SnrValue = snr;
	State = RX;
}

void OnTxTimeout(void) {
	Radio.Sleep();
	State = TX_TIMEOUT;

//	PRINTF("OnTxTimeout\n");
}

void OnRxTimeout(void) {
	Radio.Sleep();
	State = RX_TIMEOUT;
//	PRINTF("OnRxTimeout\n");
}

void OnRxError(void) {
	Radio.Sleep();
	State = RX_ERROR;
//	PRINTF("OnRxError\n");
}

static void OnledEvent(void) {
	LED_Toggle(LED_BLUE);
	LED_Toggle(LED_RED1);
	LED_Toggle(LED_RED2);
	LED_Toggle(LED_GREEN);

	TimerStart(&timerLed);
}

static void LoraTxData(lora_AppData_t *AppData, FunctionalState* IsTxConfirmed) {
	/* USER CODE BEGIN 3 */
	uint16_t pressure = 0;
	int16_t temperature = 0;
	uint16_t humidity = 0;
	uint8_t batteryLevel;
//  sensor_t sensor_data;

#ifdef USE_B_L072Z_LRWAN1
	TimerInit(&TxLedTimer, OnTimerLedEvent);

	TimerSetValue(&TxLedTimer, 200);

	LED_On(LED_RED1);

	TimerStart(&TxLedTimer);
#endif
#ifndef CAYENNE_LPP
	int32_t latitude, longitude = 0;
	uint16_t altitudeGps = 0;
#endif
//  BSP_sensor_Read( &sensor_data );

#ifdef CAYENNE_LPP
	uint8_t cchannel = 0;
//  temperature = ( int16_t )( sensor_data.temperature * 10 );     /* in °C * 10 */
//  pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
//  humidity    = ( uint16_t )( sensor_data.humidity * 2 );        /* in %*2     */
	uint32_t i = 0;

	batteryLevel = HW_GetBatteryLevel(); /* 1 (very low) to 254 (fully charged) */

	AppData->Port = LPP_APP_PORT;

	*IsTxConfirmed = LORAWAN_CONFIRMED_MSG;
	AppData->Buff[i++] = cchannel++;
	AppData->Buff[i++] = LPP_DATATYPE_BAROMETER;
	AppData->Buff[i++] = (pressure >> 8) & 0xFF;
	AppData->Buff[i++] = pressure & 0xFF;
	AppData->Buff[i++] = cchannel++;
	AppData->Buff[i++] = LPP_DATATYPE_TEMPERATURE;
	AppData->Buff[i++] = (temperature >> 8) & 0xFF;
	AppData->Buff[i++] = temperature & 0xFF;
	AppData->Buff[i++] = cchannel++;
	AppData->Buff[i++] = LPP_DATATYPE_HUMIDITY;
	AppData->Buff[i++] = humidity & 0xFF;
	AppData->Buff[i++] = cchannel++;
	AppData->Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT;
	AppData->Buff[i++] = batteryLevel * 100 / 254;
	AppData->Buff[i++] = cchannel++;
	AppData->Buff[i++] = LPP_DATATYPE_DIGITAL_OUTPUT;
	AppData->Buff[i++] = AppLedStateOn;
#else
	temperature = ( int16_t )( sensor_data.temperature * 100 ); /* in °C * 100 */
	pressure = ( uint16_t )( sensor_data.pressure * 100 / 10 ); /* in hPa / 10 */
	humidity = ( uint16_t )( sensor_data.humidity * 10 ); /* in %*10     */
	latitude = sensor_data.latitude;
	longitude= sensor_data.longitude;
	uint32_t i = 0;

	batteryLevel = HW_GetBatteryLevel( ); /* 1 (very low) to 254 (fully charged) */

	AppData->Port = LORAWAN_APP_PORT;

	*IsTxConfirmed = LORAWAN_CONFIRMED_MSG;

#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID )
	AppData->Buff[i++] = AppLedStateOn;
	AppData->Buff[i++] = ( pressure >> 8 ) & 0xFF;
	AppData->Buff[i++] = pressure & 0xFF;
	AppData->Buff[i++] = ( temperature >> 8 ) & 0xFF;
	AppData->Buff[i++] = temperature & 0xFF;
	AppData->Buff[i++] = ( humidity >> 8 ) & 0xFF;
	AppData->Buff[i++] = humidity & 0xFF;
	AppData->Buff[i++] = batteryLevel;
	AppData->Buff[i++] = 0;
	AppData->Buff[i++] = 0;
	AppData->Buff[i++] = 0;
#else
	AppData->Buff[i++] = AppLedStateOn;
	AppData->Buff[i++] = ( pressure >> 8 ) & 0xFF;
	AppData->Buff[i++] = pressure & 0xFF;
	AppData->Buff[i++] = ( temperature >> 8 ) & 0xFF;
	AppData->Buff[i++] = temperature & 0xFF;
	AppData->Buff[i++] = ( humidity >> 8 ) & 0xFF;
	AppData->Buff[i++] = humidity & 0xFF;
	AppData->Buff[i++] = batteryLevel;
	AppData->Buff[i++] = ( latitude >> 16 ) & 0xFF;
	AppData->Buff[i++] = ( latitude >> 8 ) & 0xFF;
	AppData->Buff[i++] = latitude & 0xFF;
	AppData->Buff[i++] = ( longitude >> 16 ) & 0xFF;
	AppData->Buff[i++] = ( longitude >> 8 ) & 0xFF;
	AppData->Buff[i++] = longitude & 0xFF;
	AppData->Buff[i++] = ( altitudeGps >> 8 ) & 0xFF;
	AppData->Buff[i++] = altitudeGps & 0xFF;
#endif
#endif
	AppData->BuffSize = i;

	/* USER CODE END 3 */
}

static void LoraRxData(lora_AppData_t *AppData) {
	/* USER CODE BEGIN 4 */
	switch (AppData->Port) {
	case LORAWAN_APP_PORT:
		if (AppData->BuffSize == 1) {
			AppLedStateOn = AppData->Buff[0] & 0x01;
			if (AppLedStateOn == RESET) {
				PRINTF("LED OFF\n\r");
				LED_Off(LED_BLUE);

			} else {
				PRINTF("LED ON\n\r");
				LED_On(LED_BLUE);
			}
			//GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
		}
		break;
	case LPP_APP_PORT: {
		AppLedStateOn = (AppData->Buff[2] == 100) ? 0x01 : 0x00;
		if (AppLedStateOn == RESET) {
			PRINTF("LED OFF\n\r");
			LED_Off(LED_BLUE);

		} else {
			PRINTF("LED ON\n\r");
			LED_On(LED_BLUE);
		}
		break;
	}
	default:
		break;
	}
	/* USER CODE END 4 */
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent(void) {
	LED_Off(LED_RED1);
}
#endif

