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
	#include "uart.h"

	#if defined( USE_BAND_868 )

	#define RF_FREQUENCY                                868000000 // Hz

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

	#else
		#error "Please define a modem in the compiler options."
	#endif

	typedef enum
	{
		LOWPOWER,
		RX,
		RX_TIMEOUT,
		RX_ERROR,
		TX,
		TX_TIMEOUT,
	}States_t;

	#define RX_TIMEOUT_VALUE                            1000
	#define BUFFER_SIZE                                 64 // Define the payload size here
	uint16_t BufferSize = BUFFER_SIZE;
	uint8_t Buffer[BUFFER_SIZE];

	States_t State = LOWPOWER;

	int8_t RssiValue = 0;
	int8_t SnrValue = 0;

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

	/* SPI handler declaration */
	extern SPI_HandleTypeDef hspi2;

	/* Buffer used for transmission */
	#define BUFFERSIZE                       39
	//#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)
	/* Exported macro ------------------------------------------------------------*/
	#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

	/* Buffer used for reception */
	uint8_t buffLora[40];
	uint8_t RxReady[5];
	uint8_t parsingBuff[BUFFERSIZE];
	uint8_t bufferGPS[80];

	uint8_t ReadyMsg[] = "READY";
	uint8_t OKMsg[] = "OK";

	extern UART_HandleTypeDef huart1;

	int recibidoMaster = 0;
	int recibidoSlave = 0;
	int recibidoReady = 0;
	int enviadoReady = 0;
	int errorReady = 0;
	int transmite = 0;
	int estado = 0;
	int i = 0;

	int IDMaster;
	char IDMasterLora[1];

	int ID;
	char IDLora[1];

	int IDSlave;
	char IDSlaveLora[1];

	char hora[8];
	char lat[10];
	char latC[1];
	char lon[10];
	char lonC[1];
	char buffGPS[40];
	char ReadyID[6];
	char OK_ID[3];


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

		SPI_Config();
		SPI_Init();

		PRINTF("VERSION: %X\n\r", VERSION);

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

		/* Slave */
		bool isMaster = false;
		ID = 1;
		sprintf(IDLora,"%d", ID);
		IDMaster = ID-1;
		sprintf(IDMasterLora,"%d", IDMaster);
		IDSlave = ID+1;
		sprintf(IDSlaveLora,"%d", IDSlave);

		while (1) {
			switch (State) {
			case RX:
				if (isMaster == true) {
					if (BufferSize > 0) {
						if ((strncmp((const char*) Buffer, (const char*) ReadyMsg,5) == 0) && Buffer[5] == IDSlaveLora[0]) {
							PRINTF(" Master: %s\r\n", Buffer);
							Radio.Send(misDat[i].datos, BUFFERSIZE);
							enviadoReady = 1;
							recibidoMaster = 1;
							errorReady = 1;
							DelayMs(1);
						}
						else if ((recibidoMaster == 1)&& (strncmp((const char*) Buffer,(const char*) OKMsg, 2) == 0) && Buffer[2] == IDSlaveLora[0]) {
							PRINTF(" Master: %s\r\n", Buffer);
							Radio.Send(misDat[i].datos, BUFFERSIZE);
							isMaster = false;
							DelayMs(1);
						}
						Radio.Rx( RX_TIMEOUT_VALUE);
						memset(Buffer, '\0', BUFFER_SIZE);
					}
				} else {
					if (BufferSize > 0) {
						PRINTF("%s\r\n", Buffer);
						if ((strncmp((const char*) Buffer, (const char*) ReadyMsg,5) == 0) && Buffer[5] == IDMasterLora[0]) {
							PRINTF(" Esclavo: %s\r\n", Buffer);
							sprintf(ReadyID, "%s%d", ReadyMsg, ID);
							Radio.Send(ReadyID, 6);
							recibidoSlave = 1;
							DelayMs(1);
						}
						else if ((recibidoSlave == 1) && (strncmp((const char*) Buffer,(const char*) "GPS", 3) == 0)) {
							PRINTF("%s\r\n", Buffer);
							sprintf(OK_ID, "%s%d", OKMsg, ID);
							Radio.Send(OK_ID, 3);
							DelayMs(1);
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
						sprintf(ReadyID, "%s%d", ReadyMsg, ID);
						Radio.Send(ReadyID, 6);
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
	}

	void OnRxTimeout(void) {
		Radio.Sleep();
		State = RX_TIMEOUT;
	}

	void OnRxError(void) {
		Radio.Sleep();
		State = RX_ERROR;
	}
