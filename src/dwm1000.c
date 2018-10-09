
#include <inttypes.h>
#include <string.h>
#include "main.h"
#include "stm32f0xx_hal.h"

#include "dwm1000.h"

/* ----------------------------------------------------------------------- */
uint8_t _sysctrl[LEN_SYS_CTRL];

uint8_t _deviceMode;

SPI_HandleTypeDef* _deviceHandle;


/* ---------------------- Functions -------------------------------------- */

/* ------------------------------------- */
/* STATE MANAGEMENT FUNCTION             */
/* ------------------------------------- */
void DWM_Init(void){

  DWM_reset();

	uint8_t SPIRxBuffer8[4];

  // Check DW ID
	DWM_ReadSPI_ext(DEV_ID, NO_SUB, SPIRxBuffer8, DEV_ID_LEN);
	uint32_t deviceID =(SPIRxBuffer8[3] << 24) | (SPIRxBuffer8[2] << 16) | (SPIRxBuffer8[1] << 8) | SPIRxBuffer8[0];

	if (deviceID != 0xDECA0130){
		printf("Wrong DW ID \n");
		printf("%" PRIx32 "\n", deviceID);
		// infinite loop
		while (1){
			HAL_GPIO_TogglePin(GPIOC, LD3_Pin);
			HAL_Delay(100);
		}
	}
 // RXAUTR: Receiver auto-re-enable
	uint8_t sysCfg[SYS_CFG_LEN];
  DWM_ReadSPI_ext(SYS_CFG, NO_SUB, sysCfg, SYS_CFG_LEN);
	sysCfg[3] = 0x20;
	sysCfg[1] = 0x12;
	sysCfg[0] = 0x00;
	setBit(sysCfg,SYS_CFG_LEN,22,1);//setting 110kbps
  DWM_WriteSPI_ext(SYS_CFG, NO_SUB, sysCfg, SYS_CFG_LEN);

	// CHAN_CTRL
	uint8_t chanCtrl[CHAN_CTRL_LEN];
	DWM_ReadSPI_ext(CHAN_CTRL, NO_SUB, chanCtrl, CHAN_CTRL_LEN);
	chanCtrl[0] = 0x55;
	chanCtrl[2] &= 0xC5;
	chanCtrl[2] |= 0x04;
  DWM_WriteSPI_ext(CHAN_CTRL, NO_SUB, chanCtrl, CHAN_CTRL_LEN);

	// F_CTRL
	uint8_t fctrl[TX_FCTRL_LEN];
	DWM_ReadSPI_ext(TX_FCTRL, NO_SUB, fctrl, TX_FCTRL_LEN);
	fctrl[0] = 0x0C;
	fctrl[1] = 0x80;
	fctrl[2] = 0x15;
	fctrl[3] = 0x00;
	fctrl[4] = 0x00;
	DWM_WriteSPI_ext(TX_FCTRL, NO_SUB, fctrl, TX_FCTRL_LEN);

	// setup Rx Timeout 5ms
#ifdef MASTER_BOARD
	uint8_t timeout[] = {0x88, 0x13};
	DWM_WriteSPI_ext(RX_FWTO,NO_SUB, timeout, 2);
	DWM_ReadSPI_ext(SYS_CFG, NO_SUB, sysCfg, SYS_CFG_LEN);
	setBit(sysCfg, SYS_CFG_LEN, 28,1);
	DWM_WriteSPI_ext(SYS_CFG, NO_SUB, sysCfg, SYS_CFG_LEN);
#endif

  // setup of the irq
	uint8_t sysMask[SYS_MASK_LEN];
	DWM_ReadSPI_ext(SYS_MASK, NO_SUB, sysMask, SYS_MASK_LEN);
	setBit(sysMask,SYS_MASK_LEN,7,1);//TX OK
	setBit(sysMask,SYS_MASK_LEN,14,1);//RX OK NO ERROR
#ifdef MASTER_BOARD
	setBit(sysMask,SYS_MASK_LEN,12,1);// RX ERROR
	setBit(sysMask,SYS_MASK_LEN,17,1);// RX Timeout
#endif
  DWM_WriteSPI_ext(SYS_MASK, NO_SUB, sysMask, SYS_MASK_LEN);

  // antenna delay
	uint16_t delayuint16 = ANTENNA_DELAY;
	uint8_t delayuint8[2];
	delayuint8[1] = (delayuint16 & 0xFF00) >>8;
	delayuint8[0] = (delayuint16 & 0xFF);
	DWM_WriteSPI_ext(TX_ANTD, NO_SUB, delayuint8, 2);
	//DWM_WriteSPI_ext(LDE_CTRL, 0x1804, delayuint8,2);

	// ERIC - Check SYS_STATUS
		// clear IRQ flags on DW
	uint8_t ack[4];
	memset(ack, 0, 4);
	setBit(ack, 4, 7, 1);
	setBit(ack, 4, 14, 1);
	DWM_WriteSPI_ext(SYS_STATUS, NO_SUB, ack, 4);
	HAL_GPIO_WritePin(GPIOC, LD6_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void idle() {
	// Set SYS_CTRL to 0
	memset(_sysctrl, 0, LEN_SYS_CTRL);
	// Set bit TRXOFF to 1
	setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, 1);
	// Set the device in IDLE mode
	_deviceMode = IDLE_MODE;
	// Update the DWM1000 module
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

void DWM_reset(void){
	uint8_t pmscCtrl0[PMSC_CTRL0_LEN];
	// Getting PMSC_CTRL0 register
	DWM_ReadSPI_ext(PMSC, PMSC_CTRL0, pmscCtrl0, PMSC_CTRL0_LEN);

	// Set SYSCLKS bits to 01
	pmscCtrl0[0] &= 0xFC;
	pmscCtrl0[0] |= 0x01;
	DWM_WriteSPI_ext(PMSC, PMSC_CTRL0, pmscCtrl0, PMSC_CTRL0_LEN);
	HAL_Delay(1);

	// Clear SOFTRESET bits
	pmscCtrl0[3] &= 0x0F;
	DWM_WriteSPI_ext(PMSC, PMSC_CTRL0, pmscCtrl0, PMSC_CTRL0_LEN);
	HAL_Delay(1);

	// Set SOFTRESET bits
	pmscCtrl0[3] |= 0xF0;
	pmscCtrl0[0] &= 0xFC;
	DWM_WriteSPI_ext(PMSC, PMSC_CTRL0, pmscCtrl0, PMSC_CTRL0_LEN);
	HAL_Delay(5);

        // Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)
	uint8_t TxUint8[4];

	TxUint8[0] = 0xF6;
	TxUint8[1] = 0x00;
	TxUint8[2] = 0x01;
	TxUint8[3] = 0x03;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_deviceHandle, TxUint8, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	TxUint8[0] = 0xED;
	TxUint8[1] = 0x06;
	TxUint8[2] = 0x00;
	TxUint8[3] = 0x80;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_deviceHandle, TxUint8, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(1);

	TxUint8[0] = 0xF6;
	TxUint8[1] = 0x00;
	TxUint8[2] = 0x00;
	TxUint8[3] = 0x02;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_deviceHandle, TxUint8, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

}

void DWM_Enable_Rx(void){
	idle();
	uint8_t TxBuf8[4];
	memset(TxBuf8, 0, 4);
	setBit(TxBuf8,4,8,1);
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, TxBuf8, 4);
	_deviceMode = RX_MODE;
}

void DWM_Disable_Rx(void){
	uint8_t TxBuf8[4];
	memset(TxBuf8, 0, 4);
	setBit(TxBuf8,4,8,0);
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, TxBuf8, 4);
	_deviceMode = IDLE_MODE;
}
/* ------------------------------------- */
/*  BITs AND BYTEs  FUNCTIONS            */
/* ------------------------------------- */
void setBit(uint8_t *data, uint16_t len, uint8_t bit, uint8_t val) {
	uint16_t idx;
	uint8_t shift;

	idx = bit>>3;
	if (idx >= len) return;

	uint8_t* targetByte = &data[idx];
	shift = bit%8;
	if (val) {
		Bitset(*targetByte, shift);
	} else {
		Bitclear(*targetByte, shift);
	}
}


/* ------------------------------------- */
/*  RAW READ-WRITE FUNCTIONS             */
/* ------------------------------------- */
void DWM_ReadSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len) {
	uint8_t header[3];
	memset(header,0,3);
	uint16_t i = 0;
	uint8_t headerLen = 1;
	uint8_t DUMMY_BYTE[len];
	memset(DUMMY_BYTE, 0, len);

	if (offset == NO_SUB) {
		header[0] = address | READ;
	} else {
		header[0] = address | READ_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}
	}
	__disable_irq();
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// Hack write byte by byte to avoid hardfault on HAL_SPI_Transmit with non 2 bytes aligned data
	for (i=0; i < headerLen; i++) {
		HAL_SPI_Transmit(_deviceHandle, &header[i], 1, HAL_MAX_DELAY);
	}
	for (i=0; i < len; i++) {
		HAL_SPI_TransmitReceive(_deviceHandle, &DUMMY_BYTE[i], &data[i], 1, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	__enable_irq();
}

void DWM_WriteSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len) {
	uint8_t header[3];
	uint8_t headerLen = 1;
	uint16_t i = 0;

	if (offset == NO_SUB) {
		header[0] = address | WRITE;
	} else {
		header[0] = address | WRITE_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}
	}
	__disable_irq();
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// Hack write byte by byte to avoid hardfault on HAL_SPI_Transmit with non 2 bytes aligned data
	for (i=0; i < headerLen; i++) {
		HAL_SPI_Transmit(_deviceHandle, &header[i], 1, HAL_MAX_DELAY);
	}
	for (i=0; i < len; i++) {
		HAL_SPI_Transmit(_deviceHandle, &data[i], 1, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	__enable_irq();
}


/* ------------------------------------- */
/*  DWM1000 COMMUNICATIONS	             */
/* ------------------------------------- */
void DWM_SendData(uint8_t* data, uint8_t len){ // data limited to 125 byte long

	uint8_t fctrl = len+2; // FCS is 2-bytes long
	/* ** ERIC **/
	idle();
	/* ** ERIC ** */
	DWM_WriteSPI_ext(TX_BUFFER, NO_SUB, data, len);
	// maj frame length
	DWM_WriteSPI_ext(TX_FCTRL, NO_SUB, &fctrl, 1);

	// START SENDING
	// Set bit TXSTRT to 1
	setBit(_sysctrl, LEN_SYS_CTRL, TXSTRT_BIT, 1);
	// Remove Force idle mode and wait for response
	setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, 0);
	setBit(_sysctrl, LEN_SYS_CTRL, WAIT4RESP_BIT, 1);
	// Set the device in TX mode
	_deviceMode = TX_MODE;
	// Update the DWM1000 module
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

void DWM_ReceiveData(uint8_t* buffer){
	// Get frame length
	uint8_t flen;
	DWM_ReadSPI_ext(RX_FINFO, NO_SUB, &flen, 1);
	flen = flen-2; // FCS 2 Byte long

	//reading data
	DWM_ReadSPI_ext(RX_BUFFER, NO_SUB, buffer, flen);
	for (int i =0; i< flen ; i++){

	}
}
