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
	// Set RX 110kbps
	uint8_t sysCfg[2];
	sysCfg[1] = 0x00;
	sysCfg[0] = 1<<6;
	DWM_WriteSPI_ext(SYS_CFG, 0x02, sysCfg, 2);

	// CHAN_CTRL: set RXPRF 01
	uint8_t chanCtrl[1];
	chanCtrl[0] = 1<<2;
	DWM_WriteSPI_ext(CHAN_CTRL, 0x02, chanCtrl, 1);

/****************************************************************************************
 * *************************************************************************************/

	// DEFAULT CONFIGURATION THAT SOULD BE MODIFIED (SECTION 2.5.5 OF USER MANUAL)
	// AGC_TUNE1
	uint8_t agc_tune1[2] = {0x70, 0X88};
	DWM_WriteSPI_ext(AGC_CTRL, 0x04, agc_tune1, 2);

	//AGC_TUNE2
	uint8_t agc_tune2[4] = {0x07, 0xA9, 0x02, 0x25};
	DWM_WriteSPI_ext(AGC_CTRL, 0x0C, agc_tune2, 4);

	// DRX_TUNE2
        uint8_t drx_tune2[4] = {0x2d, 0x00, 0x1a, 0x31};
        DWM_WriteSPI_ext(DRX_CONF, 0x08, drx_tune2, 4);

        // LDE_CFG1: NTM
        uint8_t lde_cfg1[1] = {0x6d};
        DWM_WriteSPI_ext(LDE_CTRL, 0x0806, lde_cfg1, 1);

        // LDE_CFG2
        uint8_t lde_cfg2[2] = {0x07, 0x16};
        DWM_WriteSPI_ext(LDE_CTRL, 0x1806, lde_cfg2, 2);

        // TX_POWER
        uint8_t tx_power[4] = {0x48, 0x28, 0x08, 0x0e};
        DWM_WriteSPI_ext(TX_POWER,NO_SUB, tx_power, 4);

        // RF_TXCTRL
        uint8_t rf_txctrl[3] = {0xe0, 0x3f, 0x1e};
        DWM_WriteSPI_ext(RF_CONF, 0x0c, rf_txctrl, 3);

        // TC_PGDELAY
        uint8_t tc_pgdelay[1] = {0xc0};
        DWM_WriteSPI_ext(TX_CAL, 0x0b, tc_pgdelay, 1);

        // FS_PLLTUNE
        uint8_t fs_plltune[1] = {0xbe};
        DWM_WriteSPI_ext(FS_CTRL, 0x0b, fs_plltune, 1);

/****************************************************************************************
 * *************************************************************************************/

	// TX_FCTRL: TR & TXBR to 110k
	uint8_t fctrl[1] = {0x80};
	DWM_WriteSPI_ext(TX_FCTRL, 0x01, fctrl, 1);

	// setup Rx Timeout 5ms
#ifdef MASTER_BOARD
	uint8_t timeout[] = {0x88, 0x13};
	DWM_WriteSPI_ext(RX_FWTO,NO_SUB, timeout, 2);
	DWM_ReadSPI_ext(SYS_CFG, 0x03, sysCfg, 1);
	sysCfg[0] |= 1<<4;
	DWM_WriteSPI_ext(SYS_CFG, 0x03, sysCfg, 1);
#endif

	// setup of the irq
	uint8_t sysMask[SYS_MASK_LEN];
	DWM_ReadSPI_ext(SYS_MASK, NO_SUB, sysMask, SYS_MASK_LEN);
	setBit(sysMask,SYS_MASK_LEN,7,1);// TX OK
	setBit(sysMask,SYS_MASK_LEN,10,1);// RX OK
	setBit(sysMask,SYS_MASK_LEN,13,1);// RX OK
	setBit(sysMask,SYS_MASK_LEN,14,1);// RX OK
	setBit(sysMask,SYS_MASK_LEN,12,1);// RX ERROR
	setBit(sysMask,SYS_MASK_LEN,15,1);// RX ERROR
	setBit(sysMask,SYS_MASK_LEN,16,1);// RX ERROR
	setBit(sysMask,SYS_MASK_LEN,17,1);// RX TIMEOUT
	setBit(sysMask,SYS_MASK_LEN,18,1);// RX ERROR
	setBit(sysMask,SYS_MASK_LEN,21,1);// RX ERROR
	setBit(sysMask,SYS_MASK_LEN,26,1);// RX ERROR
	DWM_WriteSPI_ext(SYS_MASK, NO_SUB, sysMask, SYS_MASK_LEN);

	// antenna delay
	uint8_t delayuint8[2];
	delayuint8[1] = (ANTENNA_DELAY & 0xFF00) >>8;
	delayuint8[0] = (ANTENNA_DELAY & 0xFF);
	DWM_WriteSPI_ext(TX_ANTD, NO_SUB, delayuint8, 2);

	HAL_GPIO_WritePin(GPIOC, LD6_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	_deviceMode = IDLE_MODE;
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
	uint8_t pmscCtrl0[2];

	// Set SYSCLKS bits to 01
	pmscCtrl0[0] = 0x01;
	DWM_WriteSPI_ext(PMSC, PMSC_CTRL0, pmscCtrl0, 1);
	HAL_Delay(1);

	// Clear SOFTRESET bits
	pmscCtrl0[0] = 0x00;
	DWM_WriteSPI_ext(PMSC, 0x03, pmscCtrl0, 1),
	HAL_Delay(1);

	// Set SOFTRESET bits
	pmscCtrl0[0] = 0xF0;
	DWM_WriteSPI_ext(PMSC, 0x03, pmscCtrl0, 1);
	HAL_Delay(5);

        // Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)

	pmscCtrl0[0] = 0x01;
	pmscCtrl0[1] = 0x03;

	DWM_WriteSPI_ext(PMSC, PMSC_CTRL0, pmscCtrl0, 2);

	HAL_Delay(1);

	uint8_t opt_ctrl[2];
	opt_ctrl[0] = 0x00;
	opt_ctrl[1] = 0x80;

	DWM_WriteSPI_ext(OTP_IF, 0x06, opt_ctrl, 2);

	HAL_Delay(1);

	pmscCtrl0[0] = 0x00;
	pmscCtrl0[1] = 0x02;

	DWM_WriteSPI_ext(PMSC, PMSC_CTRL0, pmscCtrl0, 2);

}

void DWM_Reset_Rx(){
	uint8_t pmscCtrl0[] = {0x0E};
	DWM_WriteSPI_ext(PMSC, 0x03, pmscCtrl0, 1);
	HAL_Delay(1);

	pmscCtrl0[0] = 0xF0;
	DWM_WriteSPI_ext(PMSC, 0x03, pmscCtrl0, 1);
	_deviceMode = IDLE_MODE;
}
void DWM_Enable_Rx(void){
	idle();
	uint8_t TxBuf8[4];
	memset(TxBuf8, 0, 4);
	setBit(TxBuf8,4,8,1);
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, TxBuf8, 4);
	_deviceMode = RX_MODE;
}

/*void DWM_Disable_Rx(void){
	uint8_t TxBuf8[4];
	memset(TxBuf8, 0, 4);
	setBit(TxBuf8,4,8,0);
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, TxBuf8, 4);
	_deviceMode = IDLE_MODE;
}*/
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
		HAL_SPI_Receive(_deviceHandle, &data[i], 1, HAL_MAX_DELAY);
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
	flen -= 2; // FCS 2 Byte long

	//reading data
	DWM_ReadSPI_ext(RX_BUFFER, NO_SUB, buffer, flen);
}
