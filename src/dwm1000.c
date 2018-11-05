#include <inttypes.h>
#include <string.h>
#include "main.h"
#include "stm32f0xx_hal.h"

#include "dwm1000.h"

/* ----------------------------------------------------------------------- */


SPI_HandleTypeDef* _deviceHandle;
static struct Config config;

static void DWM_Reset(void);



struct Channel {
	enum CHANNEL channel;
	uint8_t preambleCode;
	enum PE preambleSize;
	enum BITRATE bitRate;
	enum PRF prf;
};
struct Transmitter {
	struct Channel channel;
};

struct Receiver {
	struct Channel channel;
	uint16_t frameTimeoutDelay;
	uint16_t pac;
	uint16_t sfd;
};

struct Config {
	struct Transmitter transmitter;
	struct Receiver receiver;
	enum CHANNEL channel;
};

static uint8_t CHANNEL_ValidPreambleCode(struct Channel* channel, uint8_t preambleCode) {
	switch (channel->prf) {
	case _16MHZ:
		switch (channel->channel) {
		case _1:
			if (preambleCode != 1U && preambleCode != 2U) {
				preambleCode = 1U;
			}
			break;
		case _2: case _5:
			if (preambleCode != 3U && preambleCode != 4U) {
				preambleCode = 3U;
			}
			break;
		case _3:
			if (preambleCode != 5U && preambleCode != 6U) {
				preambleCode = 5U;
			}
			break;
		case _4: case _7:
			if (preambleCode != 7U && preambleCode != 8U) {
				preambleCode = 7U;
			}
			break;
		}
		break;
	case _64MHZ:
		switch (channel->channel) {
		case _1: case _2: case _3: case _5:
			if (preambleCode < 9U || preambleCode >  12U) {
				preambleCode = 9U;
			}
			break;
		case _4: case _7:
			if (preambleCode < 17U || preambleCode >  20U) {
				preambleCode = 17U;
			}
			break;
		}
		break;
	}
	return preambleCode;
}

static int CHANNEL_IsValidPreambleCode(struct Channel* channel,  uint8_t preambleCode) {
	return CHANNEL_ValidPreambleCode(channel, preambleCode) == preambleCode;
}

static enum PE CHANNEL_ValidPreambleSize(struct Channel* channel, enum PE preambleSize) {
	switch (preambleSize) {
	case _1536: case _2048: case _4096:
		if (channel->bitRate != _110KBPS) {
			preambleSize = _1024;
		}
		break;
	case _128: case _256: case _512: case _1024:
		if (channel->bitRate == _110KBPS) {
			preambleSize = _1536;
		}
		break;
	case _64:
		switch (channel->bitRate) {
		case _850KBPS:
			preambleSize = _128;
			break;
		case _110KBPS:
			preambleSize = _1536;
			break;
		case _6800KBPS:
			break;
		}
		break;
	}
	return preambleSize;
}

static int CHANNEL_IsValidPreambleSize(struct Channel* channel, enum PE preambleSize) {
	return CHANNEL_ValidPreambleSize(channel, preambleSize) == preambleSize;
}

static void DWM_UpdateChannel(void){
	uint8_t chan[] = {((config.channel << 4)|config.channel)};
	DWM_WriteSPI_ext(CHAN_CTRL, NO_SUB, chan, 1);
	uint8_t pllTune[1];
	uint8_t pllCfg[4];
	switch (config.channel) {
		case _1:
			int2Bytes(0x09000407U, pllCfg);
			pllTune[0] = 0x1EU;
			break;
		case _2: case _4:
			int2Bytes(0x08400508U, pllCfg);
			pllTune[0] = 0x26U;
			break;
		case _3:
			int2Bytes(0x08401009U, pllCfg);
			pllTune[0] = 0x56U;
			break;
		case _5: case _7:
			int2Bytes(0x0800041DU, pllCfg);
			pllTune[0] = 0xBEU;
			break;
	}
	DWM_WriteSPI_ext(FS_CTRL, 0x07, pllCfg, 4);
	DWM_WriteSPI_ext(FS_CTRL, 0x0B, pllTune, 1);
}

static void CHANNEL_Init(struct Channel* channel) {
	channel->channel = _5;
	channel->prf = _16MHZ;
	channel->bitRate = _6800KBPS;
	channel->preambleCode = CHANNEL_ValidPreambleCode(channel, 4U);
	channel->preambleSize = CHANNEL_ValidPreambleSize(channel, _128);
}

static void TRANSMITTER_UpdateTxPower() {
	uint8_t txPower[4];
	switch (config.transmitter.channel.bitRate) {
	case _6800KBPS:
		switch (config.transmitter.channel.prf) {
		case _16MHZ:
			switch (config.transmitter.channel.channel) {
			case _1: case _2:
				int2Bytes(0x15355575U, txPower);
				break;
			case _3:
				int2Bytes(0x0F2F4F6FU, txPower);
				break;
			case _4:
				int2Bytes(0x1F1F3F5FU, txPower);
				break;
			case _5:
				int2Bytes(0x0E082848U, txPower);
				break;
			case _7:
				int2Bytes(0x32527292U, txPower);
				break;
			}
			break;
		case _64MHZ:
			switch (config.transmitter.channel.channel) {
			case _1: case _2:
				int2Bytes(0x07274767U, txPower);
				break;
			case _3:
				int2Bytes(0x2B4B6B8BU, txPower);
				break;
			case _4:
				int2Bytes(0x3A5A7A9AU, txPower);
				break;
			case _5:
				int2Bytes(0x25456585U, txPower);
				break;
			case _7:
				int2Bytes(0x5171B1D1U, txPower);
				break;
			}
			break;
		}
		break;
	default:
		switch (config.transmitter.channel.prf) {
		case _16MHZ:
			switch (config.transmitter.channel.channel) {
			case _1: case _2:
				int2Bytes(0x75757575U, txPower);
				break;
			case _3:
				int2Bytes(0x6F6F6F6FU, txPower);
				break;
			case _4:
				int2Bytes(0x5F5F5F5FU, txPower);
				break;
			case _5:
				int2Bytes(0x48484848U, txPower);
				break;
			case _7:
				int2Bytes(0x92929292U, txPower);
				break;
			}
			break;
		case _64MHZ:
			switch (config.transmitter.channel.channel) {
			case _1: case _2:
				int2Bytes(0x67676767U, txPower);
				break;
			case _3:
				int2Bytes(0x8B8B8B8BU, txPower);
				break;
			case _4:
				int2Bytes(0x9A9A9A9AU, txPower);
				break;
			case _5:
				int2Bytes(0x85858585U, txPower);
				break;
			case _7:
				int2Bytes(0xD1D1D1D1U, txPower);
				break;
			}
			break;
		}
		break;
	}
	DWM_WriteSPI_ext(TX_POWER, NO_SUB, txPower, 4);
}

static void TRANSMITTER_UpdateChannel() {
	uint8_t tc[1];
	uint8_t txctrl[4];
	switch (config.transmitter.channel.channel) {
		case _1:
			int2Bytes(0x00005C40U, txctrl);
			tc[0] =  0xC9U;
			break;
		case _2:
			int2Bytes(0x00045CA0U, txctrl);
			tc[0] =  0xC2U;
			break;
		case _3:
			int2Bytes(0x00086CC0U, txctrl);
			tc[0] =  0xC5U;
			break;
		case _4:
			int2Bytes(0x00045C80U, txctrl);
			tc[0] =  0x95U;
			break;
		case _5:
			int2Bytes(0x001E3FE0U, txctrl);
			tc[0] =  0xC0U;
			break;
		case _7:
			int2Bytes(0x001E7DE0U, txctrl);
			tc[0] =  0x93U;
			break;
	}
	DWM_WriteSPI_ext(RF_CONF,  0x0C, txctrl,  3);
	DWM_WriteSPI_ext(TX_CAL,  0x0B, tc,  1);
	TRANSMITTER_UpdateTxPower();
}

static void TRANSMITTER_UpdateBitrate(){
	DWM_WriteSPI_ext(TX_FCTRL,  1, (uint8_t[]){((config.transmitter.channel.bitRate << 5) | (1<<7))},  1);
	TRANSMITTER_UpdateTxPower();
}

static void TRANSMITTER_UpdatePreambleCode(){
	uint8_t chan[2];
	DWM_ReadSPI_ext(CHAN_CTRL, 2, chan, 2);
	uint8_t preambleCode = config.transmitter.channel.preambleCode;
	chan[0] &= 0x3F;
	chan[1] &= 0xF8;
	chan[0] |= preambleCode << 6;
	chan[1] |= preambleCode >> 2;
	DWM_WriteSPI_ext(CHAN_CTRL, 2, chan, 2);
}

static void TRANSMITTER_UpdatePreambleSize(){
	DWM_WriteSPI_ext(TX_FCTRL, 2,(uint8_t[]){(config.transmitter.channel.prf |
			(config.transmitter.channel.preambleSize << 2))}, 1);
}

static void TRANSMITTER_UpdatePrf(){
	DWM_WriteSPI_ext(TX_FCTRL, 2, (uint8_t[]){(config.transmitter.channel.prf |
			(config.transmitter.channel.preambleSize << 2))}, 1);
	TRANSMITTER_UpdateTxPower();
}

void TRANSMITTER_SetPreambleCode(uint8_t preambleCode) {
	config.transmitter.channel.preambleCode = CHANNEL_ValidPreambleCode(&(config.transmitter.channel), preambleCode);
	TRANSMITTER_UpdatePreambleCode();
}

void TRANSMITTER_SetPreambleSize(enum PE preambleSize) {
	config.transmitter.channel.preambleSize = CHANNEL_ValidPreambleSize(&(config.transmitter.channel), preambleSize);
	TRANSMITTER_UpdatePreambleSize();
}

void TRANSMITTER_SetChannel(enum CHANNEL channel) {
	config.transmitter.channel.channel = channel;
	if (!CHANNEL_IsValidPreambleCode(&(config.transmitter.channel), config.transmitter.channel.preambleCode)) {
		TRANSMITTER_SetPreambleCode(config.transmitter.channel.preambleCode);
	}
	TRANSMITTER_UpdateChannel();
}

void TRANSMITTER_SetBitRate(enum BITRATE bitRate) {
	config.transmitter.channel.bitRate = bitRate;
	if (!CHANNEL_IsValidPreambleSize(&(config.transmitter.channel), config.transmitter.channel.preambleSize)) {
		TRANSMITTER_SetPreambleSize(config.transmitter.channel.preambleSize);
	}
	TRANSMITTER_UpdateBitrate();
}

void TRANSMITTER_SetPrf(enum PRF prf) {
	config.transmitter.channel.prf = prf;
	if (!CHANNEL_IsValidPreambleCode(&(config.transmitter.channel), config.transmitter.channel.preambleCode)) {
		TRANSMITTER_SetPreambleCode(config.transmitter.channel.preambleCode);
	}
	TRANSMITTER_UpdatePrf();
}

static void TRANSMITTER_Init(void) {
	CHANNEL_Init(&(config.transmitter.channel));
	TRANSMITTER_UpdateChannel();
	TRANSMITTER_UpdateBitrate();
	TRANSMITTER_UpdatePrf();
	TRANSMITTER_UpdatePreambleCode();
	TRANSMITTER_UpdatePreambleSize();
}

static void RECEIVER_UpdateSFDTimeout() {
	uint16_t preambleSize = 4096;
	uint8_t timeout[2];
	switch(config.receiver.channel.preambleSize) {
		case _64:
			preambleSize = 64;
			break;
		case _128:
			preambleSize = 128;
			break;
		case _256:
			preambleSize = 256;
			break;
		case _512:
			preambleSize = 512;
			break;
		case _1024:
			preambleSize = 1024;
			break;
		case _1536:
			preambleSize = 1536;
			break;
		case _2048:
			preambleSize = 2048;
			break;
		case _4096:
			preambleSize = 4096;
			break;
	}
		short2Bytes(preambleSize + config.receiver.sfd + 1 - config.receiver.pac, timeout);
                DWM_WriteSPI_ext(DRX_CONF, 0x20, timeout, 2);
}

static void RECEIVER_UpdatePacSize() {
	uint8_t drxTune2[4];
	switch (config.receiver.channel.prf) {
	case _16MHZ:
		switch (config.receiver.channel.preambleSize) {
		case _64: case _128:
			int2Bytes(0x311A002DU, drxTune2);
			config.receiver.pac = 8;
			break;
		case _256: case _512:
			int2Bytes(0x331A0052U, drxTune2);
			config.receiver.pac = 16;
			break;
		case _1024:
			int2Bytes(0x351A009AU, drxTune2);
			config.receiver.pac = 32;
			break;
		case _1536: case _2048: case _4096:
			int2Bytes(0x371A011DU, drxTune2);
			config.receiver.pac = 64;
			break;
		}
		break;
	case _64MHZ:
		switch (config.receiver.channel.preambleSize) {
		case _64: case _128:
			int2Bytes(0x313B006BU, drxTune2);
			config.receiver.pac = 8;
			break;
		case _256: case _512:
			int2Bytes(0x333B00BEU, drxTune2);
			config.receiver.pac = 16;
			break;
		case _1024:
			int2Bytes(0x353B015EU, drxTune2);
			config.receiver.pac = 32;
			break;
		case _1536: case _2048: case _4096:
			int2Bytes(0x373B0296U, drxTune2);
			config.receiver.pac = 64;
			break;
		}
		break;
	}
	DWM_WriteSPI_ext(DRX_CONF, 0x08, drxTune2, 4);
}

static void RECEIVER_UpdateFrameTimeoutDelay() {
	uint8_t sysCfg[1];
	DWM_ReadSPI_ext(SYS_CFG, 0x03, sysCfg, 0x01);
	sysCfg[0] |= (1 << 4);
	uint8_t rxfwto[2];
	short2Bytes(config.receiver.frameTimeoutDelay, rxfwto);
	if (config.receiver.frameTimeoutDelay == 0) {
		sysCfg[0] &= ~(1 << 4);
	}

	DWM_WriteSPI_ext(RX_FWTO, NO_SUB, rxfwto, 0x02);
	DWM_WriteSPI_ext(SYS_CFG, 0x03, sysCfg, 0x01);
}

static void RECEIVER_UpdateChannel() {
	uint8_t rxCtrl[1];
	switch (config.receiver.channel.channel) {
	case _1: case _2: case _3: case _5:
		rxCtrl[0] = 0xD8;
		break;
	case _4: case _7:
		rxCtrl[0] = 0xBC;
		break;
	}
	DWM_WriteSPI_ext(RF_CONF, 0x0B, rxCtrl, 1);
}

static void RECEIVER_UpdateBitrate() {
	uint8_t sys[1] = {0};
	uint8_t drxTune0b[2];
	switch (config.receiver.channel.bitRate) {
	case _110KBPS:
		short2Bytes(0x000AU, drxTune0b);
		sys[0] = 1 << 6;
		config.receiver.sfd = 64;
		break;
	case _850KBPS: case _6800KBPS:
		short2Bytes(0x0001U, drxTune0b);
		config.receiver.sfd = 8;
		break;
	}
	DWM_WriteSPI_ext(SYS_CFG, 2, sys, 1);
	DWM_WriteSPI_ext(DRX_CONF, 0x02, drxTune0b, 2);
	RECEIVER_UpdateSFDTimeout();
}

static void RECEIVER_UpdatePreambleCode() {
	uint8_t chan[1];
	DWM_ReadSPI_ext(CHAN_CTRL, 3, chan, 1);
	uint8_t preambleCode = config.receiver.channel.preambleCode;
	chan[0] &= 0x07;
	chan[0] |= preambleCode << 3;
	DWM_WriteSPI_ext(CHAN_CTRL, 3, chan, 1);
}

static void RECEIVER_UpdatePreambleSize() {
	uint8_t drxTune1b[2];
	switch (config.receiver.channel.preambleSize) {
	case _64:
		short2Bytes(0x0064, drxTune1b);
		break;
	case _128: case _256: case _512: case _1024:
		short2Bytes(0x0020, drxTune1b);
		break;
	case _1536: case _2048: case _4096:
		short2Bytes(0x0010, drxTune1b);
		break;
	}
	DWM_WriteSPI_ext(DRX_CONF, 0x06, drxTune1b, 2);
	RECEIVER_UpdatePacSize();
	RECEIVER_UpdateSFDTimeout();
}

static void RECEIVER_UpdatePrf() {
	uint8_t prf[1];
	DWM_ReadSPI_ext(CHAN_CTRL, 2, prf, 1);
	prf[0] &= 0xF3;
	prf[0] |= config.receiver.channel.prf << 2;
	DWM_WriteSPI_ext(CHAN_CTRL, 2, prf, 1);
	uint8_t drxTune1a[2];
	uint8_t agcTune1[2];
	uint8_t ldeCfg2[2];
	switch (config.receiver.channel.prf) {
	case _16MHZ:
		short2Bytes(0x0087U, drxTune1a);
		short2Bytes(0x8870U, agcTune1);
		short2Bytes(0x1607U, ldeCfg2);
		break;
	case _64MHZ:
		short2Bytes(0x008DU, drxTune1a);
		short2Bytes(0x889BU, agcTune1);
		short2Bytes(0x0607U, ldeCfg2);
		break;
	}
	DWM_WriteSPI_ext(DRX_CONF, 0x04, drxTune1a, 2);
	DWM_WriteSPI_ext(AGC_CTRL, 0x04, agcTune1, 2);
	DWM_WriteSPI_ext(LDE_CTRL, 0x1806, ldeCfg2, 2);
	RECEIVER_UpdatePacSize();
}

void RECEIVER_SetPreambleCode(uint8_t preambleCode) {
	config.receiver.channel.preambleCode = CHANNEL_ValidPreambleCode(&(config.receiver.channel), preambleCode);
	RECEIVER_UpdatePreambleCode();
}


void RECEIVER_SetPreambleSize(enum PE preambleSize) {
	config.receiver.channel.preambleSize = CHANNEL_ValidPreambleSize(&(config.receiver.channel), preambleSize);
	RECEIVER_UpdatePreambleSize();
}

void RECEIVER_SetChannel(enum CHANNEL channel) {
	config.receiver.channel.channel = channel;
	if (!CHANNEL_IsValidPreambleCode(&(config.receiver.channel), config.receiver.channel.preambleCode)) {
		RECEIVER_SetPreambleCode(config.receiver.channel.preambleCode);
	}

	RECEIVER_UpdateChannel();
}

void RECEIVER_SetBitRate(enum BITRATE bitRate) {
	config.receiver.channel.bitRate = bitRate;
	if (!CHANNEL_IsValidPreambleSize(&(config.receiver.channel), config.receiver.channel.preambleSize)) {
		RECEIVER_SetPreambleSize(config.receiver.channel.preambleSize);
	}
	RECEIVER_UpdateBitrate();
}

void RECEIVER_SetPrf(enum PRF prf) {
	config.receiver.channel.prf = prf;
	if (!CHANNEL_IsValidPreambleCode(&(config.receiver.channel), config.receiver.channel.preambleCode)) {
		RECEIVER_SetPreambleCode(config.receiver.channel.preambleCode);
	}
	RECEIVER_UpdatePrf();
}

void RECEIVER_SetRrameTimeoutDelay(uint16_t timeoutDelay) {
	config.receiver.frameTimeoutDelay = timeoutDelay;
	RECEIVER_UpdateFrameTimeoutDelay();

}

static void RECEIVER_Init(void) {
	CHANNEL_Init(&(config.receiver.channel));
	config.receiver.frameTimeoutDelay = 0;
	RECEIVER_UpdateChannel();
	RECEIVER_UpdateBitrate();
	RECEIVER_UpdatePrf();
	RECEIVER_UpdatePreambleCode();
	RECEIVER_UpdatePreambleSize();
	RECEIVER_UpdateFrameTimeoutDelay();
}

void DWM_SetChannel(enum CHANNEL channel) {
	config.channel = channel;
	TRANSMITTER_SetChannel(channel);
	RECEIVER_SetChannel(channel);
	DWM_UpdateChannel();
}

static void CONFIG_Init(void) {
	DWM_Reset();
	TRANSMITTER_Init();
	RECEIVER_Init();
	config.channel = _5;
	DWM_UpdateChannel();

}





/* ---------------------- Functions -------------------------------------- */

/* ------------------------------------- */
/* STATE MANAGEMENT FUNCTION             */
/* ------------------------------------- */
void DWM_Init(void){

	CONFIG_Init();

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
	/*// Set RX 110kbps
	uint8_t sysCfg[2];
	sysCfg[1] = 0x00;
	sysCfg[0] = 1<<6;
	DWM_WriteSPI_ext(SYS_CFG, 0x02, sysCfg, 2);

	// CHAN_CTRL: set RXPRF 01
	uint8_t chanCtrl[1];
	chanCtrl[0] = 1<<2;
	DWM_WriteSPI_ext(CHAN_CTRL, 0x02, chanCtrl, 1);*/

/****************************************************************************************
 * *************************************************************************************/

	/*// DEFAULT CONFIGURATION THAT SOULD BE MODIFIED (SECTION 2.5.5 OF USER MANUAL)
	// AGC_TUNE1
	uint8_t agc_tune1[] = {0x70, 0X88};
	DWM_WriteSPI_ext(AGC_CTRL, 0x04, agc_tune1, 2);*/

	//AGC_TUNE2
	uint8_t agc_tune2[] = {0x07, 0xA9, 0x02, 0x25};
	DWM_WriteSPI_ext(AGC_CTRL, 0x0C, agc_tune2, 4);

	/*// DRX_TUNE2
        uint8_t drx_tune2[] = {0x2d, 0x00, 0x1a, 0x31};
        DWM_WriteSPI_ext(DRX_CONF, 0x08, drx_tune2, 4);*/

        // LDE_CFG1: NTM
        uint8_t lde_cfg1[] = {0x6d};
        DWM_WriteSPI_ext(LDE_CTRL, 0x0806, lde_cfg1, 1);

        /*// LDE_CFG2
        uint8_t lde_cfg2[] = {0x07, 0x16};
        DWM_WriteSPI_ext(LDE_CTRL, 0x1806, lde_cfg2, 2);

        // TX_POWER
        uint8_t tx_power[] = {0x48, 0x28, 0x08, 0x0e};
        DWM_WriteSPI_ext(TX_POWER,NO_SUB, tx_power, 4);

        // RF_TXCTRL
        uint8_t rf_txctrl[] = {0xe0, 0x3f, 0x1e};
        DWM_WriteSPI_ext(RF_CONF, 0x0c, rf_txctrl, 3);

        // TC_PGDELAY
        uint8_t tc_pgdelay[] = {0xc0};
        DWM_WriteSPI_ext(TX_CAL, 0x0b, tc_pgdelay, 1);

        // FS_PLLTUNE
        uint8_t fs_plltune[] = {0xbe};
        DWM_WriteSPI_ext(FS_CTRL, 0x0b, fs_plltune, 1);*/

/****************************************************************************************
 * *************************************************************************************/

	/*// TX_FCTRL: TR & TXBR to 110k
	uint8_t fctrl[] = {0x80};
	DWM_WriteSPI_ext(TX_FCTRL, 0x01, fctrl, 1);*/

	// setup Rx Timeout 5ms
#ifdef MASTER_BOARD
	/*uint8_t timeout[] = {0x88, 0x13};
	DWM_WriteSPI_ext(RX_FWTO,NO_SUB, timeout, 2);
	DWM_ReadSPI_ext(SYS_CFG, 0x03, sysCfg, 1);
	sysCfg[0] |= 1<<4;
	DWM_WriteSPI_ext(SYS_CFG, 0x03, sysCfg, 1);*/
	RECEIVER_SetFrameTimeoutDelay(5000U);
#endif

	// setup of the irq
	uint8_t sysMask[SYS_MASK_LEN];
	DWM_ReadSPI_ext(SYS_MASK, NO_SUB, sysMask, SYS_MASK_LEN);
	setBit(sysMask,SYS_MASK_LEN,7,1);// TX OK
	//setBit(sysMask,SYS_MASK_LEN,10,1);// RX OK
	setBit(sysMask,SYS_MASK_LEN,13,1);// RX OK
	//setBit(sysMask,SYS_MASK_LEN,14,1);// RX OK
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
	DWM_ReadSPI_ext(DEV_ID, NO_SUB, SPIRxBuffer8, DEV_ID_LEN);
	deviceID =(SPIRxBuffer8[3] << 24) | (SPIRxBuffer8[2] << 16) | (SPIRxBuffer8[1] << 8) | SPIRxBuffer8[0];

	if (deviceID != 0xDECA0130){
		printf("Wrong DW ID \n");
		printf("%" PRIx32 "\n", deviceID);
		// infinite loop
		while (1){
			HAL_GPIO_TogglePin(GPIOC, LD3_Pin);
			HAL_Delay(100);
		}
	}

}

void idle() {
	// Set the device in IDLE mode
	uint8_t sysCtrl[1] = { 1 << TRXOFF_BIT};
	// Update the DWM1000 module
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, sysCtrl, 1);
}

static void DWM_Reset(void){
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
	uint8_t pmscCtrl0[] = {0xE0};
	DWM_WriteSPI_ext(PMSC, 0x03, pmscCtrl0, 1);

	pmscCtrl0[0] = 0xF0;
	DWM_WriteSPI_ext(PMSC, 0x03, pmscCtrl0, 1);
}
void DWM_Enable_Rx(void){
	uint8_t TxBuf8[4];
	memset(TxBuf8, 0, 4);
	setBit(TxBuf8,4,8,1);
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, TxBuf8, 4);
}

/*void DWM_Disable_Rx(void){
	uint8_t TxBuf8[4];
	memset(TxBuf8, 0, 4);
	setBit(TxBuf8,4,8,0);
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, TxBuf8, 4);
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

void int2Bytes(const uint32_t dataInt, uint8_t data[4]) {
	data[0] = dataInt;
	data[1] = dataInt >> 8;
	data[2] = dataInt >> 16;
	data[3] = dataInt >> 24;
}

void short2Bytes(const uint16_t	dataShort, uint8_t data[2]) {
	data[0] = dataShort;
	data[1] = dataShort >> 8;
}

/* ------------------------------------- */
/*  RAW READ-WRITE FUNCTIONS             */
/* ------------------------------------- */
void DWM_ReadSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len) {
	uint8_t header[3];
	memset(header,0,3);
	uint16_t i = 0;
	uint8_t headerLen = 1;

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
void DWM_SendData(uint8_t* data, uint8_t len, uint8_t waitResponse){ // data limited to 125 byte long

	uint8_t fctrl = len+2; // FCS is 2-bytes long
	/* ** ERIC **/
	/* ** ERIC ** */
	DWM_WriteSPI_ext(TX_BUFFER, NO_SUB, data, len);
	// maj frame length
	DWM_WriteSPI_ext(TX_FCTRL, NO_SUB, &fctrl, 1);

	// START SENDING
	// Set bit TXSTRT to 1

	uint8_t sysCtrl[1];
	DWM_ReadSPI_ext(SYS_CTRL, NO_SUB, sysCtrl, 1);
	sysCtrl[0] |= 1<<TXSTRT_BIT;
	sysCtrl[0] |= (waitResponse?1:0)<<WAIT4RESP_BIT;
	// Set the device in TX mode
	// Update the DWM1000 module
	DWM_WriteSPI_ext(SYS_CTRL, NO_SUB, sysCtrl, 1);
}

void DWM_ReceiveData(uint8_t* buffer){
	// Get frame length
	uint8_t flen;
	DWM_ReadSPI_ext(RX_FINFO, NO_SUB, &flen, 1);
	flen -= 2; // FCS 2 Byte long

	//reading data
	DWM_ReadSPI_ext(RX_BUFFER, NO_SUB, buffer, flen);
}
