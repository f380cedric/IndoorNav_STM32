/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define NCS_MEMS_SPI_Pin GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT1_EXTI_IRQn EXTI0_1_IRQn
#define MEMS_INT2_Pin GPIO_PIN_2
#define MEMS_INT2_GPIO_Port GPIOC
#define MEMS_INT2_EXTI_IRQn EXTI2_3_IRQn
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define B1_EXTI_IRQn EXTI0_1_IRQn
#define EXT_RESET_Pin GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOC
#define USBF4_DM_Pin GPIO_PIN_11
#define USBF4_DM_GPIO_Port GPIOA
#define USBF4_DP_Pin GPIO_PIN_12
#define USBF4_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI_IRQ_Pin GPIO_PIN_3
#define SPI_IRQ_GPIO_Port GPIOB
#define SPI_IRQ_EXTI_IRQn EXTI2_3_IRQn
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
// board switching
#define SLAVE1_BOARD
//#define SLAVE2_BOARD
//#define SLAVE3_BOARD
//#define SLAVE4_BOARD
//#define SLAVE5_BOARD
//#define SLAVE6_BOARD


// Antenna calibration
//#define ANTENNA_DELAY        65610U //Offset for error at -50cm  //0x8066 // precis 10cm
#define ANTENNA_DELAY          65670U

#define PANID 		0xDECAU
#define BROADCAST	0xFFFFU

#ifdef SLAVE1_BOARD
#define SLAVE_BOARD
#define SLAVE_COORDINATE_X 		60U
#define SLAVE_COORDINATE_Y 		321U
#define SLAVE_COORDINATE_Z 		295U
#define ADDRESS 			0x1111U
#define WAIT4RESP_DELAY			50000U
#endif

#ifdef SLAVE2_BOARD
#define SLAVE_BOARD
#define SLAVE_COORDINATE_X 		1220U
#define SLAVE_COORDINATE_Y 		46U
#define SLAVE_COORDINATE_Z 		225U
#define ADDRESS 			0x2222U
#define WAIT4RESP_DELAY			40000U

#endif

#ifdef SLAVE3_BOARD
#define SLAVE_BOARD
#define SLAVE_COORDINATE_X 		1220U
#define SLAVE_COORDINATE_Y 		699U
#define SLAVE_COORDINATE_Z 		155U
#define ADDRESS 			0x3333U
#define WAIT4RESP_DELAY			30000U
#endif

#ifdef SLAVE4_BOARD
#define SLAVE_BOARD
#define SLAVE_COORDINATE_X 		60U
#define SLAVE_COORDINATE_Y 		321U
#define SLAVE_COORDINATE_Z 		295U
#define ADDRESS 			0x4444U
#define WAIT4RESP_DELAY			20000U
#endif

#ifdef SLAVE5_BOARD
#define SLAVE_BOARD
#define SLAVE_COORDINATE_X 		60U
#define SLAVE_COORDINATE_Y 		321U
#define SLAVE_COORDINATE_Z 		295U
#define ADDRESS 			0x5555U
#define WAIT4RESP_DELAY			10000U
#endif

#ifdef SLAVE6_BOARD
#define SLAVE_BOARD
#define SLAVE_COORDINATE_X 		60U
#define SLAVE_COORDINATE_Y 		321U
#define SLAVE_COORDINATE_Z 		295U
#define ADDRESS 			0x6666U
#define WAIT4RESP_DELAY			0000U
#endif


#define TX_OK_MASK		0x00000080U // TX OK
#define RX_FINISHED_MASK	0x00006400U // RX FINISHED
#define RX_ERROR_MASK		0x24279000U // RX ERROR
// State Machine Slave
enum STATE {
STATE_INIT,
STATE_WAIT_POLL,
STATE_SEND_TIMES,
STATE_WAIT_FINAL,
STATE_END_CYCLE,
STATE_SEND_RESPONSE,
STATE_WAIT_SEND_RESPONSE,
};

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
