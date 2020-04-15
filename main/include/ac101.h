/* ========================================
 *
 * Copyright © 2019, Dana H. Myers.
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Dana H. Myers.
 *
 * ========================================
*/

#ifndef AC101_H_
#define AC101_H_


#define AC101_ADDR			0x1a				/*!< Device address*/

#define WRITE_BIT  			I2C_MASTER_WRITE
#define READ_BIT   			I2C_MASTER_READ

#define ACK_CHECK_EN   		0x1
//#define ACK_CHECK_DIS  		0x0
#define ACK_VAL    			0x0
// #define NACK_VAL   			0x1

#define CHIP_AUDIO_RS		0x00
#define PLL_CTRL1			0x01
#define PLL_CTRL2			0x02
#define SYSCLK_CTRL			0x03
#define MOD_CLK_ENA			0x04
#define MOD_RST_CTRL		0x05
#define I2S_SR_CTRL			0x06
#define I2S1LCK_CTRL		0x10
#define I2S1_SDOUT_CTRL		0x11
#define I2S1_SDIN_CTRL		0x12
#define I2S1_MXR_SRC		0x13
#define I2S1_VOL_CTRL1		0x14
#define I2S1_VOL_CTRL2		0x15
#define I2S1_VOL_CTRL3		0x16
#define I2S1_VOL_CTRL4		0x17
#define I2S1_MXR_GAIN		0x18
#define ADC_DIG_CTRL		0x40
#define ADC_VOL_CTRL		0x41
#define HMIC_CTRL1			0x44
#define HMIC_CTRL2			0x45
#define HMIC_STATUS			0x46
#define DAC_DIG_CTRL		0x48
#define DAC_VOL_CTRL		0x49
#define DAC_MXR_SRC			0x4c
#define DAC_MXR_GAIN		0x4d
#define ADC_APC_CTRL		0x50
#define ADC_SRC				0x51
#define ADC_SRCBST_CTRL		0x52
#define OMIXER_DACA_CTRL	0x53
#define OMIXER_SR			0x54
#define OMIXER_BST1_CTRL	0x55
#define HPOUT_CTRL			0x56
#define SPKOUT_CTRL			0x58
#define AC_DAC_DAPCTRL		0xa0
#define AC_DAC_DAPHHPFC 	0xa1
#define AC_DAC_DAPLHPFC 	0xa2
#define AC_DAC_DAPLHAVC 	0xa3
#define AC_DAC_DAPLLAVC 	0xa4
#define AC_DAC_DAPRHAVC 	0xa5
#define AC_DAC_DAPRLAVC 	0xa6
#define AC_DAC_DAPHGDEC 	0xa7
#define AC_DAC_DAPLGDEC 	0xa8
#define AC_DAC_DAPHGATC 	0xa9
#define AC_DAC_DAPLGATC 	0xaa
#define AC_DAC_DAPHETHD 	0xab
#define AC_DAC_DAPLETHD 	0xac
#define AC_DAC_DAPHGKPA 	0xad
#define AC_DAC_DAPLGKPA 	0xae
#define AC_DAC_DAPHGOPA 	0xaf
#define AC_DAC_DAPLGOPA 	0xb0
#define AC_DAC_DAPOPT   	0xb1
#define DAC_DAP_ENA     	0xb5

#endif  /* AC101_H_ */

