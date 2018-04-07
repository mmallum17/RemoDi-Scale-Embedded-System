/**
 ******************************************************************************
  * @file    user_diskio.h
  * @brief   This file contains the common defines and functions prototypes for  
  *          the user_diskio driver.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SD_DISKIO_H
#define __SD_DISKIO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* USER CODE BEGIN 0 */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
 /* MMC/SD command */
 #define CMD0	(0)			/* GO_IDLE_STATE */
 #define CMD1	(1)			/* SEND_OP_COND (MMC) */
 #define ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
 #define CMD8	(8)			/* SEND_IF_COND */
 #define CMD9	(9)			/* SEND_CSD */
 #define CMD10	(10)		/* SEND_CID */
 #define CMD12	(12)		/* STOP_TRANSMISSION */
 #define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
 #define CMD16	(16)		/* SET_BLOCKLEN */
 #define CMD17	(17)		/* READ_SINGLE_BLOCK */
 #define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
 #define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
 #define ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
 #define CMD24	(24)		/* WRITE_BLOCK */
 #define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
 #define CMD32	(32)		/* ERASE_ER_BLK_START */
 #define CMD33	(33)		/* ERASE_ER_BLK_END */
 #define CMD38	(38)		/* ERASE */
 #define CMD55	(55)		/* APP_CMD */
 #define CMD58	(58)		/* READ_OCR */

 /* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */
/* Exported functions ------------------------------------------------------- */
extern Diskio_drvTypeDef  SD_Driver;

/* USER CODE END 0 */
   
#ifdef __cplusplus
}
#endif

#endif /* __SD_DISKIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
