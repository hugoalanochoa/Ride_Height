/**
  ******************************************************************************
  * @file    ISL29501.h
  * @author  ochoaha
  * @date    May 22, 2017
  * @brief   API functions for intersil ISL29501
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 MSD Performance </center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of MSD Performance nor the names of its contributors
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



/*
**====================================================================================
** Double inclusion protection
**====================================================================================
*/
#ifndef SRC_ISL29501_H_
#define SRC_ISL29501_H_


/*
**====================================================================================
** Imported definitions
**====================================================================================
*/
#include "stm32xxx_hal.h"

extern I2C_HandleTypeDef  XNUCLEO53L0A1_hi2c;


/*
**====================================================================================
** Public type definitions
**====================================================================================
*/
#define ISL29501_I2C_ADDRESS       (uint8_t) 0xAE

/*
**====================================================================================
** Public constant definitions for external access
**====================================================================================
*/
#define ISL_SS(level)           HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,level)
#define ISL_CEN(level)          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,level)

/*
**====================================================================================
** Function prototype declarations for external access
**====================================================================================
*/


/**
 ** Initializes sensor
 **
 ** \details
 **
 ** \return    Nothing
 **************************************************************************************/
extern
int
SensorInit (
            void);

/**
** Reads Sensor Registers
**
** \details
**
** \return    Nothing
**************************************************************************************/
extern
int
SensorRead (
            uint8_t  regadd,             /**< Register Address  */
            uint8_t  *data,              /**< Data buffer  */
            uint     n_bytes);           /**< Number of bytes to read */

/**
** Writes Sensor Registers
**
** \details
**
** \return    Nothing
**************************************************************************************/
extern
int
SensorWrite (
             uint8_t regadd, /**< Register Address  */
             uint8_t *data, /**< Data buffer  */
             uint n_bytes); /**< Number of bytes to write */

extern
uint32_t
SensorMeasure (
               void);
extern
uint32_t SensorMeasureF (void);


extern
void
SensorCrossTalkCal (
                    void);

extern
void
SensorMagitudeCal (
                   void);

extern
void
SensorDistanceCal (
                   void);

extern
void
SensorDistCal2 (
                void);


extern
double
Bytes2Double3S (
                uint8_t exp,
                uint8_t msb,
                uint8_t lsb);

extern
double
Bytes2Double2U (
                uint8_t msb,
                uint8_t lsb);
extern
void
Double2Bytes2U (
                double dNum,
                uint8_t* baResult);

extern
void
Double2Bytes3S (
               double dNum,
               uint8_t* baResult);


#endif /* SRC_ISL29501_H_ */
