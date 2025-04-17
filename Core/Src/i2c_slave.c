/*
 * i2c_slave.c
 *
 *  Created on: Jun 9, 2023
 *      Author: controllerstech
 */

#include "main.h"
#include "i2c_slave.h"
#include "string.h"

uint32_t I2C_REGISTERS[41] = {0}; // Declare 41 registers

extern I2C_HandleTypeDef hi2c1;

#define RxSIZE  42  // Handle up to 41 registers + 1 for the address
uint8_t RxData[RxSIZE];
uint8_t rxcount = 0;
uint8_t txcount = 0;
uint8_t startPosition = 0;
uint8_t bytesRrecvd = 0;

void process_data(void)
{
    int startREG = RxData[0];  // Get the starting register address
    int numREG = rxcount - 1;  // Get the number of registers written
    int endREG = startREG + numREG - 1;  // Calculate the end register

    if (endREG > 40)  // There are now a total of 41 registers (0-40)
    {
        // Clear everything and return
        memset(RxData, '\0', RxSIZE);
        rxcount = 0;
        return;
    }

    int indx = 1;  // Set the index to 1 to start reading from RxData[1]
    for (int i = 0; i < numREG; i++)
    {
    	uint32_t value = 0; // Initialize the 32-bit value
    	// Combine 4 bytes into a uint32_t value
		for (int byteIndex = 0; byteIndex < 4 && indx < rxcount; byteIndex++)
		{
			value |= (uint32_t)RxData[indx++] << (byteIndex * 8);
		}
        I2C_REGISTERS[startREG++] = value;  // Store the data in I2C_REGISTERS
    }

    // Control PB0, PB1, PB2, PB10, PB11, PB12, PB13, PB14 based on I2C_REGISTERS 8-15
    for (int i = 0; i < 8; i++)
    {
        uint16_t pin;
        if (i < 3) // PB0, PB1, PB2
        {
            pin = GPIO_PIN_0 << i;
        }
        else if (i == 3) // PB10
        {
            pin = GPIO_PIN_10;
        }
        else if (i == 4) // PB11
        {
            pin = GPIO_PIN_11;
        }
        else // PB12-PB14
        {
            pin = GPIO_PIN_12 << (i - 5);
        }

        HAL_GPIO_WritePin(GPIOB, pin, I2C_REGISTERS[8 + i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // Read PA0-PA7 (input) into I2C_REGISTERS 0-7
//    for (int i = 0; i < 8; i++)
//    {
//        I2C_REGISTERS[i] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0 << i);
//    }

    // Read PC15, PC14, PF1, PF0, PB15, PA8, PA9, PA10 (input) into I2C_REGISTERS 16-23
//    I2C_REGISTERS[16] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
//    I2C_REGISTERS[17] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
//    I2C_REGISTERS[18] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);
//    I2C_REGISTERS[19] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
//    I2C_REGISTERS[20] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
//    I2C_REGISTERS[21] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
//    I2C_REGISTERS[22] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
//    I2C_REGISTERS[23] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

    // Configure PF7, PA15, PB3, PB4, PB5, PB8, PB9, PC13 as digital inputs (I2C_REGISTERS 24-31)
    I2C_REGISTERS[24] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7);  // PF7
    I2C_REGISTERS[25] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15); // PA15
    I2C_REGISTERS[26] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);  // PB3
    I2C_REGISTERS[27] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);  // PB4
    I2C_REGISTERS[28] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);  // PB5
    I2C_REGISTERS[29] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);  // PB8
    I2C_REGISTERS[30] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);  // PB9
    I2C_REGISTERS[31] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // PC13

}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

//void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
//{
//    if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // Master wants to transmit
//    {
//        rxcount = 0;  // Reset the Rx count
//        HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData + rxcount, 1, I2C_FIRST_FRAME);
//    }
//    else
//    {
//        // Handle read request from master
//        startPosition = RxData[0];  // The first byte is the register address
//        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)&I2C_REGISTERS[startPosition], 1, I2C_FIRST_FRAME);
//    }
//}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // Master wants to transmit
    {
        rxcount = 0;  // Reset the Rx count
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData + rxcount, 1, I2C_FIRST_FRAME);
    }
    else
    {
        // Handle read request from master
        startPosition = RxData[0];  // The first byte is the register address
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *)&I2C_REGISTERS[startPosition], sizeof(uint32_t), I2C_FIRST_FRAME);
    }
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    rxcount++;
    if (rxcount < RxSIZE)
    {
        if (rxcount == RxSIZE - 1)
        {
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData + rxcount, 1, I2C_LAST_FRAME);
        }
        else
        {
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData + rxcount, 1, I2C_NEXT_FRAME);
        }
    }

    if (rxcount == RxSIZE)
    {
        process_data();  // Process received data
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    uint32_t errorcode = HAL_I2C_GetError(hi2c);
    if (errorcode == HAL_I2C_ERROR_AF)  // Acknowledge failure
    {
        // Process the received data in case of an ACK failure
        process_data();
    }
    else if (errorcode == HAL_I2C_ERROR_BERR)  // Bus error
    {
        HAL_I2C_DeInit(hi2c);
        HAL_I2C_Init(hi2c);  // Reinitialize the I2C
        memset(RxData, '\0', RxSIZE);  // Reset the Rx buffer
        rxcount = 0;  // Reset the count
    }

    HAL_I2C_EnableListen_IT(hi2c);  // Re-enable listen mode
}
