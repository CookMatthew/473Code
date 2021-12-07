/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== rfEasyLinkEchoRx_nortos.c ========
 */
/* Standard C Libraries */
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/Power.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/interrupt.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "smartrf_settings/smartrf_settings.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

/* Undefine to not use async mode */
#define RFEASYLINKECHO_ASYNC

#define RFEASYLINKECHO_PAYLOAD_LENGTH     30

/* Pin driver handle */
static PIN_Handle pinHandle;
static PIN_State pinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

static volatile bool bEchoDoneFlag;

static bool bBlockTransmit = false;

EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};

#ifdef RFEASYLINKECHO_ASYNC
void echoTxDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED2 to indicate Echo TX, clear LED1 */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
    }
    else
    {
        /* Set LED1 and clear LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
    }

    bEchoDoneFlag = true;
}

void echoRxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED2 to indicate RX, clear LED1 */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        /* Copy contents of RX packet to TX packet */
        memcpy(&txPacket.payload, rxPacket->payload, rxPacket->len);
        /* Permit echo transmission */
        bBlockTransmit = false;
    }
    else
    {
        /* Set LED1 and clear LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        /* Block echo transmission */
        bBlockTransmit = true;
    }

    bEchoDoneFlag = true;
}
#endif //RFEASYLINKECHO_ASYNC

#include <ti/drivers/i2c.h>
#include <ti/drivers/i2c/I2CCC26xx.h>
#include "board.h"
#define USB_ADDR 0x22
#define IMU_ADDR 0x69
#define ALT_ADDR 0x76
#define EXPAND_ADDR 0x22

void *mainThread(void *arg0)
{

    uint8_t         txBuffer[15];
    uint8_t         rxBuffer[2];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* Call driver init functions */
    I2C_init();

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA;
    pinCfg.pinSCL = Board_I2C0_SCL;
    i2cParams.custom = &pinCfg;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        //Error Initializing I2C
        while (1);
    }
    else {
        //I2C Initialized!
    }

    txBuffer[0] = 'H'; //Tx data
    txBuffer[1] = 'e';
    txBuffer[2] = 'l';
    txBuffer[3] = 'l';
    txBuffer[4] = 'o';
    txBuffer[5] = ' ';
    txBuffer[6] = 'w';
    txBuffer[7] = 'o';
    txBuffer[8] = 'r';
    txBuffer[9] = 'l';
    txBuffer[10] = 'd';
    txBuffer[11] = '\n';
    i2cTransaction.slaveAddress = USB_ADDR; //device address
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 12; //number of bytes to send
    i2cTransaction.readBuf = rxBuffer; //memory location to save read data
    i2cTransaction.readCount = 0; //num of bytes to save


    if (I2C_transfer(i2c, &i2cTransaction)) {
        // Transaction was a success, handle the data
    }
    else {
        I2C_close(i2c);
        return NULL;
        // Transaction failed
    }

    // IMU Transaction
    int x;
    for (x = 59; x < 73; x++){
        txBuffer[0] = x; //Tx data

        i2cTransaction.slaveAddress = IMU_ADDR; //device address
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1; //number of bytes to send
        i2cTransaction.readBuf = rxBuffer; //memory location to save read data
        i2cTransaction.readCount = 1; //num of bytes to save

        if (I2C_transfer(i2c, &i2cTransaction)) {
                // Transaction was a success, handle the data
            }
        else {
            I2C_close(i2c);
            return NULL;
            // Transaction failed
        }
    }






    /* Deinitialized I2C */
    I2C_close(i2c);

    return (NULL);
}

/*void *mainThread(void *arg0)
{
    uint32_t absTime;
     Open LED pins
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL)
    {
        while(1);
    }

     Clear LED pins
    PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);

#ifndef RFEASYLINKECHO_ASYNC
    EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}};
#endif //RFEASYLINKECHO_ASYNC

    // Initialize the EasyLink parameters to their default values
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);


     * Initialize EasyLink with the settings found in easylink_config.h
     * Modify EASYLINK_PARAM_CONFIG in easylink_config.h to change the default
     * PHY

    if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success){
        while(1);
    }


     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);


    while(1) {
#ifdef RFEASYLINKECHO_ASYNC
        // Set the echo done flag to false, callback will
        // set it to true
        bEchoDoneFlag = false;

        // Wait to receive a packet
        EasyLink_receiveAsync(echoRxDoneCb, 0);

         Wait indefinitely for Rx
        while(bEchoDoneFlag == false){
            bool previousHwiState = IntMasterDisable();

             * Tricky IntMasterDisable():
             * true  : Interrupts were already disabled when the function was
             *         called.
             * false : Interrupts were enabled and are now disabled.

            IntMasterEnable();
            Power_idleFunc();
            IntMasterDisable();

            if(!previousHwiState)
            {
                IntMasterEnable();
            }
        };
#else
        rxPacket.absTime = 0;
        EasyLink_Status result = EasyLink_receive(&rxPacket);

        if (result == EasyLink_Status_Success)
        {
             Toggle LED2 to indicate RX, clear LED1
            PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
             Copy contents of RX packet to TX packet
            memcpy(&txPacket.payload, &rxPacket.payload, rxPacket.len);
             Permit echo transmission
            bBlockTransmit = false;
        }
        else
        {
             Set LED1 and clear LED2 to indicate error
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
             Block echo transmission
            bBlockTransmit = true;
        }
#endif // RFEASYLINKECHO_ASYNC

        if(bBlockTransmit == false)
        {
             Switch to Transmitter and echo the packet if transmission
             * is not blocked

            txPacket.len = RFEASYLINKECHO_PAYLOAD_LENGTH;


             * Address filtering is enabled by default on the Rx device with the
             * an address of 0xAA. This device must set the dstAddr accordingly.

            txPacket.dstAddr[0] = 0xaa;

             Set Tx absolute time to current time + 100ms
            if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
            {
                // Problem getting absolute time
            }
            txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(100);

#ifdef RFEASYLINKECHO_ASYNC
            // Set the echo done flag to false, callback will
            // set it to true
            bEchoDoneFlag = false;
            EasyLink_transmitAsync(&txPacket, echoTxDoneCb);

             Wait for Tx to complete. A Successful TX will cause the echoTxDoneCb
             * to be called and the bEchoDoneFlag to be set

            while(bEchoDoneFlag == false){
                bool previousHwiState = IntMasterDisable();

                 * Tricky IntMasterDisable():
                 * true  : Interrupts were already disabled when the function was
                 *         called.
                 * false : Interrupts were enabled and are now disabled.

                IntMasterEnable();
                Power_idleFunc();
                IntMasterDisable();

                if(!previousHwiState)
                {
                    IntMasterEnable();
                }
            };


#else
            EasyLink_Status result = EasyLink_transmit(&txPacket);

            if (result == EasyLink_Status_Success)
            {
                 Toggle LED2 to indicate Echo TX, clear LED1
                PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
            }
            else
            {
                 Set LED1 and clear LED2 to indicate error
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
            }
#endif //RFEASYLINKECHO_ASYNC
        }
    }
}*/
