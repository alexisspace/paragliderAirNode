/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

volatile APP_DATA appData = {0, 0, 0, 0};

DRV_SPI_CLIENT_DATA SPI_cfgObj={
 .baudRate = 0,
 .operationStarting = APP_SPI_BufferEventHandler,
 .operationEnded = NULL,
};

DRV_HANDLE  SPI_handle;
char   SPI_ReadBuffer[SPI_READ_BUFFER_SIZE], SPI_WriteBuffer[SPI_WRITE_BUFFER_SIZE];
DRV_SPI_BUFFER_HANDLE SPI_bufferHandle;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
// Buffer event callback function
void  APP_SPI_BufferEventHandler(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void * context)
{ 
	switch(event)
	{
		case DRV_SPI_BUFFER_EVENT_PROCESSING:
            // Assert CSN line (0)
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, CSN_PIN, 0);
            BSP_LEDToggle(YELLOW);  // This is to verify callback is executed at beginning
            appData.nRF_status = 1; // Set busy status
		break;
        
		case DRV_SPI_BUFFER_EVENT_COMPLETE:
            // Release CSN line (1)
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, CSN_PIN, 1);
            BSP_LEDToggle(GREEN);
            appData.nRF_status = 0; // Set idle status

            // Update status register value
            appData.data = SPI_ReadBuffer[0];
		break;
        
		case DRV_SPI_BUFFER_EVENT_ERROR:
		break;
	}
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
void nRF24L01p_PowerUp(void)
{
    // SETUP_RETR: 500 us for auto-retransmit delay
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | SETUP_RETR;
    SPI_WriteBuffer[1] = 0b00010011;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);
    
    // TX_ADDR: Only LSB writen
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | TX_ADDR;
    SPI_WriteBuffer[1] = EARTH_NODE_ADDR;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);    

    // RX_ADDR_P0: This should be the same TX_ADDR to in order for the AKC packet to be received
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | RX_ADDR_P0;
    SPI_WriteBuffer[1] = EARTH_NODE_ADDR;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);     
    
    // RX_ADDR_P1: This is the address the earth node will be sending to.
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | RX_ADDR_P1;
    SPI_WriteBuffer[1] = AIR_NODE_ADDR;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);   
    
    // DYNPD: Enable dynamic payloads on P0 and P1
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | DYNPD;
    SPI_WriteBuffer[1] = 0x03;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);     
    
    // FEATURE: 
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | FEATURE;
    SPI_WriteBuffer[1] = 0b00000111;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);
    
    // CONFIG: Power up
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | CONFIG;
    SPI_WriteBuffer[1] = 0b00001010;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);
    
    // CE PIN set LOW to enter standby-I MODE
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, CE_PIN, 0);
}


void nRF24L01p_PTX_config(void)
{
    // CONFIG: PRIM_RX bit set LOW
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | CONFIG;
    SPI_WriteBuffer[1] = 0b00001010;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);
    // CE PIN set LOW to enter standby-I
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, CE_PIN, 0);
}

void nRF24L01p_PRX_config(void)
{
    // CONFIG: PRIM_RX bit set HIGH
    while(appData.nRF_status);  // Wait for SPI driver to be idle
    SPI_WriteBuffer[0] = W_REGISTER | CONFIG;
    SPI_WriteBuffer[1] = 0b00001011;
    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);
    // CE PIN set HIGH to enter receive mode
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, CE_PIN, 1);
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    SPI_handle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE );	// Open driver instance
    DRV_SPI_ClientConfigure(SPI_handle,  &SPI_cfgObj);	// Register callback functions
    vTaskDelay(300 / portTICK_PERIOD_MS);  // Wait for the nRF24L01+ to initiate
    
    // Config nRF24L01+ registers to operate in the application
    nRF24L01p_PowerUp();
    nRF24L01p_PRX_config();     // Set PRX mode
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    unsigned char n, n_payload;

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            // Use the task inherent semaphore 1 
            // Wait for INT2 external interrupt, nRF24L01+ interrupt request.
            ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); /* Block indefinitely. */

            while(appData.nRF_status);  // Wait for SPI driver to be idle
            SPI_WriteBuffer[0] = NRF_NOP_CMD;   // Read the STATUS register
            DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 1, SPI_ReadBuffer, 1,
                    APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);

            while(appData.nRF_status);  // Wait for SPI driver to be idle
            // Guardar el registro STATUS para verificar la fuente de interrupcion
            n = appData.data>>4; // Take the relevant bits only

            // Data ready RX FIFO. Asserted when new data arrives RX FIFO   
            if(n & 0x04){
                //The RX_DR IRQ is asserted by a new packet arrival event. 
                //The procedure for handling this interrupt should be: 
                //1) read payload through SPI, 
                // Read payload width
                while(appData.nRF_status); // Wait for SPI to be idle
                SPI_WriteBuffer[0] = R_RX_PL_WID;   // 
                DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 1, SPI_ReadBuffer, 2,
                    APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);

                // Read actual payload data
                while(appData.nRF_status); // Wait for SPI to be idle
                n_payload = SPI_ReadBuffer[1]; // Save payload width     
                if(n_payload < 32){ // Check if it is a valid payload length, otherwise there is an error
                    SPI_WriteBuffer[0] = R_RX_PAYLOAD;
                    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 1, SPI_ReadBuffer, n_payload+1,
                        APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);                

                    // Process command (maybe in the next app.state)
                    appData.cmd = SPI_ReadBuffer[1];
                    appData.state = APP_PROCESS_CMD;

                }else{
                   // Flush RX FIFO on nRF24L01+
                   SPI_WriteBuffer[0] = FLUSH_RX;
                   DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 1, SPI_ReadBuffer, 1,
                        APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);
                }
                //2) clear RX_DR IRQ,
                while(appData.nRF_status);  // Wait for SPI to be idle
                SPI_WriteBuffer[0] = W_REGISTER | STATUS_REG;
                SPI_WriteBuffer[1] = 0x40;       // Write STATUS_REG to clear interrupt sources
                   DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
                        APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);

                //3) read FIFO_STATUS to check if there are more payloads available in RX FIFO, 
                while(appData.nRF_status); // Wait for SPI to be idle
                SPI_WriteBuffer[0] = R_REGISTER | FIFO_STATUS;
                DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 1, SPI_ReadBuffer, 2,
                    APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);
                while(appData.nRF_status); // Wait for SPI to be idle
                if(SPI_ReadBuffer[1] & 0x01){
                    // RX FIFO empty
                }    
                //4) if there are more data in RX FIFO, repeat from step 1).            
            }

            // Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX.
            if(n & 0x02){
                while(appData.nRF_status);  // Wait for SPI to be idle
                SPI_WriteBuffer[0] = W_REGISTER | STATUS_REG;
                SPI_WriteBuffer[1] = 0x20;       // Write STATUS_REG to clear interrupt sources
                DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
                        APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);            
            }
            // Maximum number of TX retransmits interrupt
            if(n & 0x01){
                while(appData.nRF_status);  // Wait for SPI to be idle
                SPI_WriteBuffer[0] = W_REGISTER | STATUS_REG;
                SPI_WriteBuffer[1] = 0x10;       // Write STATUS_REG to clear interrupt sources
                DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, 2, SPI_ReadBuffer, 1,
                        APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);              
            }
            // 
            
                break;
        }
        
        case APP_PROCESS_CMD:
        {
        /* TODO: implement your application state machine.*/
            switch(appData.cmd)
            {
                case GET_GPS_DATA:
                {   
                    // Send GPS data to earth node. NOTE: It will be sent on the second request
                    while(appData.nRF_status);  // Wait for SPI to be idle
                    SPI_WriteBuffer[0] = W_ACK_PAYLOAD | 0x01; // Pipe 0x01 ACK payload
                    n_payload = sprintf(&SPI_WriteBuffer[1],"LatLon\n"); // Example string
                    DRV_SPI_BufferAddWriteRead2(SPI_handle, SPI_WriteBuffer, n_payload+2, SPI_ReadBuffer, 1,
                            APP_SPI_BufferEventHandler, NULL, &SPI_bufferHandle);                    
                    appData.cmd = 0; // Can be replaced with an enum type
                    break;
                }
            }

        /* The default state should never be executed. */
    
            break;
        }
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
