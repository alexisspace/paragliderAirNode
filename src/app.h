/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// Application definitions
#define RED  0
#define YELLOW 1
#define GREEN 2
#define BYTES_TO_READ 1
#define GPS_BUFFER_SIZE 80
#define KNOTS2MS 0.514444       // knots to m/s
#define D2R  17.4533e-3 // Degrees to radian conversion factor
#define SPI_READ_BUFFER_SIZE 10
#define SPI_WRITE_BUFFER_SIZE 10

// nRF24L01+ register address, commands and other configurations
// Registers addres
#define CONFIG       0x00
#define EN_AA        0x01
#define EN_RXADDR    0x02
#define SETUP        0x03
#define SETUP_RETR   0x04
#define RF_CH        0x05
#define RF_SETUP     0x06
#define STATUS_REG   0x07
#define OBSERVE_TX   0x08
#define RPD          0x09
#define RX_ADDR_P0   0x0A
#define RX_ADDR_P1   0x0B
#define RX_ADDR_P2   0x0C
#define RX_ADDR_P3   0x0D
#define RX_ADDR_P4   0x0E
#define RX_ADDR_P5   0x0F
#define TX_ADDR      0x10
#define RX_PW_P0     0x11
#define RX_PW_P1     0x12
#define DYNPD        0x1C
#define FEATURE      0x1D
#define FIFO_STATUS  0x17

// Commands
#define R_REGISTER   0x00
#define W_REGISTER   0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX     0b11100001 
#define FLUSH_RX     0b11100010
#define REUSE_TX_PL  0b11100011
#define R_RX_PL_WID  0b01100000
#define W_ACK_PAYLOAD   0b10101000
#define W_TX_PAYLOAD_NOACK 0b10110000
#define NRF_NOP_CMD          0b11111111

// PIN definitions
#define CSN_PIN PORTS_BIT_POS_14
#define CE_PIN PORTS_BIT_POS_15

// Other constans
#define AIR_NODE_ADDR   0xE5
#define EARTH_NODE_ADDR 0xAB
#define GET_GPS_DATA    0x11


// Global variables
TaskHandle_t pxTaskNavPredictionHandle, pxTaskNavCorrectionHandle;
TaskHandle_t px_APP_TasksHandle; 
float mag_data[3], Pos[3], Vel[3], BaroAlt;

// Exposed fuctions prototypes


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
	APP_STATE_SERVICE_TASKS,
    APP_PROCESS_CMD,

	/* TODO: Define states used by the application state machine. */

} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;
    uint8_t nRF_status; // Indicate if it is busy
    uint8_t data;       // Place holder for nRF24L01+ status register
    uint8_t cmd;
    

    /* TODO: Define any additional data used by the application. */

} APP_DATA;

volatile APP_DATA appData;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
void  APP_SPI_BufferEventHandler(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void * context);
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

void nRF24L01p_PowerUp(void);
void nRF24L01p_PTX_config(void);
void nRF24L01p_PRX_config(void);

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

