/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"
#include "insgps.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************


 
static void _SYS_Tasks ( void );
static void _APP_Tasks(void);
static void vTaskMyDelay(void*);
static void vTaskNavPrediction(void *);
static void vTaskNavCorrection(void *);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    BaseType_t xReturned;
    
    //BSP_LEDToggle(YELLOW);
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                240, NULL, 1, NULL);

    /* Create OS Thread for APP Tasks. */
    xTaskCreate((TaskFunction_t) _APP_Tasks,
                "APP Tasks",
                480, NULL, 1, &px_APP_TasksHandle);
    
    xReturned = xTaskCreate((TaskFunction_t) vTaskMyDelay,
                "Delay",
                128, NULL, 1, NULL);
    
    
    // Navigation prediction task
    xReturned = xTaskCreate((TaskFunction_t) vTaskNavPrediction,
                "Predic",
                580, NULL, 1, &pxTaskNavPredictionHandle);

    // Navigation correction task
    xReturned = xTaskCreate((TaskFunction_t) vTaskNavCorrection,
                "Correc",
                240, NULL, 1, &pxTaskNavCorrectionHandle);

    
    //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_2);
    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( void)
{
    while(1)
    {
        /* Maintain system services */
        SYS_DEVCON_Tasks(sysObj.sysDevcon);

        /* Maintain Device Drivers */

        /* Maintain Middleware */

        /* Task Delay */
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _APP_Tasks ( void )

  Summary:
    Maintains state machine of APP.
*/

static void _APP_Tasks(void)
{
    while(1)
    {
        APP_Tasks();
        //vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void vTaskMyDelay(void* pvParameters)
{
    TickType_t xLastWakeTime;
        
    xLastWakeTime = xTaskGetTickCount();  
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, (333 / portTICK_PERIOD_MS));       
        // Toggle LED1
        Nop();
        //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_1);
        //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G, CSN_PIN);
        //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G, CE_PIN);

        //taskYIELD();
    }

}

/*IMU measurements are processed*/
static void vTaskNavPrediction(void * pvParameters)
{
    float gyro_data[3], accel_data[3], dT;
    
    
    // Initialize position
    INSGPSInit();
    
    gyro_data[0] = 0.1;
    gyro_data[1] = 0.3;
    gyro_data[2] = 0.5;
    
    accel_data[0] = 0.3;
    accel_data[1] = 0.6;
    accel_data[2] = 0.8;
    
    dT = 10e-3;
    mag_data[0] = 0.1;
    mag_data[1] = 0.01;
    mag_data[2] = 0.2;
    
    Pos[0] = 1.2;
    Pos[1] = 1.33;
    Pos[2] = 2.5;
    
    Vel[0] = 3.3;
    Vel[1] = 2.3;
    Vel[2] = 5.7;
    
    BaroAlt = 2.1;
            

    INSStatePrediction(gyro_data, accel_data, dT);

    while(1)
    {
        // Wait for INT1 external interrupt, IMU data ready.
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); /* Block indefinitely. */ 
        
        // Read IMU data
        
        // Compute navigation estimate
        INSStatePrediction(gyro_data, accel_data, dT);

        
    }
}

// GPS measurements are included in the state estimation
static void vTaskNavCorrection(void * pvParameters)
{
    DRV_HANDLE usart_gps_handle;
    uint8_t msg_gps[GPS_BUFFER_SIZE], c_tmp[4];
    uint8_t k;
    uint8_t *c_ptr1;
    DRV_USART_BUFFER_HANDLE gps_buffer_handle;
    bool msgReceived, validData;
    char *c_ptr2;
    unsigned char b_cnt; // byte counter
    float v_mag, v_dir, lat, lon;
    

    
    // USART module opening
    usart_gps_handle = DRV_USART_Open(DRV_USART_INDEX_0,
            DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_BLOCKING);
    
    // GPS config
    
    // Receive GPS messages
    while(1){
    
        msgReceived = false;
        b_cnt = 0;  // Buffer byte element pointer
        do{
            // Next function call blocks the task until BYTES_TO_READ are available
            DRV_USART_Read(usart_gps_handle, &msg_gps[b_cnt], BYTES_TO_READ); 

            if((msg_gps[b_cnt] == '$')){
                b_cnt = 0; // GPS beginning message
            }
            if(msg_gps[b_cnt] == '\n'){
                msgReceived = true;
                msg_gps[b_cnt+1] = NULL; // Terminate string with null
            }
            b_cnt++; 
            if(b_cnt >= GPS_BUFFER_SIZE - 1){
                b_cnt = 0;
            }
        }while(!msgReceived);

        //BSP_LEDToggle(GREEN);
        
        // Extract Lat, Lon, vel north, vel east from GPS RMC message
        c_ptr1 = strtok(msg_gps, ","); // Tokenize GPS message
        validData = true;
        for(k = 0; (c_ptr1 != NULL) && (validData); k++){
            switch(k){
                case 2: // Read status: A = data valid; V = data not valid
                    if(strstr(c_ptr1, "V") != NULL){
                        validData = false;
                    }
                    break;
                case 3: // Latitude (ddmm.mmmm)
                    strncpy(c_tmp, c_ptr1, 2); // Copy first two characters
                    c_tmp[2] = 0; // Null terminate the string
                    lat = strtof(c_tmp,&c_ptr2) + strtof(c_ptr1+2,&c_ptr2)/60.0;
                    break;
                case 4: // Latitude N/S indicator: N+; S-
                    if(strstr(c_ptr1, "S") != NULL){
                        lat = -lat;
                    }
                    break;
                case 5: // Longitude (dddmm.mmmm)
                    strncpy(c_tmp, c_ptr1, 3); // Copy first three characters
                    c_tmp[3] = 0; // Null terminate the string
                    lon = strtof(c_tmp,&c_ptr2) + strtof(c_ptr1+3,&c_ptr2)/60.0;                    
                    break;
                case 6: // Longitude East/West: E=east+; W=west-
                    if(strstr(c_ptr1, "W") != NULL){
                        lon = -lon;
                    }
                    break;
                case 7: // Speed over ground:
                    v_mag = strtof(c_ptr1,&c_ptr2);
                    break;
                case 8: // Course over ground: True. 
                    /*SiRF Technology Inc. does not support magnetic declination.
                    * All "course over ground2 data are geodetic WGS48
                     *  directions.*/
                    v_dir = strtof(c_ptr1,&c_ptr2);
                    v_dir = v_dir*D2R;  // Degrees to radian conversion
                    break;
            }
            c_ptr1 = strtok(NULL, ","); // Read next token
        }
        
        
        // Compute correction step
        FullCorrection(mag_data, Pos, Vel, BaroAlt);
    }
    
    
}


/*******************************************************************************
 End of File
 */

