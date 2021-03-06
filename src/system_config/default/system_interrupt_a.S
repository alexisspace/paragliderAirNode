/*******************************************************************************
 System Interrupt Source File

  File Name:
    sys_interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the 
    interrupt sub-system.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
/*
*********************************************************************************************************
*                                           INCLUDES
*********************************************************************************************************
*/
#include <xc.h>



#include "ISR_Support.h"


/* External Interrupt Instance 0 Interrupt */
   .extern  IntHandlerExternalInterruptInstance0

   .section	.vector_7,code, keep
   .equ     __vector_dispatch_7, IntVectorExternalInterruptInstance0
   .global  __vector_dispatch_7
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorExternalInterruptInstance0

IntVectorExternalInterruptInstance0:
    la    $26,  _IntHandlerExternalInterruptInstance0         
    jr    $26
    nop
	.end	IntVectorExternalInterruptInstance0

   .section	.ExternalInterruptInstance0_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerExternalInterruptInstance0

_IntHandlerExternalInterruptInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerExternalInterruptInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerExternalInterruptInstance0


/* External Interrupt Instance 1 Interrupt */
   .extern  IntHandlerExternalInterruptInstance1

   .section	.vector_11,code, keep
   .equ     __vector_dispatch_11, IntVectorExternalInterruptInstance1
   .global  __vector_dispatch_11
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorExternalInterruptInstance1

IntVectorExternalInterruptInstance1:
    la    $26,  _IntHandlerExternalInterruptInstance1         
    jr    $26
    nop
	.end	IntVectorExternalInterruptInstance1

   .section	.ExternalInterruptInstance1_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerExternalInterruptInstance1

_IntHandlerExternalInterruptInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerExternalInterruptInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerExternalInterruptInstance1



/* USART Instance 0 Interrupt */

   .extern  IntHandlerDrvUsartInstance0

   .section	.vector_24,code, keep
   .equ     __vector_dispatch_24, IntVectorDrvUsartInstance0
   .global  __vector_dispatch_24
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartInstance0

IntVectorDrvUsartInstance0:
    la    $26,  _IntHandlerDrvUsartInstance0         
    jr    $26
    nop
	.end	IntVectorDrvUsartInstance0

   .section	.DrvUsartInstance0_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerDrvUsartInstance0

_IntHandlerDrvUsartInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerDrvUsartInstance0




/* SPI Instance 0 Interrupt */

   .extern  IntHandlerSPIInstance0

   .section	.vector_23,code, keep
   .equ     __vector_dispatch_23, IntVectorSPIInstance0
   .global  __vector_dispatch_23
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIInstance0

IntVectorSPIInstance0:
    la    $26,  _IntHandlerSPIInstance0         
    jr    $26
    nop
	.end	IntVectorSPIInstance0

   .section	.SPIInstance0_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandlerSPIInstance0

_IntHandlerSPIInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandlerSPIInstance0


/*******************************************************************************
 End of File
 */

