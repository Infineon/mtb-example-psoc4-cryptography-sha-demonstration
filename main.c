/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4 Cryptography: SHA
 * Demonstration Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
*******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "string.h"
#include "stdio.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
/* The input message size (inclusive of the string terminating character '\0').
 * Edit this macro to suit your message size */
#define MAX_MESSAGE_SIZE                    (100u)

/* Size of the message digest for SHA-256 encryption */
#define MESSAGE_DIGEST_SIZE                 (32u)

#define UART_TIMEOUT_MS                     (0u)

#define SCREEN_HEADER "\r\n__________________________________________________"\
                      "____________________________\r\n*\t\tCE236441 PSoC 4 "\
                      "Cryptography: SHA demonstration\r\n*\r\n*\tThis code example "\
                      "shows how to generate a 32-byte hash value for an\r\n*\t"\
                      "arbitrary user input message using the SHA2 algorithm "\
                      "in PSoC 4\r\n*\tUART Terminal Settings: Baud Rate - "\
                      "115200 bps, 8N1\r\n*"\
                      "\r\n__________________________________________________"\
                      "____________________________\r\n"

#define SCREEN_HEADER1 "\r\n\n_______________________________________________" \
                       "____________________________\r\n"

/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
#define CLEAR_SCREEN         "\x1b[2J\x1b[;H"

/* Number of bytes per line to be printed on the UART terminal */
#define BYTES_PER_LINE                        (16u)

/* The get_char function timed out with no received data */
#define CY_RSLT_ERR_CSP_UART_GETC_TIMEOUT               \
        (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_DRIVER_SCB, 1))

/*******************************************************************************
 * Data type definitions
 ********************************************************************************/
/* Data type definition to track the state machine accepting the user message */
typedef enum
{
    MESSAGE_ENTER_NEW,
    MESSAGE_READY,
    MESSAGE_NOT_READY
} message_status_t;

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void print_msg_digest(uint8_t *data, uint8_t len);

cy_rslt_t uart_getc(uint8_t *value, uint32_t time_out);

cy_rslt_t uart_putc(uint32_t value);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* UART context structure */
cy_stc_scb_uart_context_t uart_context;

/* Variables to hold the user message and the corresponding message digest */
CY_ALIGN(4) uint8_t message[MAX_MESSAGE_SIZE];
CY_ALIGN(4) uint8_t hash[MESSAGE_DIGEST_SIZE];

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* System entrance point. This function performs
*    1.Initializes the BSP.
*    2.Enable the crypto block.
*    3.Perform the message digest generation using SHA-256 algorithm.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_rslt_t uart_result = CY_RSLT_SUCCESS;
    cy_en_crypto_status_t crypto_status = CY_CRYPTO_NOT_INITIALIZED;

    /* Variable to track the status of the message entered by the user. */
    message_status_t msg_status = MESSAGE_ENTER_NEW;

    uint8_t msg_size = 0;

    /* Initialize the device and board peripherals. */
    result = cybsp_init();

    /* Board init failed. Stop program execution. */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initializes the SCB for debug UART port */
    result = Cy_SCB_UART_Init(UART_HW, &UART_config, &uart_context);

    /* SCB UART init failed. Stop program execution */
    if (result != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen. */
    Cy_SCB_UART_PutString(UART_HW, CLEAR_SCREEN);

    Cy_SCB_UART_PutString(UART_HW, SCREEN_HEADER);

    /* Enable the Crypto block. */
    crypto_status = Cy_Crypto_Enable(CRYPTO);

    /* Crypto init failed. Stop program execution */
    if (crypto_status != CY_CRYPTO_SUCCESS)
    {
        CY_ASSERT(0);
    }

    for (;;)
    {
        switch (msg_status)
        {
        case MESSAGE_ENTER_NEW:
            memset(message, 0, MAX_MESSAGE_SIZE);
            msg_size = 0;
            Cy_SCB_UART_PutString(UART_HW, "\r\nEnter the message:\r\n");
            msg_status = MESSAGE_NOT_READY;
            break;

        case MESSAGE_NOT_READY:
            uart_result = uart_getc(&message[msg_size], UART_TIMEOUT_MS);
            if (uart_result == CY_RSLT_SUCCESS)
            {
                /* Check if the Enter key is pressed. If pressed,
                 * set the message status as MESSAGE_READY.
                 */
                if (message[msg_size] == '\n' || message[msg_size] == '\r')
                {
                    message[msg_size] = '\0';
                    msg_status = MESSAGE_READY;
                }
                else
                {
                    /* Check if the Backspace key is pressed. If pressed,
                     * decrement the message size and print a space character
                     * to overwrite the previous character.
                     */
                    if (message[msg_size] != '\b')
                    {
                        uart_putc(message[msg_size]);
                        msg_size++;
                    }
                    else if (msg_size > 0)
                    {
                        msg_size--;
                        Cy_SCB_UART_PutString(UART_HW, "\b \b");
                    }

                    /* Check if size of the message exceeds MAX_MESSAGE_SIZE
                     * (inclusive of the string terminating character '\0')
                     */
                    if (msg_size > (MAX_MESSAGE_SIZE - 1))
                    {
                        Cy_SCB_UART_PutString(UART_HW, "\r\n\nMessage length exceeds maximum characters!!!"
                                                       " Please enter a shorter message\r\nor edit the macro"
                                                       " MAX_MESSAGE_SIZE to suit your message size\r\n");
                        msg_status = MESSAGE_ENTER_NEW;
                        break;
                    }
                }
            }
            /* Time out occurred. */
            else if (uart_result == CY_RSLT_ERR_CSP_UART_GETC_TIMEOUT)
            {
                Cy_SCB_UART_PutString(UART_HW, "\r\n\nMessage entry timed out!"
                                               " Please enter the message within the timeout period\r\n"
                                               "or edit the macro UART_TIMEOUT_MS to change the"
                                               " timeout period.\r\n\n");
                /* set Message status to accept new message */
                msg_status = MESSAGE_ENTER_NEW;
            }
            break;

        case MESSAGE_READY:
            /* Perform the message digest generation using SHA-256
             * algorithm.
             */
            crypto_status = Cy_Crypto_Sha(CRYPTO, message, msg_size,
                                          hash, CY_CRYPTO_MODE_SHA256);

            if (crypto_status == CY_CRYPTO_SUCCESS)
            {
                Cy_SCB_UART_PutString(UART_HW, "\r\n\nHash value for the message:\r\n\n");
                print_msg_digest(hash, MESSAGE_DIGEST_SIZE);
            }

            msg_status = MESSAGE_ENTER_NEW;
            break;
        }
    }
}

/*******************************************************************************
* Function Name: print_msg_digest()
********************************************************************************
* Summary: Function used to display the data in hexadecimal format
*
* Parameters:
*  uint8_t *data - Pointer to location of data to be printed
*  uint8_t  len  - length of data to be printed
*
* Return:
*  void
*
*******************************************************************************/
void print_msg_digest(uint8_t *data, uint8_t len)
{
    char print[10];
    for (uint32_t i = 0; i < len; i++)
    {
        if ((i % BYTES_PER_LINE) == 0)
        {
            Cy_SCB_UART_PutString(UART_HW, "\r\n");
        }

        sprintf(print, "0x%02X ", *(data + i));
        Cy_SCB_UART_PutString(UART_HW, print);
    }
    Cy_SCB_UART_PutString(UART_HW, "\r\n");
    Cy_SCB_UART_PutString(UART_HW, SCREEN_HEADER1);
}

/*******************************************************************************
* Function Name: uart_getc
********************************************************************************
*
* Description:
*  Get a character. This is a blocking call which waits till a character is
*  received.
*
* Parameters:
*  uint8_t    *value - The value read from the serial port.
*  uint32_t  time_out - The time in ms to spend attempting to receive from serial 
*                      port. Zero is wait forever.
*
* Return:
*  The status of the getc request.
*
*******************************************************************************/
cy_rslt_t uart_getc(uint8_t *value, uint32_t time_out)
{
    uint32_t read_value = Cy_SCB_UART_Get(UART_HW);
    uint32_t time_out_Ticks = time_out;
    while (read_value == CY_SCB_UART_RX_NO_DATA)
    {
        if (time_out != 0UL)
        {
            if (time_out_Ticks > 0UL)
            {
                Cy_SysLib_Delay(1);
                time_out_Ticks--;
            }
            else
            {
                return CY_RSLT_ERR_CSP_UART_GETC_TIMEOUT;
            }
        }
        read_value = Cy_SCB_UART_Get(UART_HW);
    }
    *value = (uint8_t)read_value;
    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: uart_putc
********************************************************************************
*
* Description:
*  Send a character. This is a blocking call which waits till the character is sent
*  out from the UART completely.
*
* Parameters:
* uint32_t  value - The character to be sent.
*
* Return:
*  The status of the putc request.
*
*******************************************************************************/
cy_rslt_t uart_putc(uint32_t value)
{
    uint32_t count = 0;
    while (count == 0)
    {
        count = Cy_SCB_UART_Put(UART_HW, value);
    }

    return CY_RSLT_SUCCESS;
}

/* [] END OF FILE */

