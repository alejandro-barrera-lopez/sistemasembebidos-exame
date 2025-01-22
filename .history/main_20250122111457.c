/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "lcd.h"

#include "pin_mux.h"
/**
 * @brief A tarefa consiste en programar o acendido e apagado dun dos dous LEDs
 (un calquera, o que prefirades) cunha frecuencia determinada de entre as seguintes:

    1 Hz (1 acendido cada segundo)
    2 Hz (2 acendidos cada segundo, isto é, acéndese cada medio segundo)
    0.5 Hz (1 acendido cada 2 segundos)
    0 Hz (o LED permanece apagado)

O sistema permitirá conmutar entre eses 4 estados do LED mediante os dous botóns,
seguindo esta progresión:

0 Hz <-> 0.5 Hz <-> 1 Hz <-> 2 Hz

O botón da dereita, incremente a velocidade (ata o máximo de 2 Hz), e o da esquerda
diminúea (ata estar o LED apagado). O sistema comezará acendendo o LED cunha
frecuencia de 1 Hz.

O LCD da placa mostrará en todo momento os Hz cos que está a funcionar o LED.

Podedes empregar o temporizador da placa que prefirades para levar a conta do tempo,
agás o SysTick.
 *
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RED_LED 29U
#define GREEN_LED 5U
#define SW1 3U  // Botón dereito - LED verde

typedef enum
{
    LED_OFF = 0,
    LED_05HZ = 2,
    LED_1HZ = 1,
    LED_2HZ = 3
} led_state_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile led_state_t led_state = LED_1HZ;

/*******************************************************************************
 * Code
 ******************************************************************************/
void setup_io(void);
void disable_button_interrupts(void);
void disable_watchdog(void);



/*!
 * @brief Main function
 */
int main(void)
{
  char ch;

  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  PRINTF("Plantilla exame Sistemas Embebidos: 1a oportunidade 24/25 Q1\r\n");

  while (1)
    {
      ch = GETCHAR();
      PUTCHAR(ch);
    }

  disable_button_interrupts();
}
