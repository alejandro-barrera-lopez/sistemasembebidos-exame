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

#include "pin_mux.h"
/**
 * @brief Este exercicio consiste na implementación dun autómata simple que simule un
 sistema de seguridade con dúas portas. O sistema permite abrir e pechar cada unha das
 portas, alertando mediante os LEDs se o habitáculo está asegurado (ambas portas ben
 pechadas) ou non.

    O sistema fará uso dos dous botóns da placa, xestionados mediante interrupcións:
        - O botón esquerdo (collendo a placa pola zona do slider, a parte contraria
        aos conectores USB) permite abrir/pechar a porta 1. Cando se pulsa, abre a
        porta se estaba pechada, e péchaa se estaba aberta.
        - O botón dereito fai o mesmo coa porta 2.
    A saída do sistema indicará o estado das portas deste xeito:
        - UNSAFE: sinal LED verde ACENDIDO, o que indica que unha ou as dúas portas
        están abertas. Se as dúas portas están ben pechadas, este LED estaría APAGADO.
        - SAFE: sinal LED vermello ACENDIDO, cando as dúas portas están ben pechadas.
        En caso contrario (algunha das portas está aberta), APAGADO.

    As dúas portas están inicialmente abertas. Só un dos dous LEDs debería estar
    acendido nun momento dado.
 *
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_GREEN (1U << 5)  // PTD5
#define LED_RED   (1U << 29) // PTE29
#define BTN_RIGHT (1U << 3)  // PTC3
#define BTN_LEFT  (1U << 12) // PTC12


typedef enum
{
  PORTA_ABERTA = 0,
  PORTA_PECHADA
} porta_state_t;

typedef enum
{
  UNSAFE = 0,
  SAFE
} seguridade_state_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/



/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile porta_state_t porta1_state = PORTA_ABERTA;
volatile porta_state_t porta2_state = PORTA_ABERTA;
volatile seguridade_state_t seguridade_state = UNSAFE;

/*******************************************************************************
 * Code
 ******************************************************************************/
void init_leds(void) {
    // Enable clock for PORTD and PORTE
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    // Configure PTD5 (green LED) as GPIO
    PORTD->PCR[5] = PORT_PCR_MUX(1);
    GPIOD->PDDR |= LED_GREEN;

    // Configure PTE29 (red LED) as GPIO
    PORTE->PCR[29] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= LED_RED;

    // Initialize both LEDs to off state
    GPIOD->PSOR = LED_GREEN;
    GPIOE->PSOR = LED_RED;
}

void init_buttons(void) {
    // Enable clock for PORTC
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Configure PTC3 (right button) and PTC12 (left button) as GPIO with pull-up
    PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    // Configure buttons as inputs
    GPIOC->PDDR &= ~(BTN_LEFT | BTN_RIGHT);

    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void setup_io(void)
{
    // Habilitar reloxos de portos
    SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK);

    // Configurar LEDs
    PORTD->PCR[LED_GREEN] = PORT_PCR_MUX(1U);
    PORTE->PCR[LED_RED] = PORT_PCR_MUX(1U);
    GPIOE->PDDR = (1U << LED_RED);
    GPIOD->PDDR = (1U << LED_GREEN);

    // Apagar LEDs inicialmente
    GPIOE->PSOR = (1U << LED_RED);
    GPIOD->PSOR = (1U << LED_GREEN);

    // Configurar botóns
    PORTC->PCR[BTN_RIGHT] = PORT_PCR_MUX(1U);
    PORTC->PCR[BTN_LEFT] = PORT_PCR_MUX(1U);
    PORTC->PCR[BTN_RIGHT] |= (PORT_PCR_PE(1U) | PORT_PCR_PS(1U));
    PORTC->PCR[BTN_LEFT] |= (PORT_PCR_PE(1U) | PORT_PCR_PS(1U));
    PORTC->PCR[BTN_RIGHT] |= PORT_PCR_IRQC(0xA);
    PORTC->PCR[BTN_LEFT] |= PORT_PCR_IRQC(0xA);
    GPIOC->PDDR &= ~(1U << BTN_RIGHT);
    GPIOC->PDDR &= ~(1U << BTN_LEFT);

    // Habilitar interrupcións
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void disable_button_interrupts(void)
{
    PORTC->PCR[BTN_RIGHT] &= ~PORT_PCR_IRQC_MASK;  // Desactivar interrupción no botón dereito
    PORTC->PCR[BTN_LEFT] &= ~PORT_PCR_IRQC_MASK;  // Desactivar interrupción no botón esquerdo
}

void inline disable_watchdog(void) {
    // Disable watchdog timer
    SIM->COPC = 0;
}

void actualizar_leds(void) {
    if (seguridade_state == SAFE) {
        GPIO_PortSet(GPIOE, LED_RED);
        GPIO_PortClear(GPIOE, LED_GREEN);
    } else {
        GPIO_PortSet(GPIOE, LED_GREEN);
        GPIO_PortClear(GPIOE, LED_RED);
    }
}


void PORTC_PORTD_IRQHandler(void) {
  /* Clear external interrupt flag. */
  // GPIO_PortClearInterruptFlags(GPIOB, 1U << 2);
  // /* Cambiar estado da porta 1 */
  // if (porta1_state == PORTA_ABERTA) {
  //   porta1_state = PORTA_PECHADA;
  // } else {
  //   porta1_state = PORTA_ABERTA;
  // }

  // /* Cambiar estado de seguridade */
  // if (porta1_state == PORTA_PECHADA && porta2_state == PORTA_PECHADA) {
  //   seguridade_state = SAFE;
  // } else {
  //   seguridade_state = UNSAFE;
  // }

  // /* Actualizar LEDs */
  // actualizar_leds();

  GPIOD->PCOR = (1U << LED_GREEN); // Encender verde
  GPIOE->PCOR = (1U << LED_RED); // Apagar vermello

}


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

  disable_watchdog();
  setup_io();


  PRINTF("Plantilla exame Sistemas Embebidos: 1a oportunidade 24/25 Q1\r\n");

  while (1)
    {
      ch = GETCHAR();
      PUTCHAR(ch);
    }
}
