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
#define LED_GREEN (5U)  // PTD5
// #define LED_RED   (29U) // PTE29
#define BTN_RIGHT (3U)  // PTC3
#define BTN_LEFT  (12U) // PTC12

typedef enum
{
    LED_OFF = 0,
    LED_05HZ,
    LED_1HZ,
    LED_2HZ
} led_state_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void irclk_ini(void);
void setup_io(void);
void disable_button_interrupts(void);
void disable_watchdog(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile led_state_t led_state = LED_1HZ;

/*******************************************************************************
 * Code
 ******************************************************************************/
void irclk_ini()
{
  // Habilitar reloxo interno
  MCG->C1 = MCG->C1 | MCG_C1_IRCLKEN(1);
  MCG->C1 = MCG->C1 | MCG_C1_IREFSTEN(1);

  // // Limpar bit IRCS
  MCG->C2 = MCG->C2 & ~MCG_C2_IRCS_MASK;
}

void setup_io(void) {
    // Habilitar reloxos de portos
    SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK);

    // Configurar LED verde
    PORTD->PCR[LED_GREEN] = PORT_PCR_MUX(1U);
    GPIOD->PDDR = (1U << LED_GREEN);
    GPIOD->PCOR = (1U << LED_GREEN); // Apagado por defecto

    // Configurar botóns
    PORTC->PCR[BTN_RIGHT] = PORT_PCR_MUX(1U);
    PORTC->PCR[BTN_LEFT] = PORT_PCR_MUX(1U);
    PORTC->PCR[BTN_RIGHT] |= (PORT_PCR_PE(1U) | PORT_PCR_PS(1U));
    PORTC->PCR[BTN_LEFT] |= (PORT_PCR_PE(1U) | PORT_PCR_PS(1U));
    PORTC->PCR[BTN_RIGHT] |= PORT_PCR_IRQC(0xA);
    PORTC->PCR[BTN_LEFT] |= PORT_PCR_IRQC(0xA);
    GPIOC->PDDR &= ~(1U << BTN_RIGHT);
    GPIOC->PDDR &= ~(1U << BTN_LEFT);

    // Habilitar interrupcións para os botóns
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);

    // Inicializar LCD
    irclk_ini();
    lcd_ini();
}

void inline disable_button_interrupts(void)
{
    PORTC->PCR[BTN_RIGHT] &= ~PORT_PCR_IRQC_MASK;  // Desactivar interrupción no botón dereito
    PORTC->PCR[BTN_LEFT] &= ~PORT_PCR_IRQC_MASK;  // Desactivar interrupción no botón esquerdo
}

void inline disable_watchdog(void) {
    // Disable watchdog timer
    SIM->COPC = 0;
}

void display_led_state(void)
{
  lcd_set(led_state == LED_05HZ ? 5 : 0, 3);

  switch (led_state) {
    case LED_OFF:
      lcd_set(0, 2);
      break;
    case LED_05HZ:
      lcd_set(0, 2);
      break;
    case LED_1HZ:
      lcd_set(1, 2);
      break;
    case LED_2HZ:
      lcd_set(2, 2);
      break;
    default:
      break;
  }

  SegLCD_DP2_On();

  print_debug();
}

void print_debug(void) {
    PRINTF("LED state: %d\r\n", led_state);
}

void PORTC_PORTD_IRQHandler(void) {
  // Comprobar botón esquerdo (SW3 - LED vermello)
  if ((PORTC->PCR[BTN_LEFT] >> PORT_PCR_ISF_SHIFT) & 0x1U)
  {
    PRINTF("Botón esquerdo pulsado\r\n");
    led_state = (led_state == LED_OFF) ? LED_2HZ : (led_state - 1);
    PORTC->PCR[BTN_LEFT] |= PORT_PCR_ISF(1); // Limpar interrupción
  }
  if ((PORTC->PCR[BTN_RIGHT] >> PORT_PCR_ISF_SHIFT) & 0x1U)
  {
    PRINTF("Botón dereito pulsado\r\n");
    led_state = (led_state == LED_2HZ) ? LED_OFF : (led_state + 1);
    PORTC->PCR[BTN_RIGHT] |= PORT_PCR_ISF(1); // Limpar interrupción
  }

  display_led_state();
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
  display_led_state();

  while (1)
    {
      ch = GETCHAR();
      PUTCHAR(ch);
    }

  // disable_button_interrupts();
}
