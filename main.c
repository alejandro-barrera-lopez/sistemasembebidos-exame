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
 * @brief A tarefa consiste en levar a cabo o benchmarking das distintas
 implementacións dunha función reverse_int() (invertir os bits dunha palabra de
 32 bits) presentadas nas slides8.pdf, de acordo co exercicio proposto na última
 diapositiva (e tendo en conta a última versión do PDF, con data de onte mesmo,
 21 de xaneiro).

Medirase o rendemento de 4 implementacións desta función, dúas en C e dúas en
ensamblador, empregando o temporizador SysTick para medir os ciclos de reloxo
consumidos en cada unha delas. Estas son as catro implementacións, cada unha
delas estará nun arquivo fonte diferente:

    reverse_int1(), nun arquivo fonte reverse1.c. A versión en C baseada nun bucle,
      mostrada na diapositiva 2.
    reverse_int2(), nun arquivo fonte reverse2.s. A versión en ASM máis optimizada
      da diapositiva 5 (a da dereita, vaia).
    reverse_int3(), nun arquivo fonte reverse3.s. A versión ASM sen bucle da
      diapositiva 7, que precisará varios axustes para correr no noso Cortex-M0+.
    reverse_int4(), nun arquivo fonte reverse4.c. A versión sen bucle en C
      (diapositiva 8).

O programa correrá unha vez cada unha desas implementacións, medindo os ciclos que
lle leva a cada unha, e presentará pola interfaz de porto serie o resultado do
benchmark,.
 *
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SYSTICK_CLOCK    48000000U  // Ajusta esto según tu frecuencia de reloj

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
unsigned int reverse_int1(unsigned int num);
unsigned int reverse_int2(unsigned int num);
unsigned int reverse_int3(unsigned int num);
unsigned int reverse_int4(unsigned int num);

static inline unsigned int measure_time(unsigned int (*func)(unsigned int), unsigned int value);

/*******************************************************************************
 * Variables
 ******************************************************************************/
const unsigned int num = 0x5ca1ab1e;


/*******************************************************************************
 * Code
 ******************************************************************************/
static inline unsigned int measure_time(unsigned int (*func)(unsigned int), unsigned int value) {
    unsigned int start, end, result;

    // Configura o SysTick para contar ciclos
    SysTick->LOAD = 0xFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;

    start = SysTick->VAL;
    result = func(value);
    end = SysTick->VAL;

    // Deshabilita o SysTick
    SysTick->CTRL = 0;

    // Calcula ticks
    return start - end;
}

/*!
 * @brief Main function
 */
int main(void)
{
  unsigned int ticks, result;

  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  PRINTF("\r\nBenchmarking reverse_int routines:\r\n");
  PRINTF("Number to reverse: %u\r\n", num);

  // Test reverse_int1
  ticks = measure_time(reverse_int1, num);
  result = reverse_int1(num);
  PRINTF("Elapsed ticks with reverse_int1(): %u (%u)\r\n", ticks, result);

  // Test reverse_int2
  ticks = measure_time(reverse_int2, num);
  result = reverse_int2(num);
  PRINTF("Elapsed ticks with reverse_int2(): %u (%u)\r\n", ticks, result);

  // Test reverse_int3
  ticks = measure_time(reverse_int3, num);
  result = reverse_int3(num);
  PRINTF("Elapsed ticks with reverse_int3(): %u (%u)\r\n", ticks, result);

  // Test reverse_int4
  ticks = measure_time(reverse_int4, num);
  result = reverse_int4(num);
  PRINTF("Elapsed ticks with reverse_int4(): %u (%u)\r\n", ticks, result);

  while (1) {
      __WFI();
  }
}
