.syntax unified
    .cpu cortex-m0
    .thumb
    .global reverse_int3
    .type reverse_int3, %function

reverse_int3:
    push {r4, lr}

    ldr r1, =0x55555555
    movs r2, r0
    ands r2, r1
    lsrs r0, r0, #1
    ands r0, r1
    lsls r2, r2, #1
    orrs r0, r2

    ldr r1, =0x33333333
    movs r2, r0
    ands r2, r1
    lsrs r0, r0, #2
    ands r0, r1
    lsls r2, r2, #2
    orrs r0, r2

    ldr r1, =0x0F0F0F0F
    movs r2, r0
    ands r2, r1
    lsrs r0, r0, #4
    ands r0, r1
    lsls r2, r2, #4
    orrs r0, r2

    movs r1, r0
    lsrs r0, #16
    lsls r2, r1, #16
    orrs r0, r2

    movs r1, r0
    lsrs r0, #8
    lsls r2, r1, #8
    movs r1, #0xFF
    lsls r1, #8
    bics r0, r1
    orrs r0, r2

    pop {r4, pc}

    .align 4
constants:
    .word 0x55555555
    .word 0x33333333
    .word 0x0F0F0F0F
