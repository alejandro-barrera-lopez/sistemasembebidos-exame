.syntax unified
    .cpu cortex-m0
    .thumb
    .global reverse_int2
    .type reverse_int2, %function

reverse_int2:
    movs r3, r0
    push {lr}
    movs r2, #32
    movs r0, #0

.L2:
    lsls r0, r0, #1
    lsrs r3, r3, #1
    bcc .L3
    adds r0, r0, #1
.L3:
    subs r2, #1
    bne .L2

    pop {pc}
