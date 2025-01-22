// Devolve o enteiro invertido bit a bit
unsigned int reverse_int4(unsigned int n)
{
    // Paso 1: Invertir bits pares e impares
    n = ((n & 0x55555555) << 1) | ((n >> 1) & 0x55555555);

    // Paso 2: Invertir grupos de 2 bits, para obter cuartetos xa invertidos
    n = ((n & 0x33333333) << 2) | ((n >> 2) & 0x33333333);

    // Paso 3: Invertir cuartetos, obtendo 4 bytes xa completamente invertidos
    n = ((n & 0x0F0F0F0F) << 4) | ((n >> 4) & 0x0F0F0F0F);

    // Paso 4: Invertir a orde dos 4 bytes
    n = __builtin_bswap32(n);
    // Función propia de GCC para invertir bytes nunha palabra de 32 bits
    // https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html

    return n;
}