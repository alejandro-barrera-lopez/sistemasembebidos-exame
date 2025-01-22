unsigned int reverse_int1(unsigned int in)
{
    unsigned int out = 0;
    // Devolve o enteiro invertido bit a bit

    for (unsigned int i=0; i<32; i++) {
        out <<= 1;
        out |= in & 1;
        in >>= 1;
    }

    return out;
}