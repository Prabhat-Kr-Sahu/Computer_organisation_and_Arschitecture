// test_multiply.c
// Chains of dependent multiplications implemented via repeated addition.
// Small test using volatile globals so compiler doesn't optimize away memory accesses.

volatile int a = 4;
volatile int b = 3;
volatile int c;
volatile int d;
volatile int e;

// Magic finish address that the testbench watches.
// Writing the final result here signals test completion.
// Ensure your data memory maps this address (0x00000FFC -> word index 1023 for 1024-word memory).
#define TEST_FINISH_ADDR ((volatile int *)0x00000FFC)

// Simple multiply using repeated addition. Handles sign of operands.
int multiply(int x, int y)
{
    int sign = 1;
    if (x < 0)
    {
        x = -x;
        sign = -sign;
    }
    if (y < 0)
    {
        y = -y;
        sign = -sign;
    }
    int result = 0;
    for (int i = 0; i < y; ++i)
        result += x;
    return sign * result;
}

int main()
{
    // Chain of dependent multiplications:
    // a = 4, b = 3
    // c = a * b = 4 * 3 = 12
    // d = c * a = 12 * 4 = 48
    // e = d * b = 48 * 3 = 144
    c = multiply(a, b);
    d = multiply(c, a);
    e = multiply(d, b);

    // Write the final result to the "finish" address.
    // Expected final value: 144 decimal == 0x00000090
    *TEST_FINISH_ADDR = e;

    // Halt (infinite loop) — testbench will detect the store and finish simulation.
    while (1)
        ;
}
