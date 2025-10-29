// Use 'volatile' to ensure the compiler doesn't optimize
// away your calculations.
volatile int a = 10;
volatile int b = 12;
volatile int c;
volatile int d;
volatile int e;

// This is the "magic" address your testbench will watch.
// When you write to this, the testbench knows the test is done.
// #define TEST_FINISH_ADDR ((volatile int*)0x80000000)

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
    // This is your "chain of dependent multiplications" implemented via loop-add
    c = multiply(a, b); // c = 10 * 12 = 120
    d = multiply(c, a); // d = 120 * 10 = 1200
    e = multiply(d, b); // e = 1200 * 12 = 14400

    // Write the final result to the "finish" address.
    // 14400 in hex is 0x3840
    *TEST_FINISH_ADDR = e;

    // Infinite loop to "halt" the processor.
    while (1)
        ;
}