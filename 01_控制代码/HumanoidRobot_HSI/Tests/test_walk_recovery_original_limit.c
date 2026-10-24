/* Run the original recovery regression without changing its 5-degree limit. */
#define main original_three_modes_main
#include "test_three_modes.c"
#undef main
int main(void) { TestWalkRecoveryContinuity(); return 0; }
