void debug_setup(void);

#ifdef DEBUG_UART

#include <stdio.h>

#else

#define printf(...)

#endif
