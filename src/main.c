// FreeRTOS headers
#include "FreeRTOS.h"
// #include "portable.h"
// #include "portmacro.h"
// #include "projdefs.h"
// #include "queue.h"
// #include "semphr.h"
// #include "task.h"

// Toolchain headers
#include <stddef.h>
#include <stdint.h>



int main(void) {
  vTaskStartScheduler();
  while (1)
    ;
}

