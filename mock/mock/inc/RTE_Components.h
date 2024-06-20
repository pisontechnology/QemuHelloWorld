
/*
 * I think this points to CMSIS RTOS dependencies.
 * It points to FreeRTOS stuff in here because that's backend of CMSIS-RTOS we're using
 * Project: 'RTX_CM' No clue what this is, copied from another project
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

#include "FreeRTOSConfig.h"

void vPortFree(void *pv) {}

#endif /* RTE_COMPONENTS_H */
