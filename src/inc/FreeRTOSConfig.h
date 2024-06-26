#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifndef CMSIS_device_header
#define CMSIS_device_header "cmsis.h"
#endif /* CMSIS_device_header */

/*-----------musca-a specific defines---------------*/
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configKERNEL_PROVIDED_STATIC_MEMORY 1
#define USE_FreeRTOS_HEAP_4


#define configENABLE_TRUSTZONE                   0
#define configRUN_FREERTOS_SECURE_ONLY           0
#define configENABLE_FPU                         0
#define configENABLE_MPU                         0

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 48000000 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 5 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 130 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 10 * 1024 ) )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_TRACE_FACILITY                0
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1

/* Software timer definitions. */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                5
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xQueueGetMutexHolder            1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_eTaskGetState                   1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
  /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
  #define configPRIO_BITS       __NVIC_PRIO_BITS
#else
  #define configPRIO_BITS       4        /* 15 priority levels */
#endif

/* The lowest priority. */
#define configKERNEL_INTERRUPT_PRIORITY         ( 31 << (8 - configPRIO_BITS) )
/* Priority 5, or 160 as only the top three bits are implemented. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( 5 << (8 - configPRIO_BITS) )

#endif /* FREERTOS_CONFIG_H */