/*
 * FreeRTOS V11.3.0
 * Configuration for STM32H743ZITx (Cortex-M7, 400MHz)
 *
 * This file configures FreeRTOS for the 4-Axis SV660N EtherCAT
 * Master Motion Control System.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    extern uint32_t SystemCoreClock;
#endif

/*-----------------------------------------------------------
 * Kernel configuration
 *----------------------------------------------------------*/
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( SystemCoreClock )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 8 )
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 256 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 24 * 1024 ) )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_TRACE_FACILITY                1
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1
#define configGENERATE_RUN_TIME_STATS           0

#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/* Software timer definitions. */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* Set the following definitions to 1 to include the API function, or zero
   to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xQueueGetMutexHolder            1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskGetIdleTaskHandle          0

/*-----------------------------------------------------------
 * Cortex-M7 specific definitions
 * STM32H7 implements 4 priority bits (16 levels, 0=NMI...15=idle)
 *----------------------------------------------------------*/
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS                     __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS                     4
#endif

/* The lowest interrupt priority (highest numeric value). */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     0x0F

/* The highest interrupt priority that can call interrupt-safe FreeRTOS API
   functions (FromISR). Lower numeric = higher priority.
   Interrupts with priority 0..4 are above this and CANNOT call FromISR.
   Interrupts with priority 5..15 CAN call FromISR functions. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

/* Derived hardware-level values (shifted to 8-bit NVIC register format) */
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/*-----------------------------------------------------------
 * Hooks and asserts
 *----------------------------------------------------------*/
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

/* Normal assert() semantics without relying on the provision of an assert.h
   header file. */
#define configASSERT( x ) \
    if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/*-----------------------------------------------------------
 * FreeRTOS port interrupt handlers → CMSIS standard names
 * The port.c (ARM_CM4F) defines vPortSVCHandler, xPortPendSVHandler,
 * xPortSysTickHandler. These macros map them to the expected vector names.
 *----------------------------------------------------------*/
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/*-----------------------------------------------------------
 * Optional: runtime stats timer (currently disabled)
 *----------------------------------------------------------*/
#if configGENERATE_RUN_TIME_STATS
    extern volatile uint32_t CPU_RunTime;
    #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  (CPU_RunTime = 0ul)
    #define portGET_RUN_TIME_COUNTER_VALUE()           CPU_RunTime
#endif

/*-----------------------------------------------------------
 * STM32H7 FreeRTOS port note:
 * This project uses the ARM_CM4F RVDS port which works on Cortex-M7
 * (both are ARMv7-M architecture). The port.c M7 revision asserts
 * have been removed for STM32H7 compatibility.
 *----------------------------------------------------------*/

#endif /* FREERTOS_CONFIG_H */
