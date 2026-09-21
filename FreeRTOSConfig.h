#pragma once

// Minimale FreeRTOS-Konfiguration für den RP2040-Port (SMP-Kernel,
// hier aber bewusst auf 1 Kern beschränkt - siehe configNUM_CORES unten).
// NOCH NICHT AUF ECHTER HARDWARE GETESTET - erster Build/Flash-Versuch
// wird zeigen, ob configCPU_CLOCK_HZ/Stackgrößen passen. Bitte Ergebnis
// (inkl. evtl. nötiger Anpassungen) ins DEVLOG eintragen, wie bei den
// bisherigen Schritten.

#define configUSE_PREEMPTION                    1
#define configUSE_TICKLESS_IDLE                 0

// Default-Systemtakt des Pico (125 MHz). Falls du set_sys_clock_khz()
// nutzt, hier anpassen - sonst läuft der FreeRTOS-Tick falsch.
#define configCPU_CLOCK_HZ                      125000000
#define configTICK_RATE_HZ                      1000

#define configMAX_PRIORITIES                    4
#define configMINIMAL_STACK_SIZE                256
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   (64 * 1024)

#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

// Bewusst Single-Core (siehe DEVLOG): der Audio-Task braucht exklusiven,
// unkontendierten Zugriff auf den ADC. Zwei Kerne würden das Problem
// (gemeinsam genutzte ADC-Peripherie zwischen Audio- und Control-Task)
// nicht lösen, nur verschieben - der ADC bleibt ein einzelnes
// Hardware-Peripheriegerät, egal auf wie vielen Kernen Code läuft.
#define configNUM_CORES                         1
#define configTICK_CORE                         0
#define configRUN_MULTIPLE_PRIORITIES           0
#define configUSE_CORE_AFFINITY                 0

#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE

// Interop-Schicht aus, mit der man rohe pico_sync-Primitiven (mutex_t/
// semaphore_t aus dem Pico-SDK selbst) blockierend in FreeRTOS-Tasks
// nutzen könnte - wir verwenden ausschließlich FreeRTOS-eigene
// Primitiven (Queues), brauchen das also nicht. Wichtiger Nebeneffekt:
// umgeht einen Compile-Fehler im RP2040-Port (portable/ThirdParty/GCC/
// RP2040/port.c), wo dieser Codepfad event_groups.h nutzt, ohne vorher
// timers.h einzubinden ("implicit declaration of
// xEventGroupSetBitsFromISR/xTimerPendFunctionCallFromISR").
#define configSUPPORT_PICO_SYNC_INTEROP         0
#define configSUPPORT_PICO_TIME_INTEROP         0

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_eTaskGetState                   1

// portDISABLE_INTERRUPTS() statt taskDISABLE_INTERRUPTS(): letzteres ist
// ein Macro aus task.h, das configASSERT() an dieser Stelle noch nicht
// kennt - portmacro.h (RP2040-SMP-Port) nutzt configASSERT bereits
// intern (z.B. in vPortRecursiveLock), BEVOR task.h überhaupt included
// ist. portDISABLE_INTERRUPTS() ist direkt im Port definiert und daher
// unabhängig von der Include-Reihenfolge in main.cpp verfügbar.
#define configASSERT(x) if((x)==0) { portDISABLE_INTERRUPTS(); for(;;); }
