 ;
 ;  user/Arch/stm32f10x_hd.s
 ;
 ;  Copyright (C) YYC
 ;
 ;  2012.4.9
 ;
 ;  启动文件：堆栈初始化及向量表初始化。
 ; 

;* <<< Use Configuration Wizard in Context Menu >>> 
; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp
                                                  
; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

				PRESERVE8
                THUMB
	   
		IMPORT  OSPendSV
		IMPORT  SysTickHandler
  		IMPORT  WWDG_IRQHandler
  		IMPORT  PVD_IRQHandler
  		IMPORT  TAMPER_IRQHandler
  		IMPORT  RTC_IRQHandler
  		IMPORT  FLASH_IRQHandler
  		IMPORT  RCC_IRQHandler
  		IMPORT  EXTI0_IRQHandler
  		IMPORT  EXTI1_IRQHandler
  		IMPORT  EXTI2_IRQHandler
  		IMPORT  EXTI3_IRQHandler
  		IMPORT  EXTI4_IRQHandler
  		IMPORT  DMA1_Channel1_IRQHandler
  		IMPORT  DMA1_Channel2_IRQHandler
  		IMPORT  DMA1_Channel3_IRQHandler
  		IMPORT  DMA1_Channel4_IRQHandler
  		IMPORT  DMA1_Channel5_IRQHandler
  		IMPORT  DMA1_Channel6_IRQHandler
  		IMPORT  DMA1_Channel7_IRQHandler
  		IMPORT  ADC1_2_IRQHandler
  		IMPORT  USB_HP_CAN1_TX_IRQHandler
  		IMPORT  USB_LP_CAN1_RX0_IRQHandler
  		IMPORT  CAN1_RX1_IRQHandler
  		IMPORT  CAN1_SCE_IRQHandler
  		IMPORT  EXTI9_5_IRQHandler
  		IMPORT  TIM1_BRK_IRQHandler
  		IMPORT  TIM1_UP_IRQHandler
  		IMPORT  TIM1_TRG_COM_IRQHandler
  		IMPORT  TIM1_CC_IRQHandler
  		IMPORT  TIM2_IRQHandler
  		IMPORT  TIM3_IRQHandler
  		IMPORT  TIM4_IRQHandler
  		IMPORT  I2C1_EV_IRQHandler
  		IMPORT  I2C1_ER_IRQHandler
  		IMPORT  I2C2_EV_IRQHandler
  		IMPORT  I2C2_ER_IRQHandler
  		IMPORT  SPI1_IRQHandler
  		IMPORT  SPI2_IRQHandler
  		IMPORT  USART1_IRQHandler
  		IMPORT  USART2_IRQHandler
  		IMPORT  USART3_IRQHandler
  		IMPORT  EXTI15_10_IRQHandler
  		IMPORT  RTCAlarm_IRQHandler
  		IMPORT  USBWakeUp_IRQHandler
        IMPORT     TIM8_BRK_IRQHandler        ; TIM8 Break
        IMPORT     TIM8_UP_IRQHandler         ; TIM8 Update
        IMPORT     TIM8_TRG_COM_IRQHandler    ; TIM8 Trigger and Commutation
        IMPORT     TIM8_CC_IRQHandler         ; TIM8 Capture Compare
        IMPORT     ADC3_IRQHandler            ; ADC3
        IMPORT     FSMC_IRQHandler            ; FSMC
        IMPORT     SDIO_IRQHandler            ; SDIO
        IMPORT     TIM5_IRQHandler            ; TIM5
        IMPORT     SPI3_IRQHandler            ; SPI3
        IMPORT     UART4_IRQHandler           ; UART4
        IMPORT     UART5_IRQHandler           ; UART5
        IMPORT     TIM6_IRQHandler            ; TIM6
        IMPORT     TIM7_IRQHandler            ; TIM7
        IMPORT     DMA2_Channel1_IRQHandler   ; DMA2 Channel1
        IMPORT     DMA2_Channel2_IRQHandler   ; DMA2 Channel2
        IMPORT     DMA2_Channel3_IRQHandler   ; DMA2 Channel3
        IMPORT     DMA2_Channel4_5_IRQHandler

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size
__Vectors 
;		DCD     Stack + Stack_Size              ; Top of Stack
				DCD		__initial_sp
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     SVC_Handler                ; SVCall Handler
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                DCD     OSPendSV             		; PendSV Handler
                DCD     SysTickHandler            ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler            ; Window Watchdog
                DCD     PVD_IRQHandler             ; PVD through EXTI Line detect
                DCD     TAMPER_IRQHandler          ; Tamper
                DCD     RTC_IRQHandler             ; RTC
                DCD     FLASH_IRQHandler           ; Flash
                DCD     RCC_IRQHandler             ; RCC
                DCD     EXTI0_IRQHandler           ; EXTI Line 0
                DCD     EXTI1_IRQHandler           ; EXTI Line 1
                DCD     EXTI2_IRQHandler           ; EXTI Line 2
                DCD     EXTI3_IRQHandler           ; EXTI Line 3
                DCD     EXTI4_IRQHandler           ; EXTI Line 4
                DCD     DMA1_Channel1_IRQHandler   ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler   ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler   ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler   ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler   ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler   ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler   ; DMA1 Channel 7
                DCD     ADC1_2_IRQHandler          ; ADC1 & ADC2
                DCD     USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
                DCD     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
                DCD     CAN1_RX1_IRQHandler        ; CAN1 RX1
                DCD     CAN1_SCE_IRQHandler        ; CAN1 SCE
                DCD     EXTI9_5_IRQHandler         ; EXTI Line 9..5
                DCD     TIM1_BRK_IRQHandler        ; TIM1 Break
                DCD     TIM1_UP_IRQHandler         ; TIM1 Update
                DCD     TIM1_TRG_COM_IRQHandler    ; TIM1 Trigger and Commutation
                DCD     TIM1_CC_IRQHandler         ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler            ; TIM2
                DCD     TIM3_IRQHandler            ; TIM3
                DCD     TIM4_IRQHandler            ; TIM4
                DCD     I2C1_EV_IRQHandler         ; I2C1 Event
                DCD     I2C1_ER_IRQHandler         ; I2C1 Error
                DCD     I2C2_EV_IRQHandler         ; I2C2 Event
                DCD     I2C2_ER_IRQHandler         ; I2C2 Error
                DCD     SPI1_IRQHandler            ; SPI1
                DCD     SPI2_IRQHandler            ; SPI2
                DCD     USART1_IRQHandler          ; USART1
                DCD     USART2_IRQHandler          ; USART2
                DCD     USART3_IRQHandler          ; USART3
                DCD     EXTI15_10_IRQHandler       ; EXTI Line 15..10
                DCD     RTCAlarm_IRQHandler        ; RTC Alarm through EXTI Line
                DCD     USBWakeUp_IRQHandler       ; USB Wakeup from suspend
                DCD     TIM8_BRK_IRQHandler        ; TIM8 Break
                DCD     TIM8_UP_IRQHandler         ; TIM8 Update
                DCD     TIM8_TRG_COM_IRQHandler    ; TIM8 Trigger and Commutation
                DCD     TIM8_CC_IRQHandler         ; TIM8 Capture Compare
                DCD     ADC3_IRQHandler            ; ADC3
                DCD     FSMC_IRQHandler            ; FSMC
                DCD     SDIO_IRQHandler            ; SDIO
                DCD     TIM5_IRQHandler            ; TIM5
                DCD     SPI3_IRQHandler            ; SPI3
                DCD     UART4_IRQHandler           ; UART4
                DCD     UART5_IRQHandler           ; UART5
                DCD     TIM6_IRQHandler            ; TIM6
                DCD     TIM7_IRQHandler            ; TIM7
                DCD     DMA2_Channel1_IRQHandler   ; DMA2 Channel1
                DCD     DMA2_Channel2_IRQHandler   ; DMA2 Channel2
                DCD     DMA2_Channel3_IRQHandler   ; DMA2 Channel3
                DCD     DMA2_Channel4_5_IRQHandler ; DMA2 Channel4 & Channel5
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler
Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
                IMPORT  system_init
                LDR     R0, = system_init
                BLX     R0               
                LDR     R0, =__main
                BX      R0
                ENDP
                
; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP		
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP    
						
				ALIGN

;********************************************************************************************
;*  The function expected of the C library startup 
;*  code for defining the stack and heap memory locations. 
;********************************************************************************************
                 IF      :DEF:__MICROLIB           
                
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                
                 ELSE
				 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                 LDR     R0, =Heap
                 LDR     R1, =(Stack + Stack_Size)
                 LDR     R2, =(Heap +  Heap_Size)
                 LDR     R3, = Stack
                 BX      LR

                 ALIGN
				 
				 ENDIF
					
                 END

;******************* (C) COPYRIGHT 2012 Lycanthia *****END OF FILE*****
