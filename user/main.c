/*
 *  user/main.c
 *
 *  Copyright (C) YYC
 *
 *  2012.4.9
 *
 *  主程序C语言入口, uC/OS-II启动.
 * 
 */

#include <Cfg/config.h>
#include <Task/mainloop.h>
#include <Task/spi.h>
#include <Task/uart.h>

#define OS_TASK_INIT_PRIO 			1
#define OS_INIT_TASK_STACK_SIZE		64      /* 初始化任务堆栈大小   */
#define OS_USER_TASK_STK_SIZE 128

static OS_STK        InitTaskStk[OS_INIT_TASK_STACK_SIZE] = { 0 };    /* 初始化任务堆栈               */
static OS_STK				Task_MainStk[OS_USER_TASK_STK_SIZE]	 = { 0 };
static OS_STK				Task_SpiStk[OS_USER_TASK_STK_SIZE]	 = { 0 };
static OS_STK				Task_UartStk[OS_USER_TASK_STK_SIZE]	 = { 0 };
/* 
 * 
 * Private function prototypes
 *
 */
static void    application_init( void );

static void    init_task_core( void *pdata );

void tick_init( void );


/**
  * @brief  C入口函数
  * @param  None
  * @retval None
  */
int main( void )
{
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock(  );
    OSInit(  );                 /* 初始化OS       */
    tick_init(  );           /* 初始化OS Tick  */
         
    OSTaskCreateExt( init_task_core,
                     ( void * ) 0,
                     ( OS_STK * ) & InitTaskStk[OS_INIT_TASK_STACK_SIZE - 1],
                     OS_TASK_INIT_PRIO,
                     OS_TASK_INIT_PRIO,
                     ( OS_STK * ) & InitTaskStk[0],
                     OS_INIT_TASK_STACK_SIZE,
                     ( void * ) 0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR );

//    OSTaskCreate( Task_Main, ( void * ) 0, &Task_MainStk[OS_USER_TASK_STK_SIZE - 1], 10 );      //主循环
			OSTaskCreate( Task_Spi, ( void * ) 0, &Task_SpiStk[OS_USER_TASK_STK_SIZE - 1], 11 );      //主循环
//	  OSTaskCreate( Task_Uart, ( void * ) 0, &Task_UartStk[OS_USER_TASK_STK_SIZE - 1], 12 );      //主循环
    OSStart(  );                /* 启动多任务环境 */
    return ( 0 );
}



/**
  * @brief  初始化任务核心函数：(1) 初始化应用  (2) 初始化操作系统组件
  * @param  None
  * @retval None
  */
static void init_task_core( void *pdata )
{

    pdata = pdata;              /* 防止编译器警告 */
	
    application_init(  );

    while ( 1 )                 //nocase
    {
        OSTaskSuspend( OS_PRIO_SELF );  /* 挂起初始化任务               */
    }
}


/**
  * @brief  基本应用初始化
  * @param  None
  * @retval None
  */
static void application_init( void )
{
	spi1_init();
	spi2_init();
	uart1_init();
	uart2_init();
	uart3_init();

}

void memclr(u8 *p,u8 len)
{
	while(len--)
		*p++ = 0;
}
void mcpy(u8 *d,u8 *s,u8 len)
{
	while(len--)
		*d++ = *s++;	
}
/***************************************************************/
