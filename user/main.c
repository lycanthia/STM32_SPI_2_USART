/*
 *  user/main.c
 *
 *  Copyright (C) YYC
 *
 *  2012.4.9
 *
 *  ������C�������, uC/OS-II����.
 * 
 */

#include <Cfg/config.h>
#include <Task/mainloop.h>
#include <Task/spi.h>
#include <Task/uart.h>

#define OS_TASK_INIT_PRIO 			1
#define OS_INIT_TASK_STACK_SIZE		64      /* ��ʼ�������ջ��С   */
#define OS_USER_TASK_STK_SIZE 128

static OS_STK        InitTaskStk[OS_INIT_TASK_STACK_SIZE] = { 0 };    /* ��ʼ�������ջ               */
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
  * @brief  C��ں���
  * @param  None
  * @retval None
  */
int main( void )
{
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock(  );
    OSInit(  );                 /* ��ʼ��OS       */
    tick_init(  );           /* ��ʼ��OS Tick  */
         
    OSTaskCreateExt( init_task_core,
                     ( void * ) 0,
                     ( OS_STK * ) & InitTaskStk[OS_INIT_TASK_STACK_SIZE - 1],
                     OS_TASK_INIT_PRIO,
                     OS_TASK_INIT_PRIO,
                     ( OS_STK * ) & InitTaskStk[0],
                     OS_INIT_TASK_STACK_SIZE,
                     ( void * ) 0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR );

//    OSTaskCreate( Task_Main, ( void * ) 0, &Task_MainStk[OS_USER_TASK_STK_SIZE - 1], 10 );      //��ѭ��
			OSTaskCreate( Task_Spi, ( void * ) 0, &Task_SpiStk[OS_USER_TASK_STK_SIZE - 1], 11 );      //��ѭ��
//	  OSTaskCreate( Task_Uart, ( void * ) 0, &Task_UartStk[OS_USER_TASK_STK_SIZE - 1], 12 );      //��ѭ��
    OSStart(  );                /* ���������񻷾� */
    return ( 0 );
}



/**
  * @brief  ��ʼ��������ĺ�����(1) ��ʼ��Ӧ��  (2) ��ʼ������ϵͳ���
  * @param  None
  * @retval None
  */
static void init_task_core( void *pdata )
{

    pdata = pdata;              /* ��ֹ���������� */
	
    application_init(  );

    while ( 1 )                 //nocase
    {
        OSTaskSuspend( OS_PRIO_SELF );  /* �����ʼ������               */
    }
}


/**
  * @brief  ����Ӧ�ó�ʼ��
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
