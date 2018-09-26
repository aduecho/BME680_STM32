#include "sw_delay.h"


void  delayus (uint32_t ulTime)
{
    uint32_t i = 0;

    while(ulTime--) {
      for(i=0; i < 7; i++);
    }
}

void  delayms (uint32_t ulTime)
{
    uint32_t i = 0;

    while(ulTime--) {
      for(i=0; i < 12002; i++);
    }
}

 

void WatchDog_Init(u8 prer, u16 reld)	//Tout=((4*2^prer)*reld) /40
{
   IWDG->KR=0x5555; //�������RR��RLR�Ĵ���
   IWDG->PR=prer;  //���÷�Ƶϵ��
   IWDG->RLR=reld; //�趨��������ʼֵ
   IWDG->KR=0xaaaa; //���μ��س�ʼֵ
   IWDG->KR=0xcccc;  //�������Ź���ʱ��
}
/*
Tout=((4*2^prer)*rlr) /40 ms
*/

void WatchDog_Feed(void)
{
 IWDG->KR=0xaaaa;
}

