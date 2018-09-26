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
   IWDG->KR=0x5555; //允许访问RR和RLR寄存器
   IWDG->PR=prer;  //设置分频系数
   IWDG->RLR=reld; //设定计数器初始值
   IWDG->KR=0xaaaa; //初次加载初始值
   IWDG->KR=0xcccc;  //启动看门狗定时器
}
/*
Tout=((4*2^prer)*rlr) /40 ms
*/

void WatchDog_Feed(void)
{
 IWDG->KR=0xaaaa;
}

