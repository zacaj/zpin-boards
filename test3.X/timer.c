#ifndef ZTIMER_H
#define ZTIMER_H

uint32_t (*callbacks[5])(void*);
uint32_t ticksLeft[5];
void *timerData[5];

void initTimers() {
    ConfigIntTimer1(T1_INT_PRIOR_2);
    ConfigIntTimer2(T2_INT_PRIOR_2);
    ConfigIntTimer3(T3_INT_PRIOR_2);
    ConfigIntTimer4(T4_INT_PRIOR_2);
    ConfigIntTimer5(T5_INT_PRIOR_2);
    for(int i=0; i<5; i++)
        callbacks[i]=NULL;
}

void timerTick(int i) {
    i--;
    if(!callbacks[i])
        return;
    
    ticksLeft[i]--;
    if(ticksLeft[i]==0) {
        ticksLeft[i] = callbacks[i](timerData[i]);
        if(!ticksLeft[i]) {
            switch(i) {
                case 0: CloseTimer1(); DisableIntT1; break;
                case 1: CloseTimer2(); DisableIntT2; break;
                case 2: CloseTimer3(); DisableIntT3; break;
                case 3: CloseTimer4(); DisableIntT4; break;
                case 4: CloseTimer5(); DisableIntT5; break;
            }
            callbacks[i] = NULL;
        }
    }
}

/*
 * calls \func\ with parameter \data\ in \ms\ milliseconds
 * 
 * if \func\ returns non-zero, repeats the timer in that many ms
 * 
 * returns false if no timer was available
 */
uint8_t callIn(uint32_t (*func)(void*), void* d, uint32_t ms) {
    int i=0;
    while(callbacks[i] && i<5) i++;
    if(i==5)
        return 0;
    
    callbacks[i]=func;
    timerData[i]=d;
    ticksLeft[i]=ms;
    
    switch(i) {
        case 0: OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 156); EnableIntT1; break;
        case 1: OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 156); EnableIntT2; break;
        case 2: OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, 156); EnableIntT3; break;
        case 3: OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_256, 156); EnableIntT4; break;
        case 4: OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_256, 156); EnableIntT5; break;
    }    
    
    return i+1;
}

void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1Handler(void)
{
    mT1ClearIntFlag();
    timerTick(1);
}
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) Timer2Handler(void)
{
    mT2ClearIntFlag();
    timerTick(2);
}
void __ISR(_TIMER_3_VECTOR, IPL2SOFT) Timer3Handler(void)
{
    mT3ClearIntFlag();
    timerTick(3);
}
void __ISR(_TIMER_4_VECTOR, IPL2SOFT) Timer4Handler(void)
{
    mT4ClearIntFlag();
    timerTick(4);
}
void __ISR(_TIMER_5_VECTOR, IPL2SOFT) Timer5Handler(void)
{
    mT5ClearIntFlag();
    timerTick(5);
}
#endif