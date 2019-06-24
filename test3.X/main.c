#include "../test3.X/common.c"

uint32_t lastChange=0;

int main(void)
{
    initPic32();
    
    ODCB |= BIT_12;
    
    while(1) {
        if(msElapsed - lastChange > 500) {
            lastChange = msElapsed;
            PORTToggleBits(bB, 0xFFFF);
        }
    }
}
