#include "../test3.X/common.c"

Pin select = { IOPORT_A, p0 };
Pin clock = { IOPORT_B, p14 };
Pin sdi = { IOPORT_A, p1};
Pin led = { IOPORT_B, p15};

#define LEN(i) if(len<i) return i-len;
#define read32(i) (((cmd[i])|(cmd[i+1]<<8)|(cmd[i+2]<<16)|(cmd[i+3]<<24)))

/*
 * cmd: array of bytes that make of the command
 * len: number of bytes in cmd, >=1
 * return: amount of bytes to read
 *         0 if command is done
 */
uint8_t commandReceived(uint8_t* cmd, uint8_t len);

void loop();

void init();

void send(uint8_t* data, uint8_t len) {
    for(int i=0; i<len; i++) {
        SpiChnPutC(1, data[i]);
        SpiChnGetC(1);
    }
}

int main(void)
{
    initPic32();
    
    initOut(led, 1);
    
    PORTSetPinsDigitalIn(IOPORT_A, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_A, BIT_1);
    PORTSetPinsDigitalIn(IOPORT_B, BIT_14);
    
    PPSInput(1, SS1, RPA0);
    PPSInput(2, SDI1, RPA1);
    
#ifndef NO_SDO
    PPSOutput(3, RPA2, SDO1);
#endif
    
    SpiChnOpen(1, SPI_OPEN_MODE8|SPI_OPEN_SLVEN|SPI_OPEN_SSEN
#ifdef NO_SDO
            |SPI_OPEN_DISSDO
#endif
            , 2);
    
    init();
    
    uint8_t lastByteReceived = 0;
    uint8_t cmd[8];
    uint8_t length=0;
    uint8_t toRead = 0;
    while(1) {
        if(SpiChnTxBuffEmpty(1))
            SpiChnPutC(1, lastByteReceived);
        lastByteReceived = 0;
        
        while(!SpiChnDataRdy(1)) {
            loop();

            setOut(led, msElapsed % 200 < 100);
        }
        uint8_t byte = SpiChnGetC(1);
        cmd[length++] = byte;
        if(toRead)
            toRead--;
        if(toRead==0)
            toRead = commandReceived(cmd, length);
    }
}
