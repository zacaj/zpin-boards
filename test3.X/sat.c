#include "../test3.X/common.c"

Pin select = { IOPORT_A, p0 };
Pin clock = { IOPORT_B, p14 };
Pin sdi = { IOPORT_A, p1};

Pin led = { IOPORT_B, p15};

#define LEN(i) if(len!=i) { prefix = 'L'; out[0] = i; return 1; }
#define read32(i) (((cmd[i])|(cmd[i+1]<<8)|(cmd[i+2]<<16)|(cmd[i+3]<<24)))

uint8_t out[9]; // write responses to commandRecieved() here
uint8_t prefix;
/*
 * cmd: array of bytes that make of the command
 * len: number of bytes in cmd, >=1
 * return: number of bytes to send back, of any
 */
uint8_t commandReceived(uint8_t* cmd, uint8_t len);

void loop();

void init();

void send(uint8_t* data, uint8_t len) {
    SpiChnPutC(1, 0xFF);
    SpiChnGetC(1);
    SpiChnPutC(1, prefix);
    SpiChnGetC(1);
    SpiChnPutC(1, len);
    SpiChnGetC(1);
    uint8_t checksum = 0;
    for(int i=0; i<len; i++) {
        SpiChnPutC(1, data[i]);
        SpiChnGetC(1);
        checksum += data[i];
    }
    SpiChnPutC(1, checksum);
    SpiChnGetC(1);
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
    uint8_t cmd[32];
    uint8_t i=0;
    uint8_t checksum = 0;
    uint8_t theirSum = 0;
    uint8_t toRead = 0;
    uint32_t recStart = 0;
    uint8_t respSize = 0;
    while(1) {
        if(SpiChnTxBuffEmpty(1))
            SpiChnPutC(1, lastByteReceived);
        lastByteReceived = 0;
        
        if (recStart && msElapsed - recStart > 10) {
            //timeout
            i = 0;
            recStart = 0;
            toRead = 0;
        }

        while(!SpiChnDataRdy(1)) {
            loop();

            setOut(led, msElapsed % 200 < 100);
        }
        uint8_t byte = SpiChnGetC(1);
        if (i == 0) { // command start
            recStart = msElapsed;
            if (byte != 'S') {
                i = 0;
                recStart = 0;
                toRead = 0;
            }
            i++;
        }
        else if (i == 1) { // cmd length
            toRead = byte;
            checksum = 0;
            i++;
        }
        else if (i-2 < toRead) { // read cmd
            cmd[i-2] = byte;
            checksum += byte;
            i++;
        }
        else if (i-2 == toRead) { // checksum
            theirSum = byte;
            i++;
        }
        else { // done
            if (byte == 'E') {
                if (checksum == theirSum) {
                    respSize = commandReceived(cmd, i);
                    prefix = 'R';
                }
                else {
                    prefix = 'C';
                    respSize = 0;
                }
                send(out, respSize + 1);
            }
            else {
                i = 0;
                recStart = 0;
                toRead = 0;
            }
        }
    }
}
