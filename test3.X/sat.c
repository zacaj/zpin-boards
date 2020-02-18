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

uint32_t recStart = 0, recLast = 0, respStart = 0;

uint8_t sendByte(uint8_t data) {
    SpiChnPutC(1, data);
    while(!SpiChnDataRdy(1)) {
        if (respStart && msElapsed - respStart > 200) {
            return 0;
        }
    }
    SpiChnGetC(1);
    return 1;
}

uint8_t send(uint8_t* data, uint8_t len) {
    respStart = msElapsed;
    /*uint8_t out[len+3+len?1:0];
    uint8_t checksum = 0;
    out[0] = 0xFE;
    out[1] = prefix;
    out[2] = len;
    for(uint8_t i=0; i<len; i++) {
        out[i+3] = data[i];
        checksum += data[i];
    }
    if(len)
        out[len+3] = checksum;
    SpiChnPutS(1, out, len+3+len?1:0);*/
    uint8_t checksum = 0;
    SpiChnWriteC(1, 0xFE);
    SpiChnWriteC(1, prefix);
    SpiChnWriteC(1, len);
    for(uint8_t i=0; i<len; i++) {
        SpiChnWriteC(1, data[i]);
        checksum += data[i];
    }
    if(len)
        SpiChnWriteC(1, checksum);
    
    while(!SpiChnTxBuffEmpty(1)) {
        if (respStart && msElapsed - respStart > 200) {
            return 0;
        }
    }
    
//    if (!sendByte(0xFF)) return 0;
//    if (!sendByte(prefix)) return 0;
//    if (!sendByte(len)) return 0;
//    uint8_t checksum = 0;
//    for(int i=0; i<len; i++) {
//        if (!sendByte(data[i])) return 0;
//        checksum += data[i];
//    }
//    if (len && !sendByte(checksum)) return 0;
    return 1;
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
            
            |SPI_OPEN_ENHBUF|SPI_CONFIG_SMP_END|SPI_CONFIG_CKE_REV
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
    uint8_t respSize = 0;
    while(1) {
        if(SpiChnTxBuffEmpty(1) && !i)
            SpiChnPutC(1, 'e');
        lastByteReceived = 0;
        
        if (SpiChnGetRov(1, 0)) {
            i = 0;
        }
        
        if (i && ((recStart && msElapsed - recStart > 200) || (recLast && msElapsed - recLast > 20))) {
            //timeout
            
            i = 0;
            recStart = 0;
            recLast = 0;
            respStart = 0;
        }

        if(!SpiChnDataRdy(1)) {
            if (i) continue;
            loop();

            setOut(led, msElapsed % 200 < 100 && !i);
            continue;
        }
        // got data
        recLast = msElapsed;
        
        setOut(led, 0);
        uint8_t byte = SpiChnGetC(1);
        if (i == 0) { // command start
            recStart = msElapsed;
            if (byte != 'S') {
                i = 0;
                recStart = 0;
                recLast = 0;
                continue;
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
                    prefix = 'R';
                    respSize = commandReceived(cmd, toRead);
                }
                else {
                    prefix = 'C';
                    respSize = 0;
                }
                uint32_t now = msElapsed;
                
                uint8_t success = send(out, respSize);
                
                if (!success) {
                    setOut(led, 1);
                    while(msElapsed - now < 1000);
                }
                i = 0;
                recStart = 0;
                recLast = 0;
                respStart = 0;
            }
            else {
                i = 0;
                recStart = 0;
                recLast = 0;
                respStart = 0;
            }
        }
    }
}
