#include "../test3.X/common.c"
#define DISABLE_IRQ 1
#define ENABLE_GVD 1
Pin led = { IOPORT_B, p0 };
Pin tx = { IOPORT_B, p7 };
Pin rx = { IOPORT_B, p2 };
Pin sdo = { IOPORT_B, p11 };
Pin sdi = { IOPORT_B, p8 };
Pin sck = { IOPORT_B, p14 };
Pin inCS = { IOPORT_B, p4 };
Pin inReset = { IOPORT_B, p5 };
#ifndef DISALBE_IRQ
Pin inInt = { IOPORT_B, p9 };
#endif
#ifdef ENABLE_GVD
Pin gvd = { IOPORT_B, p9 };
#endif

u8 enableSolenoids = 1;

void loop();

void init();

uint16_t ledSpeed = 700;
boolean ledInvert = false;

void setLed() {
    setOut(led, !(ledInvert != (msElapsed % ledSpeed < (ledSpeed/6))));
}

int main(void)
{
    initPic32();

    initOut(led, 1);

//    PORTSetPinsDigitalIn(IOPORT_A, BIT_0);
//    PORTSetPinsDigitalIn(IOPORT_A, BIT_1);
//    PORTSetPinsDigitalIn(IOPORT_B, BIT_14);
//
//    PPSInput(1, SS1, RPA0);
//    PPSInput(2, SDI1, RPA1);
//
//#ifndef NO_SDO
//    PPSOutput(3, RPA2, SDO1);
//#endif
//
//    SpiChnOpen(1, SPI_OPEN_MODE8|SPI_OPEN_SLVEN|SPI_OPEN_SSEN
//            
//            |SPI_OPEN_ENHBUF|SPI_CONFIG_SMP_END|SPI_CONFIG_CKE_REV
//#ifdef NO_SDO
//            |SPI_OPEN_DISSDO
//#endif
//            , 2);

    init();

    while(1) {
        loop();
        setLed();
    }
}

#ifndef DISABLE_UART
#define QUEUE_LEN 5
#define MESSAGE_LEN 32
char sendMessage[QUEUE_LEN][MESSAGE_LEN+1];
char* sendAt = NULL;
u8 queueStart = 0, queueEnd = 0;
char recvMessage[MESSAGE_LEN+1];
char* recvAt = NULL;
u8 recvLen = 0;
u32 recvStartTime = 0;
#define MESSAGE_START 254
#define MESSAGE_END 253

void sendByte() {
    if (sendAt) {
        u8 byte = *sendAt;
        INTEnable(INT_SOURCE_UART_TX(UART1), INT_ENABLED);
        UARTSendDataByte(UART1, byte);
        if (byte == '\0' || byte == MESSAGE_END)
            sendAt = NULL;
        else {
            sendAt++;
            if (sendAt >= sendMessage[queueStart]+MESSAGE_LEN)
                sendAt = NULL;
        }
        
        if (!sendAt && queueStart != queueEnd) {
            sendMessage[queueStart][0] = 'X';
            queueStart++;
            if (queueStart >= QUEUE_LEN)
                queueStart = 0;
            if (queueStart != queueEnd)
                sendAt = sendMessage[queueStart];
        }
    }
    if (!sendAt) {
//        INTClearFlag(INT_SOURCE_UART_TX(UART1));
        INTEnable(INT_SOURCE_UART_TX(UART1), INT_DISABLED);
    }
    INTClearFlag(INT_SOURCE_UART_TX(UART1));
}

void send(u8* m, u8 len) {
    ledInvert = !ledInvert;
    u8 checksum = 0;
    u8 i=0;
    u8 queueAt = queueEnd++;
    if (queueEnd >= QUEUE_LEN)
        queueEnd = 0;
    sendMessage[queueAt][0] = MESSAGE_START;
    for (;i<MESSAGE_LEN-3;i++) {
        if ((len && i==len) || (!len && !m[i])) {
            break;
        }
        checksum += m[i];
        sendMessage[queueAt][i+1] = m[i];
    }
    if (!checksum || checksum >= MESSAGE_END) checksum = 1;
    sendMessage[queueAt][i+1] = checksum;
    sendMessage[queueAt][i+2] = MESSAGE_END;
    sendMessage[queueAt][i+3] = 0;
    if (!sendAt) {
        sendAt = sendMessage[queueAt];
//        INTClearFlag(INT_SOURCE_UART_TX(UART1));
        INTEnable(INT_SOURCE_UART_TX(UART1), INT_ENABLED);
        while(!UARTTransmissionHasCompleted(UART1));
        sendByte();
    }
}

void gotMessage(u8* data);
void __ISR(_UART_1_VECTOR, IPL2SOFT) IUart1Handler(void)
{
    if (INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        sendByte();
    }
    if (INTGetFlag(INT_SOURCE_UART_RX(UART1))) {
        u8 byte = UARTGetDataByte(UART1);
        if (byte == MESSAGE_START) {
            recvAt = recvMessage;
            recvStartTime = dmsElapsed;
        }
        else if (recvAt - recvMessage == recvLen+1 && recvLen) {
            if (byte == MESSAGE_END) {
                *recvAt = byte;
                gotMessage(recvMessage);
            }
            recvAt = NULL;
        }
        else if (recvAt) {
            if (recvAt == recvMessage)
                recvLen = byte;
            *recvAt = byte;
            recvAt++;
            if (recvAt >= recvMessage+MESSAGE_LEN)
                recvAt = NULL;
        }
        else {
            
        }
    }
    INTClearFlag(INT_SOURCE_UART_TX(UART1));
    INTClearFlag(INT_SOURCE_UART_RX(UART1));
    INTClearFlag(INT_SOURCE_UART_ERROR(UART1));
    INTClearFlag(INT_SOURCE_UART(UART1));
}
#endif

//
typedef struct {
    Pin pin;
    
    u32 onSince; // ms.  set = firing, 0 = not firing
    
    u32 strokeLength; // -1 = infinite, 0 = disabled
    u16 strokeOnDms;
    u16 strokeOffDms; // 0 = no pwm, strokeOnDms ignored
    
    u32 holdLength;
    u16 holdOnDms;
    u16 holdOffDms;
    
    u32 triggerSw; // bitfield, 1 = stroke will begin if this switch turns on, hold will end if this switch turns off
    u32 repulseSw;
    u32 lastOpened; // ms
    u8 minOffMs; // min time the switch must be open before triggering again
    u8 minOnDms; // min time the coil must be energized before it can untrigger
} Solenoid;
//
#define SOL_DEFAULT(port, pin) { { port, pin }, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0 }
//
#define SOL_NUM 14
Solenoid solenoid[SOL_NUM] = {
    SOL_DEFAULT(IOPORT_B, p3),
    SOL_DEFAULT(IOPORT_B, p1),
    SOL_DEFAULT(IOPORT_A, p2),
    SOL_DEFAULT(IOPORT_A, p0),
    SOL_DEFAULT(IOPORT_B, p13),
    SOL_DEFAULT(IOPORT_B, p10),
    SOL_DEFAULT(IOPORT_A, p1),
    SOL_DEFAULT(IOPORT_B, p9), // interrupt
    SOL_DEFAULT(IOPORT_A, p3),
    SOL_DEFAULT(IOPORT_A, p4),
    SOL_DEFAULT(IOPORT_B, p12),
    SOL_DEFAULT(IOPORT_B, p15),
    SOL_DEFAULT(IOPORT_B, p5),
    SOL_DEFAULT(IOPORT_B, p6),
};

void initSolenoids() {
    for (int i=0; i<SOL_NUM; i++) {
        if (i==7) continue;
        initOut(solenoid[i].pin, 0);
        solenoid[i].strokeLength = 0;
        solenoid[i].strokeOffDms = 0;
        solenoid[i].strokeOnDms = 2;
        solenoid[i].holdLength = 0;
        solenoid[i].holdOffDms = 0;
        solenoid[i].holdOnDms = 0;
        solenoid[i].lastOpened = 0;
        solenoid[i].minOffMs = 1;
        solenoid[i].minOnDms = 0;
        solenoid[i].onSince = 0;
        solenoid[i].repulseSw = 0;
        solenoid[i].triggerSw = 0;
// stress test        
//        solenoid[i].onSince=1;
//        solenoid[i].strokeOnDms = 1;
//        solenoid[i].strokeOffDms = 1;
//        solenoid[i].strokeLength = -1;
    }
}

void gotMessage(u8* data) {
    ledInvert = !ledInvert;
    u8 len = data[0];
    u8 i=0; 
    u8 checksum = 0;
    for (;i<len; i++) {
//        if (!data[i] || data[i]==MESSAGE_END) break;
        checksum += data[i];
    }
//    checksum -= data[i-1];
    if (!checksum || checksum >= MESSAGE_END) checksum = 1;
    if (checksum != data[len]) {
        char msg[MESSAGE_LEN];
        sprintf(msg, "___cksm");
        msg[0] = data[0];
        msg[1] = data[len];
        msg[2] = data[3];
        send(msg, 7);
        return;
    }
    
    u8 num = data[3];
    if (num >= SOL_NUM) {
        send("#num?", 0);
        return;
    }
    if (num == 7 && data[1] != 'A') return;
    switch (data[1]) {
        case 'A': {// AK#
            char msg[MESSAGE_LEN];
            sprintf(msg, "#ack %x %x", num, recvStartTime);
            msg[0] = data[0];
            send(msg, 0);
            initSolenoids();
            break;
        }
        case 'D': // DS#
            solenoid[num].onSince = 0;
            setOut(solenoid[num].pin, 0);
            solenoid[num].holdLength = 0;
            solenoid[num].strokeLength = 0;
            solenoid[num].triggerSw = 0;
            solenoid[num].repulseSw = 0;
            break;
        case 'C': {//CS#{sl,sl,son,soff,hl,hl,hon,hoff}
            u16 length = (data[4]<<8)|data[5];
            if (length == -1)
                solenoid[num].strokeLength = -1;
            else
                solenoid[num].strokeLength = length;
            solenoid[num].strokeOnDms = data[6];
            solenoid[num].strokeOffDms = data[7];
            length = (data[8]<<8)|data[9];
            if (length == -1)
                solenoid[num].holdLength = -1;
            else
                solenoid[num].holdLength = length;
            solenoid[num].holdOnDms = data[10];
            solenoid[num].holdOffDms = data[11];
            solenoid[num].triggerSw = 0;
            solenoid[num].repulseSw = 0;
            solenoid[num].minOffMs = 0;
            break;
        }
        case 'F': // FS#
            solenoid[num].onSince = msElapsed;
//            setOut(solenoid[i].pin, enableSolenoids);
            break;
        case 'O': // OS#
            solenoid[num].onSince = 0;
//            setOut(solenoid[num].pin, 0);
            break;
        case 'T': // TS#{trigger x4, repulse x4, minoffdms}
            solenoid[num].triggerSw = (data[4]<<24)|(data[5]<<16)|(data[6]<<8)|(data[7]);
            solenoid[num].repulseSw = (data[8]<<24)|(data[9]<<16)|(data[10]<<8)|(data[11]);
            solenoid[num].minOffMs = data[12];
            break;            
        default:
            send("#command?", 0);
    }    
}

#if 1
typedef enum {
    IODIRA  = 0x00,
    IODIRB  = 0x01,
    IPOLA   = 0x02,
    IPOLB   = 0x03,
    GPINTENA= 0x04,
    GPINTENB= 0x05,
    DEFVALA = 0x06,
    DEFVALB = 0x07,
    INTCONA = 0x08,
    INTCONB = 0x09,
    IOCON   = 0x0A,
    GPPUA   = 0x0C,
    GPPUB   = 0x0D,
    INTFA   = 0x0E,
    INTFB   = 0x0F,
    INTCAPA = 0x10,
    INTCAPB = 0x11,
    GPIOA   = 0x12,
    GPIOB   = 0x13,
    OLATA   = 0x14,
    OLATB   = 0x15,
} InAddr;
typedef enum {
    A = 0, B = 1,
} InBank;

u8 inRead(InBank in, InAddr address) {
    u8 control = 0b01000001 | (in? 0b10 : 0);
    u8 junk;
    setOut(inCS, 0);
    SpiChnPutC(1, control);
    SpiChnPutC(1, address);
    while (SPI1STATbits.SPIBUSY);
    junk = SPI1BUF;
    
    SPI1STATbits.SPIROV=0;
    
    SPI1BUF = 0b10101010;
    while (SPI1STATbits.SPIBUSY);
    setOut(inCS, 1);
    return SPI1BUF;
}
u16 inRead2x(InBank in, InAddr address) {
    u8 control = 0b01000001 | (in? 0b10 : 0);
    u8 junk;
    setOut(inCS, 0);
    SpiChnPutC(1, control);
    SpiChnPutC(1, address);
    while (SPI1STATbits.SPIBUSY);
    junk = SPI1BUF;
    
    SPI1STATbits.SPIROV=0;
    
    SPI1BUF = 0b10101010;
    while (SPI1STATbits.SPIBUSY);
    u8 a = SPI1BUF;
    
    SPI1BUF = 0b10101010;
    while (SPI1STATbits.SPIBUSY);
    setOut(inCS, 1);
    return (SPI1BUF<<8) | a;
}
void inWrite(InBank in, InAddr address, u8 value) {
    u8 control = 0b01000000 | (in? 0b10 : 0);
    setOut(inCS, 0);//while(!SpiChnTxBuffEmpty(1));while (SPI1STATbits.SPIBUSY);
//    for(int i=0;i<5;i++);
    SpiChnPutC(1, control);//for(int i=0;i<5;i++);//while(!SpiChnTxBuffEmpty(1));while (SPI1STATbits.SPIBUSY);SpiChnReadC(1);
    SpiChnPutC(1, address);//for(int i=0;i<5;i++);//while(!SpiChnTxBuffEmpty(1));while (SPI1STATbits.SPIBUSY);SpiChnReadC(1);
    SpiChnPutC(1, value);//for(int i=0;i<5;i++);//while(!SpiChnTxBuffEmpty(1));while (SPI1STATbits.SPIBUSY);SpiChnReadC(1);
//    for(int i=0;i<5;i++);
    //while(!SpiChnTxBuffEmpty(1));
    while (SPI1STATbits.SPIBUSY);
    setOut(inCS, 1);
//    for(int i=0;i<10;i++);
}


void inWrite2x(InBank in, InAddr address, u8 value) {
    inWrite(in, address, value);
    inWrite(in, address+1, value);
}

void inWrite2xBoth(InAddr address, u8 value) {
    inWrite2x(A, address, value);
    inWrite2x(B, address, value);
}

u32 inState = 0;

void updateSolenoid(Solenoid* s, int num) {
    if (num==7) return;
    u32 ms = msElapsed;
    u32 dms = dmsElapsed;
    char message[MESSAGE_LEN];
    
    if (!s->onSince) {
        if (s->triggerSw & inState) {
            if (ms - s->lastOpened >= s->minOffMs) {
                s->onSince = msElapsed;
                setOut(s->pin, enableSolenoids);
                sprintf(message, "#TRIGGER %i", num);
                send(message, 0);
            }
        }
    }
    
    if (!s->onSince) return;
    
    if (ms - s->onSince < s->strokeLength || s->strokeLength==-1) {
        if (s->strokeOffDms) {
            setOut(s->pin, dms % (s->strokeOffDms + s->strokeOnDms) < s->strokeOnDms);
        }
        else {
            // already on, ignore it
            setOut(s->pin, enableSolenoids);
        }
    }
    else if (ms - s->onSince < s->strokeLength + s->holdLength || s->holdLength==-1) {
        if (s->triggerSw && !(s->triggerSw & inState)) {
            s->onSince = 0;
            sprintf(message, "#UNtrigger %i", num);
            send(message, 0);
        }
        else if (s->holdOffDms) {
            setOut(s->pin, dms % (s->holdOffDms + s->holdOnDms) < s->holdOnDms);
        }
        else {
            setOut(s->pin, enableSolenoids);
        }
    }
    else {
        setOut(s->pin, 0);
        if (s->triggerSw && !(s->triggerSw & inState)) {
            s->onSince = 0;
        }
    }
    
    if (!s->onSince) {
        setOut(s->pin, 0);
    }
}

void checkInputs(InAddr source) {
    source = GPIOA;
    u32 state;
    u16 a, b;
    u32 last = 0xFFFFFFFF;
    
    u8 i = 2;
    while(1) {
        a = inRead2x(A, source) ^ 0xFFFF;
        b = inRead2x(B, source) ^ 0xFFFF;
        state = (a | (b<<16));
        if (state == inState) return;
        if (state == last) break;
        last = state;
        if (--i==0) return;
    }    
    
    u32 changed = inState ^ state;
    u32 on = changed & state;
    u32 off = changed & (0xFFFFFFFF ^ state);
    
    // trigger solenoids
    u8 triggered = -1;
    u8 untriggered = -1;
    if (a != 0xFFFF && b != 0xFFFF) {
        for (int i=0; i<SOL_NUM; i++) {
            if (i==7) continue;
            if (solenoid[i].triggerSw & changed) {
                updateSolenoid(solenoid+i, i);
            }

            if (solenoid[i].triggerSw & off) {
                solenoid[i].lastOpened = msElapsed;
            }
        }

        inState = state;
    }
    else {
        triggered = inRead(A, IOCON);
        untriggered = inRead(B, IOCON);
    }
    
    char msg[MESSAGE_LEN];
    sprintf(msg, source==INTCAPA? "#iq %x %x %x %x" : "#sw %x %x %x %x", state, dmsElapsed, triggered, untriggered);
//    msg[0] = '#';
//    msg[1] = 's';
//    msg[2] = 'w';
//    msg[3] = ' ';
//    *(u32*)(msg+4) = state;
//    *(u32*)(msg+8) = (u32)dmsElapsed;
//    msg[12] = triggered;
//    msg[13] = untriggered;
    
    send(msg, 0);
    
    if (source == INTCAPA)
        checkInputs(GPIOA);
}

#ifndef DISABLE_IRQ
void __attribute__((vector(_EXTERNAL_1_VECTOR), interrupt(IPL4SOFT), nomips16, no_fpu))  IInInterruptHandler(void)
{
    if (INTGetFlag(INT_SOURCE_EX_INT(1))) {
        checkInputs(INTCAPA);
//        INTEnable(INT_SOURCE_EX_INT(1), INT_ENABLED);
    }
    INTClearFlag(INT_SOURCE_EX_INT(1));
}
#endif
#endif

void init() {
    initSolenoids();
    
//    solenoid[0].holdLength = 0;
//    solenoid[0].strokeLength = 100;
//    solenoid[0].strokeOffDms = 0;
//    solenoid[0].strokeOnDms = 2;
//    solenoid[0].triggerSw = 0b1;
//    
//    solenoid[1].holdLength = -1;
//    solenoid[1].strokeLength = 0;
//    solenoid[1].holdOffDms = 0;
//    solenoid[1].holdOnDms = 2;
//    solenoid[1].triggerSw = 0b1;
#ifdef ENABLE_GVD
    initIn(gvd);
    CNPDBbits.CNPDB9 = 1; // pulldown on B9
#endif
    
#ifndef DISABLE_UART
    initOut(tx, 1);
    initIn(rx);
//    PPSOutput(1, RPB7, U1TX);
    RPB7R = 0b0001; // U1TX
    U1RXR = 0b0100; // B2
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY); 
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY); 
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2); 
    UARTSetDataRate(UART1, SYS_FREQ, 115200/2); 

    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    U1TXREG = 'h';
    while(!UARTTransmissionHasCompleted(UART1));
    UARTSendDataByte(UART1, 'i');
    while(!UARTTransmissionHasCompleted(UART1));
    
//    int i=0;
//    while(i<25) {
//        waitUs(10000);
//        UARTSendDataByte(UART1, 'a'+i);
//        i++;
//    }
    
  INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
  INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_1);
//    IEC1bits.U1TXIE = 1;
//     INTEnable(INT_SOURCE_UART_TX(UART1), INT_ENABLED);
    INTClearFlag(INT_SOURCE_UART_RX(UART1));
    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    
    send("#hello", 0);
//    CNPDB=0;
//    for(int i=0; i<16; i++) {
//        solenoid[i].mode = Disabled;
//        initSolenoid(&solenoid[i]);
////        setOut(solenoid[i].pin, !solenoid[i].on);
//    }
#endif
#ifndef DISABLE_IO
#ifndef DISABLE_IRQ
    initIn(inInt);
    CNPUBbits.CNPUB9 = 1; // pullup on B9
    INTCONbits.INT1EP = 0; // interrupt on falling edge
    INT1R = 0b0100; // B9
    INTSetVectorPriority(INT_SOURCE_EX_INT(1), INT_PRIORITY_LEVEL_4);
    INTSetVectorSubPriority(INT_SOURCE_EX_INT(1), INT_SUB_PRIORITY_LEVEL_1);
    INTClearFlag(INT_SOURCE_EX_INT(1));
    INTEnable(INT_SOURCE_EX_INT(1), INT_ENABLED);
#endif
    
//    initOut(sdo, 1);
//    initOut(sck, 1);
//    initIn(sdi);
    PPSUnLock;
//    initOut(inReset, 1);
    initOut(inCS, 1);
    RPB11R = 0b0011; // SDO 1
    SDI1R = 0b0100; // B8
//    setOut(inReset, 0);
//    for(int i=0;i<10000;i++);
//    setOut(inReset, 1);
//    for(int i=0;i<10000;i++);
//    setOut(inCS, 0);
    SpiChnOpenEx(1, SPI_OPEN_ON|SPI_OPEN_MODE8|SPI_OPEN_MSTEN|SPI_OPEN_CKE_REV|SPI_OPEN_SMP_END, SPI_OPEN2_IGNROV|SPI_OPEN2_IGNTUR, 4);
//    setOut(inCS, 1);
//    for(int i=0;i<10000;i++);
    
    while(inRead(A, IODIRA)!= 0xFF || inRead(A, IPOLA)!=0);
    while(inRead(B, IODIRA)!= 0xFF || inRead(B, IPOLA)!=0);
    
    
    inWrite(A, IOCON, 0b01001100); // INTs tied, INT active low, seq read, addr enabled
//    inWrite(B, IOCON, 0b01001100); // INTs tied, INT active low, seq read, addr enabled
//    inWrite(A, IOCON+1, 0b01101110); // INTs tied, INT active low, seq read
    inWrite2xBoth(IODIRA, 0xFF); // 0 = output
//    inWrite4(IODIRA, 0x00); // 0 = output
    inWrite2xBoth(IPOLA, 0b00000000); // 0 = not inverted
//    inWrite2xBoth(GPIOA, 0b11110000);
//    inWrite(A, GPIOA, 0b00111100);
//    inWrite(A, GPIOB, 0b10101100);
//    inWrite4(OLATA, 0b01110000);
    inWrite2xBoth(GPPUA, 0b11111111); // 1 = pull up enabled
    inWrite2xBoth(IPOLA, 0x00); // 0 = non-inverted
#ifdef DISABLE_IRQ
    inWrite2xBoth(GPINTENA, 0x00); // 1 = enable interrupt
#else
    inWrite2xBoth(GPINTENA, 0xFF); // 1 = enable interrupt
#endif
    inWrite2xBoth(INTCONA, 0x00); // 0 = interrupt on any change
    
//    u8 ctrl = inRead(A, IOCON);
    
    u8 ctrl= inRead(A, IOCON);
    u8 a= inRead(A, GPIOA);
    u8 b= inRead(A, GPIOB);
//    while(1) {
//        ctrl = inRead(A, IOCON);
//    }
#endif
#ifdef TEST
    for(int i=0; i<16; i++) {
        solenoid[i].mode = Momentary;
        solenoid[i].onTime = 250;
    }
#endif
    /*solenoid[0].mode = Triggered;
    solenoid[0].minOnTime = 100;
    solenoid[0].maxOnTime = 100;
    solenoid[0].triggeredBy = 15;
    solenoid[0].cooldownTime = 10;

    solenoid[15].mode = Input;
    solenoid[15].settleTime = 5;/**/

    /*
    
    solenoid[0].mode = OnOff;

    solenoid[14].mode = Triggered;
    solenoid[14].minOnTime = 0;
    solenoid[14].triggeredBy = 15;
    solenoid[15].mode = Input;

    solenoid[12].mode = Triggered;
    solenoid[12].minOnTime = 75;
    solenoid[12].maxOnTime = 75;
    solenoid[12].triggeredBy = 13;
    solenoid[13].mode = Input;
    solenoid[13].settleTime = 10;*/


//    for(int i=0; i<16; i++) {
//        initSolenoid(&solenoid[i]);
//    }

    //turnOnSolenoid(&solenoid[0]);
}


uint32_t last=0;
//#define TEST
uint8_t state=0;
uint32_t I = 0;
void loop() {
    I++;
//    setOut(solenoid[SOL_NUM-3].pin, I&1);
    if (U1ASTAbits.URXDA) {
        UARTGetDataByte(UART1);
    }
    U1ASTAbits.OERR = 0;
    
#ifdef ENABLE_GVD
    if (!getIn(gvd)) {
        if (enableSolenoids) {
            enableSolenoids = 0;
            for (int i=0; i<SOL_NUM; i++) {
                if (i==7) continue;
                Solenoid* s = &solenoid[i];
                setOut(s->pin, 0);
            }
            send("#lost gv", 0);
            ledSpeed = 100;
        }
    }
    else if (!enableSolenoids) {
        enableSolenoids = 1;
        ledSpeed = 700;
    }
#endif
    
//    if (!getIn(inInt)) {
        checkInputs(GPIOA);
//    }
    
//    ledInvert = !!(inState & (1<<0));
    
//    if(msElapsed-last>1500) {
//        last = msElapsed;
//        solenoid[0].onSince = msElapsed;
//    }
//    if (!sendAt && queueStart == queueEnd) {
//        u16 in = inRead2x(A, GPIOA);
//        char msg[10];
//        sprintf(msg, "#sw %x", in);
//        send(msg);
//    }
#ifdef TEST
    if(msElapsed-last>150) {
        last = msElapsed;

        /*if(state) {
           // uint8_t cmd = Ob(01010001);
            uint8_t cmd = Ob(01011001);
            commandReceived(&cmd, 1);
            cmd = Ob(00110000)|12;
            commandReceived(&cmd, 1);
        }
        else {
            uint8_t cmd = Ob(01011001);
            commandReceived(&cmd, 1);
            cmd = Ob(01000000)|12;
            commandReceived(&cmd, 1);
        }*/
        //fireSolenoid(&solenoid[0]);

        //state=!state;
//    for(int i=0; i<16; i++) {
//        fireSolenoid(&solenoid[i], 0);
//    }
        fireSolenoid(&solenoid[I], 0);

        I++;
        if (I >= 16) I = 0;

    }
#endif
    
    for (int i=0; i<SOL_NUM; i++) {
        if (i==7) continue;
        Solenoid* s = &solenoid[i];
        updateSolenoid(s, i);
    }
}

void crashed() {
    for(int i=0; i<SOL_NUM; i++) {
        solenoid[i].onSince = 0;
        setOut(solenoid[i].pin, 0);
    }
    ledSpeed = 2000;
//    while (1) {
//        setLed();
//    }
}