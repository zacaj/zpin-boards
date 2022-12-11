#include "../test3.X/common.c"

Pin led = { IOPORT_B, p0 };
Pin tx = { IOPORT_B, p7 };
Pin rx = { IOPORT_B, p2 };
Pin sdo = { IOPORT_B, p11 };
Pin sdi = { IOPORT_B, p8 };
Pin sck = { IOPORT_B, p14 };
Pin inCS = { IOPORT_B, p4 };
Pin inReset = { IOPORT_B, p5 };
Pin inInt = { IOPORT_B, p9 };


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

#define MESSAGE_LEN 10
char sendMessage[MESSAGE_LEN+1];
char* sendAt = NULL;
char recvMessage[MESSAGE_LEN+1];
char* recvAt = NULL;
#define MESSAGE_START 255
#define MESSAGE_END 254

void sendByte() {
    if (sendAt) {
        u8 byte = *sendAt;
        if (*sendAt == '\0' || *sendAt == '\n')
            sendAt = NULL;
        else {
            sendAt++;
            if (sendAt >= sendMessage+MESSAGE_LEN)
                sendAt = NULL;
        }
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
        UARTSendDataByte(UART1, byte);
    }
    if (!sendAt) {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
        INTEnable(INT_SOURCE_UART_TX(UART1), INT_DISABLED);
    }
}

void send(char* m) {
    while(sendAt);
    strcpy(sendMessage, m);
    sendAt = sendMessage;
    INTClearFlag(INT_SOURCE_UART_TX(UART1));
    INTEnable(INT_SOURCE_UART_TX(UART1), INT_ENABLED);
    sendByte();
}

void __ISR(_UART_1_VECTOR, IPL2SOFT) IUart1Handler(void)
{
    if (INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        sendByte();
    }
    if (INTGetFlag(INT_SOURCE_UART_RX(UART1))) {
        u8 byte = UARTGetDataByte(UART1);
        if (byte == MESSAGE_START)
            recvAt = recvMessage;
        else if (byte == MESSAGE_END) {
            *recvAt = 0;
//            send(recvMessage);
            recvAt = NULL;
        }
        else if (recvAt) {
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
    u32 repluseSw;
} Solenoid;
//
#define SOL_DEFAULT(port, pin) { { port, pin }, 0, 0, 1, 0, 0, 1, 0, 0, 0 }
//
#define SOL_NUM 13
Solenoid solenoid[SOL_NUM] = {
    SOL_DEFAULT(IOPORT_B, p3),
    SOL_DEFAULT(IOPORT_B, p1),
    SOL_DEFAULT(IOPORT_A, p2),
    SOL_DEFAULT(IOPORT_A, p4),
    SOL_DEFAULT(IOPORT_B, p13),
    SOL_DEFAULT(IOPORT_B, p10),
//    SOL_DEFAULT(IOPORT_B, p9), // interrupt
    SOL_DEFAULT(IOPORT_B, p5),
    SOL_DEFAULT(IOPORT_A, p3),
    SOL_DEFAULT(IOPORT_B, p6),
    SOL_DEFAULT(IOPORT_B, p12),
    SOL_DEFAULT(IOPORT_B, p15),
    SOL_DEFAULT(IOPORT_A, p0),
    SOL_DEFAULT(IOPORT_A, p1),
};



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

void checkInputs(InAddr source) {
    u32 state = 0xFFFF ^ inRead2x(A, source);
    if (state == inState) return;
    
    u32 changed = inState ^ state;
    u32 on = changed & state;
    u32 off = changed & (0xFFFF ^ state);
    
    // trigger solenoids
    for (int i=0; i<SOL_NUM; i++) {
        if (solenoid[i].triggerSw & on) {
            if (!solenoid[i].onSince) {
                solenoid[i].onSince = msElapsed;
                setOut(solenoid[i].pin, 1);
            }
        }
        
        if (solenoid[i].triggerSw & off) {
            if (msElapsed - solenoid[i].onSince > solenoid[i].strokeLength) {
                solenoid[i].onSince = 0;
                setOut(solenoid[i].pin, 0);
            }
        }
    }
    
    inState = state;
    // todo: queue these
//    char msg[10];
//    sprintf(msg, "in %x", state);
//    send(msg);
    
    if (source == INTCAPA)
        checkInputs(GPIOA);
}

void __attribute__((vector(_EXTERNAL_1_VECTOR), interrupt(IPL4SOFT), nomips16, no_fpu))  IInInterruptHandler(void)
{
    if (INTGetFlag(INT_SOURCE_EX_INT(1))) {
        checkInputs(INTCAPA);
//        INTEnable(INT_SOURCE_EX_INT(1), INT_ENABLED);
    }
    INTClearFlag(INT_SOURCE_EX_INT(1));
}

void init() {
    for (int i=0; i<SOL_NUM; i++) {
        initOut(solenoid[i].pin, 0);
        solenoid[i].strokeLength = 0;
        solenoid[i].strokeOffDms = 0;
        solenoid[i].strokeOnDms = 2;
    }
    
    solenoid[0].holdLength = 0;
    solenoid[0].strokeLength = 40;
    solenoid[0].strokeOffDms = 0;
    solenoid[0].strokeOnDms = 2;
    solenoid[0].triggerSw = 0b1;
    
    solenoid[1].holdLength = -1;
    solenoid[1].strokeLength = 0;
    solenoid[1].holdOffDms = 0;
    solenoid[1].holdOnDms = 2;
    solenoid[1].triggerSw = 0b1;
    
    
    initIn(inInt);
    CNPUBbits.CNPUB9 = 1; // pullup on B9
    INTCONbits.INT1EP = 0; // interrupt on falling edge
    INT1R = 0b0100; // B9
    INTSetVectorPriority(INT_SOURCE_EX_INT(1), INT_PRIORITY_LEVEL_4);
    INTSetVectorSubPriority(INT_SOURCE_EX_INT(1), INT_SUB_PRIORITY_LEVEL_1);
    INTClearFlag(INT_SOURCE_EX_INT(1));
    INTEnable(INT_SOURCE_EX_INT(1), INT_ENABLED);
    
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
    SpiChnOpenEx(1, SPI_OPEN_ON|SPI_OPEN_MODE8|SPI_OPEN_MSTEN|SPI_OPEN_CKE_REV|SPI_OPEN_SMP_END, SPI_OPEN2_IGNROV|SPI_OPEN2_IGNTUR, 8);
//    setOut(inCS, 1);
//    for(int i=0;i<10000;i++);
    
    
    inWrite(A, IOCON, 0b01001100); // INTs tied, INT active low, seq read
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
    inWrite2xBoth(GPINTENA, 0xFF); // 1 = enable interrupt
    inWrite2xBoth(INTCONA, 0x00); // 0 = interrupt on any change
    
//    u8 ctrl = inRead(A, IOCON);
    
    u8 ctrl= inRead(A, IOCON);
    u8 a= inRead(A, GPIOA);
    u8 b= inRead(A, GPIOB);
//    while(1) {
//        ctrl = inRead(A, IOCON);
//    }
    initOut(tx, 1);
    initIn(rx);
//    PPSOutput(1, RPB7, U1TX);
    RPB7R = 0b0001; // U1TX
    U1RXR = 0b0100; // B2
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY); 
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY); 
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1); 
    UARTSetDataRate(UART1, SYS_FREQ, 9600); 

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
//    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    
    send("hello");
//    CNPDB=0;
//    for(int i=0; i<16; i++) {
//        solenoid[i].mode = Disabled;
//        initSolenoid(&solenoid[i]);
////        setOut(solenoid[i].pin, !solenoid[i].on);
//    }
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
    if (U1ASTAbits.URXDA) {
        UARTGetDataByte(UART1);
    }
    U1ASTAbits.OERR = 0;
    
    if (!getIn(inInt)) {
        checkInputs(INTCAPA);
    }
    
    ledInvert = !!(inState & (1<<0));
    
//    if(msElapsed-last>1500) {
//        last = msElapsed;
//        solenoid[0].onSince = msElapsed;
//    }
    
//    u16 in = inRead2x(A, GPIOA);
//    char msg[10];
//    sprintf(msg, "in %x", in);
//    send(msg);
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

    u32 ms = msElapsed;
    u32 dms = dmsElapsed;
    
    for (int i=0; i<SOL_NUM; i++) {
        Solenoid* s = &solenoid[i];
        if (!s->onSince) continue;
        if (ms - s->onSince < s->strokeLength || s->strokeLength==-1) {
            if (s->strokeOffDms) {
                setOut(s->pin, dms % (s->strokeOffDms + s->strokeOnDms) < s->strokeOnDms);
            }
            else {
                // already on, ignore it
                setOut(s->pin, 1);
            }
        }
        else if (ms - s->onSince < s->strokeLength + s->holdLength || s->holdLength==-1) {
            if (s->triggerSw && !(s->triggerSw & inState)) {
                setOut(s->pin, 0);
                s->onSince = 0;
            }
            else if (s->holdOffDms) {
                setOut(s->pin, dms % (s->holdOffDms + s->holdOnDms) < s->holdOnDms);
            }
            else {
                setOut(s->pin, 1);
            }
        }
        else {
            setOut(s->pin, 0);
            s->onSince = 0;
        }
    }
//        switch(s->mode) {
//            case Disabled:
//            case Momentary:
//                break;
//            case Input: {
//                uint8_t input = getIn(s->pin);
//                if (input) {
//                }
//                else {
//
//                }
//                break;
//            }
//            case OnOff:
//                if (s->onSince) {
//                    if (s->maxOnTime && msElapsed-s->onSince > s->maxOnTime) {
//                        if (s->pulseOffTime) { // pwm
//                            if (dmsElapsed % (s->pulseOffTime + s->pulseOnTime) < s->pulseOnTime)
//                               setOut(s->pin, s->on);
//                            else
//                                setOut(s->pin, !s->on);    
//                        }
//                        else {
//                            turnOffSolenoid(s);
//                        }
//                    }
//                }
//                break;
//#if 0
//            case Triggered: {
//                if (s->triggeredBy >= 16)
//                    break;
//                Solenoid* trigger = &solenoid[s->triggeredBy];
//                uint8_t input = getIn(trigger->pin) ^ trigger->inverted;
//                if (!input) {
//                    if (msElapsed > s->lastOnAt + trigger->settleTime) {
//                        s->disabled = 0;
//                    }
//                    turnOffSolenoid(s);
//                }
//                else {
//                    if (s->onSince) { // already on
//                        if (s->maxOnTime && msElapsed - s->onSince > s->maxOnTime) {
//                            if (s->pulseOffTime) { // pwm
//                                uint32_t t = msElapsed - s->onSince - s->maxOnTime;
//                                if (dmsElapsed % (s->pulseOffTime + 1) == 0)
//                                   setOut(s->pin, s->on);
//                                else
//                                    setOut(s->pin, !s->on);    
//                            }
//                            else {
//                                turnOffSolenoid(s);
//                                s->disabled = 1;
//                            }
//                        }
//                    }
//                    else if (!s->disabled && msElapsed > s->lastOnAt + trigger->settleTime) {
//                        turnOnSolenoid(s);
//                    }
//                    s->lastOnAt = msElapsed;
//                }
//                break;
//            }
//#endif
//        }
//    }

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