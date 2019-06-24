#include "../test3.X/sat.c"
void error() {
    
}

#define ON 0
#define OFF 1

typedef struct {
    Pin pin;
    uint32_t onTime, offTime;
    uint32_t maxOnTime, minOnTime; //0=disabled
    uint8_t triggers; // if !0, this is an input that triggers solenoid \triggers-1\
    
    uint32_t dontFireBefore;
    uint32_t onSince;
    uint8_t triggered;
} Solenoid;

//note v4 pins
Solenoid solenoid[16] = {
    { { IOPORT_B, p6 , 0}, 25, 250, 0, 0, 0, 0, 0 , 0},
    { { IOPORT_B, p7 , 0}, 25, 250, 0, 0, 0,0, 0 , 0},
    { { IOPORT_B, p8 , 0}, 25, 250, 0, 0, 0,0, 0 , 0},
    { { IOPORT_B, p11 , 0}, 25, 250, 0, 0, 0,0, 0 , 0}, // low power + connector
    { { IOPORT_B, p9 , 0}, 25, 250, 0, 0, 0,0, 0 , 0},
    { { IOPORT_B, p13 , 0}, 25, 250, 0, 0, 0,0, 0 , 0},
    
    { { IOPORT_B, p0 , 0}, 25, 250, 0, 0, 0,0, 0 , 0},
    { { IOPORT_B, p1 , 0}, 25, 250, 0, 0, 0,0, 0 , 0},
    { { IOPORT_B, p2 , 0}, 25, 250, 100, 0, 0,0, 0 , 0},
    { { IOPORT_B, p3 , 0}, 25, 250, 100, 0, 0,0, 0 , 0}, //low power
    { { IOPORT_A, p3 , 0}, 25, 250, 100, 0, 0,0, 0 , 0},
    { { IOPORT_B, p4 , 0}, 25, 250, 100, 0, 0,0, 0 , 0},
    
    { { IOPORT_B, p12 , 0}, 25, 250, 100, 0, 0,0, 0 , 0},
    { { IOPORT_B, p10 , 0}, 25, 250, 100, 0, 0,0, 0 , 0}, // low power + connector
    { { IOPORT_A, p4 , 0}, 25, 250, 100, 0, 0,0, 0 , 0},
    { { IOPORT_B, p5 , 0},25, 250, 100, 0, 0,0, 0 , 0}, // low power
};


uint32_t turnOffSolenoid(Solenoid *s) {  
    if(s->triggers) return 0;
    if(s->minOnTime && s->onSince && msElapsed-s->onSince < s->minOnTime) {
        return 0;
    }
    
    setOut(s->pin, OFF);
    s->triggered = 0;
    s->onSince = 0;
    return 0;
}

uint32_t fireSolenoid(Solenoid *s) {
    if(s->triggers) return 0;
    if(s->triggered) return 0;
    
    if(msElapsed < s->dontFireBefore)
        callIn(fireSolenoid, s, s->dontFireBefore-msElapsed);
    else {
        s->triggered = 0;
        setOut(s->pin, ON);
        s->dontFireBefore = msElapsed + s->onTime + s->offTime;
        if(!callIn(turnOffSolenoid, s, s->onTime)) {
            setOut(s->pin, OFF);
        }
    }
    return 0;
}

void turnOnSolenoid(Solenoid *s) {
    if(s->triggers) return;
    if(msElapsed < s->dontFireBefore) return;
    if(s->onSince) return;
    
    setOut(s->pin, ON);
    s->onSince = msElapsed;
}

void initSolenoid(Solenoid *s) {
    if(s->triggers) {
        initIn(s->pin);
    }
    else {
        initOut(s->pin, OFF);
    }
}

uint8_t commandReceived(uint8_t* cmd, uint8_t len) {
    switch(cmd[0]) {
        case 0:
        case 255:
            break;
        case Ob(11111100): { //clear bookkeeping
            break;
        }
        case Ob(11111101): { //dump bookkeeping
            break;
        }
        case Ob(11111110): { //identify
            uint8_t out[2];
            out[0]=0;
            out[0] |= (0)<<4;
            out[0] |= (2)<<0;
            out[1] = 1;
            send(out, 2);
            break;
        }
        default: {
            switch(cmd[0]>>4) {
                case Ob(0000): {//fire solenoid
                    uint8_t n = cmd[0]&0xF;
                    fireSolenoid(&solenoid[n]);
                    break;
                }
                case Ob(0001): { //fire custom
                    LEN(4);
                    uint8_t n = cmd[0]&0xF;
                    uint32_t on = cmd[1];
                    uint32_t off = (cmd[2]<<8)|cmd[3];
                    Solenoid *s = &solenoid[n];
                    if(s->triggers) break;
                    setOut(s->pin, ON);
                    s->dontFireBefore = msElapsed + on + off;
                    callIn(turnOffSolenoid, s, on);
                    break;
                }
                case Ob(0010): { //fire in
                    LEN(3);
                    uint8_t n = cmd[0]&0xF;
                    uint32_t in = (cmd[1]<<8)|cmd[2];
                    callIn(fireSolenoid, (n), in);
                    break;
                }
                case Ob(0011): { //turn on solenoid
                    uint8_t n = cmd[0]&0xF;
                    Solenoid *s = &solenoid[n];
                    turnOnSolenoid(s);
                    break;
                }
                case Ob(0100): { //turn off solenoid
                    uint8_t n = cmd[0]&0xF;
                    Solenoid *s = &solenoid[n];
                    turnOffSolenoid(s);
                    break;
                }
                case Ob(0110): { //conf
                    LEN(8);
                    uint8_t n = cmd[0]&Ob(111);
                    Solenoid *s = &solenoid[n];
                    s->onTime = cmd[1];
                    s->offTime = (cmd[2]<<8)|cmd[3];
                    s->maxOnTime = (cmd[4]<<8)|cmd[5];
                    s->minOnTime = cmd[6];
                    s->triggers = cmd[7];
                    initSolenoid(s);
                    break;
                }
            }
            break;
        }
    }
}
uint32_t last=0;

uint8_t state=0;
void loop() {
    uint32_t ms = msElapsed+1;
    if(msElapsed-last>5000) {
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
        
        state=!state;
    }
    
    for(int i=0; i<16; i++) {
        Solenoid *s = &solenoid[i];
        if(s->triggers) {
            if(getIn(s->pin)) {
                turnOnSolenoid(&solenoid[s->triggers-1]);
                solenoid[s->triggers-1].triggered = 1;
            }
            else if(solenoid[s->triggers-1].triggered) {
                turnOffSolenoid(&solenoid[s->triggers-1]);
            }
        }
        else {
            if(s->onSince && s->maxOnTime) {
                if(ms - s->onSince>s->maxOnTime) {
                    turnOffSolenoid(s);
                    s->dontFireBefore = msElapsed + 10000;
                }
            }
        }
    }
    
    /*
    for(int i=0; i<4; i++) {
        if(sSolenoid[i].turnedOnAt) {
            if(ms - sSolenoid[i].turnedOnAt>sSolenoid[i].maxOnTime) {
                initOut(sSolenoid[i].pin, 0);
                setOut(specialDisable[i>=4? 1:0], 1);
            }
            /*else if(msElapsed - sSolenoid[i].turnedOnAt>sSolenoid[i].minOnTime) {
                initIn(sSolenoid[i].pin);
            }*
        }
        if(!sSolenoid[i].turnedOnAt && getIn(sSolenoid[i].pin)) {
            sSolenoid[i].turnedOnAt = ms;
        }
        if(sSolenoid[i].turnedOnAt && !getIn(sSolenoid[i].pin) && ms - sSolenoid[i].turnedOnAt>5) {
            if(ms - sSolenoid[i].turnedOnAt > sSolenoid[i].maxOnTime) {
                //if(msElapsed - sSolenoid[i].turnedOnAt > sSolenoid[i].maxOnTime + sSolenoid[i].offTime) {
                    initIn(sSolenoid[i].pin);
                    if(!specialDisabled[i>=4?1:0])
                        setOut(specialDisable[i>=4? 1:0], 0);
                    sSolenoid[i].turnedOnAt = 0;
               // }
            }
            else {
                /*if(sSolenoid[i].minOnTime && 
                    msElapsed - sSolenoid[i].turnedOnAt < sSolenoid[i].minOnTime) {
                    initOut(sSolenoid[i].pin, 1);
                }
                else *
                    sSolenoid[i].turnedOnAt = 0;
            }
        }
    }*/
}

void init() {
    CNPDB=0;
    for(int i=0; i<16; i++) {
        initOut(solenoid[i].pin, 0);
        solenoid[i].onTime = 25;
        solenoid[i].offTime = 100;
        solenoid[i].maxOnTime = 100;
        solenoid[i].minOnTime = 100;
        solenoid[i].onSince = 0;
        solenoid[i].dontFireBefore = 0;
    }
    solenoid[15].triggers = 1;
    for(int i=0; i<16; i++) {
        initSolenoid(&solenoid[i]);
    }
    
}

void crashed() {
    for(int i=0; i<16; i++) {
        initIn(solenoid[i].pin);
    }
}