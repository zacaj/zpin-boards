#include "../test3.X/sat.c"
void error() {

}

typedef enum {
    Disabled = 0,
    Input = 1,
    Momentary = 2,
    OnOff = 3,
    Triggered = 4,
} Mode;


typedef struct {
    Pin pin;
    Mode mode;
    
    uint8_t pulseOnTime; 
    uint8_t pulseOffTime; // if !0, once maxOnTime is reached coil will PWM, 1ms on, this amount off, instead of disabling
    uint32_t cooldownTime; // if !0, how long it must be off before allowing another trigger (in OnOff only used after max is reached)

    // Input
    uint8_t settleTime; // how long the input must be stably off to count as off
    uint8_t inverted; // default yes

    // Triggered:
    uint8_t triggeredBy; // number of the solenoid that will trigger it
    uint32_t minOnTime;
    // maxOnTime
    uint8_t disabled; // if set trigger is disabled until it clears

    // momentary:
    uint32_t onTime;

    // on off
    uint32_t maxOnTime; // if !0, max time it can stay on before cooldown activates

    uint32_t dontFireBefore; // if !0, will block activation before this time.  if -1 disabled while input is on
    uint32_t onSince; // if !0, the solenoid is on, and has been since this time
    uint32_t lastOnAt; // for triggered, time the switch turned off last
    uint8_t on; // whether on signal is high or low
} Solenoid;

#define SOL_DEFAULT(port, pin, lp) { { port, pin }, Disabled, 1, 0, 0, 0, 1, -1, 0, 0, 50, 0, 0, 0, 0, lp}

#define V7
#ifdef V4
//note v4 pins
Solenoid solenoid[16] = {
    SOL_DEFAULT(IOPORT_B, p6, 0),
    SOL_DEFAULT(IOPORT_B, p7, 0),
    SOL_DEFAULT(IOPORT_B, p8, 0),
    SOL_DEFAULT(IOPORT_B, p11, 1), // low power + connector
    SOL_DEFAULT(IOPORT_B, p9, 0),
    SOL_DEFAULT(IOPORT_B, p13, 0),

    SOL_DEFAULT(IOPORT_B, p0, 0),
    SOL_DEFAULT(IOPORT_B, p1, 0),
    SOL_DEFAULT(IOPORT_B, p2, 0),
    SOL_DEFAULT(IOPORT_B, p3, 1), //low power
    SOL_DEFAULT(IOPORT_A, p3, 0),
    SOL_DEFAULT(IOPORT_B, p4, 0),

    SOL_DEFAULT(IOPORT_B, p12, 0),
    SOL_DEFAULT(IOPORT_B, p10, 1), // low power + connector
    SOL_DEFAULT(IOPORT_A, p4, 0),
    SOL_DEFAULT(IOPORT_B, p5, 1), // low power
};
#endif
#ifdef V7
//note v7 pins
Solenoid solenoid[16] = {
    SOL_DEFAULT(IOPORT_B, p0, 0),
    SOL_DEFAULT(IOPORT_B, p1, 0),
    SOL_DEFAULT(IOPORT_B, p2, 0),
    SOL_DEFAULT(IOPORT_A, p3, 0),
    SOL_DEFAULT(IOPORT_B, p4, 0),
    SOL_DEFAULT(IOPORT_A, p4, 0),

    SOL_DEFAULT(IOPORT_B, p6, 0),
    SOL_DEFAULT(IOPORT_B, p7, 0),
    SOL_DEFAULT(IOPORT_B, p8, 0),
    SOL_DEFAULT(IOPORT_B, p9, 0),
    SOL_DEFAULT(IOPORT_B, p13, 0),
    SOL_DEFAULT(IOPORT_B, p12, 0),
    
    SOL_DEFAULT(IOPORT_B, p5, 1), // low power
    SOL_DEFAULT(IOPORT_B, p11, 1), // low power + connector

    SOL_DEFAULT(IOPORT_B, p3, 1), // low power
    SOL_DEFAULT(IOPORT_B, p10, 1), // low power + connector
};
#endif
#ifndef V7
#ifndef V4
#error version missing
#endif
#endif

uint32_t turnOffSolenoid(Solenoid *s) {
    if (!s->onSince) return 0;

    switch(s->mode) {
        case Input:
        case Disabled:
            return 0;
        case Momentary:
            if (s->cooldownTime) {
                s->dontFireBefore = msElapsed + s->cooldownTime;
            }
            break;
        case Triggered:
            if (s->cooldownTime) {
                s->dontFireBefore = msElapsed + s->cooldownTime;
            }
            if(s->minOnTime && s->onSince && msElapsed-s->onSince < s->minOnTime) {
                return 0;
            }
            break;
        case OnOff:
            break;
    }

    setOut(s->pin, !s->on);
    s->onSince = 0;
    return 0;
}

void turnOnSolenoid(Solenoid *s) {
    switch(s->mode) {
        case Input:
        case Disabled:
            break;
        case Momentary:
        case Triggered:
            if(s->onSince) return;
        case OnOff:
            if(msElapsed < s->dontFireBefore) return;

            setOut(s->pin, s->on);
            s->onSince = msElapsed;
            break;
    }
}

uint32_t fireSolenoid(Solenoid *s, uint32_t onTime) {
    switch(s->mode) {
        case Input:
        case Disabled:
        case Triggered:
        case OnOff:
            return 0;
        case Momentary:
            if (msElapsed < s->dontFireBefore) {
                return 0;
            }
            if (s->onSince)
                return 0;
            break;
    }
    // only momentary
    if (!onTime)
        onTime = s->onTime;

    turnOnSolenoid(s);

    if(!callIn(turnOffSolenoid, s, onTime)) {
        setOut(s->pin, !s->on);
    }
    return 0;
}

void initSolenoid(Solenoid *s) {
    s->onSince = 0;
    s->dontFireBefore = 0;
    s->disabled = 0;
    s->lastOnAt = 0;
    switch(s->mode) {
        case Input:
            initIn(s->pin);
            break;
        case Disabled:
            //initIn(s->pin);
            //break;
        case OnOff:
        case Momentary:
        case Triggered:
            initOut(s->pin, !s->on);
            break;
    }
}

uint8_t commandReceived(uint8_t* cmd, uint8_t len) {
    switch(cmd[0]) {
        case 255:
            break;
        case Ob(11111100): { //clear bookkeeping
            break;
        }
        case Ob(11111101): { //dump bookkeeping
            break;
        }
        case Ob(11111110): { //identify
            out[0]=0;
            out[0] |= (0)<<4; // board revision
            out[0] |= (5)<<0; // board type id
            out[1] = 2; // api revision
            return 2;
            break;
        }
        default: {
            switch(cmd[0]>>4) {
                case Ob(0000): {//fire solenoid
                    uint8_t n = cmd[0]&0xF;
                    fireSolenoid(&solenoid[n], 0);
                    break;
                }
                case Ob(0001): { //fire custom
                    LEN(5);
                    uint8_t n = cmd[0]&0xF;
                    uint32_t on = read32(1);
                    Solenoid *s = &solenoid[n];
                    fireSolenoid(s, on);
                    break;
                }
                case Ob(0010): { //fire in
                    LEN(3);
                    uint8_t n = cmd[0]&0xF;
                    uint32_t in = (cmd[1]<<8)|cmd[2];
                    Solenoid *s = &solenoid[n];
                    callIn(fireSolenoid, s, in);
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
                    uint8_t n = cmd[0]&0xF;
                    Solenoid *s = &solenoid[n];
                    Mode mode = (cmd[1]);
                    switch (mode) {
                        case Disabled:
                            LEN(7);
                            break;
                        case Input:
                            LEN(8);
                            s->settleTime = cmd[7];
                            break;
                        case Triggered:
                            LEN(16);
                            s->triggeredBy = cmd[7];
                            s->minOnTime = read32(8);
                            s->maxOnTime = read32(12);
                            break;
                        case Momentary:
                            LEN(11);
                            s->onTime = read32(7);
                            break;
                        case OnOff:
                            LEN(12);
                            s->pulseOnTime = cmd[7];
                            s->maxOnTime = read32(8);
                            break;
                    }
                    s->mode = mode;
                    s->cooldownTime = read32(2);
                    s->pulseOffTime = cmd[6];
                    initSolenoid(s);
                    break;
                }
            }
            break;
        }
    }
    return 0;
}
uint32_t last=0;

uint8_t state=0;
uint32_t I = 0;
void loop() {
    uint32_t ms = msElapsed+1;
#if 0
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
//        fireSolenoid(&solenoid[I], 0);

        I++;
        if (I >= 16) I = 0;

    }
#endif

    for (int i=0; i<16; i++) {
        Solenoid* s = &solenoid[i];
        switch(s->mode) {
            case Disabled:
            case Momentary:
                break;
            case Input: {
                uint8_t input = getIn(s->pin);
                if (input) {
                }
                else {

                }
                break;
            }
            case OnOff:
                if (s->onSince) {
                    if (s->maxOnTime && msElapsed-s->onSince > s->maxOnTime) {
                        if (s->pulseOffTime) { // pwm
                            if (dmsElapsed % (s->pulseOffTime + s->pulseOnTime) < s->pulseOnTime)
                               setOut(s->pin, s->on);
                            else
                                setOut(s->pin, !s->on);    
                        }
                        else {
                            turnOffSolenoid(s);
                        }
                    }
                }
                break;
#if 0
            case Triggered: {
                if (s->triggeredBy >= 16)
                    break;
                Solenoid* trigger = &solenoid[s->triggeredBy];
                uint8_t input = getIn(trigger->pin) ^ trigger->inverted;
                if (!input) {
                    if (msElapsed > s->lastOnAt + trigger->settleTime) {
                        s->disabled = 0;
                    }
                    turnOffSolenoid(s);
                }
                else {
                    if (s->onSince) { // already on
                        if (s->maxOnTime && msElapsed - s->onSince > s->maxOnTime) {
                            if (s->pulseOffTime) { // pwm
                                uint32_t t = msElapsed - s->onSince - s->maxOnTime;
                                if (dmsElapsed % (s->pulseOffTime + 1) == 0)
                                   setOut(s->pin, s->on);
                                else
                                    setOut(s->pin, !s->on);    
                            }
                            else {
                                turnOffSolenoid(s);
                                s->disabled = 1;
                            }
                        }
                    }
                    else if (!s->disabled && msElapsed > s->lastOnAt + trigger->settleTime) {
                        turnOnSolenoid(s);
                    }
                    s->lastOnAt = msElapsed;
                }
                break;
            }
#endif
        }
    }

}

void init() {
    CNPDB=0;
    for(int i=0; i<16; i++) {
        solenoid[i].mode = Disabled;
        initSolenoid(&solenoid[i]);
//        setOut(solenoid[i].pin, !solenoid[i].on);
    }
//    for(int i=0; i<16; i++) {
//        solenoid[i].mode = Momentary;
//        solenoid[i].onTime = 250;
//    }
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


    for(int i=0; i<16; i++) {
        initSolenoid(&solenoid[i]);
    }

    //turnOnSolenoid(&solenoid[0]);
}

void crashed() {
    for(int i=0; i<16; i++) {
        solenoid[i].mode = Disabled;
        setOut(solenoid[i].pin, !solenoid[i].on);
    }
}