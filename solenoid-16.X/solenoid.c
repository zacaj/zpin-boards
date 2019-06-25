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

#define ON 0
#define OFF 1

typedef struct {
    Pin pin;
    Mode mode;

    uint32_t cooldownTime; // if !0, how long it must be off before allowing another trigger (in OnOff only used after max is reached)

    // Input
    uint8_t settleTime; // how long the input must be stably off to count as off
    
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
} Solenoid;

#define SOL_DEFAULT(port, pin) { { port, pin }, Disabled, 0, 0, 2, -1, 0, 0, 50, 0, 0, 0, 0}
//note v4 pins
Solenoid solenoid[16] = {
    SOL_DEFAULT(IOPORT_B, p6),
    SOL_DEFAULT(IOPORT_B, p7),
    SOL_DEFAULT(IOPORT_B, p8),
    SOL_DEFAULT(IOPORT_B, p11), // low power + connector
    SOL_DEFAULT(IOPORT_B, p9),
    SOL_DEFAULT(IOPORT_B, p13),

    SOL_DEFAULT(IOPORT_B, p0),
    SOL_DEFAULT(IOPORT_B, p1),
    SOL_DEFAULT(IOPORT_B, p2),
    SOL_DEFAULT(IOPORT_B, p3), //low power
    SOL_DEFAULT(IOPORT_A, p3),
    SOL_DEFAULT(IOPORT_B, p4),

    SOL_DEFAULT(IOPORT_B, p12),
    SOL_DEFAULT(IOPORT_B, p10), // low power + connector
    SOL_DEFAULT(IOPORT_A, p4),
    SOL_DEFAULT(IOPORT_B, p5), // low power
};


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

    setOut(s->pin, OFF);
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
        case OnOff:
            if(msElapsed < s->dontFireBefore) return;
            if(s->onSince) return;

            setOut(s->pin, ON);
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
        setOut(s->pin, OFF);
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
            initIn(s->pin);
            break;
        case OnOff:
        case Momentary:
        case Triggered:
            initOut(s->pin, OFF);
            break;
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
                    fireSolenoid(&solenoid[n], 0);
                    break;
                }
                case Ob(0001): { //fire custom
                    LEN(4);
                    uint8_t n = cmd[0]&0xF;
                    uint32_t on = cmd[1];
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
                    Mode mode = (cmd[0]&0xF0)>>4;
                    switch (mode) {
                        case Disabled:
                            break;
                            LEN(6);
                            s->settleTime = cmd[5];
                        case Input:
                            break;
                        case Triggered:
                            LEN(14);
                            s->triggeredBy = cmd[5];
                            s->minOnTime = read32(7);
                            s->maxOnTime = read32(8);
                            break;
                        case Momentary:
                            LEN(9);
                            s->onTime = read32(5);
                            break;
                        case OnOff:
                            LEN(9);
                            s->maxOnTime = read32(5);
                            break;
                    }
                    s->mode = mode;
                    s->cooldownTime = read32(1);
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
                        turnOffSolenoid(s);
                    }
                }
                break;
            case Triggered: {
                if (s->triggeredBy >= 16)
                    break;
                Solenoid* trigger = &solenoid[s->triggeredBy];
                uint8_t input = getIn(trigger->pin);
                if (!input) {
                    if (msElapsed > s->lastOnAt + trigger->settleTime) {
                        s->disabled = 0;
                    }
                    turnOffSolenoid(s);
                }
                else {
                    if (s->onSince) { // already on
                        if (s->maxOnTime && msElapsed - s->onSince > s->maxOnTime) {
                            turnOffSolenoid(s);
                            s->disabled = 1;
                        }
                    }
                    else if (!s->disabled && msElapsed > s->lastOnAt + trigger->settleTime) {
                        turnOnSolenoid(s);
                    }
                    s->lastOnAt = msElapsed;
                }
                break;
            }
                
        }
    }

}

void init() {
    CNPDB=0;
    
    solenoid[0].mode = Triggered;
    solenoid[0].minOnTime = 100;
    solenoid[0].maxOnTime = 100;
    solenoid[0].triggeredBy = 15;
    solenoid[0].cooldownTime = 10;

    solenoid[15].mode = Input;
    solenoid[15].settleTime = 5;

    for(int i=0; i<16; i++) {
        initSolenoid(&solenoid[i]);
    }

}

void crashed() {
    for(int i=0; i<16; i++) {
        initIn(solenoid[i].pin);
    }
}