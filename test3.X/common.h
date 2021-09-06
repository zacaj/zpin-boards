
#ifndef COMMON_H
#define	COMMON_H


#define _SUPPRESS_PLIB_WARNING

//#include <peripheral/adc10.h>
//#include <peripheral/bmx.h>
//#include <peripheral/cmp.h>
//#include <peripheral/cvref.h>
//#include <peripheral/dma.h>
//#include <peripheral/i2c.h>
//#include <peripheral/incap.h>
#include <peripheral/int.h>
//#include <peripheral/nvm.h>
//#include <peripheral/outcompare.h>
//#include <peripheral/pcache.h>
//#include <peripheral/pmp.h>
#include <peripheral/ports.h>
#include <peripheral/pps.h>
//#include <peripheral/power.h>
//#include <peripheral/reset.h>
//#include <peripheral/rtcc.h>
#include <peripheral/spi.h>
#include <peripheral/system.h>
#include <peripheral/timer.h>
//#include <peripheral/uart.h>
//#include <peripheral/wdt.h>
//#include <peripheral/eth.h>
//#include <peripheral/CAN.h>
#include <stdint.h>

#define Ob(x)  ((unsigned)Ob_(0 ## x ## uL))
#define Ob_(x) ((x & 1) | (x >> 2 & 2) | (x >> 4 & 4) | (x >> 6 & 8) |		\
	(x >> 8 & 16) | (x >> 10 & 32) | (x >> 12 & 64) | (x >> 14 & 128))


#define bA IOPORT_A
#define bB IOPORT_B
#define p15                       (1 << 15)
#define p14                       (1 << 14)
#define p13                       (1 << 13)
#define p12                       (1 << 12)
#define p11                       (1 << 11)
#define p10                       (1 << 10)
#define p9                        (1 << 9)
#define p8                        (1 << 8)
#define p7                        (1 << 7)
#define p6                        (1 << 6)
#define p5                        (1 << 5)
#define p4                        (1 << 4)
#define p3                        (1 << 3)
#define p2                        (1 << 2)
#define p1                        (1 << 1)
#define p0                        (1 << 0)




typedef struct {
    IoPortId port;
    uint32_t pin;
} Pin;

typedef enum  {
    false = 0,
    true
} boolean;

#define SYS_FREQ         (40000000L)

#define msElapsed ((ReadCoreTimer()/20000))
#define dmsElapsed ((ReadCoreTimer()/2000))
#define cmsElapsed ((ReadCoreTimer()/200))
#define usElapsed ((ReadCoreTimer()/20))
void waitUs(uint32_t us);
void initPic32();
void setOut(Pin pin, uint8_t value);
void initIn(Pin pin);
void initOut(Pin pin, uint8_t initialValue);
#endif	
