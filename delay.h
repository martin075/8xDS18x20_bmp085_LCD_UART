#ifndef _DELAY_H_
#define _DELAY_H_

#include <inttypes.h>
#include <avr/io.h>

/* RTB - needed a shorter delay that generated by C function
 * call delayloop16( 1)
 */
 #define DELAY_1US() __asm__ __volatile__ ( \
					"nop \n\t"  \
					"nop \n\t"  \
					"nop \n\t"  \
					"nop \n\t"  \
					"nop \n\t"  \
					"nop \n\t"  \
					"nop \n\t"  \
					"nop \n\t"  \
					"nop \n\t"  \
					::)                           

/* delay function for microsec
 * 4 cpu cycles per loop + 1 cycles(?) overhead 
 * when a constant is passed.
 */
static inline void delayloop16(uint16_t count)
{
	asm volatile (  "cp  %A0,__zero_reg__ \n\t"  \
                     "cpc %B0,__zero_reg__ \n\t"  \
                     "breq L_Exit_%=       \n\t"  \
                     "L_LOOP_%=:           \n\t"  \
                     "sbiw %0,1            \n\t"  \
                     "brne L_LOOP_%=       \n\t"  \
                     "L_Exit_%=:           \n\t"  \
                     : "=w" (count)
					 : "0"  (count)
                   );                            
}
// delayloop16(x) eats 4 cycles per x
#define DELAY_US_CONV(us) ((uint16_t)(((((us)*1000L)/(1000000000/F_CPU))-1)/4))
#define delay_us(us)	  delayloop16(DELAY_US_CONV(us))

/* delay function for millisec
  (6 cycles per x + 20(?) overhead) */
void delayloop32( uint32_t l); // not inline
#define DELAY_MS_CONV(ms) ( (uint32_t) (ms*(F_CPU/6000L)) ) 
#define delay_ms(ms)  delayloop32(DELAY_MS_CONV(ms))

#endif	// _DELAY_H_
