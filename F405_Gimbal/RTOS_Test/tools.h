#ifndef _TOOLS_H
#define _TOOLS_H

#ifndef TRUE
#define TRUE 1
#endif // !
#ifndef FALSE
#define FALSE 0
#endif // !
#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef PI_2
#define PI_2 6.2831853072f
#endif

/* 普通延时 */
void delay_ms(unsigned long t);
void delay_us(unsigned long t);
void delay_us_f(float us);

#endif // !_TOOLS_H
