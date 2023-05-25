#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "struct_typedef.h"

extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);

}
#endif
#endif
