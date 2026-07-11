#ifndef __LED_H
#define	__LED_H

#include "stm32h7xx.h"

//���Ŷ���
/*******************************************************/

//RUN
#define RUN_PIN                  GPIO_PIN_6               
#define RUN_GPIO_PORT            GPIOG                      
#define RUN_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE()
//ERR
#define ERR_PIN                  GPIO_PIN_7               
#define ERR_GPIO_PORT            GPIOG                      
#define ERR_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE()
// DO2
#define DO2_PIN                  GPIO_PIN_2                 
#define DO2_GPIO_PORT            GPIOC                     
#define DO2_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()

// DO3
#define DO3_PIN                  GPIO_PIN_3                 
#define DO3_GPIO_PORT            GPIOC                 
#define DO3_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()

// DO4
#define DO4_PIN                  GPIO_PIN_0                 
#define DO4_GPIO_PORT            GPIOA                       
#define DO4_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

// DO5
#define DO5_PIN                  GPIO_PIN_4               
#define DO5_GPIO_PORT            GPIOA                      
#define DO5_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

// DO6
#define DO6_PIN                  GPIO_PIN_5                 
#define DO6_GPIO_PORT            GPIOA                     
#define DO6_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

// DO7
#define DO7_PIN                  GPIO_PIN_6                
#define DO7_GPIO_PORT            GPIOA                 
#define DO7_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

// DO8
#define DO8_PIN                  GPIO_PIN_0                 
#define DO8_GPIO_PORT            GPIOB                       
#define DO8_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

// DO9
#define DO9_PIN                  GPIO_PIN_1                 
#define DO9_GPIO_PORT            GPIOB                       
#define DO9_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

// DO10
#define DO10_PIN                  GPIO_PIN_11                 
#define DO10_GPIO_PORT            GPIOF                       
#define DO10_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

// DO11
#define DO11_PIN                  GPIO_PIN_12                 
#define DO11_GPIO_PORT            GPIOF                       
#define DO11_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

// DO12
#define DO12_PIN                  GPIO_PIN_13                 
#define DO12_GPIO_PORT            GPIOF                       
#define DO12_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

// DO13
#define DO13_PIN                  GPIO_PIN_14                 
#define DO13_GPIO_PORT            GPIOF                       
#define DO13_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

// DO14
#define DO14_PIN                  GPIO_PIN_15                 
#define DO14_GPIO_PORT            GPIOF                      
#define DO14_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

// DO15
#define DO15_PIN                  GPIO_PIN_0                 
#define DO15_GPIO_PORT            GPIOG                       
#define DO15_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE()

// DO16
#define DO16_PIN                  GPIO_PIN_1                 
#define DO16_GPIO_PORT            GPIOG                       
#define DO16_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE()

//ETH_RST
#define ETH_RST                  GPIO_PIN_10                 
#define ETH_RST_Compatible           GPIO_PIN_12 
#define ETH_RST_GPIO_PORT            GPIOB                       
#define ETH_RST_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()
/************************************************************/


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define OFF  GPIO_PIN_RESET
#define ON GPIO_PIN_SET

/* ���κ꣬��������������һ��ʹ�� */
#define DO2(a)	HAL_GPIO_WritePin(DO2_GPIO_PORT,DO2_PIN,a)

#define DO3(a)	HAL_GPIO_WritePin(DO3_GPIO_PORT,DO3_PIN,a)

#define DO4(a)	HAL_GPIO_WritePin(DO4_GPIO_PORT,DO4_PIN,a)

#define DO5(a)	HAL_GPIO_WritePin(DO5_GPIO_PORT,DO5_PIN,a)

#define DO6(a)	HAL_GPIO_WritePin(DO6_GPIO_PORT,DO6_PIN,a)

#define DO7(a)	HAL_GPIO_WritePin(DO7_GPIO_PORT,DO7_PIN,a)

#define DO8(a)	HAL_GPIO_WritePin(DO8_GPIO_PORT,DO8_PIN,a)

#define DO9(a)	HAL_GPIO_WritePin(DO9_GPIO_PORT,DO9_PIN,a)

#define DO10(a)	HAL_GPIO_WritePin(DO10_GPIO_PORT,DO10_PIN,a)

#define DO11(a)	HAL_GPIO_WritePin(DO11_GPIO_PORT,DO11_PIN,a)

#define DO12(a)	HAL_GPIO_WritePin(DO12_GPIO_PORT,DO12_PIN,a)

#define DO13(a)	HAL_GPIO_WritePin(DO13_GPIO_PORT,DO13_PIN,a)

#define DO14(a)	HAL_GPIO_WritePin(DO14_GPIO_PORT,DO14_PIN,a)

#define DO15(a)	HAL_GPIO_WritePin(DO15_GPIO_PORT,DO15_PIN,a)

#define DO16(a)	HAL_GPIO_WritePin(DO16_GPIO_PORT,DO16_PIN,a)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)				{p->BSRR=i;}			  //����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)				{p->BSRR=i<<16;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define DO2_TOGGLE		digitalToggle(DO2_GPIO_PORT,DO2_PIN)
#define DO2_ON			digitalHi(DO2_GPIO_PORT,DO2_PIN)
#define DO2_OFF				digitalLo(DO2_GPIO_PORT,DO2_PIN)

#define DO3_TOGGLE		digitalToggle(DO3_GPIO_PORT,DO3_PIN)
#define DO3_ON			digitalHi(DO3_GPIO_PORT,DO3_PIN)
#define DO3_OFF				digitalLo(DO3_GPIO_PORT,DO3_PIN)

#define DO4_TOGGLE		digitalToggle(DO4_GPIO_PORT,DO4_PIN)
#define DO4_ON			digitalHi(DO4_GPIO_PORT,DO4_PIN)
#define DO4_OFF				digitalLo(DO4_GPIO_PORT,DO4_PIN)

#define DO5_TOGGLE		digitalToggle(DO5_GPIO_PORT,DO5_PIN)
#define DO5_ON			digitalHi(DO5_GPIO_PORT,DO5_PIN)
#define DO5_OFF				digitalLo(DO5_GPIO_PORT,DO5_PIN)

#define DO6_TOGGLE		digitalToggle(DO2_GPIO_PORT,DO2_PIN)
#define DO6_ON			digitalHi(DO6_GPIO_PORT,DO6_PIN)
#define DO6_OFF				digitalLo(DO6_GPIO_PORT,DO6_PIN)

#define DO7_TOGGLE		digitalToggle(DO7_GPIO_PORT,DO7_PIN)
#define DO7_ON			digitalHi(DO7_GPIO_PORT,DO7_PIN)
#define DO7_OFF				digitalLo(DO7_GPIO_PORT,DO7_PIN)

#define DO8_TOGGLE		digitalToggle(DO8_GPIO_PORT,DO8_PIN)
#define DO8_ON			digitalHi(DO8_GPIO_PORT,DO8_PIN)
#define DO8_OFF				digitalLo(DO8_GPIO_PORT,DO8_PIN)

#define DO9_TOGGLE		digitalToggle(DO9_GPIO_PORT,DO9_PIN)
#define DO9_ON			digitalHi(DO9_GPIO_PORT,DO9_PIN)
#define DO9_OFF				digitalLo(DO9_GPIO_PORT,DO9_PIN)

#define DO10_TOGGLE		digitalToggle(DO10_GPIO_PORT,DO10_PIN)
#define DO10_ON			digitalHi(DO10_GPIO_PORT,DO10_PIN)
#define DO10_OFF				digitalLo(DO10_GPIO_PORT,DO10_PIN)

#define DO11_TOGGLE		digitalToggle(DO11_GPIO_PORT,DO11_PIN)
#define DO11_ON			digitalHi(DO11_GPIO_PORT,DO11_PIN)
#define DO11_OFF				digitalLo(DO11_GPIO_PORT,DO11_PIN)

#define DO12_TOGGLE		digitalToggle(DO12_GPIO_PORT,DO12_PIN)
#define DO12_ON			digitalHi(DO12_GPIO_PORT,DO12_PIN)
#define DO12_OFF				digitalLo(DO12_GPIO_PORT,DO12_PIN)

#define DO13_TOGGLE		digitalToggle(DO13_GPIO_PORT,DO13_PIN)
#define DO13_ON			digitalHi(DO13_GPIO_PORT,DO13_PIN)
#define DO13_OFF				digitalLo(DO13_GPIO_PORT,DO13_PIN)


#define DO14_TOGGLE		digitalToggle(DO14_GPIO_PORT,DO14_PIN)
#define DO14_ON			digitalHi(DO14_GPIO_PORT,DO14_PIN)
#define DO14_OFF				digitalLo(DO14_GPIO_PORT,DO14_PIN)


#define DO15_TOGGLE		digitalToggle(DO15_GPIO_PORT,DO15_PIN)
#define DO15_ON			digitalHi(DO15_GPIO_PORT,DO15_PIN)
#define DO15_OFF				digitalLo(DO15_GPIO_PORT,DO15_PIN)

#define DO16_TOGGLE		digitalToggle(DO16_GPIO_PORT,DO16_PIN)
#define DO16_ON			digitalHi(DO16_GPIO_PORT,DO16_PIN)
#define DO16_OFF				digitalLo(DO16_GPIO_PORT,DO16_PIN)

#define RUN_TOGGLE		digitalToggle(RUN_GPIO_PORT,RUN_PIN)
#define RUN_ON			digitalHi(RUN_GPIO_PORT,RUN_PIN)
#define RUN_OFF				digitalLo(RUN_GPIO_PORT,RUN_PIN)

#define ERR_TOGGLE		digitalToggle(ERR_GPIO_PORT,ERR_PIN)
#define ERR_ON			digitalHi(ERR_GPIO_PORT,ERR_PIN)
#define ERR_OFF				digitalLo(ERR_GPIO_PORT,ERR_PIN)


#define ETH_RST_TOGGLE		digitalToggle(ETH_RST_GPIO_PORT,ETH_RST)
#define ETH_RST_ON			digitalHi(ETH_RST_GPIO_PORT,ETH_RST)
#define ETH_RST_OFF				digitalLo(ETH_RST_GPIO_PORT,ETH_RST)

#define ETH_RST_Compatible_ON			digitalHi(ETH_RST_GPIO_PORT,ETH_RST_Compatible)
#define ETH_RST_Compatible_OFF			digitalLo(ETH_RST_GPIO_PORT,ETH_RST_Compatible)
//(ȫ����)
#define LED_ALLON	\
					DO2_ON\
					DO3_ON\
					DO4_ON\
					DO5_ON\
					DO6_ON\
					DO7_ON\
					DO8_ON\
					DO9_ON\
					DO10_ON\
					DO11_ON\
					DO12_ON\
					DO13_ON\
					DO14_ON\
					DO15_ON\
					DO16_ON
					
//(ȫ���ر�)
#define LED_ALLOFF	\
					DO2_OFF\
					DO3_OFF\
					DO4_OFF\
					DO5_OFF\
					DO6_OFF\
					DO7_OFF\
					DO8_OFF\
					DO9_OFF\
					DO10_OFF\
					DO11_OFF\
					DO12_OFF\
					DO13_OFF\
					DO14_OFF\
					DO15_OFF\
					DO16_OFF

#define LED_ALLTOGGLE \
					DO2_TOGGLE\
					DO3_TOGGLE\
					DO5_TOGGLE\
					DO6_TOGGLE\
					DO7_TOGGLE\
					DO8_TOGGLE\
					DO9_TOGGLE\
					DO10_TOGGLE\
					DO11_TOGGLE\
					DO12_TOGGLE\
					DO13_TOGGLE\
					DO14_TOGGLE\
					DO15_TOGGLE\
					DO16_TOGGLE



/*******************************************************/
#define DI1_PIN                  GPIO_PIN_5                 
#define DI1_GPIO_PORT            GPIOF                      
#define DI1_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

#define DI2_PIN                  GPIO_PIN_4               
#define DI2_GPIO_PORT            GPIOF                 
#define DI2_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

#define DI3_PIN                  GPIO_PIN_3              
#define DI3_GPIO_PORT            GPIOF                
#define DI3_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

#define DI4_PIN                  GPIO_PIN_2               
#define DI4_GPIO_PORT            GPIOF                 
#define DI4_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

#define DI5_PIN                  GPIO_PIN_1              
#define DI5_GPIO_PORT            GPIOF                 
#define DI5_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

#define DI6_PIN                  GPIO_PIN_0              
#define DI6_GPIO_PORT            GPIOF                 
#define DI6_GPIO_CLK_ENABLE()    __GPIOF_CLK_ENABLE()

#define DI7_PIN                  GPIO_PIN_15              
#define DI7_GPIO_PORT            GPIOC                 
#define DI7_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()

#define DI8_PIN                  GPIO_PIN_14              
#define DI8_GPIO_PORT            GPIOC                 
#define DI8_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()

#define DI9_PIN                  GPIO_PIN_13              
#define DI9_GPIO_PORT            GPIOC                 
#define DI9_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()

#define DI10_PIN                  GPIO_PIN_6              
#define DI10_GPIO_PORT            GPIOE                 
#define DI10_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()

#define DI11_PIN                  GPIO_PIN_5              
#define DI11_GPIO_PORT            GPIOE                
#define DI11_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()

#define DI12_PIN                  GPIO_PIN_4              
#define DI12_GPIO_PORT            GPIOE                 
#define DI12_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()

#define DI13_PIN                  GPIO_PIN_3              
#define DI13_GPIO_PORT            GPIOE                 
#define DI13_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()

#define DI14_PIN                  GPIO_PIN_2              
#define DI14_GPIO_PORT            GPIOE                 
#define DI14_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()

#define DI15_PIN                  GPIO_PIN_1              
#define DI15_GPIO_PORT            GPIOE                 
#define DI15_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()

#define DI16_PIN                  GPIO_PIN_0              
#define DI16_GPIO_PORT            GPIOE                 
#define DI16_GPIO_CLK_ENABLE()    __GPIOE_CLK_ENABLE()




/*******************************************************/

 /** �������±��ú�
	* ��������Ϊ�ߵ�ƽ������ KEY_ON=1�� KEY_OFF=0
	* ����������Ϊ�͵�ƽ���Ѻ����ó�KEY_ON=0 ��KEY_OFF=1 ����
	*/
#define KEY_ON	1
#define KEY_OFF	0

void MX_GPIO_Init(void);

//void Key_GPIO_Config(void);
//uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

#endif /* __LED_H */
