#ifndef KOG_PIN_DEFINE_H
#define KOG_PIN_DEFINE_H

//asw0104

#define LED1_R  8
#define LED1_G  7

#define LED2_R  16
#define LED2_G  17

#define LED3_R  25
#define LED3_G  26

#define LED4_R  2
#define LED4_G  31

#define RELAY_A 6
#define RELAY_B 3
#define RELAY_C 28
#define RELAY_D 27

#define ALL_LED_R ((1 << LED1_R) | (1 << LED2_R) | (1 << LED3_R) | (1 << LED4_R))
#define ALL_LED_G ((1 << LED1_G) | (1 << LED2_G) | (1 << LED3_G) | (1 << LED4_G))

#endif