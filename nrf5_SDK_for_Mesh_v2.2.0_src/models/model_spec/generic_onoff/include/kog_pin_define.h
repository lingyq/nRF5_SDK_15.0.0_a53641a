#ifndef KOG_PIN_DEFINE_H
#define KOG_PIN_DEFINE_H

//asw0104

#define LED1_R  9
#define LED1_G  10

#define LED2_R  2
#define LED2_G  31

#define LED3_R  30
#define LED3_G  29

#define LED4_R  16
#define LED4_G  7

#define RELAY_A 25
#define RELAY_B 26
#define RELAY_C 27
#define RELAY_D 28

#define ALL_LED_R ((1 << LED1_R) | (1 << LED2_R) | (1 << LED3_R) | (1 << LED4_R))
#define ALL_LED_G ((1 << LED1_G) | (1 << LED2_G) | (1 << LED3_G) | (1 << LED4_G))

#endif