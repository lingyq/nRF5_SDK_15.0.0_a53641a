#ifndef KOG_PIN_DEFINE_H
#define KOG_PIN_DEFINE_H

//arc0102

#define LED1_R  10
#define LED1_G  9

#define LED2_R  31
#define LED2_G  30

#define ALL_LED_R ((1 << LED1_R) | (1 << LED2_R))
#define ALL_LED_G ((1 << LED1_G) | (1 << LED2_G))

#endif