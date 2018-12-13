#ifndef KOG_PIN_DEFINE_H
#define KOG_PIN_DEFINE_H

//Holyiot_150972
#define LED1_R  26
#define LED1_G  27
#define LED1_B  28

#define LED2_R  11
#define LED2_G  12
#define LED2_B  13

#define RELAY_A 4
#define RELAY_B 5

#define ALL_LED_R ((1 << LED1_R) | (1 << LED2_R))
#define ALL_LED_G ((1 << LED1_G) | (1 << LED2_G))
#define ALL_LED_B ((1 << LED1_B) | (1 << LED2_B))

#endif