#define _SYS_FREQUENCY          72    // in MHz

// RST_PORT/RST_PIN = PB6 = SWIM_RST
// SWIM_IN_PORT/SWIM_IN_PIN = PB7 = SWIM_IN, via TIM4 input capture
// SWIM_OUT_PORT/SWIM_OUT_PIN = PB11 = SWIM (out), via TIM2.ch4 output compare

#define RST_PORT              GPIOB
#define RST_PIN               GPIO6 // stlink=GPIO_PIN_6, versaloon=GPIO_PIN_11, gebruikt als SRST
#define SWIM_OUT_PORT         GPIOB
#define SWIM_OUT_PIN          GPIO11 // stlink=GPIO_PIN_11, versaloon=GPIO_PIN_4

#define RST_SETINPUT()        gpio_set_mode(RST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, RST_PIN)
#define RST_SETINPUT_PU()     do {\
                                gpio_set_mode(RST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, RST_PIN); \
                                gpio_set(RST_PORT, RST_PIN);\
                              }while(0)
#define RST_SETINPUT_PD()     do {\
                                gpio_set_mode(RST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, RST_PIN); \
                                gpio_clear(RST_PORT, RST_PIN);\
                              }while(0)
#define RST_SETOUTPUT()       gpio_set_mode(RST_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RST_PIN)
#define RST_SETOUTPUT_OD()    gpio_set_mode(RST_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, RST_PIN)
#define RST_SET()             gpio_set(RST_PORT, RST_PIN)
#define RST_CLR()             gpio_clear(RST_PORT, RST_PIN)
#define RST_GET()             gpio_get(RST_PORT, RST_PIN)

// SYNCSW in PWM mode
#define SWIM_IN_PORT   GPIOB
#define SWIM_IN_PIN    GPIO7 // stlink=GPIO_PIN_7, versaloon=GPIO_PIN_6

#define SWIM_IN_SETINPUT()    gpio_set_mode(SWIM_IN_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SWIM_IN_PIN)
#define SWIM_IN_SETINPUT_PU() do {\
                                gpio_set_mode(SWIM_IN_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SWIM_IN_PIN);\
                                gpio_set(SWIM_IN_PORT, SWIM_IN_PIN);\
                              }while(0)
#define SWIM_IN_SETINPUT_PD() do {\
                                gpio_set_mode(SWIM_IN_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SWIM_IN_PIN); \
                                gpio_clear(SWIM_IN_PORT, SWIM_IN_PIN);\
                              }while(0)
#define SWIM_IN_SETOUTPUT()   gpio_set_mode(SWIM_IN_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SWIM_IN_PIN)
#define SWIM_IN_SET()         gpio_set(SWIM_IN_PORT, SWIM_IN_PIN)
#define SWIM_IN_CLR()         gpio_clear(SWIM_IN_PORT, SWIM_IN_PIN)
#define SWIM_IN_GET()         gpio_get(SWIM_IN_PORT, SWIM_IN_PIN)

#define SWIM_OUT_TIMER_MHZ          _SYS_FREQUENCY
#define SWIM_OUT_TIMER              TIM2 // stlink=TIM2, versaloon=TIM3
#define SWIM_OUT_TIMER_DMA_UPDATE   DMA_CHANNEL2 // stlink=DMA1_Channel2, versaloon=DMA1_Channel3
#define SWIM_OUT_TIMER_DMA_COMPARE  DMA_CHANNEL6 // not used for swim don't care
#define SWIM_IN_TIMER               TIM4
#define SWIM_IN_TIMER_RISE_DMA      DMA_CHANNEL1 // stlink=DMA1_Channel1, versaloon=DMA1_Channel4
#define SWIM_IN_TIMER_FALL_DMA      DMA_CHANNEL4 // stlink=DMA1_Channel4, versaloon=DMA1_Channel1
