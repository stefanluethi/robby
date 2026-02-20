# Hardware Map

## Pin map of STM32F723 DISCO board:

| Designator  | Direction | Function | Pin  | 
|-------------|-----------|----------|------|
| LED 1 | out | GPIO | PA7 |
| LED 2 | out | GPIO | PB1 |
| Serial Tx | out | USART6_TX | PC6 |
| Serial Rx | in | USART6_RX | PC7 |

## Pin map of Motor Base board:

| Designator | Direction | Function | Pin  | 
|------------|-----------|----------|------|
| M1, M2 Sleep | out | GPIO | PH3 |
| M1 PWM | out | TIM9_CH1 | PE5 |
| M2 PWM | out | TIM9_CH2 | PE6 |
| M1 Direction | out | GPIO | PE3 |
| M2 Direction | out | GPIO | PE4 |
| M1 ENC A | in | TIM2_CH1 | PA5 |
| M1 ENC B | in | TIM2_CH2 | PA1 |
| M2 ENC A | in | TIM3_CHxy | PB4 |
| M2 ENC B | in | TIM3_CHxy | PB5 |
| M1 Current | in | ADC1_IN4 | PA4 |
| M2 Current | in | ADC1_IN6 | PA6 |
| M1, M2 Fault | in | GPIO | PC5 |
| VIN | in | ADC3_IN8 | PF10 |
| Buzzer | out | TIM12_CH1 | PH6 |
| I2C SCL | in/out | I2C2_SCL | PH4 |
| I2C SDA | in/out | I2C2_SDA | PH5 |

