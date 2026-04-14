#include "else.h"
//·äÃùÆ÷
void beep_once(void) {
    gpio_set_level(BEEP_PIN, 0);
    system_delay_ms(50);
    gpio_set_level(BEEP_PIN, 1);
    system_delay_ms(50);
    gpio_set_level(BEEP_PIN, 0);
}
// µçÑ¹¼ì²â
void voltage_detect()
{
    float voltage;
    int adc_data= adc_convert(ADC1_CH0_P10);
    voltage =  (float)11 * 3.3 * adc_data / 4095 ;
    if (voltage>11.1) {
        while(1) {
            gpio_toggle_level(LED1);
            system_delay_ms(3000);
            printf("Battery voltage is normal: %.2f\r\n", voltage);
        }
    }
    else
    {
        while(1) {
            gpio_toggle_level(LED1);
            system_delay_ms(3000);
            printf("Battery voltage is too low: %.2f\r\n", voltage);
        }
    }
}