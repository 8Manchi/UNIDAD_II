
/* Registros usados (direcciones en memoria) */
#define DDRB   (*(volatile unsigned char *)0x24)

#define ADCL   (*(volatile unsigned char *)0x78)
#define ADCH   (*(volatile unsigned char *)0x79)
#define ADCSRA (*(volatile unsigned char *)0x7A)
#define ADMUX  (*(volatile unsigned char *)0x7C)

#define TCCR1A (*(volatile unsigned char *)0x80)
#define TCCR1B (*(volatile unsigned char *)0x81)
#define OCR1AL (*(volatile unsigned char *)0x88)

/* ADC: referencia AVcc, canal ADC0 */
void ADC_init(void) {
    /* REFS0 = bit 6 → referencia AVcc, MUX = 0 (ADC0) */
    ADMUX = (1 << 6);

    /* ADEN = bit 7 (habilita ADC)
     * ADPS2:0 = 111 (bits 2,1,0) → prescaler 128 (16MHz/128=125kHz)
     */
    ADCSRA = (1 << 7) | (1 << 2) | (1 << 1) | (1 << 0);
}

/* Lectura de 10 bits en ADC0 */
unsigned int ADC_read(void) {
    unsigned int value;

    /* ADSC = bit 6 → iniciar conversión */
    ADCSRA |= (1 << 6);

    /* Esperar a que ADSC vuelva a 0 */
    while (ADCSRA & (1 << 6)) {
        /* espera activa */
    }

    /* Leer primero ADCL y luego ADCH */
    value  = ADCL;
    value |= ((unsigned int)ADCH << 8);

    return value;    /* 0–1023 */
}

/* PWM en OC1A (PB1) – Fast PWM 8 bits, ~976 Hz */
void PWM1_init(void) {
    /* PB1 = bit 1 de DDRB → salida */
    DDRB |= (1 << 1);

    /* TCCR1A:
     * WGM10 = bit 0 → Fast PWM 8 bits (junto con WGM12)
     * COM1A1 = bit 7 → salida OC1A no invertida
     */
    TCCR1A = (1 << 0) | (1 << 7);

    /* TCCR1B:
     * WGM12 = bit 3 → completa Fast PWM 8 bits
     * CS11:CS10 = 11 (bits 1 y 0) → prescaler 64
     * fPWM ≈ 16MHz / (64 * 256) ≈ 976 Hz
     */
    TCCR1B = (1 << 3) | (1 << 1) | (1 << 0);
}

int main(void) {
    unsigned int adc_value;
    unsigned char duty;

    ADC_init();
    PWM1_init();

    while (1) {
        /* Leer potenciómetro en A0 (ADC0) → 0–1023 */
        adc_value = ADC_read();

        /* Pasar de 10 bits (0–1023) a 8 bits (0–255) → desplazar 2 */
        duty = (unsigned char)(adc_value >> 2);

        /* Duty cycle del PWM en OC1A */
        OCR1AL = duty;   /* solo parte baja en modo 8 bits */
    }

    return 0;
}
