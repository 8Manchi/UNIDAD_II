// -------------------------
// Definiciones de pines LCD
// -------------------------
#define LCD_DATA_PORT   PORTD
#define LCD_DATA_DDR    DDRD

#define LCD_RS_BIT      0     // PB0 -> RS
#define LCD_E_BIT       1     // PB1 -> E

// -------------------------
// Retardo simple tipo PIC
// -------------------------
void delay_ms(unsigned int ms) {
    while (ms--) {
        for (volatile unsigned long i = 0; i < 1000UL; i++) {
            __asm__ __volatile__("nop");
        }
    }
}

// -------------------------
// LCD – Rutinas básicas
// -------------------------
void lcd_pulse_enable(void) {
    PORTB |=  (1 << LCD_E_BIT);
    delay_ms(1);
    PORTB &= ~(1 << LCD_E_BIT);
    delay_ms(1);
}

void lcd_send(unsigned char value, unsigned char isData) {
    if (isData)
        PORTB |=  (1 << LCD_RS_BIT);   // RS = 1 -> dato
    else
        PORTB &= ~(1 << LCD_RS_BIT);   // RS = 0 -> comando

    LCD_DATA_PORT = value;             // Todo el puerto D son datos
    lcd_pulse_enable();
}

void lcd_command(unsigned char cmd) {
    lcd_send(cmd, 0);
}

void lcd_data(unsigned char data) {
    lcd_send(data, 1);
}

void lcd_init(void) {
    delay_ms(50);          // Esperar a que el LCD arranque

    lcd_command(0x38);     // Function set: 8-bit, 2 líneas, 5x8
    lcd_command(0x0C);     // Display ON, cursor OFF, blink OFF
    lcd_command(0x06);     // Entry mode: cursor incrementa, sin shift
    lcd_command(0x01);     // Clear display
    delay_ms(2);
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
    unsigned char addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_command(0x80 | addr);
}

void lcd_print_str(const char *s) {
    while (*s) {
        lcd_data(*s++);
    }
}

void lcd_print_uint(unsigned int value) {
    char buf[6];  // máximo 65535
    int i = 0;

    if (value == 0) {
        lcd_data('0');
        return;
    }

    while (value > 0 && i < 5) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }

    while (i > 0) {
        lcd_data(buf[--i]);
    }
}

// -------------------------
// ADC en canal ADC0 (PC0 / A0)
// Referencia = AVCC (≈ 5V)
// -------------------------
void adc_init(void) {
    // PC0 como entrada (potenciómetro)
    DDRC &= ~(1 << PC0);

    // ADMUX:
    // REFS1:0 = 01 -> referencia AVCC (5V) con capacitor en AREF
    // MUX[3:0] = 0000 -> canal ADC0
    ADMUX = (1 << REFS0);

    // ADCSRA:
    // ADEN = 1  -> habilitar ADC
    // ADPS2:0 = 111 -> prescaler 128 (16 MHz / 128 = 125 kHz)
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Lectura bloqueante del ADC en ADC0
unsigned int adc_read(void) {
    unsigned char low, high;

    ADCSRA |= (1 << ADSC);            // Iniciar conversión

    while (ADCSRA & (1 << ADSC)) {
        // Esperar a que ADSC se ponga en 0 (fin de conversión)
    }

    // Leer ADCL primero, luego ADCH
    low  = ADCL;
    high = ADCH;

    return ((unsigned int)high << 8) | low;   // 0..1023
}

// -------------------------
// setup() y loop()
// -------------------------
void setup() {
    // LCD: PORTD como salida (D0..D7)
    LCD_DATA_DDR = 0xFF;

    // RS y E como salidas en PORTB
    DDRB |= (1 << LCD_RS_BIT) | (1 << LCD_E_BIT);

    // Inicializar LCD y ADC
    lcd_init();
    adc_init();

    // Mensaje inicial (sin "LCDx16")
    lcd_set_cursor(0, 0);
    lcd_print_str("ADC0 REF=5V");
    lcd_set_cursor(1, 0);
    lcd_print_str("Listo...");
    delay_ms(1000);
}

void loop() {
    unsigned int adcValue;
    unsigned long mV;

    // Leer ADC0
    adcValue = adc_read();

    // Convertir a milivolts (Vref = 5000 mV)
    mV = ((unsigned long)adcValue * 5000UL) / 1023UL;

    // Línea 1: valor RAW del ADC
    lcd_set_cursor(0, 0);
    lcd_print_str("ADC: ");
    lcd_print_uint(adcValue);
    lcd_print_str("     ");   // borrar restos

    // Línea 2: valor en mV
    lcd_set_cursor(1, 0);
    lcd_print_str("mV: ");
    lcd_print_uint((unsigned int)mV);
    lcd_print_str("     ");

    delay_ms(200);
}
