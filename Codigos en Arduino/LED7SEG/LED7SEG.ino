// --- Definiciones de hardware ---
#define BTN_BIT     2        // PD2: botón con pull-down externo
#define SEG_PORT    PORTC    // Segmentos a–f en PORTC
#define SEG_DDR     DDRC
#define SEG_MASK    0x3F     // PC0..PC5 (a..f). El segmento g va en PD7

#define SEG_G_BIT   7        // PD7: segmento g (pin digital 7)

// --- Retardo tosco aproximado ---
void delay_ms(unsigned int ms) {
    while (ms--) {
        // Bucle vacío; no es exacto pero sirve para rebote
        for (volatile unsigned int i = 0; i < 8000U; i++) {
            __asm__ __volatile__("nop");
        }
    }
}

// Tabla de segmentos para dígitos 0–9
// g f e d c b a  (1 = segmento encendido) - display cátodo común
const unsigned char digitLUT[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

void display_digit(unsigned char digit) {
    if (digit > 9) return;

    unsigned char value = digitLUT[digit];

    // Segmentos a–f en PC0..PC5
    SEG_PORT = (SEG_PORT & ~SEG_MASK) | (value & SEG_MASK);

    // Segmento g en PD7 (bit 6 de value)
    if (value & 0b01000000)
        PORTD |=  (1 << SEG_G_BIT);   // encender g
    else
        PORTD &= ~(1 << SEG_G_BIT);   // apagar g
}

void setup(void) {
    // --- Botón en PD2 como entrada, con pull-down externo ---
    DDRD  &= ~(1 << BTN_BIT);   // PD2 entrada
    PORTD &= ~(1 << BTN_BIT);   // sin pull-up (porque usas pull-down de 10k a GND)

    // --- Segmentos a–f (PC0..PC5) como salidas ---
    SEG_DDR  |= SEG_MASK;       // PC0..PC5 salidas
    SEG_PORT &= ~SEG_MASK;      // todos apagados

    // --- Segmento g (PD7) como salida ---
    DDRD  |=  (1 << SEG_G_BIT);
    PORTD &= ~(1 << SEG_G_BIT);

    // Mostrar 0 al inicio
    display_digit(0);
}

void loop(void) {
    static unsigned char count = 0;

    // Leer botón (pull-down: 0 = suelto, 1 = presionado)
    if (PIND & (1 << BTN_BIT)) {          // ¿está en 1?
        // Esperar por rebote de presión
        delay_ms(20);

        // Confirmar que sigue presionado
        if (PIND & (1 << BTN_BIT)) {
            // --- Pulsación válida: incrementar UNA sola vez ---
            count++;
            if (count > 9) count = 0;
            display_digit(count);

            // Esperar a que SUELTE el botón (mientras esté en 1, no hacemos nada)
            while (PIND & (1 << BTN_BIT)) {
                // bucle bloqueante hasta que deje de presionar
            }

            // Debounce al soltar
            delay_ms(20);
        }
    }

    // Pequeño respiro para no leer demasiado rápido (opcional)
    // delay_ms(1);
}
