#define LED_BIT 5   // PB5 (pin 13)
#define BTN_BIT 2   // PD2

// Retardo simple tipo PIC
void delay_ms(unsigned int ms) {
    while (ms--) {
        for (volatile unsigned long i = 0; i < 1000UL; i++) {
            __asm__ __volatile__("nop");
        }
    }
}

// --- Lectura estable estilo PIC ---
// Retorna: 0, 1, o 0xFF si es inestable
unsigned char get_input_debounced(void) {
    unsigned char a = (PIND & (1 << BTN_BIT)) ? 1 : 0;
    delay_ms(20);
    unsigned char b = (PIND & (1 << BTN_BIT)) ? 1 : 0;

    if (a == b) return b;
    else return 0xFF;
}

void setup() {
    // LED PB5 salida
    DDRB |= (1 << LED_BIT);

    // LED inicialmente apagado (activo-HIGH)
    PORTB &= ~(1 << LED_BIT);

    // Botón en PD2 como entrada con pull-down externo
    DDRD &= ~(1 << BTN_BIT);
    PORTD &= ~(1 << BTN_BIT);   // sin pull-up

    // -------------------------
    // Agregado: Monitor Serial
    // -------------------------
    Serial.begin(9600);
    Serial.println("Sistema iniciado");
}

void loop() {
    static unsigned char lastStable = 0;   // último valor estable
    static unsigned char lastProcessed = 0; // para detectar flanco

    unsigned char stable = get_input_debounced();

    if (stable != 0xFF) {
        // Solo usaremos valores estables
        
        // Detectar flanco de subida: 0 -> 1
        if (stable == 1 && lastProcessed == 0) {

            // --- TOGGLE ---
            // LED activo-HIGH: PB5=1 → enciende, PB5=0 → apaga
            PORTB ^= (1 << LED_BIT);

            // -------------------------------------------
            // Agregado: imprimir estado del LED en Serial
            // -------------------------------------------
            if (PINB & (1 << LED_BIT))
                Serial.println("LED encendido");
            else
                Serial.println("LED apagado");

            // Actualizamos
            lastProcessed = 1;
        }
        else if (stable == 0) {
            // Botón suelto, reseteamos el detector de flancos
            lastProcessed = 0;
        }

        lastStable = stable;
    }
}

