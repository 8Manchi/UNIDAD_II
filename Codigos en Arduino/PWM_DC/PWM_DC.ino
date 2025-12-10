// ---------------------------------------------------------
// Control de posición de motor DC con encoder incremental
// ATmega328P (Arduino Uno) - Nivel de registros
// ---------------------------------------------------------

#include <avr/io.h>         // Definiciones de registros del ATmega328P
#include <avr/interrupt.h>  // Manejo de interrupciones (ISR, sei, cli)
#include <stdlib.h>         // Funciones de conversión (atol)

// -------------------- Variables globales -----------------

volatile long encoderCount = 0;   // Contador de cuentas del encoder (ticks)
volatile int8_t encoderDir = 0;   // Último sentido de giro (+1 o -1)

// Parámetros de control de posición
long setpoint = 0;                 // Posición objetivo en cuentas
const long POSITION_TOLERANCE = 5; // Margen de error aceptable (±5 cuentas)
const uint8_t Kp = 3;              // Ganancia proporcional
const uint8_t PWM_MAX = 255;       // Límite superior de PWM (8 bits)

// Prototipos de funciones
void motorStop(void);
void motorForward(void);
void motorBackward(void);
void setPWM(uint8_t duty);
long getEncoderCount(void);
void readSetpointFromSerial(void);

// ---------------------------------------------------------
// setup()
//  Configuración inicial del microcontrolador:
//  - GPIO para encoder y driver de motor
//  - Interrupción externa INT0 para el encoder
//  - Timer1 en Fast PWM 8 bits (salida OC1A)
//  - UART para comunicación por monitor serie
// ---------------------------------------------------------

void setup(void)
{
  cli(); // Deshabilita interrupciones globales mientras se configura todo

  // --------------- Configuración de GPIO -----------------
  // Encoder:
  //   Canal A (C1) -> PD2 (INT0, pin digital 2)
  //   Canal B (C2) -> PD3 (pin digital 3)
  //
  // DDRD: 0 = entrada, 1 = salida
  DDRD &= ~((1 << DDD2) | (1 << DDD3));       // PD2 y PD3 como entradas
  // PORTD: en entradas, bit = 1 activa pull-up interno
  PORTD |= (1 << PORTD2) | (1 << PORTD3);     // Habilitar pull-up en PD2 y PD3

  // Driver de motor (puente H):
  //   D7 (PD7) -> IN1
  //   D8 (PB0) -> IN2
  //   D9 (PB1/OC1A) -> ENA (PWM)
  DDRD |= (1 << DDD7);                        // PD7 como salida (IN1)
  DDRB |= (1 << DDB0) | (1 << DDB1);          // PB0 (IN2) y PB1 (OC1A) como salidas

  // Apagar motor al inicio (condición segura)
  motorStop();
  setPWM(0);

  // -------- Configuración de interrupción externa INT0 ----
  // INT0 se usa para leer el canal A del encoder (PD2).
  //
  // EICRA - External Interrupt Control Register A:
  //   ISC01 ISC00 = 11 -> disparo por flanco de subida en INT0
  EICRA |= (1 << ISC01) | (1 << ISC00);

  // EIMSK - External Interrupt Mask Register:
  //   INT0 = 1 -> habilita la interrupción externa INT0
  EIMSK |= (1 << INT0);

  // EIFR - External Interrupt Flag Register:
  //   INTF0 = 1 -> limpia una posible bandera pendiente de INT0
  EIFR  |= (1 << INTF0);

  // -------- Configuración del Timer1 para PWM en OC1A -----
  //
  // Se configura Fast PWM de 8 bits con salida no inversora en OC1A (PB1).
  // Frecuencia determinada por F_CPU y prescaler; resolución de 0 a 255.
  //
  TCCR1A = 0;
  TCCR1B = 0;

  // Modo Fast PWM 8 bits:
  //   WGM13:0 = 0b0101
  //   -> WGM10 = 1 (TCCR1A), WGM12 = 1 (TCCR1B)
  TCCR1A |= (1 << WGM10);
  TCCR1B |= (1 << WGM12);

  // Modo no inversor en OC1A:
  //   COM1A1:0 = 10 -> salida activa en alto proporcional al duty
  TCCR1A |= (1 << COM1A1);

  // Prescaler del Timer1:
  //   CS12 CS11 CS10
  //   0     1    0  -> prescaler = 8
  TCCR1B |= (1 << CS11);

  // Duty cycle inicial en 0 (motor apagado)
  OCR1A = 0;

  // -------- UART (Serial) para setpoint y monitoreo --------
  Serial.begin(9600);
  Serial.println("Inicio - Control de posicion con encoder");
  Serial.println("Escribe un numero de cuentas y presiona Enter.");

  sei(); // Habilita interrupciones globales
}

// ---------------------------------------------------------
// loop()
//  Bucle principal de control:
//  - Lee el setpoint desde el monitor serie
//  - Obtiene la posicion actual (encoderCount)
//  - Calcula el error y decide sentido de giro
//  - Calcula PWM proporcional al error (con saturacion)
//  - Detiene el motor cuando el error es pequeño
// ---------------------------------------------------------

void loop(void)
{
  // Leer (si existe) un nuevo setpoint desde el monitor serie
  readSetpointFromSerial();

  // Obtener posicion actual de manera atomica
  long posicion = getEncoderCount();
  long error = setpoint - posicion;
  long absError = (error >= 0) ? error : -error;

  if (absError <= POSITION_TOLERANCE)
  {
    // Ya se encuentra dentro del margen de la posicion objetivo
    motorStop();
    setPWM(0);
  }
  else
  {
    // Seleccionar sentido de giro segun el signo del error
    if (error > 0)
    {
      motorForward();
    }
    else
    {
      motorBackward();
    }

    // Control proporcional simple:
    //   PWM = Kp * |error|, limitado a PWM_MAX
    uint32_t pwm = (uint32_t)absError * Kp;
    if (pwm > PWM_MAX)
    {
      pwm = PWM_MAX;
    }
    setPWM((uint8_t)pwm);
  }

  // Enviar informacion al monitor serie (para observar el comportamiento)
  Serial.print("Posicion: ");
  Serial.print(posicion);
  Serial.print("  Setpoint: ");
  Serial.println(setpoint);
}

// ---------------------------------------------------------
// Rutina de servicio de interrupcion del encoder
//  INT0_vect se ejecuta cada vez que ocurre un flanco de
//  subida en el canal A del encoder (PD2).
//  El estado del canal B (PD3) indica el sentido del giro.
// ---------------------------------------------------------

ISR(INT0_vect)
{
  // Leer canal B (C2) desde el registro PIND
  if (PIND & (1 << PIND3))
  {
    // Canal B en alto -> giro "positivo"
    encoderCount++;
    encoderDir = 1;
  }
  else
  {
    // Canal B en bajo -> giro "negativo"
    encoderCount--;
    encoderDir = -1;
  }
}

// ---------------------------------------------------------
// Funciones auxiliares para control del motor y PWM
// ---------------------------------------------------------

// Detiene el motor: ambas entradas del driver en 0
void motorStop(void)
{
  PORTD &= ~(1 << PORTD7); // IN1 = 0 (PD7)
  PORTB &= ~(1 << PORTB0); // IN2 = 0 (PB0)
}

// Gira el motor en sentido "adelante"
void motorForward(void)
{
  PORTD |=  (1 << PORTD7); // IN1 = 1
  PORTB &= ~(1 << PORTB0); // IN2 = 0
}

// Gira el motor en sentido "atras"
void motorBackward(void)
{
  PORTD &= ~(1 << PORTD7); // IN1 = 0
  PORTB |=  (1 << PORTB0); // IN2 = 1
}

// Ajusta el duty cycle del PWM (0-255)
void setPWM(uint8_t duty)
{
  OCR1A = duty;
}

// Copia atomica de la variable de posicion (encoderCount)
// al ser de 32 bits, se protege el acceso deshabilitando
// brevemente las interrupciones.
long getEncoderCount(void)
{
  long value;
  uint8_t sreg = SREG; // Guardar estado de interrupciones
  cli();
  value = encoderCount;
  SREG = sreg;         // Restaurar SREG (incluye I-bit)
  return value;
}

// ---------------------------------------------------------
// Lectura del setpoint desde el monitor serie
//  El usuario escribe un numero y presiona Enter.
//  La cadena se convierte a long y se almacena en setpoint.
// ---------------------------------------------------------

void readSetpointFromSerial(void)
{
  static char buffer[16];    // Buffer para texto recibido
  static uint8_t index = 0;  // Indice de escritura en el buffer

  while (Serial.available() > 0)
  {
    char c = Serial.read();

    // Fin de linea -> procesar numero completo
    if (c == '\n' || c == '\r')
    {
      if (index > 0)
      {
        buffer[index] = '\0';     // Terminar cadena
        long value = atol(buffer); // Convertir a long
        setpoint = value;          // Actualizar setpoint
        index = 0;                 // Reiniciar buffer

        Serial.print("Nuevo setpoint: ");
        Serial.println(setpoint);
      }
    }
    else
    {
      // Acumular caracteres mientras haya espacio en el buffer
      if (index < sizeof(buffer) - 1)
      {
        buffer[index++] = c;
      }
    }
  }
}
