# Control de posición de motor DC con encoder incremental  
### ATmega328P (Arduino Uno) · PWM · Interrupciones a nivel de registros

Este proyecto implementa un sistema de **control de posición en lazo cerrado** utilizando un motor DC con encoder incremental, un driver L293D y el microcontrolador **ATmega328P**.  
Toda la configuración de **GPIO, interrupciones y PWM** se realizó **a nivel de registros**, sin emplear funciones de alto nivel como `pinMode`, `digitalWrite`, `analogWrite` o `attachInterrupt`.

---

## 📌 Objetivo del proyecto

- Leer un encoder incremental (canales A y B).
- Contar pulsos usando interrupciones externas (INT0).
- Determinar sentido de giro desde la señal en cuadratura.
- Generar PWM por hardware mediante el **Timer1 (OC1A)**.
- Controlar un motor DC a través de un **driver L293D**.
- Implementar un controlador **proporcional** para alcanzar un setpoint.
- Recibir el setpoint desde el **monitor serial** del Arduino.
- Mostrar en tiempo real: posición actual, setpoint y correcciones.


---

## 🛠️ Hardware utilizado

- Arduino Uno (ATmega328P)
- Motor DC con encoder incremental integrado
- Driver L293D o equivalente
- Fuente de 9–12 V para el motor
- Cableado Dupont y protoboard

---

## 🔌 Diagrama de conexiones (texto)

### Encoder → Arduino Uno

| Encoder | Pin Arduino | Pin ATmega328P | Función |
|--------|-------------|----------------|---------|
| Canal A | D2 | PD2 / INT0 | Interrupción externa (conteo) |
| Canal B | D3 | PD3 | Sentido de giro |
| Vcc | 5V | — | Alimentación |
| GND | GND | — | Tierra |

### Driver L293D → Arduino Uno

| Driver | Pin Arduino | Pin ATmega328P | Función |
|--------|-------------|----------------|---------|
| IN1 | D7 | PD7 | Dirección |
| IN2 | D8 | PB0 | Dirección |
| ENA (PWM) | D9 | PB1 (OC1A) | PWM para potencia |
| VCC1 | 5V | — | Lógica del driver |
| VCC2 | 12V | — | Motor |
| GND | GND común | — | Referencia |

### Motor DC

- Motor → Pines 1Y y 2Y del L293D  
- Encoder → D2, D3, +5V, GND  

> **IMPORTANTE:** Todas las tierras deben estar unidas.

---

## 🧠 Resumen del funcionamiento del software

### Lectura del encoder
- INT0 detecta flancos del canal A.
- La ISR compara el estado del canal B para determinar el sentido.
- Se incrementa o decrementa `encoderCount`.

### Generación de PWM
- Timer1 configurado en **Fast PWM 8 bits**.
- Salida PWM por **OC1A (D9)**.
- Duty cycle controlado mediante `OCR1A`.

### Control de posición
1. El usuario escribe un setpoint en el monitor serial.
2. El microcontrolador calcula:  

error = setpoint – posicion_actual

3. Si el error es positivo → motorForward()  
Si es negativo → motorBackward()
4. PWM proporcional:  

5. Si `|error| ≤ 5`, el motor se detiene.

---

## ▶️ Cómo usar el proyecto

1. Abre el archivo `.ino` en **Arduino IDE**.
2. Selecciona **Arduino Uno** como placa.
3. Carga el programa.
4. Abre el **monitor serial a 9600 baudios**.
5. Escribe un número (ejemplo: `5000`) y presiona **Enter**.
6. El motor girará hasta alcanzar la posición indicada.

---

## 📊 Resultados esperados

- El contador del encoder sube o baja según el sentido del motor.
- No se pierden pulsos gracias a interrupciones.
- El motor llega al setpoint con buena precisión.
- El PWM proporcional produce una aproximación suave.
- Ejemplo típico del monitor serial:

Posicion: 4964 Setpoint: 5000
Posicion: 4964 Setpoint: 5000
...



---

## 🚀 Mejoras posibles

- Implementar un **control PID**.
- Usar un driver MOSFET más eficiente.
- Filtrar digitalmente las señales del encoder.
- Crear interfaz gráfica para monitoreo.
- Añadir entradas físicas para el setpoint.

---

## 🧾 Autor

**Diego Rodolfo De Jesús Flores Martínez**  
Universidad Politécnica de Victoria  
Materia: *Programación de Sistemas Embebidos*

---

## 📄 Licencia

MIT
