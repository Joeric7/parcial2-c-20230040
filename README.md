# Monitor de señal analógica con alarma configurable

**Examen práctico de Microcontroladores: Code Challenge integrador (GPIO, TIMER, UART, ADC)**

| Campo | Valor |
|---|---|
| Nombre | Joeric Mena |
| Matrícula | 2023-0040 |
| Placa | NUCLEO-F303RE |
| Microcontrolador | STM32F303RE |
| IDE / biblioteca | STM32CubeIDE, STM32Cube HAL (STM32F3) |
| Terminal serial | Terminal con marcas de tiempo, 115200 8N1 |

---

## 1. Mapa de pines

| Función | Pin | Periférico / modo | Notas |
|---|---|---|---|
| Potenciómetro (cursor) | **PA0** | ADC1, canal 1 (`ADC_CHANNEL_1`), single-ended | Extremos a 3V3 y GND |
| Pulsador | **PC13** | GPIO entrada, EXTI13 (flanco de subida y bajada) | Botón B1 de la placa, con pull-up externa en la placa. **Activo en bajo** (presionado = 0) |
| LED de actividad | **PA5** | GPIO salida push-pull | LED LD2 de la placa. Activo en alto |
| LED de alarma | **PB3** | GPIO salida push-pull | LED externo con resistencia limitadora. Activo en alto |
| UART TX | **PA2** | USART2_TX | Hacia ST-LINK (puerto COM virtual) |
| UART RX | **PA3** | USART2_RX | Desde ST-LINK (puerto COM virtual) |

### Esquema de conexiones

```
        3V3 ───┬──────────────┐
               │              │
             ┌─┴─┐            │
   10 kΩ pot │   ├── cursor ──┼──► PA0 (ADC1_IN1)
             └─┬─┘            │
               │              │
        GND ───┴──────────────┘

   PB3 ──►|── LED ──[R]── GND         (LED de alarma, activo en alto)
   PA5 ──► LD2 (integrado)            (LED de actividad)
   PC13 ◄── B1 (integrado, pull-up en la placa, activo en bajo)
   PA2/PA3 ◄──► ST-LINK VCP ◄──► USB ◄──► PC (115200 8N1)
```

---

## 2. Configuración de periféricos

### 2.1 Reloj del sistema

- Fuente HSI (8 MHz) → PLL ×9, `PREDIV = 1` → **SYSCLK = HCLK = 72 MHz**.
- APB1 = 36 MHz; APB2 = 72 MHz.
- Reloj de periféricos: `TIM34 = HCLK` (72 MHz), `USART2 = PCLK1`, `ADC12 = PLL/1`.

### 2.2 TIMER: TIM4 (base de tiempo de 1 ms)

TIM4 es un temporizador de hardware de 16 bits. Genera una interrupción de actualización cada 1 ms que incrementa el contador global `g_ms`. A partir de este tick, el lazo principal programa todas las tareas.

**Cálculo:**

```
f_TIM4       = 72 MHz
Prescaler    = 71   → f_cnt = 72 MHz / (71+1) = 1 MHz  (1 µs por cuenta)
Period (ARR) = 999  → desborde cada (999+1) cuentas = 1000 µs = 1 ms

f_tick = 72 000 000 / ((71+1) × (999+1)) = 1000 Hz  →  T = 1 ms
```

**Tareas derivadas del tick:**

| Tarea | Período | Cuenta de ticks |
|---|---|---|
| Muestra ADC | 100 ms | 100 |
| Cambio del LED de actividad | 500 ms | 500 |
| Telemetría UART | 1000 ms | 1000 |
| Antirrebote del pulsador | muestreo cada 1 ms | confirmación a 30 ticks |

- Ruta de la ISR: `TIM4_IRQHandler` → `HAL_TIM_PeriodElapsedCallback()` → `g_ms++`.
- El lazo principal procesa un tick por iteración con `(g_ms - h_tick) >= 1`; si se atrasa, recupera los ticks perdidos sin perder la fase.
- No se usa `HAL_Delay()` ni el SysTick de HAL como base de tiempo.

### 2.3 ADC: ADC1

| Parámetro | Valor |
|---|---|
| Canal | ADC1_IN1 (PA0) |
| Resolución **N** | **12 bits** (valores 0…4095) |
| **VREF** | **3.3 V** (VDDA de la placa) |
| Rango de entrada | 0 V … 3.3 V |
| Modo | Conversión única por software, una por cada tarea de 100 ms |
| Tiempo de muestreo | 61.5 ciclos |
| Reloj ADC | Asíncrono (PLL/1) |
| Calibración | `HAL_ADCEx_Calibration_Start()` al arrancar (single-ended) |

**Conversión a porcentaje (enteros, sin desbordamiento):**

```
PCT = (100 × ADC + 4095/2) / 4095        // redondeo entero, uint32_t
```

`100 × 4095 = 409 500` no cabe en 16 bits, por eso el cálculo se hace en `uint32_t`. Ejemplo: ADC = 2866 → PCT = 70.

### 2.4 GPIO y antirrebote

- **Pulsador (PC13):** entrada con interrupción EXTI en ambos flancos. La ISR solo levanta una bandera (`g_flanco`).
- `boton_actualizar()` corre cada 1 ms y compara la lectura con el estado confirmado:
  - Un nuevo estado se acepta tras **30 ms consecutivos** de lectura estable.
  - Un flanco reinicia la cuenta de 30 ms, así los rebotes la anulan.
  - Solo la **pulsación confirmada** genera un evento; mantener presionado no repite, y la liberación también debe confirmarse antes de aceptar otra pulsación.
- Sin retardos bloqueantes.
- **LED de actividad (PA5):** `HAL_GPIO_TogglePin` cada 500 ms.
- **LED de alarma (PB3):** refleja la variable `alarma`.

### 2.5 UART: USART2

| Parámetro | Valor |
|---|---|
| Velocidad | 115200 baudios |
| Formato | 8 bits, sin paridad, 1 bit de parada (8N1) |
| Control de flujo | Ninguno |
| Recepción | Por interrupción, 1 byte a la vez → buffer circular de 128 bytes |
| Transmisión | Por interrupción → buffer circular de 512 bytes |

- La ISR de recepción solo guarda el byte en el buffer. **El ensamblado de líneas y la interpretación de comandos se hacen en el lazo principal.**
- `enviar()` copia el texto al buffer de salida y arranca la transmisión por interrupción, de modo que respuestas y telemetría se encolan completas, sin mezclarse en una misma línea.
- Conexión: USB del ST-LINK (niveles compatibles y GND común).

---

## 3. Protocolo UART

Líneas ASCII terminadas en **LF** o **CRLF** (el CR se ignora). Sensible a mayúsculas. Máximo 32 caracteres por línea (sin terminador). Cada respuesta termina en `\r\n`.

| Comando | Respuesta | Comportamiento |
|---|---|---|
| `STATUS` | Línea de estado (formato de telemetría) | Inmediata |
| `THR n` | `OK THR=n` | `n` entero decimal 10…95; en RUN reinicia la decisión de alarma |
| `START` | `OK RUN` | Pasa a RUN (si ya estaba en RUN conserva la alarma) |
| `STOP` | `OK PAUSE` | Pasa a PAUSE y apaga la alarma |
| Otro / inválido | `ERR` | No cambia parámetros ni detiene las tareas |

- Línea vacía: se ignora.
- Línea de más de 32 caracteres: se descarta completa hasta el LF, responde `ERR` y se puede procesar la siguiente.
- `THR` rechaza: sin argumento, texto adicional, decimales, letras, signos y valores fuera de 10…95.

**Formato de telemetría / STATUS:**

```
t=1200;ADC=2866;PCT=70;THR=70;MODE=RUN;ALARM=1
```

---

## 4. Lógica de la alarma

- **Estado inicial** tras reinicio: RUN, THR = 70 %, alarma apagada, `t = 0`.
- **RUN:** enciende con `PCT ≥ THR`; apaga con `PCT < THR − 5`; en la banda intermedia conserva el estado.
- **PAUSE:** alarma forzada a 0; ADC, LED de actividad, telemetría y comandos siguen activos.
- **PAUSE → RUN** o **cambio de THR en RUN:** `reiniciar_alarma()` decide sin histéresis con la última muestra (`PCT ≥ THR` → 1, si no 0). Desde la siguiente muestra se aplica la histéresis normal. Si no hay muestra válida, espera la primera.
- **THR en PAUSE:** se guarda el valor y la alarma sigue apagada.

---

## 5. Mapa del código (función → requisito)

Todo el código de aplicación está en `Core/Src/main.c`.

| Función | Responsabilidad |
|---|---|
| `MX_TIM4_Init()` | Configura TIM4: prescaler 71, período 999 |
| `HAL_TIM_PeriodElapsedCallback()` | ISR de 1 ms: incrementa `g_ms` |
| `main()` (bloque por ms) | Planificador: LED 500 ms, ADC 100 ms, telemetría 1000 ms, evento de botón |
| `MX_ADC1_Init()` | ADC1 canal 1, 12 bits, 61.5 ciclos |
| `leer_adc()` | Una conversión; actualiza `adc_valor`, `pct` y evalúa la alarma |
| `adc_a_pct()` | Conversión a % con redondeo en 32 bits |
| `evaluar_alarma()` | Histéresis (RUN) |
| `reiniciar_alarma()` | Decisión sin histéresis (reanudar / cambio de THR) |
| `poner_pausa()` / `poner_run()` | Cambios de modo |
| `HAL_GPIO_EXTI_Callback()` | ISR del botón: solo levanta bandera |
| `boton_actualizar()` | Antirrebote de 30 ms, una acción por pulsación |
| `HAL_UART_RxCpltCallback()` | ISR RX: byte → buffer circular |
| `rx_leer()` / `armar_linea()` | Ensamblado de líneas, LF/CRLF, límite de 32, descarte |
| `procesar_linea()` / `leer_numero()` | Interpretación y validación de comandos |
| `enviar()` / `tx_iniciar()` / `HAL_UART_TxCpltCallback()` | Transmisión por buffer circular sin bloqueo |
| `enviar_estado()` | Formato de telemetría y respuesta a `STATUS` |

---

## 6. Compilación y ejecución

1. Clonar el repositorio.
2. En STM32CubeIDE: *File → Open Projects from File System…* y seleccionar la carpeta del proyecto.
3. Compilar con *Project → Build Project* (configuración `Debug`).
4. Conectar la placa por USB y cargar con *Run → Run*.
5. Abrir la terminal serial en el puerto COM virtual del ST-LINK: **115200, 8N1, sin control de flujo**.
6. Tras el reinicio aparece telemetría cada segundo, por ejemplo:
   ```
   t=1000;ADC=12;PCT=0;THR=70;MODE=RUN;ALARM=0
   ```

---

## 7. Resultados de las pruebas P01 a P10

La verificación se realizó con la terminal serial con marcas de tiempo y los instrumentos de medición de tiempos. La evidencia de todas las pruebas está en el **video de demostración**.

### Resumen

| ID | Prueba | Resultado | Evidencia |
|---|---|---|---|
| P01 | Arranque y estado inicial | **PASS** | Video de demostración |
| P02 | Temporización (±5 %) | **PASS** | Video de demostración |
| P03 | Conversión ADC en 3 niveles | **PASS** | Video de demostración |
| P04 | Histéresis de la alarma | **PASS** | Video de demostración |
| P05 | Pulsador y antirrebote | **PASS** | Video de demostración |
| P06 | Modo PAUSE (STOP / START) | **PASS** | Video de demostración |
| P07 | Comando `THR` válido | **PASS** | Video de demostración |
| P08 | Comandos inválidos | **PASS** | Video de demostración |
| P09 | Fragmentación, terminadores y línea larga | **PASS** | Video de demostración |
| P10 | Estrés de 30 s y latencia | **PASS** | Video de demostración |

### Detalle

**P01: Arranque y estado inicial.**
*Esperado:* compila y arranca; `MODE=RUN`, `THR=70`, `ALARM=0`, PCT cercano a 0.
*Observado:* el `STATUS` enviado tras 200 ms respondió con el estado inicial esperado. **PASS.**

**P02: Temporización (±5 %).**
*Esperado:* adquisición cada 100 ms, cambio de LED cada 500 ms y telemetría cada 1000 ms, con tolerancia ±5 %.

| Evento | Nominal | Resultado |
|---|---|---|
| Adquisición ADC | 100 ms | Dentro de ±5 % |
| Cambio del LED PA5 | 500 ms | Dentro de ±5 % |
| Línea de telemetría | 1000 ms | Dentro de ±5 % |

*Observado:* las 10 líneas periódicas y los eventos medidos cumplen la tolerancia. **PASS.**

**P03: Conversión ADC en 3 niveles (±3 puntos porcentuales).**
*Esperado:* PCT = 100·Vin/VREF dentro de ±3 puntos; ADC en el rango de 12 bits; redondeo correcto.

| Punto | Resultado |
|---|---|
| ≈0 % de VREF | PCT dentro de ±3 puntos |
| ≈50 % de VREF | PCT dentro de ±3 puntos |
| ≈100 % de VREF | PCT dentro de ±3 puntos |

*Observado:* los tres niveles coinciden con la tensión medida y respetan la fórmula de redondeo. **PASS.**

**P04: Histéresis (THR = 70).**

| PCT | ALARM esperado | ALARM observado |
|---|---|---|
| 60 | 0 | 0 |
| 72 | 1 | 1 |
| 67 | 1 | 1 |
| 62 | 0 | 0 |

**PASS.**

**P05: Pulsador y antirrebote.**
*Esperado:* una alternancia por pulsación válida; ninguna repetición al mantener el botón; los rebotes no producen alternancias adicionales.
*Observado:* las 5 pulsaciones separadas produjeron una alternancia RUN/PAUSE cada una; la pulsación mantenida 2 s produjo una sola alternancia. **PASS.**

**P06: Modo PAUSE.**
*Esperado:* con PCT≈80, `STOP` apaga la alarma; en PAUSE continúan ADC, LED de actividad y telemetría; `START` restaura RUN y enciende la alarma.
*Observado:* comportamiento conforme en todos los pasos. **PASS.**

**P07: Comando `THR` válido.**

| Comando | Respuesta | Alarma |
|---|---|---|
| `THR 50` (PCT≈60) | `OK THR=50` | Enciende |
| `THR 80` | `OK THR=80` | Apaga |
| `THR 10` | `OK THR=10` | Límite inferior aceptado |
| `THR 95` | `OK THR=95` | Límite superior aceptado |

**PASS.**

**P08: Comandos inválidos.**

| Envío | Respuesta |
|---|---|
| `THR 9` | `ERR` |
| `THR 96` | `ERR` |
| `THR abc` | `ERR` |
| `THR 50x` | `ERR` |
| `THR 50 extra` | `ERR` |
| Comando desconocido | `ERR` |

*Observado:* un `ERR` por línea; THR y modo permanecen sin cambios; la aplicación sigue funcionando. **PASS.**

**P09: Fragmentación, terminadores y línea larga.**
*Observado:* `STATUS` enviado en dos fragmentos respondió correctamente; LF y CRLF funcionan igual; la línea de 33 caracteres generó `ERR`; el `STATUS` posterior respondió con normalidad y no se ejecutaron fragmentos descartados. **PASS.**

**P10: Estrés de 30 s.**
*Esperado:* sin bloqueo ni líneas corruptas; respuestas ≤100 ms; tiempos periódicos dentro de ±5 %; modo y alarma siguen las reglas.
*Observado:* con `STATUS` a 5 comandos/s, variación de la entrada y alternancia de modo, no hubo bloqueos ni líneas corruptas; las latencias y los tiempos periódicos cumplieron los límites. **PASS.**