# 🚀 Interrupt-Based I2C Communication between ESP8266 & STM32F030C8T6

A robust project demonstrating I2C communication (master-slave) between an **ESP8266** and **STM32F030C8T6**, with interrupt-based handling on the STM32 side. Includes support for ADC inputs, button controls, feedback lines, and addressable LEDs.

---

## 📌 Hardware Pin Configuration

### 🧠 STM32F030C8T6 (Slave)

| Function           | Pin(s)         |
|--------------------|----------------|
| **I2C SCL**         | PB6            |
| **I2C SDA**         | PB7            |
| **ADC Inputs**      | PA0 to PA7     |
| **SWD Debug**       | PA13 (SWDIO), PA14 (SWCLK) |
| **Buttons**         | PF7, PA15, PB3–PB5, PB8–PB9, PC13 |
| **Switches**        | PB0–PB2, PB10–PB14 |
| **Feedback (F.B.)** | PC14–PC15, PF0–PF1, PB15, PA8–PA10 |

### 🌐 ESP8266 (Master)

| Function              | Pin   |
|------------------------|-------|
| **I2C SCL**            | D1    |
| **I2C SDA**            | D2    |
| **ADC (MAINS voltage)**| A0    |
| **Auto/Manual F.B.**   | D7    |
| **WS2812 LEDs (8x)**   | D6    |

---

## 🔧 Software Architecture

### 📲 I2C Communication
- **Master**: ESP8266 using `Wire` library.
- **Slave**: STM32F0 using **interrupt-driven I2C HAL**.
- Data format: Structured packets (e.g., ADC values, button states, etc.)

### 📈 ADC Handling
- STM32 reads 8 analog inputs (PA0–PA7) continuously.
- Values are stored in buffer and available on I2C request.

### 🧠 Button Handling
- All buttons trigger **external interrupts**.
- Button press state is tracked and updated.

### ✨ WS2812 LED Control
- Driven by ESP8266 using `Adafruit_NeoPixel` or `FastLED`.
- Modes: Auto/Manual, controlled via feedback from STM32.

---

## 📡 Communication Protocol

```text
I2C Address: 0x42 (example)

Request from ESP8266:
  - 0x01 → Get ADC values
  - 0x02 → Get Button states
  - 0x03 → Get Feedback inputs

Response from STM32:
  - Payload format [Byte1, Byte2, ..., ByteN]
