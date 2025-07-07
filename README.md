<details>
<summary> <h2> 00_Hello_World - LED Blink Project </h2></summary>
  
This project demonstrates how to toggle a green LED connected to a GPIO pin using STM32 HAL functions with the STM32CubeIDE development environment. The LED blinks with a 500 ms delay interval.

## 🛠️ Hardware

- **Development Board:** STM32 Nucleo-F446RE
- **Microcontroller:** STM32F446RE (ARM Cortex-M4)
- **LED Pin:** Custom GPIO (e.g., `green_led_Pin` on `GPIOA`)
- **Power Supply:** USB connection

## 💻 Development Environment

- **IDE:** STM32CubeIDE
- **Toolchain:** STM32 HAL (Hardware Abstraction Layer)
- **Language:** C

## ⚙️ Functionality

The LED is toggled on and off every 500 milliseconds in the main loop.

```c
HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_SET);
HAL_Delay(500);
HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
HAL_Delay(500);

```
## 🔌 GPIO Configuration

![image](https://github.com/user-attachments/assets/2d9db08e-9321-4f3e-aee0-1a1410f5cf48)

The pin is configured as:
```c
GPIO_InitStruct.Pin = green_led_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
HAL_GPIO_Init(green_led_GPIO_Port, &GPIO_InitStruct);
```

</details>

<details>
  <summary><h2> 01_Knight_Rider - LED Animation Project</h2></summary>

  This project demonstrates how to control multiple LEDs connected to GPIO pins on an STM32 Nucleo-F446RE board using STM32 HAL. The LEDs blink in a forward and backward sequence, creating a simple "running light" or "bouncing" LED animation.

## 🛠️ Hardware

- **Development Board:** STM32 Nucleo-F446RE  
- **LEDs:** 4 LEDs connected to GPIOA (Red, Green, Yellow, Blue)  
- **Power Supply:** USB connection

## 💻 Development Environment

- **IDE:** STM32CubeIDE  
- **Toolchain:** STM32 HAL  
- **Language:** C

## ⚙️ Functionality

This code toggles 4 LEDs in the following sequence:
- Forward: Yellow → Blue → Green → Red  
- Backward: Green → Blue  

Each LED lights up for 200 milliseconds before turning off and moving to the next in the sequence. This creates a simple back-and-forth animation pattern.

## 🔁 LED Sequence Logic

```c
for (i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(LED_PORT, leds[i], 1);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LED_PORT, leds[i], 0);
}

for (i = 2; i > 0; i--) {
    HAL_GPIO_WritePin(LED_PORT, leds[i], 1);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LED_PORT, leds[i], 0);
}
```

## 🔌 GPIO Configuration

![image](https://github.com/user-attachments/assets/eb5e7de0-5b05-4353-84e9-e546778ca476)

| LED Color | Pin Macro        | Port  |
| --------- | ---------------- | ----- |
| Red       | `red_led_Pin`    | GPIOA |
| Green     | `green_led_Pin`  | GPIOA |
| Yellow    | `yellow_led_Pin` | GPIOA |
| Blue      | `blue_led_Pin`   | GPIOA |

These pins are initialized as push-pull output without pull-up or pull-down resistors.

```c
GPIO_InitStruct.Pin = red_led_Pin | green_led_Pin | yellow_led_Pin | blue_led_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

</details>

<details>
  <summary><h2> 02_Button - Button-Controlled LED Counter </h2></summary>

  This STM32CubeIDE project demonstrates how to use a push button to increment a counter and control multiple LEDs connected to GPIO pins on the STM32 Nucleo-F446RE board. The number of active LEDs increases with each button press, cycling through four stages.

## 🛠️ Hardware

- **Board:** STM32 Nucleo-F446RE  
- **Microcontroller:** STM32F446RE (ARM Cortex-M4)  
- **Input:** 1 Push Button (Pull-up enabled)  
- **Output:** 4 LEDs (connected to GPIOA pins PA6, PA7, PA9, PA8)  
- **Status LED:** `green_led_Pin` for button press indication

## 💻 Development Environment

- **IDE:** STM32CubeIDE  
- **Toolchain:** STM32 HAL (Hardware Abstraction Layer)  
- **Language:** C

## ⚙️ Functionality

- The program monitors a button connected to a GPIO pin (`button_Pin`) with pull-up enabled.
- When the button is pressed (logic low), a counter (`buttonCounter`) is incremented.
- The green LED is turned ON while the button is pressed.
- Based on the modulo of the counter (`buttonCounter % 4`), a set number of LEDs are turned ON:
  - 0 → 1 LED ON (PA6)
  - 1 → 2 LEDs ON (PA6, PA7)
  - 2 → 3 LEDs ON (PA6, PA7, PA9)
  - 3 → 4 LEDs ON (PA6, PA7, PA9, PA8)
- After 4 presses, the counter resets to the beginning of the cycle.

### 🔁 LED Sequence Logic (simplified)

```c
if (buttonCounter % 4 == 0) {
  LED1 = ON
} else if (buttonCounter % 4 == 1) {
  LED1, LED2 = ON
} else if (buttonCounter % 4 == 2) {
  LED1, LED2, LED3 = ON
} else if (buttonCounter % 4 == 3) {
  LED1, LED2, LED3, LED4 = ON
}
```

## 🔌 Pin Configuration

![image](https://github.com/user-attachments/assets/2b4b777b-ea3d-4060-8b54-da1cbcc4d326)

| Purpose      | GPIO Pin                    |
| ------------ | --------------------------- |
| Button Input | `button_Pin` (with pull-up) |
| Status LED   | `green_led_Pin` (GPIOA)     |
| Output LEDs  | `PA6`, `PA7`, `PA9`, `PA8`  |

These pins are configured as follows:

Input (button): `GPIO_MODE_INPUT`, `GPIO_PULLUP`

Output (LEDs): `GPIO_MODE_OUTPUT_PP`, `GPIO_NOPULL`, `MEDIUM` speed

</details>

<details>
  <summary> <h2> 03_External_Interrupt - External Interrupt LED Blink</h2></summary>

  This STM32CubeIDE project demonstrates the use of external interrupts (EXTI) on the STM32 Nucleo-F446RE board. A user button triggers an interrupt that causes a faster LED blinking sequence in addition to the default blinking pattern.

## 🛠️ Hardware

- **Board:** STM32 Nucleo-F446RE  
- **Microcontroller:** STM32F446RE (ARM Cortex-M4)  
- **Input:** Push Button connected to `PC13` (default user button)  
- **Output:** LED connected to custom GPIO pin (`led_Pin`)  

## 💻 Development Environment

- **IDE:** STM32CubeIDE  
- **Toolchain:** STM32 HAL  
- **Language:** C

## ⚙️ Functionality

- The LED toggles every 500 ms by default.
- When the user presses the button (attached to `PC13`), an **external interrupt (EXTI)** is triggered.
- The interrupt sets a flag `interruptFlag`.
- In the next loop cycle, the LED blinks **rapidly 5 times** (100 ms interval) as a response to the interrupt.

### 🔁 LED Blink Logic

```c
// Default blink every 500 ms
HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
HAL_Delay(500);

// On interrupt
if (interruptFlag) {
    interruptFlag = 0;
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
        HAL_Delay(100);
    }
}
```

## ⏱️ External Interrupt Setup
Button Pin: `buton_Pin` mapped to `PC13`

Trigger: Rising edge (`GPIO_MODE_IT_RISING`)

Pull Configuration: Pull-up enabled (`GPIO_PULLUP`)

Interrupt Handler: `HAL_GPIO_EXTI_Callback()`

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        interruptFlag = 1;
    }
}
```

## 🔌 Pin Configuration

![image](https://github.com/user-attachments/assets/df0388df-9cbb-4182-8433-941425fa8412)

| Purpose      | Pin       | Mode                   |
| ------------ | --------- | ---------------------- |
| LED Output   | `led_Pin` | GPIO\_MODE\_OUTPUT\_PP |
| Button Input | `PC13`    | GPIO\_MODE\_IT\_RISING |


</details>

<details>
  <summary><h2> 04_UART - UART Communication (Transmit & Receive) </h2></summary>

  This STM32CubeIDE project demonstrates basic UART communication using the STM32 Nucleo-F446RE board. The system waits for a 5-byte message over UART and responds with a predefined message once received.

## 🔧 Hardware

- **Board:** STM32 Nucleo-F446RE  
- **Communication Interface:** UART2 (TX/RX)  
- **Optional Output:** LED (status or activity indicator)

## 🛠️ Development Environment

- **IDE:** STM32CubeIDE  
- **Toolchain:** STM32 HAL  
- **Language:** C

## ⚙️ Functionality

- Initializes UART2 at 115200 baud rate.
- Waits for a 5-byte message via UART (blocking mode).
- Once the message is received, it transmits `"Hello!\r\n"` back over UART.

### 💬 UART Operation

```c
uint8_t messageTX[] = "Hello! \r\n";
uint8_t messageRX[5];

if (HAL_UART_Receive(&huart2, messageRX, sizeof(messageRX), HAL_MAX_DELAY) == HAL_OK) {
    HAL_UART_Transmit(&huart2, messageTX, sizeof(messageTX), HAL_MAX_DELAY);
}
```

## 🧠 UART2 Configuration

| Parameter    | Value  |
| ------------ | ------ |
| Baud Rate    | 115200 |
| Word Length  | 8 bits |
| Stop Bits    | 1      |
| Parity       | None   |
| Flow Control | None   |
| Mode         | TX/RX  |

## 🔌 Pin Configuration

![image](https://github.com/user-attachments/assets/9fe9c09f-c909-4cab-8330-9950882bd100)

| UART Function | STM32 Pin | Nucleo Pin Header |
| ------------- | --------- | ----------------- |
| TX (USART2)   | PA2       | D1                |
| RX (USART2)   | PA3       | D0                |


## 🧪 How to Test
Open a terminal tool (like PuTTY, TeraTerm, or STM32CubeMonitor).

Connect to the Nucleo board's virtual COM port at `115200 8N1`.

Send any 5-byte string (e.g., `12345`).

You will receive `Hello!` as a response over UART.

</details>

<details>
  <summary><h2> 05_UART_Interrupt - UART Command-Controlled LED </h2></summary>

  This STM32 project demonstrates UART-based LED control on the STM32 Nucleo-F446RE board. It allows you to send textual commands (`on`, `off`, or `blink <time>`) to control an onboard LED through a serial terminal.

## 🔧 Hardware

- **Board:** STM32 Nucleo-F446RE   
- **Peripheral Used:** UART2 (TX/RX), GPIO (LED Control)  

## 🛠️ Development Environment

- **IDE:** STM32CubeIDE  
- **Library:** STM32 HAL (Hardware Abstraction Layer)  
- **Language:** C

## 🚀 Features

- **Non-blocking UART Reception (Interrupt-based)**
- **Command Parsing**
- **LED Control**
- **Dynamic LED Blink Duration**

## 📡 UART Commands

| Command         | Description                             |
|-----------------|-----------------------------------------|
| `on`            | Turns the LED **on**                    |
| `off`           | Turns the LED **off**                   |
| `blink <time>`  | Turns on the LED for `<time>` seconds   |

> Example: Sending `blink 3` via serial will turn the LED on for 3 seconds.

## ⚙️ Code Explanation

- UART receive is configured in **interrupt mode** using `HAL_UART_Receive_IT`.
- Incoming characters are accumulated into a buffer until a recognizable command is detected.
- Upon recognizing:
  - `"on"`: LED is turned **on**
  - `"off"`: LED is turned **off**
  - `"blink <time>"`: LED turns **on for `<time>` seconds**, then off

```c
// Example: Handling "blink 2"
if (strncmp(messageBuffer, "blink ", 6) == 0) {
    blinkTime = atoi(&messageBuffer[6]);  // parse "2"
    interruptFlag = 1;
}
```

The blinking is handled in the main loop using:

```c
if(interruptFlag){
    HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, 1);
    HAL_Delay(blinkTime * 1000);
    HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, 0);
    interruptFlag = 0;
}
```

## 🔌 Pin Configuration

![image](https://github.com/user-attachments/assets/5e9a9cf2-ff94-4482-aa9d-d278d47d52ce)

| Function | STM32 Pin | Nucleo Pin |
| -------- | --------- | ---------- |
| TX       | PA2       | D1         |
| RX       | PA3       | D0         |

## 🧪 How to Use
1. Flash the code to your STM32 Nucleo-F446RE.
2. Open a serial terminal (e.g., PuTTY, Tera Term).
3. Set Baud rate to 115200.
4. Send the following commands:
- on
- off
- blink 5

The LED will behave accordingly.

## 📌 Notes
Input buffer size is `10 bytes`.

All commands must fit within the buffer.

The program resets the buffer after each recognized command.

Uses `string.h` and `stdlib.h` for parsing and comparison.
</details>

<details>
  <summary><h2> 06_Bluetooth_Module - HM-10 Bluetooth UART Communication </h2></summary>

  This project demonstrates **Bluetooth communication between an STM32F4 microcontroller and a smartphone** using the **HM-10 Bluetooth module** over **USART1**. The STM32 receives data sent from the phone and immediately echoes it back, enabling two-way Bluetooth-based serial communication.

## 🔧 Hardware Requirements

- 📶 **Bluetooth Module:** HM-10 (BLE)
- 🎛 **Board:** STM32 Nucleo-F446RE 
- 📡 **Communication:** UART (USART1)
- 🧠 **Library:** STM32 HAL (Hardware Abstraction Layer)
- 📲 **Phone App:** Any Bluetooth serial terminal (e.g., Serial Bluetooth Terminal on Android, BLESerial tiny on iOS)

## 🔌 Pin Configuration

| HM-10 Pin | Connects To STM32 |
|-----------|-------------------|
| VCC       | 3.3V              |
| GND       | GND               |
| TXD       | RX (e.g., PA10 - USART1 RX) |
| RXD       | TX (e.g., PA9  - USART1 TX) |

> ⚠️ **Note:** HM-10 uses 3.3V logic. Do not connect to 5V TX lines directly without a voltage divider or level shifter.

![image](https://github.com/user-attachments/assets/25b97a5d-ef52-4079-b110-836183270cad)


## 💡 How It Works

- The HM-10 module is paired with a smartphone.
- Data sent from the phone is received by STM32 via **USART1**.
- The microcontroller echoes the data back to the phone.
- This forms a simple Bluetooth serial communication loop.

### UART Settings

| Parameter     | Value     |
|---------------|-----------|
| Baud Rate     | 9600      |
| Data Bits     | 8         |
| Stop Bits     | 1         |
| Parity        | None      |
| Flow Control  | None      |

## 📟 Code Behavior

```c
uint8_t message[10];
HAL_UART_Receive(&huart1, message, 10, 1000);  // Receive 10 bytes from HM-10
HAL_UART_Transmit(&huart1, message, 10, 1000); // Echo it back to the phone
```
</details>

<details>
  <summary><h2> Register_Level_1 - Bare-Metal LED Blink </h2></summary>

  This project demonstrates how to toggle an LED using STM32F4 microcontroller registers directly without relying on the HAL (Hardware Abstraction Layer) or CMSIS drivers. The LED connected to **PA5** (typically the onboard LED on STM32F4 Discovery/Nucleo boards) is toggled with a simple software delay.

## 🛠️ Hardware Requirements

- **Board:** STM32 Nucleo-F446RE
- **LED Connection:** Onboard LED at **GPIOA Pin 5**

## 💻 Software Requirements

- **IDE:** STM32CubeIDE
- **No HAL or CMSIS functions used**
- Pure register-level programming

## 🚀 What This Code Does

1. Enables the clock for **GPIOA**.
2. Configures **PA5** as:
   - Output mode
   - Push-pull type
   - High-speed
   - No pull-up/pull-down
3. Enters an infinite loop:
   - Sets **PA5 high** (turn LED on)
   - Waits using a crude software delay loop
   - Sets **PA5 low** (turn LED off)
   - Waits again

## 🔧 Register Configuration Breakdown

```c
RCC->AHB1ENR |= (1<<0);           // Enable clock to GPIOA
GPIOA->MODER &= ~(3<<(5*2));      // Clear mode bits for PA5
GPIOA->MODER |= (1<<(5*2));       // Set PA5 as output (01)
GPIOA->OTYPER &= ~(1<<5);         // Push-pull output
GPIOA->OSPEEDR |= (3<<(5*2));     // High speed
GPIOA->PUPDR &= ~(3<<(5*2));      // No pull-up/pull-down
```

## 💡 LED Blinking Loop

```c
GPIOA->ODR |= (1<<5);             // LED ON
for (int i = 0; i < 1000000; ++i); // Delay
GPIOA->ODR &= ~(1<<5);            // LED OFF
for (int i = 0; i < 1000000; ++i); // Delay
```
## ⚠️ Notes
- This code uses bare-metal programming: no HAL, no interrupts, no external libraries.
- The LED blinking is based on an inaccurate software delay — use hardware timers for better control.
- PA5 is typically connected to the onboard LED on most STM32F4 boards, but verify your board’s pinout.

## ✅ Benefits of Register-Level Programming
- Smaller binary size
- Maximum control over hardware
- Useful for bootloaders or performance-critical code

## ❌ Limitations
- No abstraction: less portable
- More error-prone
- No safety checks
</details>

<details>
  <summary><h2> Register_Level_2 - Button Controlled LED (Bare-Metal GPIO)</h2></summary>

  This project demonstrates how to control an LED connected to **GPIOA Pin 5** using a **button connected to GPIOC Pin 13**, implemented at the register level (bare-metal) without using the HAL or CMSIS abstraction layers.

## 🛠️ Hardware Requirements

- **Development Board:** STM32 Nucleo-F446RE
- **Button:** Connected to **PC13** (typically the blue push button on Nucleo boards)
- **LED:** Connected to **PA5** (typically the onboard LED)

## 💻 Software Requirements

- **IDE:** STM32CubeIDE
- **No HAL/CMSIS functions used**
- **Pure register-level code**

## 🚀 Project Functionality

1. Configures **PC13** as a digital input with **pull-up resistor**.
2. Configures **PA5** as a digital output.
3. Continuously reads the state of the button:
   - If the button is **pressed** (PC13 pulled LOW), the LED on **PA5** is **turned on**.
   - If the button is **not pressed** (PC13 HIGH), the LED is **turned off**.

## 🔧 Register Configuration Summary

### Button (PC13) Configuration

```c
RCC->AHB1ENR |= (1 << 2);               // Enable clock for GPIOC
GPIOC->MODER &= ~(3 << (13 * 2));       // Set PC13 as input
GPIOC->PUPDR &= ~(3 << (13 * 2));       // Clear pull-up/down bits
GPIOC->PUPDR |= (1 << (13 * 2));        // Enable pull-up
```

### LED (PA5) Configuration

```c
RCC->AHB1ENR |= (1 << 0);               // Enable clock for GPIOA
GPIOA->MODER &= ~(3 << (5 * 2));        // Clear PA5 mode bits
GPIOA->MODER |= (1 << (5 * 2));         // Set PA5 as output
GPIOA->OTYPER &= ~(1 << 5);             // Push-pull
GPIOA->OSPEEDR |= (3 << (5 * 2));       // High speed
GPIOA->PUPDR &= ~(3 << (5 * 2));        // No pull-up/pull-down
```

## 🔄 Main Loop Logic

```c
while (1) {
    if (!(GPIOC->IDR & (1 << 13))) {
        GPIOA->ODR |= (1 << 5);  // Turn LED on
    } else {
        GPIOA->ODR &= ~(1 << 5); // Turn LED off
    }
}
```

## ⚠️ Notes
- PC13 is connected to the user button on many Nucleo boards.
- No debouncing is implemented — for real-world use, software or hardware debouncing may be required.
- You can easily adapt this example to other pins or external buttons/LEDs.

</details>
