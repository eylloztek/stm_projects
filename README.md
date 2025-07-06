<details>
<summary> <h3> 00_Hello_World - LED Blink Project </h3></summary>
  
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
  <summary><h3> 01_Knight_Rider - LED Animation Project</h3></summary>

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
  <summary><h3> 02_Button - Button-Controlled LED Counter </h3></summary>

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
  <summary> <h3> 03_External_Interrupt - External Interrupt LED Blink</h3></summary>

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

