<details>
<summary> <h3></h3> 00_Hello_World - LED Blink Project</h3></summary>
  
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

