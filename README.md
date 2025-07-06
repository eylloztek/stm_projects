# 00_Hello_World - LED Blink Project

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



