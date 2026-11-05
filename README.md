# STM32 Real-Time DC Motor Speed Controller Using PID

![STM32](https://img.shields.io/badge/Platform-STM32F407--Discovery-blue)
![Language](https://img.shields.io/badge/Language-Embedded%20C-green)
![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE-orange)
![Control](https://img.shields.io/badge/Control-PID-red)
![PWM](https://img.shields.io/badge/PWM-10%20kHz-blueviolet)
![Status](https://img.shields.io/badge/Project-Development-yellow)

## 1. Project Overview

This project implements a real-time DC motor speed control system using an **STM32F407-Discovery development board** and a PID (Proportional–Integral–Derivative) controller.

The system uses a quadrature rotary encoder to measure the actual motor speed, calculates the speed error, and dynamically adjusts the PWM duty cycle to maintain the desired rotational speed.

The control algorithm runs periodically using a hardware timer interrupt, while USART2 provides real-time monitoring of the motor parameters through a serial terminal.

The main objective is to develop a modular embedded firmware architecture combining hardware timer configuration, encoder interfacing, PWM generation, PID control, interrupt-driven execution, and UART communication.

## 2. Main Features

- Real-time DC motor speed regulation using PID control.
- Quadrature encoder interfacing using STM32 hardware timers.
- RPM calculation based on encoder pulse measurements.
- PWM generation for motor speed control.
- Fixed-period control loop using TIM6 interrupts.
- PID output saturation and integral anti-windup protection.
- Real-time UART telemetry using USART2.
- Modular Embedded C architecture with separate PID and motor-control modules.
- Hardware abstraction using STM32 HAL libraries.

## 3. Hardware Requirements

| Component | Description |
|---|---|
| Microcontroller | STM32F407VGT6 |
| Development Board | STM32F407-Discovery |
| DC Motor | Brushed DC motor |
| Encoder | Incremental quadrature encoder |
| Motor Driver | External H-bridge motor driver |
| Communication | USB-to-UART / USART2 connection |
| Power Supply | Appropriate regulated motor and logic supplies |

**Important:** The motor must be connected through an appropriate motor driver. The STM32 GPIO pins must not directly drive the motor.

## 4. Software Requirements

- STM32CubeIDE
- STM32CubeMX (integrated into STM32CubeIDE)
- STM32F4 HAL Driver Library
- ST-LINK debugger/programmer
- Serial terminal (PuTTY, Tera Term, or similar)

Programming language: **Embedded C**

## 5. System Architecture

The firmware is divided into three main software layers:

### A. PID Controller (`pid.c` / `pid.h`)

Responsible for calculating the control output based on the difference between the desired and measured motor speed.

Implemented functions:

- `PID_Init()` – Initializes the PID parameters and internal states.
- `PID_Update()` – Calculates the PID output.
- `PID_Reset()` – Resets the integral and previous error.
- `clamp()` – Limits a value within the configured output boundaries.

The PID equation is:

\[
u(t)=K_p e(t)+K_i\int e(t)dt+K_d\frac{de(t)}{dt}
\]

Where:

- \(K_p\): Proportional gain.
- \(K_i\): Integral gain.
- \(K_d\): Derivative gain.
- \(e(t)\): Difference between target and measured RPM.
- \(u(t)\): Controller output.

The implementation also includes an integral anti-windup mechanism to prevent excessive integral accumulation when the controller output reaches its saturation limits.

### B. Motor Control Module (`motor_control.c` / `motor_control.h`)

Responsible for connecting the PID algorithm with the encoder measurements.

Implemented functions:

- `MotorController_Init()` – Initializes the motor controller.
- `MotorController_SetTarget()` – Sets the desired motor speed.
- `MotorController_Update()` – Calculates the actual RPM and updates the PID controller.

**Encoder speed calculation:**

The current encoder count is compared with the previous encoder count to determine the number of pulses detected during the sampling period.

The RPM calculation is:

\[
RPM=\frac{\Delta Count}{CountsPerRev \times dt}\times60
\]

Current encoder configuration:

- Encoder resolution assumption: 100 pulses/revolution.
- Quadrature counting: ×4.
- Total counts per revolution: 400.

### C. Main Application (`main.c`)

The main application handles:

- STM32 peripheral initialization.
- System clock configuration.
- PWM generation through TIM3.
- Encoder acquisition through TIM4.
- Periodic control execution through TIM6.
- UART telemetry through USART2.
- Communication between the interrupt routine and the main loop.

## 6. STM32 Peripheral Configuration

The project uses the following peripheral configuration:

| Peripheral | Function | Configuration |
|---|---|---|
| TIM3 | PWM Generation | Channel 1 |
| TIM4 | Quadrature Encoder | Channel 1 and Channel 2 |
| TIM6 | Control Loop Timer | 100 Hz |
| USART2 | Serial Communication | 115200 baud |
| GPIO | Peripheral interfacing | Alternate Function / Output |

### Pin Configuration

| STM32 Pin | Peripheral Function | Description |
|---|---|---|
| PA6 | TIM3_CH1 | PWM Output |
| PB6 | TIM4_CH1 | Encoder Channel A |
| PB7 | TIM4_CH2 | Encoder Channel B |
| PA2 | USART2_TX | UART Transmission |
| PA3 | USART2_RX | UART Reception |

### System Clock Configuration

The firmware configures the STM32F407 system clock using the external high-speed oscillator and PLL.

| Clock | Frequency |
|---|---:|
| SYSCLK | 168 MHz |
| HCLK | 168 MHz |
| APB1 | 42 MHz |
| APB2 | 84 MHz |
| APB1 Timer Clock | 84 MHz |

### TIM3 – PWM Configuration

TIM3 Channel 1 generates the PWM signal used to control the motor driver.

Configuration:

- Prescaler (PSC): 83
- Auto-Reload Register (ARR): 99
- Timer Clock: 84 MHz
- PWM Frequency: 10 kHz
- Output Pin: PA6

PWM frequency calculation:

\[
f_{PWM}=\frac{84MHz}{(83+1)(99+1)}
\]

\[
f_{PWM}=10kHz
\]

The PID controller generates a PWM command between 0% and 100%.

### TIM4 – Encoder Configuration

TIM4 operates in quadrature encoder mode using both input channels.

Configuration:

- Encoder Mode: `TIM_ENCODERMODE_TI12`
- Channel 1: PB6
- Channel 2: PB7
- Prescaler: 0
- Counter Period: 65535
- Input Capture Prescaler: DIV1

The timer hardware counts encoder transitions, allowing the firmware to calculate the motor speed without manually detecting every encoder edge using GPIO interrupts.

### TIM6 – Periodic Control Loop

TIM6 generates a periodic interrupt to execute the motor-control algorithm.

Configuration:

- Timer Clock: 84 MHz
- Prescaler: 8399
- Auto-Reload Register: 99
- Interrupt Frequency: 100 Hz
- Control Period: 10 ms

Frequency calculation:

\[
f=\frac{84MHz}{(8399+1)(99+1)}
\]

\[
f=100Hz
\]

The corresponding sampling period is:

\[
dt=0.01s
\]

### USART2 – UART Communication

USART2 is configured to transmit motor information to a computer.

Configuration:

- Baud Rate: 115200
- Word Length: 8 bits
- Stop Bits: 1
- Parity: None
- Mode: TX/RX
- TX: PA2
- RX: PA3

## 7. Control Loop Operation

The complete control sequence follows these steps:

1. Initialize the STM32 peripherals and motor controller.
2. Set the desired motor speed (initial target: 1500 RPM).
3. Start TIM3 PWM generation.
4. Start TIM4 encoder interface.
5. Start TIM6 periodic interrupt.
6. Read the encoder counter every 10 ms.
7. Calculate the encoder count difference.
8. Convert encoder counts into actual RPM.
9. Calculate the speed error.
10. Execute the PID algorithm.
11. Apply the resulting PWM duty cycle to TIM3.
12. Set a flag indicating that new motor data is available.
13. Transmit the updated motor information through USART2.

The interrupt callback is implemented using:

`HAL_TIM_PeriodElapsedCallback()`

The main loop handles UART telemetry separately from the periodic control calculations.

## 8. Initial PID Parameters

The project currently uses the following initial PID gains:

| Parameter | Value |
|---|---:|
| Kp | 0.8 |
| Ki | 0.2 |
| Kd | 0.02 |
| Minimum Output | 0% |
| Maximum Output | 100% |
| Initial Target Speed | 1500 RPM |
| Maximum Requested Speed | 3000 RPM |

These are starting values and require experimental tuning according to the actual motor, encoder, motor driver, and mechanical load.

The controller includes anti-windup logic to prevent the integral term from continuing to accumulate in a direction that worsens output saturation.

## 9. UART Telemetry

The system sends motor information through USART2 every time a new control update is available.

Example output format:

```text
Target: 1500.0 RPM | Actual: 1425.0 RPM | PWM: 48.0 %
Target: 1500.0 RPM | Actual: 1462.0 RPM | PWM: 45.0 %
Target: 1500.0 RPM | Actual: 1490.0 RPM | PWM: 43.0 %
```

*The values above are illustrative examples of the output format, not measured experimental results.*

This allows the user to monitor the target speed, actual speed, and controller output through a serial terminal.

## 10. Repository Structure

```text
STM32-PID-Motor-Controller/
│
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── pid.h
│   │   └── motor_control.h
│   │
│   └── Src/
│       ├── main.c
│       ├── pid.c
│       └── motor_control.c
│
├── Drivers/
│   └── STM32F4xx_HAL_Driver/
│
├── screenshots/
│   └── Add project screenshots here
│
├── documentation/
│   └── Add wiring diagrams and technical documentation here
│
├── README.md
└── LICENSE
```

*The structure above is a suggested organization. Retain the STM32CubeIDE-generated project files and directories required to build the firmware.*

## 11. How to Build and Run

1. Clone or download this repository.
2. Open STM32CubeIDE.
3. Import the STM32CubeIDE project.
4. Connect the STM32F407-Discovery board using ST-LINK.
5. Verify the peripheral configuration and GPIO assignments.
6. Connect the quadrature encoder to PB6 and PB7.
7. Connect the motor driver PWM input to PA6.
8. Connect the motor driver to the DC motor and appropriate power supply.
9. Configure the serial terminal for USART2 at 115200 baud, 8N1.
10. Build the project.
11. Flash the firmware onto the STM32F407-Discovery board.
12. Monitor the UART output.

Before connecting the motor, verify the PWM output and motor-driver wiring. Start with a low PWM command and an appropriate current-limited power supply.

## 12. Important Configuration Notes

Before using the firmware with a real motor, verify the following:

- **Encoder resolution:** Update `ENCODER_COUNTS_PER_REV` according to the actual encoder specification and counting mode.
- **PID tuning:** Adjust Kp, Ki, and Kd experimentally.
- **Motor driver:** Ensure that the selected driver supports the motor voltage and current.
- **Encoder direction:** Verify that the encoder count changes in the expected direction.
- **PWM frequency:** Confirm that 10 kHz is suitable for the selected motor driver.
- **Control-loop timing:** Keep the PID sampling period consistent with the actual TIM6 interrupt frequency.

## 13. Future Improvements

Potential extensions for future versions include:

- Interactive target RPM adjustment through UART commands.
- PID gain adjustment without recompiling the firmware.
- UART reception using interrupts or DMA.
- Improved encoder filtering and speed measurement.
- More advanced PID tuning methods.
- Motor direction control.
- Fault detection and motor protection.
- Real-time RPM visualization using a PC monitoring interface.
- Integration with FreeRTOS for multitasking applications.

## 14. Skills Demonstrated

This project demonstrates practical knowledge of:

- Embedded C programming.
- STM32F407 microcontroller architecture.
- STM32CubeIDE and CubeMX configuration.
- STM32 HAL libraries.
- Hardware timer configuration.
- PWM generation and duty-cycle control.
- Quadrature encoder interfacing.
- PID control implementation.
- Interrupt-driven embedded programming.
- UART serial communication.
- Modular firmware development.
- Real-time embedded control systems.

## 15. Author

**Embedded Systems / Firmware Development Portfolio**

Developed using STM32F407-Discovery and Embedded C.

---

**Project:** STM32 Real-Time DC Motor Speed Controller Using PID  
**Microcontroller:** STM32F407VGT6  
**Development Environment:** STM32CubeIDE  
**Programming Language:** Embedded C

