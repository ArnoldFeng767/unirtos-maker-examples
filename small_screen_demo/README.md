# Circular Screen Multi-Tool Demo

[中文](./README_zh.md)

The circular screen multi-tool demo is a standalone small-screen smart terminal solution built on UniRTOS.

With UniRTOS as the core runtime platform, this project drives a 1.28-inch GC9A01 circular SPI color screen and a CST816T capacitive touch chip, combined with the LVGL v8.3.11 graphics library, to render a clock face, a feature-selection page, and several practical small-tool pages, along with touch interaction. The device does not rely on networking capabilities such as communication or positioning, and instead focuses on "display + touch interaction + practical small tools". It automatically shows the current date and time as the default clock face on power-up, supports switching between the clock face and the feature-selection page with an upward/downward swipe, and can further enter independent feature pages such as a stopwatch, a tomato timer, a wooden-fish counter, and a calculator. Relying on the API-rich UniRTOS, the solution has good cross-module adaptability, making it suitable for scenarios such as smartwatch dial prototyping, desktop gadgets, curio-style electronic accessories, and rapid integration of circular-screen touch interaction.

# Development Resources

| Accessory Name               | Quantity | Description                                                             | How to Obtain |
| :---------------------------- | :------- | :------------------------------------------------------------------------ | :------------- |
| QuecDuino development board  | 1        | Hardware platform for running the project; must carry module model "EG800ZCN_LA". | [Buy here](https://www.quecmall.com/goods-detail/2c90800b9e4a2da8019e71fa2b3000df) |
| GC9A01 circular touch screen module | 1  | 1.28-inch 240x240 circular IPS screen (SPI interface), integrated with a CST816T capacitive touch chip (I2C interface) that supports click/double-click/long-press/swipe gesture reporting. | Self-provided |
| USB data cable                | 1        | Used to connect the PC and the development board; a USB-A to USB-C cable of the non "charge-only" type. | Self-provided |
| PC                             | 1        | System: Windows 10/Windows 11.                                            | Self-provided |

# Quick Start

## Hardware Connection

Wiring between the circular screen (GC9A01, SPI) and the development board is as follows (pin numbers are module pin numbers, not GPIO numbers):

| Screen Pin | Board Pin | Description                          |
| :--------- | :-------- | :------------------------------------ |
| CS         | Pin64     | SPI chip select                       |
| MOSI       | Pin63     | SPI master-out-slave-in               |
| MISO       | Pin62     | SPI master-in-slave-out (unused)      |
| SCLK       | Pin49     | SPI clock                             |
| RST        | Pin22     | Reset (GPIO)                          |
| DC         | Pin23     | Command/data select (GPIO)            |
| BL         | Pin19     | Backlight control (GPIO)              |

Wiring between the touch chip (CST816T, I2C) and the development board is as follows:

| Touch Chip Pin | Board Pin | Description                         |
| :-------------- | :-------- | :------------------------------------ |
| RST             | Pin66     | Touch chip reset (GPIO)               |
| INT             | Pin67     | Gesture interrupt, falling-edge triggered (GPIO) |
| SCL             | Pin57     | I2C clock                             |
| SDA             | Pin58     | I2C data                              |

1. Connect the screen's SPI signal lines and the touch chip's I2C signal lines, reset pin, and interrupt pin according to the tables above.
2. Connect the development board and the PC with the data cable.

## Software Deployment

### Setting Up the Development Environment

Refer to the [UniRTOS Quick Start](https://docs.quectel.com/zh/UniRTOS/UniRTOS文档/快速上手/快速上手.html) documentation to learn how to set up the development environment and complete the basic development workflow.

### Pulling the Code

Open a new PowerShell window and run the following commands:

```Plain
# Pull the example repository
unirtos-cli new -r unirtos-maker-examples
# Enter the project directory
cd unirtos-maker-examples/small_screen_demo
```

### Building the Project

Run the firmware build command in the PowerShell window:

```PowerShell
unirtos-cli build -m EG800ZCN_LA -d <project directory>
```

After the build finishes, the end of the PowerShell window will show the firmware build result:

```Plain
SUCCESS: Unirtos project built successfully!
```

After the firmware builds successfully, flash it to the development board with QFlash.

### On-Board Verification

1. After the development board powers on, the screen lights up automatically and shows the circular clock face (time + date + weekday); if the RTC has not yet synced to a valid time, the clock face temporarily shows a "SYNCING" placeholder.

<img src="./media/home_page.png" alt="Clock Face" style="zoom: 50%;" />

1. Swipe up on the clock face to enter the feature-selection page, where four feature icons — wooden-fish counter, tomato timer, stopwatch, and calculator — are arranged vertically; swipe up/down to switch the currently selected item, which is enlarged and centered.

<img src="./media/function_selection_page.png" alt="Feature-Selection Page" style="zoom: 50%;" />

1. Tap/double-tap the currently selected icon on the feature-selection page to enter its independent run page:
   - **Stopwatch**: Tap/double-tap to start or pause timing; long-press the on-screen RESET button area to reset the timer to zero; swipe down to return to the feature-selection page.

     <img src="./media/stopwatch.png" alt="Stopwatch" style="zoom: 50%;" />

   - **Tomato Timer**: Defaults to a 25-minute countdown; swipe left/right to adjust the duration (effective only before the countdown starts); tap/double-tap to start or pause; long-press to reset; once the countdown finishes, it switches to an eye-catching "DONE" completion screen; swipe down to reset and restart.

     <img src="./media/timer.png" alt="Tomato Timer" style="zoom: 50%;" />

   - **Wooden-Fish Counter**: Tap anywhere on the screen to increment the count by 1, accompanied by a scale/highlight animation and a floating "+1" indicator; long-press enters a clear-confirmation state, tap again to confirm clearing; swipe down to cancel the confirmation or return to the feature-selection page.

     <img src="./media/fish.png" alt="Wooden-Fish Counter" style="zoom: 50%;" />

   - **Calculator**: Use the on-screen 4x4 digit/operator keypad to perform chained basic arithmetic operations; swipe down to return to the feature-selection page (calculation state is preserved).

     <img src="./media/calc.png" alt="Calculator" style="zoom: 50%;" />

1. Swiping down in the top area of the screen on the feature-selection page returns directly to the clock face, regardless of which item is currently selected.

### Running Log

When the device is running normally, a log tool can capture a running log similar to the one below, which can be used to confirm whether touch gesture recognition, page switching, and other states are working correctly:

<img src="./media/log.png" alt="Running Log" style="zoom: 67%;" />

## Behavior Summary

The interaction behavior on each page is as follows:

- **Clock Face**: Always shows time/date/weekday; swipe up to enter the feature-selection page.
- **Feature-Selection Page**: Icons are arranged vertically based on selection state; the selected item is enlarged, centered, and rendered on top, while adjacent items shrink progressively and shift toward the inner edge of the circular screen; the label font size scales automatically with row distance.
- **Stopwatch / Tomato Timer / Wooden-Fish Counter / Calculator Run Pages**: All support swiping down to quickly return to the feature-selection page; each page retains its own running state (elapsed time, count, or calculation result) and does not need to restart when re-entered.

## FAQ

- **The screen shows nothing**: Check the SPI wiring (especially CS/MOSI/SCLK/RST/DC/BL) against the table above, and confirm the development board model is EG800ZCN_LA.
- **Touch does not respond**: Check the I2C wiring (SCL/SDA) and the INT/RST pin connections, and confirm the touch chip model is CST816T (the chip ID register should read `0xB5`).
- **The clock face shows "SYNCING" for a long time**: This means the RTC has not yet synced to a valid time; wait for the module to register on the network and complete the time sync, or check the SIM card and network status.
