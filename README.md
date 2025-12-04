# GameBob - a custom made gameboy
<img width="1400" height="1050" alt="Image" src="https://github.com/user-attachments/assets/5f3f22c8-3ec0-42f0-b36d-c35933f03db4" />


## What is gameBob?
GameBob is a custom made gameboy based on the STM32F103RB microcontroller. 

### Core functionalities
- FreeRTOS captures user input and updates game status
- WiFi connection in order to download games from local server (OTA programming)
- Bootloader to switch between main menu and game firmware

## Building the project

### REQUIREMENTS

1. Install the **GNU Arm Embedded Toolchain** (`gcc-arm-none-eabi`).  
   - [Download from ARM](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)  

2. **CMake**  
   - [Download CMake](https://cmake.org/download/) and add it to your PATH  

3. **Ninja** 
   - [Download Ninja](https://ninja-build.org/) and add it to your PATH  

4. **cube-cmake**  
   Comes with **STM32CubeIDE / STM32CubeMX**. Make sure you can run `cube-cmake` from the terminal.

## Building the project

1. Clone the repository and then navigate to the project folder:
   ```bash
   git clone https://github.com/ErykPiechowiak/gameBobMainMenu
   cd gameBobMainMenu
    ```
2. Create build files:
   ```bash
   cube-cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -G Ninja -S . -B build/Release
   ```
3. Build the project
   ```bash
   ninja -C build/Release
   ```

   **Do the same steps with any avaliable game repository for the gameBob project.**
   
## Flashing the microcontroller

Flash region is divided into two parts: 
- from 0x08000000 to 0x0800FFFF - a main menu application acting as a bootloader. Bootloader dowloads games binary files from local http server, flashes it into memory and then runs the game code. The mainMenu (bootloader) firmware can be found here: [mainMenu Repository](https://github.com/ErykPiechowiak/gameBobMainMenu)
- from 0x08010000 to 0x0801FFFF - a game downloaded by main menu firmware or flashed directly in this region by st-link flasher. Avaliable game firmware can be found here: [Pong Repository](https://github.com/ErykPiechowiak/gameBobPong)

Use the ST-LINK flasher in order to flash the .bin files into the flash memory. **Remember about the proper addresses!**
Instruction how to configure the local server with games files and flashing them by using the built-in bootloader will be avaliable in the gameBob Server repository: [Server Repository](https://github.com/ErykPiechowiak/gameBobServer)

