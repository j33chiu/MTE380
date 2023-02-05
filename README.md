
# Setup Instructions for Nucleo F401re

1. Download and install ArduinoIDE (https://www.arduino.cc/en/software)
2. In the ide, under *file->preferences*, paste https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json under "Assitional boards manager URLs"
3. under *tools->Board->Boards Manager* search "STM32" 
4. There should be a result named "STM32 MCU based boards by STMicroelectronics", install the latest version. There may be a popup for additional dependencies, just install those as well
5. In the top left, there is a dropdown for selecting the board, find and select "Generic STM32F4 series"
6. If the board is connected, there should be a discovered port as well that needs to be selected
7. under *tools->Board Part Number* find and select "Generic F401RETx"
8. In the library manager (3rd tab on the left side) install the libraries and their required dependencies:
   * Adafruit ICM20X
   * Adafruit MPU6050
   * DFRobot_VL53L0X
9. should be able to compile and run repo code now
10. If an "STM32_Programmer_CLI.exe: not found" error occurs, download and install STMCubProgrammer from https://www.st.com/en/development-tools/stm32cubeprog.html#get-software
