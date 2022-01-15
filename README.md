# stm32f4-project
STM32F407VG micro controller used to send ADXL345 data (Accelerometer) to Laptop / microSD card over.
- If Ethernet cable is connected between STM32F4 and Laptop, the Accelerometer data is sent to PC (using TCP socket)
- When Ethernet connection goes down, Accelerometer data is saved in SD card (available in STM32F4 expansion) 
- Whenever Ethernet cable is reconnected, the data saved in SD card is sent to Laptop. After that, the Accelerometer starts again sending data to PC

![image](https://user-images.githubusercontent.com/94836571/149620294-0efefd8a-e553-4f61-af70-d4545b537607.png)

# Functional Design 
This application is realized with 3 FreeRToS Tasks :
- Acc_To_Eth task : sends accelerometer data to remote server 
- Acc_To_SD task : writes accelerometer data to microSD card
- SD_To_ETH task : sends SD data to remote server
Task communication is made with a osMessageQueue()

# ADXL345 Communication
The Micro controller communicates with the Accelerator via SPI (through a "mini" SPI driver).
The developed driver enables axis measurements reading and tap detection.

# Ethernet Communication 
Micro communicates to PC with a TCP socket from the LwIP (Light weight IP) stack.  

# microSD interface
The FatFS Library is used to read/write data from/into the microSD card

![image](https://user-images.githubusercontent.com/94836571/148701100-724250b8-dc97-463e-80a4-a5cccae53ecd.png)
