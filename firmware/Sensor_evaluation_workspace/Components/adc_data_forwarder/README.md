# RA6M5 Devkit ADC Data Forwarder
This is a simple project that gets the data from the ADC ports on a Renesas RA6-series microcontrollers, as well as compute the Root-Means Squared value of the current and prints the comma sepparated values to the USB Serial port so it the board can be used to train models using Edge Impulse Data forwarder.

## Configuration
**Launch Configuration**:ADC Debug_flat 

### ADC

#### Pin Configuration
The pin configuration depends on what pins you want to use. These examples are for the **RA6M5** MCU:
* The **ADC0 Unit** supports pins **P000** to **P015**. 
* The **ADC1 Unit** is a bit more complex, it supports pins:
	* **P000** to **P002** which can overlap with the **ADC0 Unit**
	* **P500** to **P508**
	* **P800** to **P802**

For our example we are going to select **ADC0** and **P003**.
![](https://i.imgur.com/9fbLWsb.png)

![](https://i.imgur.com/WCEXxQ6.png)

#### Stack Configuration

From the Stacks panel in the FSP Configurator add a new **r_adc** stack to your project. This can be either added to the **HAL/Common** category or to a specific thread. For simplicity's sake, we added it to **HAL/Common** so that all threads can access it. 

![](https://i.imgur.com/RlluqXb.png)

Now click on properties, and let's configure and edit the settings. 

Most importantly, we need to set the **ADC Unit** that the **r_adc stack** will use. Some pins are connected to the ADC0 Unit and some are connected to the ADC1 unit. You need to set the unit depending on what pins your peripherals are connected to. If you want to use both, you'll need to add two **r_adc** stacks. Since we are going to use **P003** we are going to select **Unit 0**

If you configured your pins correctly, once you select the correct unit, you should see this screen at the bottom of the properties panel. 
![](https://i.imgur.com/aeC3Ryq.png)

Another important setting you'll find here is the **Mode**. There's continuous scan, single scan, and group scan. Because we want to be able to control when the scan occurs, we'll select **Single Scan Mode**.

![](https://i.imgur.com/FpfEa6V.png)

Next up, we'll select what Channels can be read in a single scan:
![](https://i.imgur.com/S5uZ6CJ.png)

At this section, we'll need to give the FSP Configurator the name of the method that it should call when an ADC scan has ended, and set the FreeRTOS priority of that callback:

![](https://i.imgur.com/uOgqn2J.png)


#### Root Mean Square (RMS)
Root Mean Square, often abbreviated as RMS, is a mathematical calculation used to find the effective or average value of a set of numbers. It is commonly used in signal processing and electrical engineering to determine the magnitude of varying quantities, such as voltage or current.

The RMS value is calculated using the following steps:

* Take the square of each number in the set.
* Find the average (mean) of the squared values.
* Take the square root of the mean.
* The RMS value provides a representation of the "typical" value in a fluctuating signal, considering both positive and negative values. It is often used to measure the magnitude of alternating current (AC) or to determine the power of a signal.

#### Code Explanation

```cpp
static void startScan(){
    fsp_err_t err;

    err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    if( err == FSP_SUCCESS){
        err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
        if( err == FSP_SUCCESS){
            err = R_ADC_ScanStart(&g_adc0_ctrl);
            R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
        }
    }
}
```

```cpp
void adc_callback(adc_callback_args_t *p_args){
    if((NULL != p_args) && (ADC_EVENT_SCAN_COMPLETE == p_args->event)){
        scanEnded = true;
    }
}
```

```cpp
static void sampleADC(){
    // read current values from the ADC
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_3, &currentReading);
    scanEnded = false;

    offsetI = offsetI + ((currentReading-offsetI)/4096);
    filteredI = currentReading - offsetI;
    sqI = filteredI * filteredI;
    sumI += sqI;
    ++cycleCounter;

    // initiate a new scan when we are ready to process the next sample
    startScan();
}
```
This method is responsible for sampling the analog-to-digital converter (ADC) and performing calculations on the obtained readings.

`R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_3, &currentReading);` reads the current values from the ADC and stores the result in currentReading.

`offsetI = offsetI + ((currentReading-offsetI)/4096);` calculates the offset by gradually updating the value based on the current reading.

`filteredI = currentReading - offsetI;` calculates the filtered current value by subtracting the offset from the current reading.

`sqI = filteredI * filteredI;` computes the square of the filtered current value.

`sumI += sqI;` accumulates the squared values to calculate the sum.

`++cycleCounter;` increments the cycle counter.

`startScan();` initiates a new scan to process the next sample.

 ```c
 static double calculateCurrent(){
    double I_RATIO = ICAL *((AREF/1000.0) / (ADC_COUNTS));
    double Irms = I_RATIO * sqrt(sumI / CYCLES);
    sumI = 0;

    return Irms;
}
 ```
This method calculates the RMS current using the accumulated squared values from sampleADC().

`double I_RATIO = ICAL *((AREF/1000.0) / (ADC_COUNTS));` computes a constant ratio based on calibration values and other constants.

`double Irms = I_RATIO * sqrt(sumI / CYCLES);` calculates the RMS current by dividing the sum of squared values by the number of cycles and taking the square root. The constant ratio is then multiplied to obtain the final result.

`sumI = 0;` resets the sum of squared values for the next calculation.

These methods work together to sample and process ADC readings and calculate the RMS current based on the accumulated squared values.
 
```c
static void readADC(){
    if(scanEnded) sampleADC();
    if(cycleCounter > CYCLES){
       rmsCurrent = calculateCurrent();
       cycleCounter = 0;
    }
}
```


### USB/UART Serial Support

For USB Serial support, you'll need to add **USB PCDC (r_usb_pcdc)** and **CRC** to your solution. These stacks need to be added into a thread because they depend on some timer magic that's only available inside the threads. 
To support UART, you'll need to also add **UART (r_sci_uart)** to your solution. If you want more SCI UART ports, just add another instance, and configure their pins accordingly. The **r_sci_uart** stack can be added to the common section, or any thread. It doesn't really (as far as I know)

![](https://i.imgur.com/GchjY1m.png)

#### USB Stack Configuration 
The  **USB PCDC (r_usb_pcdc)** stack (Peripheral Communication Device Class), doesn't need any particular settings, however, the lower level of the stack **r_usb_basic** driver needs a couple of particular configurations to work on our machine. 

Make sure that your configuration looks like this: 

![](https://i.imgur.com/v6osywu.png)

The most important settings in here are the **USB Module Port** and the **USB RTOS Callback**. To access the MicroUSB port on the RA6M5 Devkit MCU daughterboard, use the **USB_IP0** setting. For the Type-C port on the full board, use **USB IP1** Port. 

The **USB RTOS Callback** is the method that FreeRTOS will call upon a new USB connection. 
Set it to `usb_cdc_rtos_callback`. This method can be found in `serial/comms/usb/usb_comms.c`

Global RTOS objects are created from the configuration interface and the E2 IDE will create all the required template code. For the implementation of USB Serial, we'll need a Queue called `g_usb_read_queue` and  a binary semaphore called `g_usb_write_complete_binary_semaphore`

![](https://i.imgur.com/jfDq0t3.png)

The last step we need to take to get the USB serial interface to work is to define it's pins. By default **USB_DP** and **USB_DM** are set, but we'll need to set the **USB_VBUS** pin. Don't worry, you won't have to scroll trough endless pages of schematics for this. It will only give you one option: **P407**. 

![](https://i.imgur.com/0KKLANf.png)

#### UART Stack Configuration 
The UART Stack configuration is pretty simple compared to the USB part of things. 
You can change the baud rate and classic serial port settings in the properties panel for **g_uart0**. 

![](https://i.imgur.com/IKogO97.png)

Additionally, you'll need to go into the Pin Configuration panel to set up SCI0 to Async UART Mode. 
![](https://i.imgur.com/g4YSQIE.png)

Click on **Generate Project Content** and let's write some code. 
## API

The API for this library is designed so it can be as similar to Arduino as possible. 
The only difference is here everything is static and we need a `SerialPort` struct to contain the mutexes and events for each port. We can bind these structs to the ports we want to use by using the following methods:

```cpp
void setupUART(const uart_instance_t* port, struct SerialPort * settings);
```

```cpp
void setupUSB(struct SerialPort * settings);
```

```cpp
SerialPort USBSerial;
SerialPort SCISerial;

setupUSB(&USBSeria1);
setupUART(&g_uart0, &SCISerial);
```

### Writing to a serial port
```cpp
void write(struct SerialPort * settings, char* str, uint8_t str_len );
```

```cpp
void _printf(struct SerialPort * settings, char* str, ... );
```

### I2C Access 
For I2C access you will need to do some additional settings. One is to set the I2C callback to the callback in method defined in `src/wire/wire.h`, namely `wire_master_callback `.

![](https://i.imgur.com/luEJzqb.png)

Additionally, the wire library lets you access the hardware interface from all of the threads, so it makes heavy use of mutexes, make sure your thread settings match the ones set in this screenshot:

![](https://i.imgur.com/jF8oAwk.png)

### Printing floats 

This simplifies access to USB serial, and allows us to `printf` to the console with a variable number of arguments. Under the hood, this uses `vsnprintf`. 
Due to the fact that `printf` is not re-entrant and uses a lot of heap space for floats, we need to do some configurations. 

#### Increase Stack/Heap size
Calling `printf` on floats, abuse the stack and heap quite a lot. That's why we need to increase both the heap size and the stack sizes for our threads. 
First, go to the BSP Tab in the **FSP Configuration** and under **RA Common** set the **Main stack size** and **Heap size** to 0x4000 (16384) bytes.

![](https://i.imgur.com/PU8uxHz.png)

Now, for each thread that uses `printf`, change the stack and heap size too. 

![](https://i.imgur.com/nymDczx.png)

Feel free to tweak the settings to your liking, but these settings have worked for me.

#### Enable newlib-nano
Go to **Project Settings > C/C++ Build > Settings > GNU Arm Cross C++ Linker > Miscellaneous** and enable :
* **Use newlib-nano** 
* **Use float with nano printf**

![](https://i.imgur.com/zvIm5Yj.png)
 
#### 2.2. Enable Newlib Reentrant 
For the thread that prints floats we also need to enable **Use Newlib Reentrant** on the **Stacks** page of **FSP Configuration** and selecting the thread we are interested in. 

![](https://i.imgur.com/H8TrWH4.png)

