# Environmental Sensor Data Forwarder

This is a simple project that gets the data from the HS300X Environmental Sensor found in the RA6M5 Devkit and prints the comma sepparated values to the USB Serial port so it the board can be used to train models using Edge Impulse Data forwarder



## Configuration
This example includes some custom wrappers i have written for accessing USB, UART and I2C interfaces, code that is mainly based on the Reloc examples. You can find the two libraries, serial, and wire, in the `src/` folder.

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


## Code

We use this function to read information from the HS300X Temperature and Humidity Sensor

```c
#define HS300X_IC2_ADDR 0x44

#define HS300X_MEAS_REQUEST 0x00
#define HS300X_DATA_LEN_BYTE 4

#define HS300X_WAKEUP_TIME_MS 1
#define HS300X_MEASURE_TIME_MS 100
#define HS300X_EVENT_WAIT_TIMEOUT_MS 1000

#define HS300X_WAKEUP_TIME_TICKS pdMS_TO_TICKS(HS300X_WAKEUP_TIME_MS)
#define HS300X_MEASURE_TIME_TICKS pdMS_TO_TICKS(HS300X_MEASURE_TIME_MS)
#define HS300X_EVENT_WAIT_TIMEOUT_TICKS pdMS_TO_TICKS(HS300X_EVENT_WAIT_TIMEOUT_MS)

static fsp_err_t hs300x_temp(float* p_temperature, float* p_humidity) {

  uint8_t buf[HS300X_DATA_LEN_BYTE];
  fsp_err_t ret = FSP_SUCCESS;

  // Initialize communication with the HS300X sensor using the I2C protocol
  wire_init_communication(HS300X_IC2_ADDR);

  // Make a dummy write request to trigger a measurement request
  // Actually, no data is required, but the lower-level I2C driver requires at least 1 byte
  buf[0] = HS300X_MEAS_REQUEST;

  ret = wire_write(buf, 1, false);

  // Delay to allow time for measurement
  vTaskDelay(HS300X_MEASURE_TIME_TICKS + HS300X_WAKEUP_TIME_TICKS);

  // Read the measurement results from the sensor
  ret = wire_read(buf, HS300X_DATA_LEN_BYTE, false);
  
  if (ret == FSP_SUCCESS) {
    // Calculate humidity
    *p_humidity = (float)((((float)(((buf[0] & 0x3f) * 0x100) + buf[1])) / 16383.0) * 100.0);
    
    // Calculate temperature
    *p_temperature = (float)((((float)((unsigned short)((buf[2]) * 0x100 + buf[3]) >> 2)) / 16383.0) * 165.0 - 40.0);
  }

  return ret;
}
```

Here's a breakdown of the code: 
* `uint8_t buf[HS300X_DATA_LEN_BYTE];` defines a buffer to hold the data read from the sensor.
* `wire_init_communication(HS300X_IC2_ADDR);` initializes the communication with the HS300X sensor using the I2C protocol. The I2C address of the sensor is specified as HS300X_IC2_ADDR.

* `buf[0] = HS300X_MEAS_REQUEST;` prepares a dummy write request in order to trigger a measurement request. The value HS300X_MEAS_REQUEST is assigned to the first element of the buffer.

* `ret = wire_write(buf, 1, false);` performs a write operation to send the dummy request to the sensor. This triggers the sensor to perform a measurement. The parameter 1 specifies the number of bytes to write, and false indicates that a repeated start condition is not required.

* `vTaskDelay(HS300X_MEASURE_TIME_TICKS + HS300X_WAKEUP_TIME_TICKS);` introduces a delay to allow time for the sensor to perform the measurement. The HS300X_MEASURE_TIME_TICKS and HS300X_WAKEUP_TIME_TICKS are predefined time values.

* `ret = wire_read(buf, HS300X_DATA_LEN_BYTE, false);` reads the measurement results from the sensor into the buffer. The HS300X_DATA_LEN_BYTE specifies the number of bytes to read, and false indicates that a repeated start condition is not required.

* If the read operation is successful `(ret == FSP_SUCCESS)`, the function calculates the humidity and temperature values based on the data in the buffer.
  * The calculated humidity value is assigned to the memory location pointed to by p_humidity.
  * The calculated temperature value is assigned to the memory location pointed to by p_temperature.



