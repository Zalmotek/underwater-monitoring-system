# RA6M5 Devkit Fusion Inference
This guide will describe the steps taken to get an Edge Impulse sensor fusion model for motor health applications working on the Reloc RA6M5 Devkit development board. It combines accelerometer information from a BNO055 IMU Sensor, temperature data from a HS300X environmental sensor, and the RMS Current measured by a non invasive current clamp.
​
## Architecture
This example splits inference and sensor acquisition into two separate threads that communicate by using a queue. For now the example is very basic, with a one-element queue, but allows for further scalability. For example having a single data aquisition thread, and multiple inference threads, running in parallel, or semi-sequentially. 
​
#### Data acquisition thread 
The data acquisition thread connects to aforementioned sensors and wraps the data into the `sensorFrame` struct.
​
If the data read from the sensor is successful, data gets sent trough the `sensorQueue` to be consumed by the inference thread. 
​
Our example is running a classifier the fused data points and tries to fit samples into the **ON** and **OFF** categories. Additionally it also computes an anomaly score
```c
struct sensorFrame {
    float accX = 0.00;
    float accY = 0.00;
    float accZ = 0.00;
    float current = 0.00;
    float temperature = 0.00;
};
```

#### Inference thread
The Inference thread checks if there is any space available in the queue `sensorsQueue` and if there is, it receives the data from the queue and stores it in the global feature array. It then checks if the feature array is filled and if it is it calls the inference function and reset the feature_ix.
The `inference()` function is then defined. This function uses the Edge Impulse SDK to create a signal from the "features" array, run the classifier on this signal, and print the results to the serial port. It creates a struct `ei_impulse_result_t` named result, creates a signal from the features frame by calling `numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, & signal)` and then runs the classifier with the `run_classifier( & signal, & result, true)` call. It then prints the return code from the classifier, how long it took to perform inference and the prediction results on the serial port.
​
## Prerequisites & Configuration
### 1. Heap & Stack Size
Both the inference functionality and calling `printf` on floats, abuse the stack and heap quite a lot. That's why we need to increase both the heap size and the stack sizes for our threads. 
​
First, go to the BSP Tab in the **FSP Configuration** and under **RA Common** set the **Main stack size** and **Heap size** to 0x4000 (16384) bytes.

![](https://i.imgur.com/PU8uxHz.png)

Now, for each thread that runs inference and/or uses `printf`, change the stack and heap size too. 

![](https://i.imgur.com/nymDczx.png)

Feel free to tweak the settings to your liking, but these settings have worked for me.

### 2. Printing floats 
This example includes some custom wrappers i have written for accessing USB, UART and I2C interfaces, code that is mainly based on the Reloc examples. 
​
This simplifies access to USB serial, and allows us to `printf` to the console with a variable number of arguments. Under the hood, this uses `vsnprintf`. 
​
Due to the fact that `printf` is not re-entrant and uses a lot of heap space for floats, we need to do some configurations. 
​
#### 2.1. Enable newlib-nano
Go to **Project Settings > C/C++ Build > Settings > GNU Arm Cross C++ Linker > Miscellaneous** and enable :
* **Use newlib-nano** 
* **Use float with nano printf**

![](https://i.imgur.com/zvIm5Yj.png)
 
#### 2.2. Enable Newlib Reentrant 
For the thread that prints floats we also need to enable **Use Newlib Reentrant** on the **Stacks** page of **FSP Configuration** and selecting the thread we are interested in. 

![](https://i.imgur.com/H8TrWH4.png)

### 3. Configure the Renesas E2 Environment to work with Edge Impulse
Next up, we need to configure the E2 environment to compile and link with the **Edge Impulse SDK**. 
​
First off, download your model from Edge Impulse, unzip the archive and move the three folders:
**edge-impulse-sdk**, **model-parameters** and **tflite-model**, to the root of your solution in the **/src** folder. 
​
Go to **Project Settings > C/C++ Build > Settings > C/C++ General > Paths and Symbols > Symbols**, add **EI_PORTING_RENESASRA65**, and set it's value to 1.
​
![](https://i.imgur.com/7bsnLra.png)
​
### 4. Modifications to the Edge Impulse SDK port for RA6M5

​
Go to `<your_project_name>/src/edge-impulse-sdk/porting/renesas-ra6m5/ei_classifier_porting.cpp`  and paste the following code: 
```c 
//#include <fsp_common_api.h>
/* Includes */
#include "../ei_classifier_porting.h"
​
#if EI_PORTING_RENESASRA65 == 1
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
​
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cstdio>
#include <cstdarg>
​
#include "hal_data.h"
​
#include "unistd.h"
//#include "peripheral/uart_ep.h"
#include <math.h>
​
​
// extern "C" void Serial_Out(char *string, int length);
extern "C" uint64_t timer_get_ms(void);
//extern "C" fsp_err_t uart_print_user_msg(uint8_t *p_msg);
​
__attribute__((weak)) EI_IMPULSE_ERROR ei_run_impulse_check_canceled() {
    return EI_IMPULSE_OK;
}
​
/**
 * Cancelable sleep, can be triggered with signal from other thread
 */
__attribute__((weak)) EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
​
    uint64_t start_time = ei_read_timer_ms();
​
    while(start_time + time_ms > ei_read_timer_ms()){};
​
    return EI_IMPULSE_OK;
}
​
uint64_t ei_read_timer_ms() {
​
    return xTaskGetTickCount() / configTICK_RATE_HZ;//Timer_getMs();
}
​
uint64_t ei_read_timer_us() {
    
    return  (xTaskGetTickCount()/ configTICK_RATE_HZ)*1000;
}
​
__attribute__((weak)) void ei_printf(const char *format, ...) {
​
    char buffer[256] = {0};
    int length;
    va_list myargs;
    va_start(myargs, format);
    length = vsnprintf(buffer, sizeof(buffer), format, myargs);
    va_end(myargs);
​
    if (length > 0){
        //uart_print_user_msg((uint8_t *)buffer);
    }
    
}
​
__attribute__((weak)) void ei_printf_float(float f) {
    float n = f;
​
    static double PRECISION = 0.00001;
    static int MAX_NUMBER_STRING_SIZE = 32;
​
    char s[MAX_NUMBER_STRING_SIZE];
​
    if (n == 0.0) {
        strcpy(s, "0");
    }
    else {
        int digit, m;
        char *c = s;
        int neg = (n < 0);
        if (neg) {
            n = -n;
        }
        // calculate magnitude
        m = log10(n);
        if (neg) {
            *(c++) = '-';
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0) {
                *(c++) = '.';
            }
            m--;
        }
        *(c) = '\0';
    }
​
    ei_printf("%s", s);
}
​
/**
 *
 * @param c
 */
void ei_putchar(char c)
{
    ei_printf("%c", c);
}
​
__attribute__((weak)) void *ei_malloc(size_t size) {
    return malloc(size);
}
​
__attribute__((weak)) void *ei_calloc(size_t nitems, size_t size) {
    return calloc(nitems, size);
}
​
__attribute__((weak)) void ei_free(void *ptr) {
    free(ptr);
}
​
#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C"
#endif
__attribute__((weak)) void DebugLog(const char* s) {
    ei_printf("%s", s);
}
​
#endif // EI_PORTING_RENESASRA65 == 1
```
### 5. Add a global queue. 
The last configuration step we need to take is to create the queue that we'll use to communicate between threads. Go to the **Stacks** tab of the **FSP Configuration** environment, and create a new object. 
![](https://i.imgur.com/3iU5KAn.png)

It's really important to set it's size accordingly. In our case, the struct we are sending trough the queue has 5 floats, so the **Item Size** should be set to **20 bytes**.
