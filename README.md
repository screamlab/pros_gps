# pros_gps

This repository is designed to interface with the BU-353N5 USB GPS device and generate maps for user applications. It provides tools to set up and run a GPS node using ROS2, facilitating easy integration and data acquisition.

## Overview
* Device: [BU-353N5](https://www.globalsat.com.tw/en/product-282242/USB-GNSS-Receiver-BU-353N5.html) 
* Purpose: Capture GPS data ans create maps within a ROS2 framework.

## Prerequisites
* Linux System
* [screamlab/tools](https://github.com/screamlab/tools) cloned locally (for udev rule generation)

## Setup environment
To ensure consistent device recognition, we'll create a persistent symbolic link (`/dev/usb_gps`) to the GPS device using rules.

1. Identify the GPS Device Name
    
    Plug in your BU-353N5 GPS device and determine its device name:
    ```
    ls /dev/ttyUSB*
    ```
    Look for a device like `/dev/ttyUSB0`. This is your GPS device.

2. Obtain `idVendor` and `idProduct`
    
    Navigate to the `screamlab/tools` directory:
    ```
    cd /path/to/screamlab/tools
    ```
    Run the `extract_usb_info.sh` script with your device name:
    ```
    ./extract_usb_info.sh /dev/ttyUSB0
    # Replace /dev/ttyUSB0 with your actual device name if different
    ```
    The script will output:
    ```
    idVendor: 067b
    idProduct: 23a3
    ```

3. Create a Udev Rule
   
    We'll create a udev rule to assign a consistent name to your GPS device.

    Option A: Manually Create the Udev Rule
    
    1. Create a new udev rule file:
        ```
        sudo nano /etc/udev/rules.d/99-usb_gps.rules
        ``` 
    
    2. Add the following line, replacing 067b and 23a3 with your actual values if they differ:
        ```
        SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23a3", SYMLINK+="usb_gps", MODE="0666"
        ```
    3. Save and close the file.

    Option B: Use `device_rule_generate.sh`

    1. Write the udev rule into  `device_rule_generate.sh` with your actual values if they differ:
        ```
        SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23a3", SYMLINK+="usb_gps", MODE="0666"
        ```
    2. Run it:
        ```
        sudo ./device_rule_generate.sh
        ```



## Get gps informatation

To set up and run the GPS node within ROS2, execute the following script:
```
./run.sh
```

This script will:
* Initialize the ROS2 environment.
* Launch the GPS node that reads data from /dev/usb_gps.
* Publish GPS data to the appropriate ROS2 topics.

## Troubleshooting

* Device Not Recognized:
    * Re-run ls /dev/ttyUSB* to check if the device is detected.
    * Double-check the idVendor and idProduct values.
