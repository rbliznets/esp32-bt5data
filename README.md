# Classes for organizing BLE data streaming channels for esp32-s3
To add to the project in the components folder from the command line, run:    

    git submodule add https://github.com/rbliznets/esp32-bt5data bt5data

##  `CBTTask` 

This header file (`CBTTask.h`) defines a singleton C++ class `CBTTask` designed to manage Bluetooth Low Energy (BLE) operations on ESP32-S3 microcontrollers using the NimBLE stack. It provides a task-based interface for handling different BLE modes, primarily focused on data exchange and iBeacon functionalities.

**Key Features:**

1.  **Singleton Pattern:** Ensures only one instance of the class manages BLE operations for the entire application.
2.  **Multi-Mode Operation:** Supports turning off BLE, acting as an iBeacon transmitter, scanning for iBeacons/MAC addresses, and operating in a data exchange mode (GATT server).
3.  **Data Streaming Channels:** Offers support for a main data channel and an optional second data channel for concurrent data exchange with a connected client.
4.  **Event-Driven Callbacks:** Uses function pointers to notify the application about incoming data (`onBLEDataRx`), connection status changes (`onBLEConnect`), and discovered iBeacon/MAC addresses (`onBeaconRx`).
5.  **Message Queue:** Internally uses a FreeRTOS queue to handle commands and data between the application and the dedicated BLE task thread.
6.  **Advertising Control:** Allows setting custom manufacturer data in the BLE advertisement payload.
7.  **iBeacon Scanning Management:** Includes optional sleep/wake cycling for the scanner to manage power consumption.

**Core Components:**

*   **`EBTMode`:** Enum defining the operational modes (Off, iBeaconTx, iBeaconRx, Data).
*   **`SBeacon`:** Structure holding iBeacon details (UUID, Major, Minor, Power, RSSI).
*   **`SMac`:** Structure holding a MAC address and its RSSI.
*   **`CBTTask`:** The main class inheriting from `CBaseTask` (likely a FreeRTOS task wrapper) and `CLock` (likely a mutex). It manages the NimBLE stack lifecycle, GATT service/characteristics, and processes internal messages.

**Public Interface:**

*   `Instance()`: Access the singleton instance.
*   `free()`: Deinitialize and destroy the singleton instance.
*   `isRun()`: Check if the task instance exists.
*   `getMode()`: Get the current operational mode.
*   `setBeacon(...)`: Configure and start iBeacon transmission or scanning.
*   `setData(...)`: Configure and start data exchange mode, setting up callbacks.
*   `sendData(...)`: Send data via the main GATT notification/indication.
*   `sendData2(...)`: Send data via the optional second GATT characteristic.
*   `setManufacturerData(...)`: Update the data included in BLE advertisements.

This class abstracts the complexities of the NimBLE API into a task-based, command-driven model suitable for embedded applications requiring BLE data streaming or iBeacon functionality.
