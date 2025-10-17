/*!
	\file
	\brief Task class for BT5 LE.
	\authors Bliznets R.A.(r.bliznets@gmail.com)
	\version 1.0.0.0
	\date 17.11.2023

	One object per application.
*/

#pragma once

#include "sdkconfig.h"
#ifdef CONFIG_BT_NIMBLE_ENABLED

#include "CBaseTask.h"
#include "CLock.h"
#include "CSoftwareTimer.h"

#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include <array>

#ifdef CONFIG_BLE_DATA_IBEACON_TX
#define MSG_INIT_BEACON_TX (10) ///< Initialize iBeacon mode command.
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
#define MSG_INIT_BEACON_RX (11) ///< Initialize iBeacon scanning mode command.
#define MSG_BEACON_DATA (12)	///< Message with iBeacon data.
#define MSG_BEACON_TIMER (13)	///< iBeacon timer message.
#define MSG_MAC_DATA (14)		///< Message with device MAC address.
#endif
#define MSG_INIT_DATA (2)	 ///< Initialize streaming channels mode command.
#define MSG_OFF (3)			 ///< Turn off BT command.
#define MSG_WRITE_DATA (4)	 ///< Message to write data to the main channel.
#define MSG_READ_DATA (5)	 ///< Message to read data from the main channel.
#define MSG_SET_ADV_DATA (6) ///< Set data for the Manufacturer specific data field in advertising.

#define MSG_INIT_DATA3 (15) ///< Set callback function for connection command.
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
#define MSG_READ_DATA2 (16)	 ///< Message to read data from the second channel.
#define MSG_INIT_DATA2 (17)	 ///< Set callback function for receiving data from the second channel command.
#define MSG_WRITE_DATA2 (18) ///< Message to write data to the second channel.
#define MSG_SKIP_WRITE (19)	 ///< Cancel write to the second channel command.
#endif

#define BTTASK_NAME "bt"			///< Task name for debugging.
#define BTTASK_STACKSIZE (4 * 1024) ///< Task stack size.
#define BTTASK_PRIOR (2)			///< Task priority.
#define BTTASK_LENGTH (30)			///< Task receive queue length.
#ifdef CONFIG_BLE_DATA_TASK0
#define BTTASK_CPU (0) ///< CPU core number.
#else
#define BTTASK_CPU (1) ///< CPU core number.
#endif

/// BT operation modes.
enum class EBTMode
{
	Off, ///< Off.
#ifdef CONFIG_BLE_DATA_IBEACON_TX
	iBeaconTx, ///< iBeacon transmitter mode.
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
	iBeaconRx, ///< iBeacon receiver mode.
#endif
	Data ///< Data exchange mode.
};

/**
 * @brief iBeacon data structure
 *
 * Contains information about an iBeacon, including UUID, major, minor numbers,
 * signal power, and received signal strength indicator (RSSI).
 */
struct SBeacon
{
	std::array<uint8_t, 16> uuid; ///< Beacon's unique identifier (16 bytes)
	uint16_t major;				  ///< Major number (beacon group)
	uint16_t minor;				  ///< Minor number (specific beacon in the group)
	int8_t power;				  ///< Transmitter power at 1 meter distance
	int8_t rssi;				  ///< Received signal strength indicator

	/**
	 * @brief Operator to compare two beacons
	 * @param other Second beacon for comparison
	 * @return true if UUIDs match, false otherwise
	 */
	bool operator==(const SBeacon &other) const
	{
		return (this->uuid == other.uuid) && (this->major == other.major) && (this->minor == other.minor) && (this->rssi == other.rssi);
	}
};

/**
 * @brief Bluetooth device MAC address structure
 *
 * Contains the device's MAC address and received signal strength.
 */
struct SMac
{
	std::array<uint8_t, 6> mac; ///< Device MAC address (6 bytes)
	int8_t rssi;				///< Received signal strength

	/**
	 * @brief Operator to compare two MAC addresses
	 * @param other Second MAC address for comparison
	 * @return true if MAC addresses match, false otherwise
	 */
	bool operator==(const SMac &other) const
	{
		return this->mac == other.mac;
	}
};

/// Data reception event function.
/*!
 * \param[in] data data.
 * \param[in] size data size.
 */
typedef void onBLEDataRx(uint8_t *data, size_t size);

/**
 * @brief Callback function for processing iBeacon data
 *
 * Called when an iBeacon or a Bluetooth device MAC address is detected.
 *
 * @param[in] data Pointer to the iBeacon data structure (can be nullptr)
 * @param[in] mac Pointer to the MAC address structure (can be nullptr)
 */
typedef void onBeaconRx(SBeacon *data, SMac *mac);

/**
 * @brief Callback function for processing connection events
 *
 * Called when a Bluetooth connection is established or disconnected.
 *
 * @param[in] connected true if the connection is established, false if disconnected
 */
typedef void onBLEConnect(bool connected);

/// BLE data channel logic class.
class CBTTask : public CBaseTask, CLock
{
private:
	static CBTTask *theSingleInstance;	///< The single object.
	static const ble_uuid16_t svs_uuid; ///< Service UUID.
	static const ble_uuid16_t chr_uuid; ///< Main channel UUID.
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	static const ble_uuid16_t chr_uuid2; ///< Second channel UUID.
#endif

	static const struct ble_gatt_svc_def gatt_svr_svcs[]; ///< Service configuration.
	static const struct ble_gatt_chr_def gatt_svr_chrs[]; ///< Service characteristics configuration.

protected:
	EBTMode mMode = EBTMode::Off; ///< Current operation mode.
	bool mConnect = false;		  ///< Connection flag.
	onBLEDataRx *mOnRx = nullptr; ///< Callback function for receiving data on the main channel.
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	onBLEDataRx *mOnRx2 = nullptr; ///< Callback function for receiving data on the second channel.
#endif
	onBLEConnect *mOnConnect = nullptr; ///< Callback function for connection events.

	uint8_t own_addr_type; ///< BLE address type.

	/// Set operation mode.
	/*!
		Initializes Nimble
	  \param[in] mode operation mode.
	  \return current operation mode
	*/
	EBTMode init_bt(EBTMode mode);

	/// Turn off.
	/*!
		Disables Nimble
	*/
	void deinit_bt();

#ifdef CONFIG_BT_NIMBLE_EXT_ADV
	ble_addr_t mAddr = {0, {0, 0, 0, 0, 0, 0}}; ///< Address for extended advertising
#endif

#ifdef CONFIG_BLE_DATA_IBEACON_TX
	uint8_t mBeaconID[16];	   ///< ID field. Retrieved from nvs "beacon" (blob) or random.
	uint16_t mBeaconMajor = 0; ///< Major field. Set during mode initialization.
	uint16_t mBeaconMinor = 0; ///< Minor field. Set during mode initialization.
	uint8_t mBeaconTx = 0;	   ///< Power at 1m field. Retrieved from nvs "btx" (u8).

	/// Stack sync callback (iBeacon).
	/*!
	 * @brief Stack synchronization callback for iBeacon transmitter mode
	 *
	 * Called when the BLE stack synchronizes in iBeacon transmitter mode.
	 */
	static void ble_on_sync_beacon();

	/// Begin advertising (iBeacon).
	/*!
	 * @brief Start advertising in iBeacon mode
	 *
	 * Starts transmitting advertisement packets in iBeacon format.
	 */
	static void ble_advertise_beacon();

	/// Set random address (iBeacon).
	/*!
	 * @brief Set random device address for iBeacon
	 *
	 * Generates and sets a random non-resolvable private address.
	 */
	static void ble_app_set_addr();
#endif

#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
	onBeaconRx *mOnBeacon = nullptr;		///< Callback function for receiving beacon data.
	uint32_t mBeaconSleepTime = 5000;		///< Sleep time between scans (ms)
	CSoftwareTimer *mBeaconTimer = nullptr; ///< Timer for controlling scanning
	bool mBeaconSleep = false;				///< Scanner sleep mode flag
	bool mBeaconFilter = true;

	/**
	 * @brief Stack synchronization callback for iBeacon receiver mode
	 *
	 * Called when the BLE stack synchronizes in iBeacon receiver mode.
	 */
	static void ble_on_sync_rx();

	/**
	 * @brief Start scanning for BLE devices
	 *
	 * Initiates the process of scanning for nearby BLE devices.
	 */
	static void ble_scan();

	/**
	 * @brief GAP event handler for scanning
	 *
	 * Processes events for discovered BLE devices and iBeacon beacons.
	 *
	 * @param[in] event Pointer to the GAP event structure
	 * @param[in] arg Additional arguments (unused)
	 * @return Return code (0 on successful processing)
	 */
	static int ble_rx_gap_event(struct ble_gap_event *event, void *arg);
#endif

	uint8_t *mManufacturerData = nullptr; ///< Manufacturer data for advertising
	uint8_t mManufacturerDataSize = 0;	  ///< Size of manufacturer data

	/// Start Nimble.
	/*!
	  \param[in] param Nimble.
	*/
	static void ble_host_task(void *param);

	/// Stack reset callback.
	/*!
	  \param[in] reason Reason code for reset.
	*/
	static void ble_on_reset(int reason);

	/// Stack sync callback.
	/*!
	 * @brief Stack synchronization callback for data transmission mode
	 *
	 * Called when the BLE stack synchronizes in data transmission mode.
	 */
	static void ble_on_sync_data();

	/// Begin advertising.
	/*!
	 * @brief Start advertising in data transmission mode
	 *
	 * Starts transmitting advertisement packets in data transmission mode.
	 */
	static void ble_advertise_data();

	/// GAP callback.
	/*!
	 * @brief GAP event handler for the data server
	 *
	 * Processes connection, disconnection, and other GAP events.
	 *
	 * @param[in] event Pointer to the GAP event structure
	 * @param[in] arg Additional arguments (unused)
	 * @return Return code (0 on successful processing)
	 */
	static int ble_server_gap_event(struct ble_gap_event *event, void *arg);

	/// Initialize service.
	/*!
	 * @brief Initialize the GATT service for data transmission
	 *
	 * Registers the custom GATT service and its characteristics.
	 *
	 * @return Return code (0 on successful initialization)
	 */
	static int gatt_svr_init();

	/// Callback function for the main channel service.
	/*!
	 * @brief Main channel characteristic access handler
	 *
	 * Handles read/write operations for the main data channel.
	 *
	 * @param[in] conn_handle Connection identifier
	 * @param[in] attr_handle Attribute identifier
	 * @param[in] ctxt GATT access context
	 * @param[in] arg Additional arguments
	 * @return Return code (0 on successful processing)
	 */
	static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	/// Callback function for the second channel service.
	/*!
	 * @brief Second channel characteristic access handler
	 *
	 * Handles read/write operations for the second data channel.
	 *
	 * @param[in] conn_handle Connection identifier
	 * @param[in] attr_handle Attribute identifier
	 * @param[in] ctxt GATT access context
	 * @param[in] arg Additional arguments
	 * @return Return code (0 on successful processing)
	 */
	static int ble_svc_gatt_handler2(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
#endif

	/// Callback function for reading data from the channel.
	/*!
	  \param[in] om data.
	  \param[in] chn channel number.
	  \return 0 if no error.
	*/
	static int gatt_svr_chr_write(struct os_mbuf *om, uint16_t chn = 1);

	/// Constructor.
	/*!
	 * @brief CBTTask class constructor
	 *
	 * Creates a new instance of the Bluetooth task class.
	 */
	CBTTask();

	/// Destructor.
	/*!
	 * @brief CBTTask class destructor
	 *
	 * Frees resources used by the Bluetooth task.
	 */
	virtual ~CBTTask();

	/// Task function.
	/*!
	 * @brief Main function for executing the Bluetooth task
	 *
	 * Processes all Bluetooth messages and events in the task loop.
	 */
	virtual void run() override;

	using CBaseTask::sendCmd;

public:
	static const char *device_name; ///< Bluetooth device name

	inline EBTMode getMode() { return mMode; };

	/// The single class instance.
	/*!
	  \return Pointer to CBTTask
	*/
	static CBTTask *Instance();

	/// Free resources.
	/*!
	 * @brief Free resources and delete the class instance
	 *
	 * Terminates the Bluetooth task and frees all associated resources.
	 */
	static void free();

	/**
	 * @brief Check if the Bluetooth task is running
	 *
	 * @return true if the task is running, false otherwise
	 */
	static inline bool isRun() { return (theSingleInstance != nullptr); };

#ifdef CONFIG_BLE_DATA_IBEACON_TX
	/// Enable iBeacon mode.
	/*!
	  \param[in] major major field.
	  \param[in] minor minor field.
	  \return true if no error.
	*/
	inline bool setBeacon(uint16_t major, uint16_t minor)
	{
		return sendCmd(MSG_INIT_BEACON_TX, major, minor);
	};
#endif

#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
	/**
	 * @brief Set iBeacon scanning mode
	 *
	 * @param[in] onBeacon Callback function for processing detected beacons
	 * @param[in] sleep Sleep time between scans in seconds
	 * @return true if the command is sent successfully
	 */
	inline bool setBeacon(onBeaconRx *onBeacon, uint16_t sleep = 5, bool filter = true)
	{
		if (filter)
			sleep |= 0x80;
		return sendCmd(MSG_INIT_BEACON_RX, sleep, (uint32_t)onBeacon);
	};
#endif

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	/// Enable data channels mode.
	/*!
	  \param[in] onRx callback for receiving data on the main channel.
	  \param[in] onRx2 callback for receiving data on the second channel.
	  \return true if no error.
	*/
	inline bool setData(onBLEDataRx *onRx, onBLEDataRx *onRx2, onBLEConnect *onConnect = nullptr)
	{
		sendCmd(MSG_INIT_DATA3, 0, (uint32_t)onConnect);
		sendCmd(MSG_INIT_DATA2, 0, (uint32_t)onRx2);
		return sendCmd(MSG_INIT_DATA, 0, (uint32_t)onRx);
	};

	/// Send data to the second channel.
	/*!
	  \param[in] data data.
	  \param[in] size data size.
	  \param[in] index data packet number.
	  \param[in] xTicksToWait message queue timeout time.
	  \return true if no error.
	*/
	bool sendData2(uint8_t *data, size_t size, uint16_t index, TickType_t xTicksToWait = portMAX_DELAY);
#else
	/// Enable data channel mode.
	/*!
	  \param[in] onRx callback for receiving data on the main channel.
	  \return true if no error.
	*/
	inline bool setData(onBLEDataRx *onRx, onBLEConnect *onConnect = nullptr)
	{
		sendCmd(MSG_INIT_DATA3, 0, (uint32_t)onConnect);
		return sendCmd(MSG_INIT_DATA, 0, (uint32_t)onRx);
	};
#endif

	/// Send data to the main channel.
	/*!
	  \param[in] data data.
	  \param[in] size data size.
	  \param[in] xTicksToWait message queue timeout time.
	  \return true if no error.
	*/
	bool sendData(uint8_t *data, size_t size, TickType_t xTicksToWait = portMAX_DELAY);

	/// Set data for advertising.
	/*!
	  \param[in] data data.
	  \param[in] size data size.
	  \param[in] xTicksToWait message queue timeout time.
	  \return true if no error.
	*/
	bool setManufacturerData(uint8_t *data, size_t size, TickType_t xTicksToWait = portMAX_DELAY);
};
#endif