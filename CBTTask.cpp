/*!
    \file
    \brief Task class for Bluetooth logic.
    \authors Bliznets R.A.(r.bliznets@gmail.com)
    \version 1.0.0.0
    \date 17.10.2023

    One object per application.
*/

#include "CBTTask.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_gap.h"

#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "console/console.h"

#include "nvs.h"
#include "esp_random.h"

#include "CTrace.h"
#include <cstring>

#ifdef CONFIG_ESP_TASK_WDT
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY
#endif

extern "C" void ble_store_config_init(void);

// Handle for reading the value of the SPP service characteristic
static uint16_t ble_spp_svc_gatt_read_val_handle = 0x30;

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16 0xABF0
/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16 0xABF1

const ble_uuid16_t CBTTask::svs_uuid = BLE_UUID16_INIT(BLE_SVC_SPP_UUID16);
const ble_uuid16_t CBTTask::chr_uuid = BLE_UUID16_INIT(BLE_SVC_SPP_CHR_UUID16);

// Second data channel (if enabled in configuration)
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
const ble_uuid16_t CBTTask::chr_uuid2 = BLE_UUID16_INIT(BLE_SVC_SPP_CHR_UUID16 + 1);
static uint16_t ble_spp_svc_gatt_read_val_handle2 = 0x31;
#endif

// Definition of GATT service characteristics
const struct ble_gatt_chr_def CBTTask::gatt_svr_chrs[] = {
    {
        /* SPP service support */
        .uuid = (ble_uuid_t *)&chr_uuid,
        .access_cb = ble_svc_gatt_handler,                     // Callback for handling characteristic access
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY, // Write and notify allowed
        .val_handle = &ble_spp_svc_gatt_read_val_handle,       // Pointer to the value handle
    },
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    {
        .uuid = (ble_uuid_t *)&chr_uuid2,
        .access_cb = ble_svc_gatt_handler2, // Callback for second channel
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &ble_spp_svc_gatt_read_val_handle2,
    },
#endif
    {
        0, /* No more characteristics in this service */
    }};

// Definition of GATT service
const struct ble_gatt_svc_def CBTTask::gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY, // Primary service
        .uuid = (ble_uuid_t *)&svs_uuid,   // Service UUID
        .characteristics = gatt_svr_chrs   // Array of characteristics
    },
    {
        0, /* No more services */
    }};

static const char *TAG = "BTTask"; // Tag for logging

const char *CBTTask::device_name = CONFIG_BLE_DATA_DEVICE_NAME; // Device name from configuration

/**
 * @brief GATT characteristic write handler
 * @param om Pointer to the data buffer
 * @param chn Channel number (1 or 2)
 * @return BLE error code
 */
int CBTTask::gatt_svr_chr_write(struct os_mbuf *om, uint16_t chn)
{
    uint16_t om_len;  // Length of received data
    int rc;           // Return code
    STaskMessage msg; // Message to send to the task

    om_len = OS_MBUF_PKTLEN(om); // Get the packet length
    if (om_len < 1)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN; // Error: empty packet
    }

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    // Determine message type based on channel
    if (chn == 2)
        allocNewMsg(&msg, MSG_READ_DATA2, om_len, true); // Second channel
    else
#endif
        allocNewMsg(&msg, MSG_READ_DATA, om_len, true); // First channel

    // Copy data from the BLE buffer to our message
    rc = ble_hs_mbuf_to_flat(om, msg.msgBody, om_len, nullptr);
    if (rc != 0)
    {
        return BLE_ATT_ERR_UNLIKELY; // Copy error
    }

    // Send message to the Bluetooth task
    CBTTask::Instance()->sendMessage(&msg, portMAX_DELAY, true);
    return 0;
}

/**
 * @brief GATT characteristic access event handler (channel 1)
 * @param conn_handle Connection identifier
 * @param attr_handle Attribute identifier
 * @param ctxt Access context
 * @param arg Additional arguments
 * @return Error code
 */
int CBTTask::ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // Handle write operation to the characteristic
        rc = gatt_svr_chr_write(ctxt->om);
        return rc;
    default:
        return BLE_ATT_ERR_UNLIKELY; // Unknown operation
    }
    return 0;
}

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
/**
 * @brief GATT characteristic access event handler (channel 2)
 * @param conn_handle Connection identifier
 * @param attr_handle Attribute identifier
 * @param ctxt Access context
 * @param arg Additional arguments
 * @return Error code
 */
int CBTTask::ble_svc_gatt_handler2(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // Handle write operation to the second characteristic
        rc = gatt_svr_chr_write(ctxt->om, 2);
        return rc;
    default:
        return BLE_ATT_ERR_UNLIKELY; // Unknown operation
    }
    return 0;
}
#endif

// Pointer to the single instance of the class
CBTTask *CBTTask::theSingleInstance = nullptr;

/**
 * @brief Get the single instance of the class (Singleton)
 * @return Pointer to the CBTTask instance
 */
CBTTask *CBTTask::Instance()
{
    if (theSingleInstance == nullptr)
    {
        // Create a new instance if it doesn't exist yet
        theSingleInstance = new CBTTask();
        // Initialize the base task
        theSingleInstance->CBaseTask::init(BTTASK_NAME, BTTASK_STACKSIZE, BTTASK_PRIOR, BTTASK_LENGTH, BTTASK_CPU);
    }
    return theSingleInstance;
};

/**
 * @brief Free resources and delete the instance
 */
void CBTTask::free()
{
    if (theSingleInstance != nullptr)
    {
        // Send a command to terminate the task
        theSingleInstance->sendCmd(MSG_END_TASK);
        // Wait for the task to finish
        do
        {
            vTaskDelay(1);
        }
#if (INCLUDE_vTaskDelete == 1)
        while (theSingleInstance->mTaskHandle != nullptr);
#else
        while (theSingleInstance->mTaskQueue != nullptr);
#endif
        // Delete the instance
        delete theSingleInstance;
        theSingleInstance = nullptr;
    }
}

/**
 * @brief Constructor for the class
 */
CBTTask::CBTTask() : CBaseTask(), CLock()
{
    // Initialize class members with default values
    mManufacturerData = nullptr;
    mManufacturerDataSize = 0;
    mMode = EBTMode::Off;
    mConnect = false;
    mOnRx = nullptr;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    mOnRx2 = nullptr;
#endif
    mOnConnect = nullptr;
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
    mOnBeacon = nullptr;
    mBeaconTimer = nullptr;
    mBeaconSleep = false;
    mBeaconSleepTime = 0;
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_TX
    mBeaconTx = 0;
    mBeaconMajor = 0;
    mBeaconMinor = 0;
#endif
}

/**
 * @brief Destructor for the class
 */
CBTTask::~CBTTask()
{
}

/**
 * @brief BLE stack reset handler
 * @param reason Reason for the reset
 */
void CBTTask::ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
}

#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
/**
 * @brief Synchronization handler for iBeacon scanning mode
 */
void CBTTask::ble_on_sync_rx()
{
    ESP_LOGD(TAG, "BLE rx");
    int rc;

    /* Ensure the correct device address is set (preferably public) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_util_ensure_addr; rc=%d", rc);
        return;
    }

    /* Start scanning to find peripheral devices */
    ble_scan();
}

/**
 * @brief Start BLE device scanning
 */
void CBTTask::ble_scan()
{
    uint8_t own_addr_type; // Type of the own device address
    int rc;

    /* Determine the address type to use for advertising (without privacy) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

#ifdef CONFIG_BT_NIMBLE_EXT_ADV
    // Extended scanning parameters
    struct ble_gap_ext_disc_params disc_params;
    disc_params.passive = 1; // Passive scanning

    /* Use default values for other parameters */
    disc_params.itvl = 0;   // BLE_GAP_SCAN_ITVL_MS(600);
    disc_params.window = 0; // BLE_GAP_SCAN_ITVL_MS(300);

    // Start extended scanning
    rc = ble_gap_ext_disc(own_addr_type, 0, 0, 1, 0, 0,
                          &disc_params, &disc_params, ble_rx_gap_event, NULL);
    assert(rc == 0);

#else
    // Standard scanning parameters
    struct ble_gap_disc_params disc_params;
    /* Filter duplicates - do not process repeated advertisements from one device */
    disc_params.filter_duplicates = 1;

    /**
     * Perform passive scanning. I.e., do not send scan requests
     * to each advertiser.
     */
    disc_params.passive = 1;

    /* Use default values for other parameters */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    // Start standard scanning
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      ble_rx_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d", rc);
    }
#endif
}

/**
 * @brief GAP event handler for scanning
 * @param event GAP event
 * @param arg Additional arguments
 * @return Return code
 */
int CBTTask::ble_rx_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields; // Advertising data fields
    int rc;
    STaskMessage msg;
    SBeacon *beacon; // Structure for iBeacon data
    SMac *mac;       // Structure for MAC address

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC_COMPLETE:
        // Scanning completed
        return 0;

#ifdef CONFIG_BT_NIMBLE_EXT_ADV
    case BLE_GAP_EVENT_EXT_DISC:
        // Extended device discovery
        if (event->ext_disc.legacy_event_type == BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND)
        {
            // Parse advertising fields
            rc = ble_hs_adv_parse_fields(&fields, event->ext_disc.data,
                                         event->ext_disc.length_data);
            if (rc == 0)
            {
                // Check if the device is an iBeacon
                if ((fields.mfg_data_len == 25) && (fields.mfg_data[0] == 0x4c) && (fields.mfg_data[1] == 0) && (fields.mfg_data[2] == 0x02) && (fields.mfg_data[3] == 0x15))
                {
                    // Create a message with iBeacon data
                    beacon = (SBeacon *)allocNewMsg(&msg, MSG_BEACON_DATA, sizeof(SBeacon), true);
                    std::memcpy(beacon->uuid, &fields.mfg_data[4], 16);              // Copy UUID
                    beacon->major = fields.mfg_data[21] + fields.mfg_data[20] * 256; // Major
                    beacon->minor = fields.mfg_data[23] + fields.mfg_data[22] * 256; // Minor
                    beacon->power = fields.mfg_data[24];                             // Power
                    beacon->rssi = event->ext_disc.rssi;                             // RSSI
                    CBTTask::Instance()->sendMessage(&msg, 10, true);                // Send message
                    return 0;
                }
            }
        }
        // Process public device MAC addresses
        if (event->ext_disc.addr.type == BLE_ADDR_PUBLIC)
        {
            mac = (SMac *)allocNewMsg(&msg, MSG_MAC_DATA, sizeof(SMac), true);
            std::memcpy(mac->mac, event->ext_disc.addr.val, 6); // Copy MAC
            CBTTask::Instance()->sendMessage(&msg, 10, true);
            mac->rssi = event->ext_disc.rssi; // RSSI
        }
        return 0;
#else
    case BLE_GAP_EVENT_DISC:
        // Standard device discovery
        // ESP_LOGW(TAG,"rssi %d, type %d",event->disc.rssi, event->disc.event_type);
        if (CBTTask::Instance()->mBeaconFilter)
        {
            // Parse advertising fields
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
            if (rc == 0)
            {
                // ESP_LOG_BUFFER_HEX("mfg", fields.mfg_data, fields.mfg_data_len);
                // Check if the device is an iBeacon
                if ((fields.mfg_data_len == 25) && (fields.mfg_data[0] == 0x4c) && (fields.mfg_data[1] == 0) && (fields.mfg_data[2] == 0x02) && (fields.mfg_data[3] == 0x15))
                {
                    // Create a message with iBeacon data
                    beacon = (SBeacon *)allocNewMsg(&msg, MSG_BEACON_DATA, sizeof(SBeacon), true);
                    std::memcpy(beacon->uuid.data(), &fields.mfg_data[4], 16);       // Copy UUID
                    beacon->major = fields.mfg_data[21] + fields.mfg_data[20] * 256; // Major
                    beacon->minor = fields.mfg_data[23] + fields.mfg_data[22] * 256; // Minor
                    beacon->power = fields.mfg_data[24];                             // Power
                    beacon->rssi = event->disc.rssi;                                 // RSSI
                    CBTTask::Instance()->sendMessage(&msg, 10, true);                // Send message
                    return 0;
                }
            }
        }
        // Process public device MAC addresses
        if (event->disc.addr.type == BLE_ADDR_PUBLIC)
        {
            // ESP_LOG_BUFFER_HEX("mac", event->disc.addr.val, 6);
            mac = (SMac *)allocNewMsg(&msg, MSG_MAC_DATA, sizeof(SMac), true);
            std::memcpy(mac->mac.data(), event->disc.addr.val, 6); // Copy MAC
            CBTTask::Instance()->sendMessage(&msg, 10, true);
            mac->rssi = event->disc.rssi; // RSSI
        }
        return 0;
#endif

    default:
        return 0;
    }
}
#endif

#ifdef CONFIG_BLE_DATA_IBEACON_TX
/**
 * @brief Synchronization handler for iBeacon transmission mode
 */
void CBTTask::ble_on_sync_beacon()
{
    ESP_LOGD(TAG, "BLE sync beacon");
    /* Generate a non-resolvable private address */
    ble_app_set_addr();
    /* Start advertising indefinitely */
    ble_advertise_beacon();
}

/**
 * @brief Set the BLE device address
 */
void CBTTask::ble_app_set_addr()
{
    ble_addr_t addr; // Address structure
    int rc;

    // Generate a random address
    rc = ble_hs_id_gen_rnd(1, &addr);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_id_gen_rnd; rc=%d", rc);
        return;
    }

    // Set the random address
    rc = ble_hs_id_set_rnd(addr.val);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_app_set_addr; rc=%d", rc);
        return;
    }
}

/**
 * @brief Start iBeacon advertising
 */
void CBTTask::ble_advertise_beacon()
{
    struct ble_gap_adv_params adv_params; // Advertising parameters
    int rc;

    // Set the advertising data for iBeacon
    rc = ble_ibeacon_set_adv_data(CBTTask::Instance()->mBeaconID,
                                  CBTTask::Instance()->mBeaconMajor,
                                  CBTTask::Instance()->mBeaconMinor,
                                  CBTTask::Instance()->mBeaconTx);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error setting advertisement iBeacon; rc=%d", rc);
        return;
    }

    /* Start advertising */
    adv_params = (struct ble_gap_adv_params){0};
    rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, nullptr, BLE_HS_FOREVER,
                           &adv_params, nullptr, nullptr);
    if (rc == 0)
        ESP_LOGD(TAG, "iBeacon advertisement started");
    else
        ESP_LOGE(TAG, "error setting advertisement iBeacon; rc=%d", rc);
}
#endif

/**
 * @brief GAP event handler for the data server
 * @param event GAP event
 * @param arg Additional arguments
 * @return Return code
 */
int CBTTask::ble_server_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        // Connection event
        if (event->connect.status != 0)
        {
            ESP_LOGW(TAG, "Connection failed");
            CBTTask::Instance()->mConnect = false;
            /* Connection failed - resume advertising */
            ble_advertise_data();
        }
        else
        {
            ESP_LOGI(TAG, "Connection established");
            CBTTask::Instance()->mConnect = true;
            // Call the connection callback
            if (CBTTask::Instance()->mOnConnect != nullptr)
                CBTTask::Instance()->mOnConnect(true);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        // Disconnection event
        ESP_LOGW(TAG, "disconnect; reason=%d", event->disconnect.reason);
        CBTTask::Instance()->mConnect = false;
        // Call the disconnection callback
        if (CBTTask::Instance()->mOnConnect != nullptr)
            CBTTask::Instance()->mOnConnect(false);
        ble_advertise_data(); // Resume advertising
        return 0;

    default:
        return 0;
    }
}

/**
 * @brief Start data advertising
 */
void CBTTask::ble_advertise_data()
{
#ifdef CONFIG_BT_NIMBLE_EXT_ADV
    int rc;
    struct ble_gap_ext_adv_params params; // Extended advertising parameters
    struct ble_hs_adv_fields adv_fields;  // Advertising data fields
    struct os_mbuf *data;                 // Buffer for advertising data

    /* Set a random (NRPA) address for the instance */
    if (CBTTask::Instance()->mAddr.type == 0)
    {
        rc = ble_hs_id_gen_rnd(1, &(CBTTask::Instance()->mAddr));
        assert(rc == 0);
    }

    /* For periodic advertising, use the instance with non-connectable advertising */
    memset(&params, 0, sizeof(params));

    /* Advertise using a random address */
    params.own_addr_type = BLE_OWN_ADDR_RANDOM;
    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_2M;
    params.sid = 1;
    params.connectable = 1; // Connectable advertising

    const char *name = ble_svc_gap_device_name(); // Get the device name

    CBTTask::Instance()->lock();
    int size = (CBTTask::Instance()->mManufacturerDataSize + strlen(name));
    // Allocate buffer for advertising data
    if (size > 20)
    {
        data = os_msys_get_pkthdr(size + (BLE_HS_ADV_MAX_SZ - 20), 0);
    }
    else
    {
        data = os_msys_get_pkthdr(BLE_HS_ADV_MAX_SZ, 0);
        params.scannable = 1;
        params.legacy_pdu = 1;
    }
    assert(data);

    /* Configure instance 1 */
    rc = ble_gap_ext_adv_configure(1, &params, NULL, CBTTask::ble_server_gap_event, NULL);
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_addr(1, &(CBTTask::Instance()->mAddr));
    assert(rc == 0);

    memset(&adv_fields, 0, sizeof(adv_fields));

    /* Advertise two flags:
     *     o Discoverability in subsequent advertising (general)
     *     o BLE only (BR/EDR not supported).
     */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN |
                       BLE_HS_ADV_F_BREDR_UNSUP;

    adv_fields.name = (const uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    // Set the service UUID
    ble_uuid16_t t[1] = {BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    adv_fields.uuids16 = t;
    adv_fields.num_uuids16 = 1;
    adv_fields.uuids16_is_complete = 1;

    // Set the manufacturer data
    adv_fields.mfg_data = CBTTask::Instance()->mManufacturerData;
    adv_fields.mfg_data_len = CBTTask::Instance()->mManufacturerDataSize;

    rc = ble_hs_adv_set_fields_mbuf(&adv_fields, data);
    CBTTask::Instance()->unlock();
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_data(1, data);
    assert(rc == 0);

    /* Start advertising */
    rc = ble_gap_ext_adv_start(1, 0, 0);
    assert(rc == 0);
#else
    // Standard advertising
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    name = ble_svc_gap_device_name();
    CBTTask::Instance()->lock();
    bool compact = (CBTTask::Instance()->mManufacturerDataSize + strlen(name)) > 16;
    struct ble_hs_adv_fields scan_response_fields;
    memset(&scan_response_fields, 0, sizeof scan_response_fields);

    /**
     *  Set the advertising data:
     *     o Flags (indicate the type of advertising and other general info).
     *     o Transmission power level.
     *     o Device name.
     *     o 16-bit service UUIDs.
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in subsequent advertising (general)
     *     o BLE only (BR/EDR not supported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the transmission power level field should be included */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)name;
    if (compact && (strlen(name) > 11))
    {
        // Compact name in main advertisement
        fields.name_len = 11;
        fields.name_is_complete = 0;

        // Full name in scan response
        scan_response_fields.name = (uint8_t *)name;
        scan_response_fields.name_len = strlen(name);
        scan_response_fields.name_is_complete = 1;
    }
    else
    {
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
    }

    // Set the service UUID
    ble_uuid16_t t[1] = {BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    fields.uuids16 = t;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    fields.mfg_data = CBTTask::Instance()->mManufacturerData;
    if (compact)
    {
        // Compact manufacturer data
        fields.mfg_data_len = 5;
        scan_response_fields.mfg_data = CBTTask::Instance()->mManufacturerData;
        scan_response_fields.name_len = CBTTask::Instance()->mManufacturerDataSize;

        rc = ble_gap_adv_rsp_set_fields(&scan_response_fields);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "error setting response data; rc=%d", rc);
        }
    }
    else
    {
        fields.mfg_data_len = CBTTask::Instance()->mManufacturerDataSize;
    }

    rc = ble_gap_adv_set_fields(&fields);
    CBTTask::Instance()->unlock();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Start advertising */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Undirected advertising
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // General discovery
    rc = ble_gap_adv_start(CBTTask::Instance()->own_addr_type, nullptr, BLE_HS_FOREVER,
                           &adv_params, ble_server_gap_event, nullptr);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGD(TAG, "Data advertisement started");
#endif
}

/**
 * @brief Synchronization handler for data transmission mode
 */
void CBTTask::ble_on_sync_data()
{
    int rc;

    // Ensure the address is set correctly
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_util_ensure_addr ; rc=%d", rc);
        return;
    }

    /* Determine the address type to use for advertising */
    rc = ble_hs_id_infer_auto(0, &CBTTask::Instance()->own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    /* Start data advertising */
    ble_advertise_data();
}

/**
 * @brief Initialize the GATT server
 * @return Error code
 */
int CBTTask::gatt_svr_init()
{
    int rc = 0;
    ble_svc_gap_init();  // Initialize GAP service
    ble_svc_gatt_init(); // Initialize GATT service

    rc = ble_gatts_count_cfg(gatt_svr_svcs); // Count service configuration

    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs); // Add services
    if (rc != 0)
    {
        return rc;
    }

    return 0;
}

/**
 * @brief Initialize Bluetooth
 * @param mode Bluetooth operation mode
 * @return Current operation mode
 */
EBTMode CBTTask::init_bt(EBTMode mode)
{
    // Check if Bluetooth is already initialized or mode is Off
    if ((mMode != EBTMode::Off) || (mode == EBTMode::Off))
        return mMode;

    esp_err_t ret = nimble_port_init(); // Initialize the NimBLE port
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
        return mMode;
    }

    // Set callback functions
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Choose the callback depending on the mode
    switch (mode)
    {
#ifdef CONFIG_BLE_DATA_IBEACON_TX
    case EBTMode::iBeaconTx:
        ble_hs_cfg.sync_cb = ble_on_sync_beacon; // iBeacon transmission mode
        break;
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
    case EBTMode::iBeaconRx:
        ble_hs_cfg.sync_cb = ble_on_sync_rx; // iBeacon scanning mode
        break;
#endif
    case EBTMode::Data:
        ble_hs_cfg.sync_cb = ble_on_sync_data; // Data transmission mode
        if (gatt_svr_init() != 0)
            ESP_LOGE(TAG, "Register custom service failed");
        if (ble_svc_gap_device_name_set(CBTTask::device_name) != 0)
            ESP_LOGE(TAG, "Set the default device name failed");
        break;
    default:
        ESP_LOGE(TAG, "Wrong mode %d ", (int)mode);
        break;
    }

    // Initialize the store configuration
    ble_store_config_init();

    // Start the BLE host task
    nimble_port_freertos_init(ble_host_task);

    mMode = mode;
    return mMode;
}

/**
 * @brief Deinitialize Bluetooth
 */
void CBTTask::deinit_bt()
{
    if (mMode == EBTMode::Off)
        return;
    nimble_port_stop();   // Stop the NimBLE port
    nimble_port_deinit(); // Deinitialize the NimBLE port
    mOnRx = nullptr;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    mOnRx2 = nullptr;
#endif
    mMode = EBTMode::Off;
    mConnect = false;
}

/**
 * @brief BLE host task
 * @param param Task parameters
 */
void CBTTask::ble_host_task(void *param)
{
    /* This function returns only when nimble_port_stop() is called */
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * @brief Main function for the Bluetooth task execution
 */
void CBTTask::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
    UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr);
#endif
    STaskMessage msg;

#ifdef CONFIG_BLE_DATA_IBEACON_TX
    // Load iBeacon parameters from NVS
    nvs_handle_t nvs_handle;
    if (nvs_open("nvs", NVS_READWRITE, &nvs_handle) == ESP_OK)
    {
        size_t sz = sizeof(mBeaconID);
        nvs_get_u8(nvs_handle, "btx", &mBeaconTx);
        if (nvs_get_blob(nvs_handle, "beacon", mBeaconID, &sz) != ESP_OK)
        {
            esp_fill_random(mBeaconID, sizeof(mBeaconID)); // Generate a random ID
            nvs_set_blob(nvs_handle, "beacon", mBeaconID, sizeof(mBeaconID));
            nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGE(TAG, "nvs_open failed");
        esp_fill_random(mBeaconID, sizeof(mBeaconID)); // Generate a random ID
    }
#endif

    struct os_mbuf *txom; // Buffer for data transmission
    int er;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    bool skip = false; // Flag to skip transmission
    int n;
#endif
    for (;;)
    {
        // Process incoming messages
        while (getMessage(&msg, TASK_MAX_BLOCK_TIME))
        {
            switch (msg.msgID)
            {
#ifdef CONFIG_BLE_DATA_IBEACON_TX
            case MSG_INIT_BEACON_TX:
                deinit_bt();
                if (mBeaconTimer != nullptr)
                {
                    delete mBeaconTimer;
                    mBeaconTimer = nullptr;
                }
                mBeaconMajor = msg.shortParam;
                mBeaconMinor = msg.paramID;
                init_bt(EBTMode::iBeaconTx);
                break;
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
            case MSG_INIT_BEACON_RX:
                deinit_bt();
                mOnBeacon = (onBeaconRx *)msg.msgBody;
                mBeaconSleepTime = (msg.shortParam & 0x7f) * 1000;
                mBeaconFilter = (msg.shortParam & 0x80) != 0;
                init_bt(EBTMode::iBeaconRx);
                if (mBeaconTimer == nullptr)
                {
                    mBeaconTimer = new CSoftwareTimer(0, MSG_BEACON_TIMER);
                }
                mBeaconTimer->start(this, ETimerEvent::SendBack, CONFIG_BLE_DATA_IBEACON_SCAN_TIMER);
                mBeaconSleep = false;
                break;
            case MSG_BEACON_DATA:
                if (mOnBeacon != nullptr)
                    mOnBeacon((SBeacon *)msg.msgBody, nullptr);
                else
                {
                    TRACEDATA("beacon", (uint8_t *)msg.msgBody, msg.shortParam);
                    vPortFree(msg.msgBody);
                }
                break;
            case MSG_MAC_DATA:
                if (mOnBeacon != nullptr)
                {
                    mOnBeacon(nullptr, (SMac *)msg.msgBody);
                }
                else
                {
                    TRACEDATA("mac", (uint8_t *)msg.msgBody, msg.shortParam);
                    vPortFree(msg.msgBody);
                }
                break;
            case MSG_BEACON_TIMER:
                if (mBeaconTimer != nullptr)
                {
                    if (mBeaconSleep)
                    {
                        if (mMode == EBTMode::Off)
                        {
                            init_bt(EBTMode::iBeaconRx);
                            mBeaconTimer->start(this, ETimerEvent::SendBack, CONFIG_BLE_DATA_IBEACON_SCAN_TIMER);
                        }
                    }
                    else if (mMode == EBTMode::iBeaconRx)
                    {
                        deinit_bt();
                        if (mOnBeacon != nullptr)
                            mOnBeacon(nullptr, nullptr);
                        mBeaconTimer->start(this, ETimerEvent::SendBack, mBeaconSleepTime);
                    }
                    mBeaconSleep = !mBeaconSleep;
                }
                break;
#endif
            case MSG_INIT_DATA:
                deinit_bt();
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
                if (mBeaconTimer != nullptr)
                {
                    delete mBeaconTimer;
                    mBeaconTimer = nullptr;
                }
#endif
                mOnRx = (onBLEDataRx *)msg.msgBody;
                init_bt(EBTMode::Data);
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
                skip = false;
#endif
                break;
            case MSG_OFF:
                deinit_bt();
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
                if (mBeaconTimer != nullptr)
                {
                    delete mBeaconTimer;
                    mBeaconTimer = nullptr;
                }
#endif
                break;
            case MSG_WRITE_DATA:
                // Send data via BLE notification
                if (mConnect)
                {
                    txom = ble_hs_mbuf_from_flat(msg.msgBody, msg.shortParam);
                    if ((er = ble_gatts_notify_custom(1, ble_spp_svc_gatt_read_val_handle, txom)) != 0)
                    {
                        TRACE_ERROR("bt: Error in sending notification", er);
                    }
                }
                else
                {
                    TRACE_WARNING("BLE Tx: not connected", msg.shortParam);
                }
                vPortFree(msg.msgBody);
                break;
            case MSG_READ_DATA:
                // Receive data via BLE
                if (mOnRx != nullptr)
                    mOnRx((uint8_t *)msg.msgBody, msg.shortParam);
                else
                {
                    TRACEDATA("BLE Rx", (uint8_t *)msg.msgBody, msg.shortParam);
                }
                vPortFree(msg.msgBody);
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
                skip = false;
#endif
                break;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
            case MSG_SKIP_WRITE:
                skip = true;
                break;
            case MSG_WRITE_DATA2:
                // Send data via the second channel
                if (mConnect && (!skip))
                {
                    for (n = 0; n < 5; n++)
                    {
                        txom = ble_hs_mbuf_from_flat(msg.msgBody, msg.shortParam);
                        if ((er = ble_gatts_notify_custom(1, ble_spp_svc_gatt_read_val_handle2, txom)) != 0)
                        {
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    TRACE_WARNING("BLE Tx: not connected", msg.shortParam);
                }
                vPortFree(msg.msgBody);
                break;
            case MSG_INIT_DATA2:
                mOnRx2 = (onBLEDataRx *)msg.msgBody;
                break;
            case MSG_READ_DATA2:
                // Receive data via the second channel
                if (mOnRx2 != nullptr)
                    mOnRx2((uint8_t *)msg.msgBody, msg.shortParam);
                else
                {
                    TRACEDATA("BLE Rx 2", (uint8_t *)msg.msgBody, msg.shortParam);
                }
                vPortFree(msg.msgBody);
                break;
#endif
            case MSG_INIT_DATA3:
                mOnConnect = (onBLEConnect *)msg.msgBody;
                break;
            case MSG_SET_ADV_DATA:
                // Set advertising data
                lock();
                if (mManufacturerData != nullptr)
                    vPortFree(mManufacturerData);
                mManufacturerData = (uint8_t *)msg.msgBody;
                mManufacturerDataSize = msg.shortParam;
                unlock();
                if ((mMode == EBTMode::Data) && (!mConnect) && (ble_hs_synced()))
                {
#ifdef CONFIG_BT_NIMBLE_EXT_ADV
                    ble_gap_ext_adv_stop(1);
#else
                    ble_gap_adv_stop();
#endif
                    ble_advertise_data();
                }
                break;
            case MSG_END_TASK:
                goto endTask;
            default:
                TRACE_WARNING("CBTTask:unknown message", msg.msgID);
                break;
            }
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
            UBaseType_t m2 = uxTaskGetStackHighWaterMark2(nullptr);
            if (m2 != m1)
            {
                m1 = m2;
                TDEC("free bttask stack", m2);
            }
#endif
        }
    }
endTask:
    deinit_bt();
    if (mManufacturerData != nullptr)
        vPortFree(mManufacturerData);
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
    if (mBeaconTimer != nullptr)
    {
        delete mBeaconTimer;
    }
#endif
}

/**
 * @brief Send data via BLE
 * @param data Pointer to data
 * @param size Data size
 * @param xTicksToWait Wait time
 * @return true if successful, false if error
 */
bool CBTTask::sendData(uint8_t *data, size_t size, TickType_t xTicksToWait)
{
    STaskMessage msg;
    uint8_t *dt = allocNewMsg(&msg, MSG_WRITE_DATA, size, true);
    std::memcpy(dt, data, size);
    return sendMessage(&msg, xTicksToWait, true);
}

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
/**
 * @brief Send data via the second BLE channel
 * @param data Pointer to data
 * @param size Data size
 * @param index Data index
 * @param xTicksToWait Wait time
 * @return true if successful, false if error
 */
bool CBTTask::sendData2(uint8_t *data, size_t size, uint16_t index, TickType_t xTicksToWait)
{
    STaskMessage msg;
    uint8_t *dt = allocNewMsg(&msg, MSG_WRITE_DATA2, size + 2, true);
    std::memcpy(&dt[2], data, size);
    dt[0] = (uint8_t)index;
    dt[1] = (uint8_t)(index >> 8);
    return sendMessage(&msg, xTicksToWait, true);
}
#endif

/**
 * @brief Set manufacturer data for advertising
 * @param data Pointer to data
 * @param size Data size
 * @param xTicksToWait Wait time
 * @return true if successful, false if error
 */
bool CBTTask::setManufacturerData(uint8_t *data, size_t size, TickType_t xTicksToWait)
{
    STaskMessage msg;
    if (data != nullptr)
    {
        uint8_t *dt = allocNewMsg(&msg, MSG_SET_ADV_DATA, size, true);
        std::memcpy(dt, data, size);
    }
    else
    {
        msg.msgID = MSG_SET_ADV_DATA;
        msg.msgBody = nullptr;
        msg.shortParam = 0;
    }
    return sendMessage(&msg, xTicksToWait, true);
}
#endif