/*!
    \file
    \brief Класс задачи под логику работы.
    \authors Близнец Р.А.(r.bliznets@gmail.com)
    \version 0.1.0.0
    \date 17.10.2023

    Один объект на приложение.
*/

#include "CBTTask.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_gap.h"
// #include "host/ble_hs.h"

#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "console/console.h"

#include "nvs.h"
#include "esp_random.h"

/////@@@@@@@@@@@@@@@@@@@
#undef max
#undef min
////////////////////////////////
#include "CTrace.h"
#include <cstring>

extern "C" void ble_store_config_init(void);

static uint16_t ble_spp_svc_gatt_read_val_handle = 0x30;

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16 0xABF0
/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16 0xABF1
const ble_uuid16_t CBTTask::svs_uuid = BLE_UUID16_INIT(BLE_SVC_SPP_UUID16);
const ble_uuid16_t CBTTask::chr_uuid = BLE_UUID16_INIT(BLE_SVC_SPP_CHR_UUID16);
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
const ble_uuid16_t CBTTask::chr_uuid2 = BLE_UUID16_INIT(BLE_SVC_SPP_CHR_UUID16 + 1);
static uint16_t ble_spp_svc_gatt_read_val_handle2 = 0x31;
#endif

const struct ble_gatt_chr_def CBTTask::gatt_svr_chrs[] = {
    {
        /* Support SPP service */
        .uuid = (ble_uuid_t *)&chr_uuid,
        .access_cb = ble_svc_gatt_handler,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &ble_spp_svc_gatt_read_val_handle,
    },
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    {
        .uuid = (ble_uuid_t *)&chr_uuid2,
        .access_cb = ble_svc_gatt_handler2,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &ble_spp_svc_gatt_read_val_handle2,
    },
#endif
    {
        0, /* No more characteristics in this service. */
    }};
const struct ble_gatt_svc_def CBTTask::gatt_svr_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = (ble_uuid_t *)&svs_uuid,
     .characteristics = gatt_svr_chrs},
    {
        0, /* No more services. */
    }};

static const char *TAG = "BTTask";

const char *CBTTask::device_name = CONFIG_BLE_DATA_DEVICE_NAME;

int CBTTask::gatt_svr_chr_write(struct os_mbuf *om, uint16_t chn)
{
    uint16_t om_len;
    int rc;
    STaskMessage msg;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < 1)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    if (chn == 2)
        allocNewMsg(&msg, MSG_READ_DATA2, om_len, true);
    else
#endif
        allocNewMsg(&msg, MSG_READ_DATA, om_len, true);

    rc = ble_hs_mbuf_to_flat(om, msg.msgBody, om_len, nullptr);
    if (rc != 0)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }
    CBTTask::Instance()->sendMessage(&msg, portMAX_DELAY, true);
    return 0;
}

int CBTTask::ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        rc = gatt_svr_chr_write(ctxt->om);
        return rc;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
int CBTTask::ble_svc_gatt_handler2(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // MODLOG_DFLT(INFO, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
        rc = gatt_svr_chr_write(ctxt->om, 2);
        return rc;
    default:
        // MODLOG_DFLT(INFO, "\nDefault Callback");
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}
#endif

CBTTask *CBTTask::theSingleInstance = nullptr;

CBTTask *CBTTask::Instance()
{
    if (theSingleInstance == nullptr)
    {
        theSingleInstance = new CBTTask();
        theSingleInstance->CBaseTask::init(BTTASK_NAME, BTTASK_STACKSIZE, BTTASK_PRIOR, BTTASK_LENGTH, BTTASK_CPU);
    }
    return theSingleInstance;
};

void CBTTask::free()
{
    if (theSingleInstance != nullptr)
    {
        theSingleInstance->sendCmd(MSG_END_TASK);
        do
        {
            vTaskDelay(1);
        }
#if (INCLUDE_vTaskDelete == 1)
        while (theSingleInstance->mTaskHandle != nullptr);
#else
        while (theSingleInstance->mTaskQueue != nullptr);
#endif
        delete theSingleInstance;
        theSingleInstance = nullptr;
    }
}

CBTTask::CBTTask() : CBaseTask(), CLock()
{
}

CBTTask::~CBTTask()
{
}

void CBTTask::ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
}

#ifdef CONFIG_BLE_DATA_IBEACON
void CBTTask::ble_on_sync_rx()
{
    ESP_LOGD(TAG, "BLE rx");
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_util_ensure_addr; rc=%d", rc);
        return;
    }

    /* Begin scanning for a peripheral to connect to. */
    ble_scan();
}

void CBTTask::ble_scan()
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      ble_rx_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d", rc);
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  ble_cts_cent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_cts_cent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
int CBTTask::ble_rx_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;
    int rc;
    STaskMessage msg;
    SBeacon *beacon;

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        if (event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND)
        {
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
            if (rc != 0)
            {
                return 0;
            }
            // MODLOG_DFLT(INFO, "DISC %d (%d)", event->disc.event_type, event->disc.rssi);
            if ((fields.mfg_data_len == 25) && (fields.mfg_data[0] == 0x4c) && (fields.mfg_data[1] == 0) && (fields.mfg_data[2] == 0x02) && (fields.mfg_data[3] == 0x15))
            {
                // TRACEDATA("bt", (uint8_t *)fields.mfg_data, fields.mfg_data_len);
                beacon = (SBeacon *)allocNewMsg(&msg, MSG_BEACON_DATA, sizeof(SBeacon), true);
                std::memcpy(beacon->uuid, &fields.mfg_data[4], 16);
                beacon->major = fields.mfg_data[21] + fields.mfg_data[20] * 256;
                beacon->minor = fields.mfg_data[23] + fields.mfg_data[22] * 256;
                beacon->power = fields.mfg_data[24];
                beacon->rssi = event->disc.rssi;
                CBTTask::Instance()->sendMessage(&msg, 100, true);
            }
        }
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        // MODLOG_DFLT(INFO, "discovery complete; reason=%d\n",
        //             event->disc_complete.reason);
        return 0;

#if CONFIG_EXAMPLE_EXTENDED_ADV
    case BLE_GAP_EVENT_EXT_DISC:
        /* An advertisment report was received during GAP discovery. */
        ext_print_adv_report(&event->disc);

        ble_cts_cent_connect_if_interesting(&event->disc);
        return 0;
#endif

    default:
        return 0;
    }
}

void CBTTask::ble_on_sync_beacon()
{
    ESP_LOGD(TAG, "BLE sync beacon");
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();
    /* Advertise indefinitely. */
    ble_advertise_beacon();
}

void CBTTask::ble_app_set_addr()
{
    ble_addr_t addr;
    int rc;

    rc = ble_hs_id_gen_rnd(1, &addr);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_id_gen_rnd; rc=%d", rc);
        return;
    }

    rc = ble_hs_id_set_rnd(addr.val);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_app_set_addr; rc=%d", rc);
        return;
    }
}

void CBTTask::ble_advertise_beacon()
{
    struct ble_gap_adv_params adv_params;
    int rc;

    rc = ble_ibeacon_set_adv_data(CBTTask::Instance()->mBeaconID, CBTTask::Instance()->mBeaconMajor, CBTTask::Instance()->mBeaconMinor, CBTTask::Instance()->mBeaconTx);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error setting advertisement iBeacon; rc=%d", rc);
        return;
    }

    /* Begin advertising. */
    adv_params = (struct ble_gap_adv_params){0};
    rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, nullptr, BLE_HS_FOREVER,
                           &adv_params, nullptr, nullptr);
    if (rc == 0)
        ESP_LOGD(TAG, "iBeacon advertisement started");
    else
        ESP_LOGE(TAG, "error setting advertisement iBeacon; rc=%d", rc);
}
#endif

int CBTTask::ble_server_gap_event(struct ble_gap_event *event, void *arg)
{
    // ESP_LOGI(TAG, "ble_server_gap_event; event=%d",
    //     event->type);

    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0)
        {
            ESP_LOGW(TAG, "Connection failed");
            CBTTask::Instance()->mConnect = false;
            /* Connection failed or if multiple connection allowed; resume advertising. */
            ble_advertise_data();
        }
        else
        {
            ESP_LOGI(TAG, "Connection established");
            CBTTask::Instance()->mConnect = true;
            if (CBTTask::Instance()->mOnConnect != nullptr)
                CBTTask::Instance()->mOnConnect(true);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "disconnect; reason=%d", event->disconnect.reason);
        CBTTask::Instance()->mConnect = false;
        if (CBTTask::Instance()->mOnConnect != nullptr)
            CBTTask::Instance()->mOnConnect(false);
        ble_advertise_data();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        // MODLOG_DFLT(INFO, "connection updated; status=%d\n",
        //             event->conn_update.status);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGD(TAG, "advertise complete; reason=%d",
                 event->adv_complete.reason);
        ble_advertise_data();
        return 0;

    case BLE_GAP_EVENT_MTU:
        // MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
        //             event->mtu.conn_handle,
        //             event->mtu.channel_id,
        //             event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        // MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
        //                   "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
        //             event->subscribe.conn_handle,
        //             event->subscribe.attr_handle,
        //             event->subscribe.reason,
        //             event->subscribe.prev_notify,
        //             event->subscribe.cur_notify,
        //             event->subscribe.prev_indicate,
        //             event->subscribe.cur_indicate);
        return 0;

    default:
        return 0;
    }
}

#ifdef CONFIG_BT_NIMBLE_EXT_ADV
static uint8_t periodic_adv_raw_data[] = {'E', 'S', 'P', '_', 'P', 'E', 'R', 'I', 'O', 'D', 'I', 'C', '_', 'A', 'D', 'V'};
#endif

void CBTTask::ble_advertise_data()
{
#ifdef CONFIG_BT_NIMBLE_EXT_ADV
    int rc;
    struct ble_gap_periodic_adv_params pparams;
    struct ble_gap_ext_adv_params params;
    struct ble_hs_adv_fields adv_fields;
    struct os_mbuf *data;
    uint8_t instance = 1;
    ble_addr_t addr;
#ifdef BT_NIMBLE_ENABLE_PERIODIC_ADV
    struct ble_gap_periodic_adv_enable_params eparams;
    memset(&eparams, 0, sizeof(eparams));
#endif

    /* set random (NRPA) address for instance */
    rc = ble_hs_id_gen_rnd(1, &addr);
    assert(rc == 0);

    /* For periodic we use instance with non-connectable advertising */
    memset(&params, 0, sizeof(params));

    /* advertise using random addr */
    params.own_addr_type = BLE_OWN_ADDR_RANDOM;
    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_2M;
    params.sid = 2;

    /* configure instance 1 */
    rc = ble_gap_ext_adv_configure(instance, &params, NULL, NULL, NULL);
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_addr(instance, &addr);
    assert(rc == 0);

    memset(&adv_fields, 0, sizeof(adv_fields));

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN |
                       BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();
    adv_fields.name = (const uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    adv_fields.tx_pwr_lvl_is_present = 1;
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    ble_uuid16_t t[1] = {BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    adv_fields.uuids16 = t;
    adv_fields.num_uuids16 = 1;
    adv_fields.uuids16_is_complete = 1;

    CBTTask::Instance()->lock();
    adv_fields.mfg_data = CBTTask::Instance()->mManufacturerData;
    adv_fields.mfg_data_len = CBTTask::Instance()->mManufacturerDataSize;

    /* Default to legacy PDUs size, mbuf chain will be increased if needed
     */
    data = os_msys_get_pkthdr(BLE_HCI_MAX_ADV_DATA_LEN, 0);
    assert(data);

    rc = ble_hs_adv_set_fields_mbuf(&adv_fields, data);
    assert(rc == 0);
    CBTTask::Instance()->unlock();

    rc = ble_gap_ext_adv_set_data(instance, data);
    assert(rc == 0);

    /* configure periodic advertising */
    memset(&pparams, 0, sizeof(pparams));
    pparams.include_tx_power = 1;
    pparams.itvl_min = BLE_GAP_ADV_ITVL_MS(120);
    pparams.itvl_max = BLE_GAP_ADV_ITVL_MS(240);

//     rc = ble_gap_periodic_adv_configure(instance, &pparams);
//     assert(rc == 0);

//     data = os_msys_get_pkthdr(sizeof(periodic_adv_raw_data), 0);
//     assert(data);

//     rc = os_mbuf_append(data, periodic_adv_raw_data, sizeof(periodic_adv_raw_data));
//     assert(rc == 0);
// #ifdef BT_NIMBLE_ENABLE_PERIODIC_ADV
//     rc = ble_gap_periodic_adv_set_data(instance, data, NULL);
// #else
//     rc = ble_gap_periodic_adv_set_data(instance, data);
// #endif
//     assert(rc == 0);

/* start periodic advertising */
#ifdef BT_NIMBLE_ENABLE_PERIODIC_ADV
    eparams.include_adi = 1;
    rc = ble_gap_periodic_adv_start(instance, &eparams);
#else
    rc = ble_gap_periodic_adv_start(instance);
#endif
    assert(rc == 0);

    /* start advertising */
    rc = ble_gap_ext_adv_start(instance, 0, 0);
    assert(rc == 0);
#else
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    name = ble_svc_gap_device_name();
    bool compact = (CBTTask::Instance()->mManufacturerDataSize + strlen(name)) > 16;
    struct ble_hs_adv_fields scan_response_fields;
    memset(&scan_response_fields, 0, sizeof scan_response_fields);

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)name;
    if(compact && (strlen(name) > 11))
    {
        fields.name_len = 11;
        fields.name_is_complete = 0;

        scan_response_fields.name = (uint8_t *)name;
        scan_response_fields.name_len = strlen(name);
        scan_response_fields.name_is_complete = 1;
    }
    else
    {
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
    }

    ble_uuid16_t t[1] = {BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    fields.uuids16 = t;
    fields.num_uuids16 = 1;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    fields.uuids16_is_complete = 1;
#else
    fields.uuids16_is_complete = 1;
#endif

    CBTTask::Instance()->lock();
    fields.mfg_data = CBTTask::Instance()->mManufacturerData;
    if(compact)
    {
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

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(CBTTask::Instance()->own_addr_type, nullptr, BLE_HS_FOREVER,
                           &adv_params, ble_server_gap_event, nullptr);
    // rc = ble_gap_adv_start(CBTTask::Instance()->own_addr_type, nullptr, 1000,
    //                        &adv_params, ble_server_gap_event, nullptr);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGD(TAG, "Data advertisement started");
#endif
}

void CBTTask::ble_on_sync_data()
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_util_ensure_addr ; rc=%d", rc);
        return;
    }

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &CBTTask::Instance()->own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    /* Begin advertising. */
    ble_advertise_data();
}

int CBTTask::gatt_svr_init()
{
    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);

    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    return 0;
}

EBTMode CBTTask::init_bt(EBTMode mode)
{
    if ((mMode != EBTMode::Off) || (mode == EBTMode::Off))
        return mMode;

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
        return mMode;
    }
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    switch (mode)
    {
#ifdef CONFIG_BLE_DATA_IBEACON
    case EBTMode::iBeaconTx:
        ble_hs_cfg.sync_cb = ble_on_sync_beacon;
        break;
    case EBTMode::iBeaconRx:
        ble_hs_cfg.sync_cb = ble_on_sync_rx;
        break;
#endif
    case EBTMode::Data:
        ble_hs_cfg.sync_cb = ble_on_sync_data;
        if (gatt_svr_init() != 0)
            ESP_LOGE(TAG, "Register custom service failed");
        if (ble_svc_gap_device_name_set(CBTTask::device_name) != 0)
            ESP_LOGE(TAG, "Set the default device name failed");
        break;
    default:
        ESP_LOGE(TAG, "Wrong mode %d ", (int)mode);
        break;
    }

    // /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);
    mMode = mode;
    return mMode;
}

void CBTTask::deinit_bt()
{
    if (mMode == EBTMode::Off)
        return;
    nimble_port_stop();
    nimble_port_deinit();
    mOnRx = nullptr;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    mOnRx2 = nullptr;
#endif
    mMode = EBTMode::Off;
    mConnect = false;
}

void CBTTask::ble_host_task(void *param)
{
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void CBTTask::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
    UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr);
#endif
    STaskMessage msg;

#ifdef CONFIG_BLE_DATA_IBEACON
    nvs_handle_t nvs_handle;
    if (nvs_open("nvs", NVS_READWRITE, &nvs_handle) == ESP_OK)
    {
        size_t sz = sizeof(mBeaconID);
        nvs_get_u8(nvs_handle, "btx", &mBeaconTx);
        if (nvs_get_blob(nvs_handle, "beacon", mBeaconID, &sz) != ESP_OK)
        {
            esp_fill_random(mBeaconID, sizeof(mBeaconID));
            nvs_set_blob(nvs_handle, "beacon", mBeaconID, sizeof(mBeaconID));
            nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGE(TAG, "nvs_open failed");
        esp_fill_random(mBeaconID, sizeof(mBeaconID));
    }
#endif

    struct os_mbuf *txom;
    int er, n;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    bool skip = false;
#endif
    for (;;)
    {
        while (getMessage(&msg, TASK_MAX_BLOCK_TIME))
        {
            switch (msg.msgID)
            {
#ifdef CONFIG_BLE_DATA_IBEACON
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
            case MSG_INIT_BEACON_RX:
                deinit_bt();
                mOnBeacon = (onBeaconRx *)msg.msgBody;
                mBeaconSleepTime = msg.shortParam * 1000;
                if (mOnBeacon != nullptr)
                    mOnBeacon(nullptr);
                init_bt(EBTMode::iBeaconRx);
                if (mBeaconTimer == nullptr)
                {
                    mBeaconTimer = new CSoftwareTimer(0, MSG_BEACON_TIMER);
                }
                mBeaconTimer->start(this, ETimerEvent::SendBack, 1000);
                mBeaconSleep = false;
                break;
            case MSG_BEACON_DATA:
                if (mOnBeacon != nullptr)
                    mOnBeacon((SBeacon *)msg.msgBody);
                else
                {
                    TRACEDATA("beacon", (uint8_t *)msg.msgBody, msg.shortParam);
                }
                vPortFree(msg.msgBody);
                // if(mBeaconTimer != nullptr)
                // {
                //     if(mMode == EBTMode::iBeaconRx)
                //     {
                //         deinit_bt();
                //         // TDEC("sleep",mBeaconSleepTime);
                //         mBeaconTimer->start(this, ETimerEvent::SendBack,mBeaconSleepTime);
                //         mBeaconSleep=true;
                //     }
                // }
                break;
            case MSG_BEACON_TIMER:
                if (mBeaconTimer != nullptr)
                {
                    if (mBeaconSleep)
                    {
                        if (mMode == EBTMode::Off)
                        {
                            init_bt(EBTMode::iBeaconRx);
                            // TDEC("rx",1000);
                            if (mOnBeacon != nullptr)
                                mOnBeacon(nullptr);
                            mBeaconTimer->start(this, ETimerEvent::SendBack, 1000);
                        }
                    }
                    else if (mMode == EBTMode::iBeaconRx)
                    {
                        deinit_bt();
                        // TDEC("sleep",mBeaconSleepTime);
                        mBeaconTimer->start(this, ETimerEvent::SendBack, mBeaconSleepTime);
                    }
                    mBeaconSleep = !mBeaconSleep;
                }
                break;
#endif
            case MSG_INIT_DATA:
                deinit_bt();
#ifdef CONFIG_BLE_DATA_IBEACON
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
#ifdef CONFIG_BLE_DATA_IBEACON
                if (mBeaconTimer != nullptr)
                {
                    delete mBeaconTimer;
                    mBeaconTimer = nullptr;
                }
#endif
                break;
            case MSG_WRITE_DATA:
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
                if (mConnect && (!skip))
                {
                    for (n = 0; n < 5; n++)
                    {
                        txom = ble_hs_mbuf_from_flat(msg.msgBody, msg.shortParam);
                        if ((er = ble_gatts_notify_custom(1, ble_spp_svc_gatt_read_val_handle2, txom)) != 0)
                        {
                            // TRACE_ERROR("bt: Error in sending notification", er);
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
                // TRACEDATA("MSG_SET_ADV_DATA",(uint8_t *)msg.msgBody,msg.shortParam);
                lock();
                if (mManufacturerData != nullptr)
                    vPortFree(mManufacturerData);
                mManufacturerData = (uint8_t *)msg.msgBody;
                mManufacturerDataSize = msg.shortParam;
                unlock();
                if ((!mConnect) && (ble_hs_synced()))
                {
                    ble_gap_adv_stop();
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
    if (mManufacturerData != nullptr)
        vPortFree(mManufacturerData);
    deinit_bt();
#ifdef CONFIG_BLE_DATA_IBEACON
    if (mBeaconTimer != nullptr)
    {
        delete mBeaconTimer;
        mBeaconTimer = nullptr;
    }
#endif
}

bool CBTTask::sendData(uint8_t *data, size_t size, TickType_t xTicksToWait)
{
    STaskMessage msg;
    uint8_t *dt = allocNewMsg(&msg, MSG_WRITE_DATA, size, true);
    std::memcpy(dt, data, size);
    return sendMessage(&msg, xTicksToWait, true);
}

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
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
