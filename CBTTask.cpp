/*!
    \file
    \brief Класс задачи под логику работы.
    \authors Близнец Р.А.(r.bliznets@gmail.com)
    \version 0.2.0.0
    \date 17.10.2023

    Один объект на приложение.
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

extern "C" void ble_store_config_init(void);

// Хендл для чтения значения характеристики SPP сервиса
static uint16_t ble_spp_svc_gatt_read_val_handle = 0x30;

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16 0xABF0
/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16 0xABF1

// Определение UUID для SPP сервиса и характеристики
const ble_uuid16_t CBTTask::svs_uuid = BLE_UUID16_INIT(BLE_SVC_SPP_UUID16);
const ble_uuid16_t CBTTask::chr_uuid = BLE_UUID16_INIT(BLE_SVC_SPP_CHR_UUID16);

// Второй канал данных (если включен в конфигурации)
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
const ble_uuid16_t CBTTask::chr_uuid2 = BLE_UUID16_INIT(BLE_SVC_SPP_CHR_UUID16 + 1);
static uint16_t ble_spp_svc_gatt_read_val_handle2 = 0x31;
#endif

// Определение характеристик GATT сервиса
const struct ble_gatt_chr_def CBTTask::gatt_svr_chrs[] = {
    {
        /* Поддержка SPP сервиса */
        .uuid = (ble_uuid_t *)&chr_uuid,
        .access_cb = ble_svc_gatt_handler,                     // Callback для обработки доступа к характеристике
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY, // Разрешена запись и уведомления
        .val_handle = &ble_spp_svc_gatt_read_val_handle,       // Указатель на хендл значения
    },
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    {
        .uuid = (ble_uuid_t *)&chr_uuid2,
        .access_cb = ble_svc_gatt_handler2, // Callback для второго канала
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &ble_spp_svc_gatt_read_val_handle2,
    },
#endif
    {
        0, /* Больше характеристик в этом сервисе нет */
    }};

// Определение GATT сервиса
const struct ble_gatt_svc_def CBTTask::gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY, // Первичный сервис
        .uuid = (ble_uuid_t *)&svs_uuid,   // UUID сервиса
        .characteristics = gatt_svr_chrs   // Массив характеристик
    },
    {
        0, /* Больше сервисов нет */
    }};

static const char *TAG = "BTTask"; // Тег для логирования

const char *CBTTask::device_name = CONFIG_BLE_DATA_DEVICE_NAME; // Имя устройства из конфигурации

/**
 * @brief Обработчик записи в характеристику GATT
 * @param om Указатель на буфер с данными
 * @param chn Номер канала (1 или 2)
 * @return Код ошибки BLE
 */
int CBTTask::gatt_svr_chr_write(struct os_mbuf *om, uint16_t chn)
{
    uint16_t om_len;  // Длина полученных данных
    int rc;           // Код возврата
    STaskMessage msg; // Сообщение для отправки в задачу

    om_len = OS_MBUF_PKTLEN(om); // Получаем длину пакета
    if (om_len < 1)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN; // Ошибка: пустой пакет
    }

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    // Определяем тип сообщения в зависимости от канала
    if (chn == 2)
        allocNewMsg(&msg, MSG_READ_DATA2, om_len, true); // Второй канал
    else
#endif
        allocNewMsg(&msg, MSG_READ_DATA, om_len, true); // Первый канал

    // Копируем данные из буфера BLE в наше сообщение
    rc = ble_hs_mbuf_to_flat(om, msg.msgBody, om_len, nullptr);
    if (rc != 0)
    {
        return BLE_ATT_ERR_UNLIKELY; // Ошибка копирования
    }

    // Отправляем сообщение в задачу Bluetooth
    CBTTask::Instance()->sendMessage(&msg, portMAX_DELAY, true);
    return 0;
}

/**
 * @brief Обработчик событий доступа к GATT характеристике (канал 1)
 * @param conn_handle Идентификатор соединения
 * @param attr_handle Идентификатор атрибута
 * @param ctxt Контекст доступа
 * @param arg Дополнительные аргументы
 * @return Код ошибки
 */
int CBTTask::ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // Обработка операции записи в характеристику
        rc = gatt_svr_chr_write(ctxt->om);
        return rc;
    default:
        return BLE_ATT_ERR_UNLIKELY; // Неизвестная операция
    }
    return 0;
}

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
/**
 * @brief Обработчик событий доступа к GATT характеристике (канал 2)
 * @param conn_handle Идентификатор соединения
 * @param attr_handle Идентификатор атрибута
 * @param ctxt Контекст доступа
 * @param arg Дополнительные аргументы
 * @return Код ошибки
 */
int CBTTask::ble_svc_gatt_handler2(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // Обработка операции записи во вторую характеристику
        rc = gatt_svr_chr_write(ctxt->om, 2);
        return rc;
    default:
        return BLE_ATT_ERR_UNLIKELY; // Неизвестная операция
    }
    return 0;
}
#endif

// Указатель на единственный экземпляр класса
CBTTask *CBTTask::theSingleInstance = nullptr;

/**
 * @brief Получение единственного экземпляра класса (Singleton)
 * @return Указатель на экземпляр CBTTask
 */
CBTTask *CBTTask::Instance()
{
    if (theSingleInstance == nullptr)
    {
        // Создаем новый экземпляр, если он еще не существует
        theSingleInstance = new CBTTask();
        // Инициализируем базовую задачу
        theSingleInstance->CBaseTask::init(BTTASK_NAME, BTTASK_STACKSIZE, BTTASK_PRIOR, BTTASK_LENGTH, BTTASK_CPU);
    }
    return theSingleInstance;
};

/**
 * @brief Освобождение ресурсов и удаление экземпляра
 */
void CBTTask::free()
{
    if (theSingleInstance != nullptr)
    {
        // Отправляем команду на завершение задачи
        theSingleInstance->sendCmd(MSG_END_TASK);
        // Ждем завершения задачи
        do
        {
            vTaskDelay(1);
        }
#if (INCLUDE_vTaskDelete == 1)
        while (theSingleInstance->mTaskHandle != nullptr);
#else
        while (theSingleInstance->mTaskQueue != nullptr);
#endif
        // Удаляем экземпляр
        delete theSingleInstance;
        theSingleInstance = nullptr;
    }
}

/**
 * @brief Конструктор класса
 */
CBTTask::CBTTask() : CBaseTask(), CLock()
{
    // Инициализация членов класса значениями по умолчанию
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
 * @brief Деструктор класса
 */
CBTTask::~CBTTask()
{
}

/**
 * @brief Обработчик сброса BLE стека
 * @param reason Причина сброса
 */
void CBTTask::ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
}

#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
/**
 * @brief Обработчик синхронизации для режима сканирования iBeacon
 */
void CBTTask::ble_on_sync_rx()
{
    ESP_LOGD(TAG, "BLE rx");
    int rc;

    /* Убедимся, что установлен правильный адрес устройства (предпочтительно публичный) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_util_ensure_addr; rc=%d", rc);
        return;
    }

    /* Начинаем сканирование для поиска периферийных устройств */
    ble_scan();
}

/**
 * @brief Запуск сканирования BLE устройств
 */
void CBTTask::ble_scan()
{
    uint8_t own_addr_type; // Тип собственного адреса устройства
    int rc;

    /* Определяем тип адреса для использования при рекламе (без приватности) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

#ifdef CONFIG_BT_NIMBLE_EXT_ADV
    // Расширенные параметры сканирования
    struct ble_gap_ext_disc_params disc_params;
    disc_params.passive = 1; // Пассивное сканирование

    /* Используем значения по умолчанию для остальных параметров */
    disc_params.itvl = 0;   // BLE_GAP_SCAN_ITVL_MS(600);
    disc_params.window = 0; // BLE_GAP_SCAN_ITVL_MS(300);

    // Запуск расширенного сканирования
    rc = ble_gap_ext_disc(own_addr_type, 0, 0, 1, 0, 0,
                          &disc_params, &disc_params, ble_rx_gap_event, NULL);
    assert(rc == 0);

#else
    // Стандартные параметры сканирования
    struct ble_gap_disc_params disc_params;
    /* Фильтруем дубликаты - не обрабатываем повторные рекламы от одного устройства */
    disc_params.filter_duplicates = 1;

    /**
     * Выполняем пассивное сканирование. Т.е. не отправляем запросы сканирования
     * каждому рекламодателю.
     */
    disc_params.passive = 1;

    /* Используем значения по умолчанию для остальных параметров */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    // Запуск стандартного сканирования
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      ble_rx_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d", rc);
    }
#endif
}

/**
 * @brief Обработчик событий GAP для сканирования
 * @param event Событие GAP
 * @param arg Дополнительные аргументы
 * @return Код возврата
 */
int CBTTask::ble_rx_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields; // Поля рекламных данных
    int rc;
    STaskMessage msg;
    SBeacon *beacon; // Структура для данных iBeacon
    SMac *mac;       // Структура для MAC адреса

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC_COMPLETE:
        // Сканирование завершено
        return 0;

#ifdef CONFIG_BT_NIMBLE_EXT_ADV
    case BLE_GAP_EVENT_EXT_DISC:
        // Расширенное обнаружение устройств
        if (event->ext_disc.legacy_event_type == BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND)
        {
            // Парсим рекламные поля
            rc = ble_hs_adv_parse_fields(&fields, event->ext_disc.data,
                                         event->ext_disc.length_data);
            if (rc == 0)
            {
                // Проверяем, является ли устройство iBeacon
                if ((fields.mfg_data_len == 25) && (fields.mfg_data[0] == 0x4c) && (fields.mfg_data[1] == 0) && (fields.mfg_data[2] == 0x02) && (fields.mfg_data[3] == 0x15))
                {
                    // Создаем сообщение с данными iBeacon
                    beacon = (SBeacon *)allocNewMsg(&msg, MSG_BEACON_DATA, sizeof(SBeacon), true);
                    std::memcpy(beacon->uuid, &fields.mfg_data[4], 16);              // Копируем UUID
                    beacon->major = fields.mfg_data[21] + fields.mfg_data[20] * 256; // Major
                    beacon->minor = fields.mfg_data[23] + fields.mfg_data[22] * 256; // Minor
                    beacon->power = fields.mfg_data[24];                             // Power
                    beacon->rssi = event->ext_disc.rssi;                             // RSSI
                    CBTTask::Instance()->sendMessage(&msg, 10, true);                // Отправляем сообщение
                    return 0;
                }
            }
        }
        // Обработка MAC адресов публичных устройств
        if (event->ext_disc.addr.type == BLE_ADDR_PUBLIC)
        {
            mac = (SMac *)allocNewMsg(&msg, MSG_MAC_DATA, sizeof(SMac), true);
            std::memcpy(mac->mac, event->ext_disc.addr.val, 6); // Копируем MAC
            CBTTask::Instance()->sendMessage(&msg, 10, true);
            mac->rssi = event->ext_disc.rssi; // RSSI
        }
        return 0;
#else
    case BLE_GAP_EVENT_DISC:
        // Стандартное обнаружение устройств
        if (event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND)
        {
            // Парсим рекламные поля
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
            if (rc == 0)
            {
                // Проверяем, является ли устройство iBeacon
                if ((fields.mfg_data_len == 25) && (fields.mfg_data[0] == 0x4c) && (fields.mfg_data[1] == 0) && (fields.mfg_data[2] == 0x02) && (fields.mfg_data[3] == 0x15))
                {
                    // Создаем сообщение с данными iBeacon
                    beacon = (SBeacon *)allocNewMsg(&msg, MSG_BEACON_DATA, sizeof(SBeacon), true);
                    std::memcpy(beacon->uuid.data(), &fields.mfg_data[4], 16);       // Копируем UUID
                    beacon->major = fields.mfg_data[21] + fields.mfg_data[20] * 256; // Major
                    beacon->minor = fields.mfg_data[23] + fields.mfg_data[22] * 256; // Minor
                    beacon->power = fields.mfg_data[24];                             // Power
                    beacon->rssi = event->disc.rssi;                                 // RSSI
                    CBTTask::Instance()->sendMessage(&msg, 100, true);               // Отправляем сообщение
                    return 0;
                }
            }
        }
        // Обработка MAC адресов публичных устройств
        if (event->disc.addr.type == BLE_ADDR_PUBLIC)
        {
            mac = (SMac *)allocNewMsg(&msg, MSG_MAC_DATA, sizeof(SMac), true);
            std::memcpy(mac->mac.data(), event->disc.addr.val, 6); // Копируем MAC
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
 * @brief Обработчик синхронизации для режима передачи iBeacon
 */
void CBTTask::ble_on_sync_beacon()
{
    ESP_LOGD(TAG, "BLE sync beacon");
    /* Генерируем неразрешаемый частный адрес */
    ble_app_set_addr();
    /* Запускаем рекламу на неопределенный срок */
    ble_advertise_beacon();
}

/**
 * @brief Установка адреса BLE устройства
 */
void CBTTask::ble_app_set_addr()
{
    ble_addr_t addr; // Структура адреса
    int rc;

    // Генерируем случайный адрес
    rc = ble_hs_id_gen_rnd(1, &addr);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_id_gen_rnd; rc=%d", rc);
        return;
    }

    // Устанавливаем случайный адрес
    rc = ble_hs_id_set_rnd(addr.val);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_app_set_addr; rc=%d", rc);
        return;
    }
}

/**
 * @brief Запуск рекламы iBeacon
 */
void CBTTask::ble_advertise_beacon()
{
    struct ble_gap_adv_params adv_params; // Параметры рекламы
    int rc;

    // Устанавливаем данные рекламы для iBeacon
    rc = ble_ibeacon_set_adv_data(CBTTask::Instance()->mBeaconID,
                                  CBTTask::Instance()->mBeaconMajor,
                                  CBTTask::Instance()->mBeaconMinor,
                                  CBTTask::Instance()->mBeaconTx);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error setting advertisement iBeacon; rc=%d", rc);
        return;
    }

    /* Начинаем рекламу */
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
 * @brief Обработчик событий GAP для сервера данных
 * @param event Событие GAP
 * @param arg Дополнительные аргументы
 * @return Код возврата
 */
int CBTTask::ble_server_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        // Событие подключения
        if (event->connect.status != 0)
        {
            ESP_LOGW(TAG, "Connection failed");
            CBTTask::Instance()->mConnect = false;
            /* Подключение не удалось - возобновляем рекламу */
            ble_advertise_data();
        }
        else
        {
            ESP_LOGI(TAG, "Connection established");
            CBTTask::Instance()->mConnect = true;
            // Вызываем callback при подключении
            if (CBTTask::Instance()->mOnConnect != nullptr)
                CBTTask::Instance()->mOnConnect(true);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        // Событие отключения
        ESP_LOGW(TAG, "disconnect; reason=%d", event->disconnect.reason);
        CBTTask::Instance()->mConnect = false;
        // Вызываем callback при отключении
        if (CBTTask::Instance()->mOnConnect != nullptr)
            CBTTask::Instance()->mOnConnect(false);
        ble_advertise_data(); // Возобновляем рекламу
        return 0;

    default:
        return 0;
    }
}

/**
 * @brief Запуск рекламы данных
 */
void CBTTask::ble_advertise_data()
{
#ifdef CONFIG_BT_NIMBLE_EXT_ADV
    int rc;
    struct ble_gap_ext_adv_params params; // Параметры расширенной рекламы
    struct ble_hs_adv_fields adv_fields;  // Поля рекламных данных
    struct os_mbuf *data;                 // Буфер для данных рекламы

    /* Устанавливаем случайный (NRPA) адрес для экземпляра */
    if (CBTTask::Instance()->mAddr.type == 0)
    {
        rc = ble_hs_id_gen_rnd(1, &(CBTTask::Instance()->mAddr));
        assert(rc == 0);
    }

    /* Для периодической рекламы используем экземпляр с неподключаемой рекламой */
    memset(&params, 0, sizeof(params));

    /* Рекламируемся используя случайный адрес */
    params.own_addr_type = BLE_OWN_ADDR_RANDOM;
    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_2M;
    params.sid = 1;
    params.connectable = 1; // Подключаемая реклама

    const char *name = ble_svc_gap_device_name(); // Получаем имя устройства

    CBTTask::Instance()->lock();
    int size = (CBTTask::Instance()->mManufacturerDataSize + strlen(name));
    // Выделяем буфер для данных рекламы
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

    /* Настраиваем экземпляр 1 */
    rc = ble_gap_ext_adv_configure(1, &params, NULL, CBTTask::ble_server_gap_event, NULL);
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_addr(1, &(CBTTask::Instance()->mAddr));
    assert(rc == 0);

    memset(&adv_fields, 0, sizeof(adv_fields));

    /* Рекламируем два флага:
     *     o Обнаруживаемость в последующей рекламе (общая)
     *     o Только BLE (BR/EDR не поддерживается).
     */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN |
                       BLE_HS_ADV_F_BREDR_UNSUP;

    adv_fields.name = (const uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    // Устанавливаем UUID сервиса
    ble_uuid16_t t[1] = {BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    adv_fields.uuids16 = t;
    adv_fields.num_uuids16 = 1;
    adv_fields.uuids16_is_complete = 1;

    // Устанавливаем производственные данные
    adv_fields.mfg_data = CBTTask::Instance()->mManufacturerData;
    adv_fields.mfg_data_len = CBTTask::Instance()->mManufacturerDataSize;

    rc = ble_hs_adv_set_fields_mbuf(&adv_fields, data);
    CBTTask::Instance()->unlock();
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_data(1, data);
    assert(rc == 0);

    /* Запускаем рекламу */
    rc = ble_gap_ext_adv_start(1, 0, 0);
    assert(rc == 0);
#else
    // Стандартная реклама
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
     *  Устанавливаем данные рекламы:
     *     o Флаги (указывают тип рекламы и другую общую информацию).
     *     o Мощность передачи.
     *     o Имя устройства.
     *     o 16-битные UUID сервисов.
     */

    memset(&fields, 0, sizeof fields);

    /* Рекламируем два флага:
     *     o Обнаруживаемость в последующей рекламе (общая)
     *     o Только BLE (BR/EDR не поддерживается).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Указываем, что поле уровня мощности передачи должно быть включено */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)name;
    if (compact && (strlen(name) > 11))
    {
        // Компактное имя в основном объявлении
        fields.name_len = 11;
        fields.name_is_complete = 0;

        // Полное имя в ответе на сканирование
        scan_response_fields.name = (uint8_t *)name;
        scan_response_fields.name_len = strlen(name);
        scan_response_fields.name_is_complete = 1;
    }
    else
    {
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
    }

    // Устанавливаем UUID сервиса
    ble_uuid16_t t[1] = {BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)};
    fields.uuids16 = t;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    fields.mfg_data = CBTTask::Instance()->mManufacturerData;
    if (compact)
    {
        // Компактные производственные данные
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

    /* Начинаем рекламу */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Ненаправленная реклама
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // Общее обнаружение
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
 * @brief Обработчик синхронизации для режима передачи данных
 */
void CBTTask::ble_on_sync_data()
{
    int rc;

    // Убеждаемся, что адрес установлен правильно
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error ble_hs_util_ensure_addr ; rc=%d", rc);
        return;
    }

    /* Определяем тип адреса для использования при рекламе */
    rc = ble_hs_id_infer_auto(0, &CBTTask::Instance()->own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    /* Начинаем рекламу данных */
    ble_advertise_data();
}

/**
 * @brief Инициализация GATT сервера
 * @return Код ошибки
 */
int CBTTask::gatt_svr_init()
{
    int rc = 0;
    ble_svc_gap_init();  // Инициализация GAP сервиса
    ble_svc_gatt_init(); // Инициализация GATT сервиса

    rc = ble_gatts_count_cfg(gatt_svr_svcs); // Подсчет конфигурации сервисов

    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs); // Добавление сервисов
    if (rc != 0)
    {
        return rc;
    }

    return 0;
}

/**
 * @brief Инициализация Bluetooth
 * @param mode Режим работы Bluetooth
 * @return Текущий режим работы
 */
EBTMode CBTTask::init_bt(EBTMode mode)
{
    // Проверяем, не инициализирован ли уже Bluetooth или не выключен ли режим
    if ((mMode != EBTMode::Off) || (mode == EBTMode::Off))
        return mMode;

    esp_err_t ret = nimble_port_init(); // Инициализация порта NimBLE
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
        return mMode;
    }

    // Устанавливаем callback функции
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Выбираем callback в зависимости от режима
    switch (mode)
    {
#ifdef CONFIG_BLE_DATA_IBEACON_TX
    case EBTMode::iBeaconTx:
        ble_hs_cfg.sync_cb = ble_on_sync_beacon; // Режим передачи iBeacon
        break;
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
    case EBTMode::iBeaconRx:
        ble_hs_cfg.sync_cb = ble_on_sync_rx; // Режим сканирования iBeacon
        break;
#endif
    case EBTMode::Data:
        ble_hs_cfg.sync_cb = ble_on_sync_data; // Режим передачи данных
        if (gatt_svr_init() != 0)
            ESP_LOGE(TAG, "Register custom service failed");
        if (ble_svc_gap_device_name_set(CBTTask::device_name) != 0)
            ESP_LOGE(TAG, "Set the default device name failed");
        break;
    default:
        ESP_LOGE(TAG, "Wrong mode %d ", (int)mode);
        break;
    }

    // Инициализация конфигурации хранения
    ble_store_config_init();

    // Запуск задачи хоста BLE
    nimble_port_freertos_init(ble_host_task);

    mMode = mode;
    return mMode;
}

/**
 * @brief Деинициализация Bluetooth
 */
void CBTTask::deinit_bt()
{
    if (mMode == EBTMode::Off)
        return;
    nimble_port_stop();   // Остановка порта NimBLE
    nimble_port_deinit(); // Деинициализация порта NimBLE
    mOnRx = nullptr;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    mOnRx2 = nullptr;
#endif
    mMode = EBTMode::Off;
    mConnect = false;
}

/**
 * @brief Задача хоста BLE
 * @param param Параметры задачи
 */
void CBTTask::ble_host_task(void *param)
{
    /* Эта функция возвращается только когда вызывается nimble_port_stop() */
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * @brief Основная функция выполнения задачи Bluetooth
 */
void CBTTask::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
    UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr);
#endif
    STaskMessage msg;

#ifdef CONFIG_BLE_DATA_IBEACON_TX
    // Загрузка параметров iBeacon из NVS
    nvs_handle_t nvs_handle;
    if (nvs_open("nvs", NVS_READWRITE, &nvs_handle) == ESP_OK)
    {
        size_t sz = sizeof(mBeaconID);
        nvs_get_u8(nvs_handle, "btx", &mBeaconTx);
        if (nvs_get_blob(nvs_handle, "beacon", mBeaconID, &sz) != ESP_OK)
        {
            esp_fill_random(mBeaconID, sizeof(mBeaconID)); // Генерируем случайный ID
            nvs_set_blob(nvs_handle, "beacon", mBeaconID, sizeof(mBeaconID));
            nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGE(TAG, "nvs_open failed");
        esp_fill_random(mBeaconID, sizeof(mBeaconID)); // Генерируем случайный ID
    }
#endif

    struct os_mbuf *txom; // Буфер для передачи данных
    int er;
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
    bool skip = false; // Флаг пропуска передачи
    int n;
#endif
    for (;;)
    {
        // Обработка входящих сообщений
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
                mBeaconSleepTime = msg.shortParam * 1000;
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
                // Отправка данных через BLE уведомление
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
                // Получение данных через BLE
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
                // Отправка данных через второй канал
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
                // Получение данных через второй канал
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
                // Установка данных рекламы
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
 * @brief Отправка данных через BLE
 * @param data Указатель на данные
 * @param size Размер данных
 * @param xTicksToWait Время ожидания
 * @return true если успешно, false если ошибка
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
 * @brief Отправка данных через второй канал BLE
 * @param data Указатель на данные
 * @param size Размер данных
 * @param index Индекс данных
 * @param xTicksToWait Время ожидания
 * @return true если успешно, false если ошибка
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
 * @brief Установка производственных данных для рекламы
 * @param data Указатель на данные
 * @param size Размер данных
 * @param xTicksToWait Время ожидания
 * @return true если успешно, false если ошибка
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