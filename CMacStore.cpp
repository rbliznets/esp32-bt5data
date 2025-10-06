/*!
    \file
    \brief Локация по BLE.
    \authors Близнец Р.А.(r.bliznets@gmail.com)
    \version 0.1.0.0
    \date 16.09.2025
*/
#include "CMacStore.h"
#include <cstring>
#include <algorithm>
#include "esp_log.h"

#if CONFIG_LOG_DEFAULT_LEVEL > 2
static const char *TAG = "CMacStore"; ///< Тег для логирования
#endif

/**
 * @brief Конструктор класса CMacStore
 *
 * Инициализирует хранилище MAC адресов и iBeacon данных с заданными параметрами.
 *
 * @param beacon Флаг включения отслеживания iBeacon
 * @param mac Флаг включения отслеживания MAC адресов
 * @param white Указатель на белый список MAC адресов (может быть nullptr)
 */
CMacStore::CMacStore(bool beacon, bool mac, std::list<std::array<uint8_t, 6>> *white) : mBeaconEnable(beacon), mMacEnable(mac), mWhiteList(white)
{
    // Проверяем, что хотя бы один режим включен
    assert(beacon || mac);

    // Создаем списки для хранения данных старых и новых сканирований
    mOldBeacons = new std::list<SBeacon>; ///< Список iBeacon с предыдущего сканирования
    mNewBeacons = new std::list<SBeacon>; ///< Список iBeacon с текущего сканирования
    mOldMacs = new std::list<SMac>;       ///< Список MAC адресов с предыдущего сканирования
    mNewMacs = new std::list<SMac>;       ///< Список MAC адресов с текущего сканирования
}

/**
 * @brief Деструктор класса CMacStore
 *
 * Освобождает память, выделенную под списки данных.
 */
CMacStore::~CMacStore()
{
    // Освобождаем память всех списков
    delete mNewMacs;
    delete mOldMacs;
    delete mNewBeacons;
    delete mOldBeacons;
    // Освобождаем белый список, если он был создан
    if (mWhiteList != nullptr)
        delete mWhiteList;
}

/**
 * @brief Добавление данных iBeacon в хранилище
 *
 * Добавляет информацию о найденном iBeacon в список новых данных,
 * если отслеживание iBeacon включено.
 *
 * @param[in] data Указатель на структуру данных iBeacon
 */
void CMacStore::addBeacon(SBeacon *data)
{
    if (mBeaconEnable)
    {
        // Добавляем данные в список новых iBeacon
        mNewBeacons->push_back(*data);
    }
}

/**
 * @brief Добавление MAC адреса в хранилище
 *
 * Добавляет MAC адрес в список новых данных с проверкой по белому списку,
 * если отслеживание MAC адресов включено.
 *
 * @param[in] mac Указатель на структуру MAC адреса
 */
void CMacStore::addMac(SMac *mac)
{
    if (mMacEnable)
    {
        // Проверяем, есть ли белый список
        if (mWhiteList != nullptr)
        {
            // Ищем MAC адрес в белом списке
            if (std::find(mWhiteList->begin(), mWhiteList->end(), mac->mac) != mWhiteList->end())
            {
                // Если адрес в белом списке, добавляем его
                mNewMacs->push_back(*mac);
            }
        }
        else
        {
            // Если белого списка нет, добавляем все адреса
            mNewMacs->push_back(*mac);
        }
    }
}

/**
 * @brief Расчет изменений в данных сканирования
 *
 * Сравнивает данные текущего и предыдущего сканирования для обнаружения изменений.
 * Если обнаружены изменения, меняет местами списки и возвращает true.
 *
 * @return true если обнаружены изменения, false если нет
 */
bool CMacStore::calculate()
{
    bool res = false; // Флаг наличия изменений

    // Проверяем изменения в данных iBeacon
    if (mBeaconEnable)
    {
        bool exit = false; // Флаг обнаружения изменений

        // Проверяем, исчезли ли какие-то iBeacon
        for (auto &var : *mOldBeacons)
        {
            if (std::find(mNewBeacons->begin(), mNewBeacons->end(), var) == mNewBeacons->end())
            {
                exit = true;
                break;
            }
        }

        // Если не исчезли, проверяем, появились ли новые iBeacon
        if (!exit)
        {
            for (auto &var : *mNewBeacons)
            {
                if (std::find(mOldBeacons->begin(), mOldBeacons->end(), var) == mOldBeacons->end())
                {
                    exit = true;
                    break;
                }
            }
        }

        // Если обнаружены изменения
        if (exit)
        {
            res = true;
            // Меняем местами списки старых и новых данных
            std::list<SBeacon> *tmp = mOldBeacons;
            mOldBeacons = mNewBeacons;
            mNewBeacons = tmp;
        }
        // Очищаем список новых данных для следующего сканирования
        mNewBeacons->clear();
    }

    // Проверяем изменения в данных MAC адресов
    if (mMacEnable)
    {
        bool exit = false; // Флаг обнаружения изменений

        // Проверяем, исчезли ли какие-то MAC адреса
        for (auto &var : *mOldMacs)
        {
            if (std::find(mNewMacs->begin(), mNewMacs->end(), var) == mNewMacs->end())
            {
                exit = true;
                break;
            }
        }

        // Если не исчезли, проверяем, появились ли новые MAC адреса
        if (!exit)
        {
            for (auto &var : *mNewMacs)
            {
                if (std::find(mOldMacs->begin(), mOldMacs->end(), var) == mOldMacs->end())
                {
                    exit = true;
                    break;
                }
            }
        }

        // Если обнаружены изменения
        if (exit)
        {
            res = true;
            // Меняем местами списки старых и новых данных
            std::list<SMac> *tmp = mOldMacs;
            mOldMacs = mNewMacs;
            mNewMacs = tmp;
        }
        // Очищаем список новых данных для следующего сканирования
        mNewMacs->clear();
    }

    return res;
}

#if CONFIG_LOG_DEFAULT_LEVEL > 2
/**
 * @brief Отладочный вывод данных
 *
 * Выводит в лог информацию о найденных MAC адресах и iBeacon для отладки.
 */
void CMacStore::debug()
{
    ESP_LOGI(TAG, "== olds ==");
    // Выводим информацию о MAC адресах
    for (auto &var : *mOldMacs)
    {
        ESP_LOGW(TAG, "mac rssi:%d dBm", var.rssi);
        ESP_LOG_BUFFER_HEX(TAG, var.mac.data(), 6);
    }

    // Выводим информацию о iBeacon
    for (auto &var : *mOldBeacons)
    {
        ESP_LOGE(TAG, "iBeacon %d:%d rssi:%d dBm, pwr: %d dBm", var.major, var.minor, var.rssi, var.power);
        ESP_LOG_BUFFER_HEX(TAG, var.uuid.data(), 16);
    }
}
#endif

/**
 * @brief Получение сериализованных данных
 *
 * Формирует бинарный буфер с данными о найденных устройствах для передачи.
 *
 * @param[out] size Размер сформированного буфера
 * @return Указатель на буфер с данными (необходимо освободить delete[])
 */
uint8_t *CMacStore::getData(uint16_t &size)
{
    // Рассчитываем общий размер данных:
    // 3 байта заголовка + (кол-во MAC * 7 байт) + (кол-во iBeacon * 22 байта)
    size = 3 + mOldMacs->size() * 7 + mOldBeacons->size() * 22;

    // Если нет данных, возвращаем nullptr
    if (size == 3)
        return nullptr;

    // Выделяем память под буфер данных
    uint8_t *data = new uint8_t[size];

    // Формируем заголовок
    data[0] = 0x08;                // Идентификатор формата данных
    data[1] = mOldMacs->size();    // Количество MAC адресов
    data[2] = mOldBeacons->size(); // Количество iBeacon

    uint16_t index = 3; // Индекс для записи данных

    // Копируем данные MAC адресов
    for (auto &var : *mOldMacs)
    {
        std::memcpy(&data[index], var.mac.data(), 6); // MAC адрес (6 байт)
        index += 6;
        data[index] = (uint8_t)var.rssi; // Уровень сигнала (1 байт)
        index++;
    }

    // Копируем данные iBeacon
    for (auto &var : *mOldBeacons)
    {
        std::memcpy(&data[index], var.uuid.data(), 16); // UUID (16 байт)
        index += 16;
        std::memcpy(&data[index], &var.major, 2); // Major номер (2 байта)
        index += 2;
        std::memcpy(&data[index], &var.minor, 2); // Minor номер (2 байта)
        index += 2;
        data[index] = (uint8_t)var.power; // Мощность передатчика (1 байт)
        index++;
        data[index] = (uint8_t)var.rssi; // Уровень сигнала (1 байт)
        index++;
    }

    return data;
}

json CMacStore::getJSON()
{
    json beacon = json::array();

    for (auto &var : *mOldMacs)
    {
        json j;
        std::string str = "";
        char tmp[4];
        for (auto &x : var.mac)
        {
            std::sprintf(tmp, "%02x", x);
            str += tmp;
        }
        j["mac"] = str;
        j["rssi"] = var.rssi;
        beacon.push_back(j);
    }

    for (auto &var : *mOldBeacons)
    {
        json j;
        std::string str = "";
        char tmp[4];
        for (auto &x : var.uuid)
        {
            std::sprintf(tmp, "%02x", x);
            str += tmp;
        }
        j["uuid"] = str;
        j["major"] = var.major;
        j["minor"] = var.minor;
        j["pwr"] = var.power;
        j["rssi"] = var.rssi;
        beacon.push_back(j);
    }

    return beacon;
}

json CMacStore::data2json(uint8_t *data)
{
    json beacon = json::array();
    uint16_t szmac = data[1];
    uint16_t szgbeacon = data[2];
    uint16_t index = 3;
    for (uint16_t i = 0; i < szmac; i++)
    {
        json j;
        std::string str = "";
        char tmp[4];
        for (size_t i = 0; i < 6; i++)
        {
            std::sprintf(tmp, "%02x", data[index + i]);
            str += tmp;
        }
        j["mac"] = str;
        j["rssi"] = (int8_t)(data[index + 6]);
        index += 7;
        beacon.push_back(j);
    }
    for (uint16_t i = 0; i < szgbeacon; i++)
    {
        json j;
        std::string str = "";
        char tmp[4];
        for (size_t i = 0; i < 16; i++)
        {
            std::sprintf(tmp, "%02x", data[index + i]);
            str += tmp;
        }
        j["uuid"] = str;
        j["major"] = (data[index + 16]) + (data[index + 17] << 8);
        j["minor"] = (data[index + 18]) + (data[index + 19] << 8);
        j["pwr"] = (int8_t)(data[index + 20]);
        j["rssi"] = (int8_t)(data[index + 21]);
        index += 22;
        beacon.push_back(j);
    }
    return beacon;
}

void CMacStore::clear()
{
    mOldBeacons->clear();
    mOldMacs->clear();
}
