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

static const char* TAG = "CMacStore";

CMacStore::CMacStore(bool beacon, bool mac, std::list<std::array<uint8_t, 6>> *white) : mBeaconEnable(beacon), mMacEnable(mac), mWhiteList(white)
{
    assert(beacon || mac);

    mOldBeacons = new std::list<SBeacon>;
    mNewBeacons = new std::list<SBeacon>;
    mOldMacs = new std::list<SMac>;
    mNewMacs = new std::list<SMac>;
}

CMacStore::~CMacStore()
{
    delete mNewMacs;
    delete mOldMacs;
    delete mNewBeacons;
    delete mOldBeacons;
    if (mWhiteList != nullptr)
        delete mWhiteList;
}

void CMacStore::addBeacon(SBeacon *data)
{
    if (mBeaconEnable)
    {
        mNewBeacons->push_back(*data);
    }
}

void CMacStore::addMac(SMac *mac)
{
    if (mMacEnable)
    {
        if (mWhiteList != nullptr)
        {
            if (std::find(mWhiteList->begin(), mWhiteList->end(), mac->mac) != mWhiteList->end())
            {
                mNewMacs->push_back(*mac);
            }
        }
        else
        {
            mNewMacs->push_back(*mac);
        }
    }
}

bool CMacStore::calculate()
{
    bool res = false;
    if (mBeaconEnable)
    {
        bool exit = false;
        for (auto &var : *mOldBeacons)
        {
            if (std::find(mNewBeacons->begin(), mNewBeacons->end(), var) == mNewBeacons->end())
            {
                exit = true;
                break;
            }
        }
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
        if (exit)
        {
            res = true;
            std::list<SBeacon> *tmp = mOldBeacons;
            mOldBeacons = mNewBeacons;
            mNewBeacons = tmp;
        }
        mNewBeacons->clear();
    }
    if (mMacEnable)
    {
        bool exit = false;
        for (auto &var : *mOldMacs)
        {
            if (std::find(mNewMacs->begin(), mNewMacs->end(), var) == mNewMacs->end())
            {
                exit = true;
                break;
            }
        }
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
        if (exit)
        {
            res = true;
            std::list<SMac> *tmp = mOldMacs;
            mOldMacs = mNewMacs;
            mNewMacs = tmp;
        }
        mNewMacs->clear();
    }
    return res;
}

#if CONFIG_LOG_DEFAULT_LEVEL > 2
void CMacStore::debug()
{
    for (auto &var : *mOldMacs)
    {
        ESP_LOGI(TAG,"mac rssi:%d dBm",var.rssi);
        ESP_LOG_BUFFER_HEX(TAG,var.mac.data(), 6);
    }
    for (auto &var : *mOldBeacons)
    {
        ESP_LOGI(TAG,"iBeacon 0x%04X:0x%04X rssi:%d dBm, pwr: %d dBm",var.major, var.minor, var.rssi, var.power);
        ESP_LOG_BUFFER_HEX(TAG,var.uuid.data(), 16);
    }
}
#endif

uint8_t *CMacStore::getData(uint16_t &size)
{
    size = 3 + mOldMacs->size() * 7 + mOldBeacons->size() * 22;
    if (size == 3)
        return nullptr;
    uint8_t *data = new uint8_t[size];
    data[0] = 0x08;
    data[1] = mOldMacs->size();
    data[2] = mOldBeacons->size();
    uint16_t index = 3;
    for (auto &var : *mOldMacs)
    {
        std::memcpy(&data[index],var.mac.data(),6);
        index += 6;
        data[index]=(uint8_t)var.rssi;
        index++;
    }
    for (auto &var : *mOldBeacons)
    {
        std::memcpy(&data[index],var.uuid.data(),16);
        index += 16;
        std::memcpy(&data[index],&var.major,2);
        index += 2;
        std::memcpy(&data[index],&var.minor,2);
        index += 2;
        data[index]=(uint8_t)var.power;
        index++;
        data[index]=(uint8_t)var.rssi;
        index++;
    }
    return data;
}
