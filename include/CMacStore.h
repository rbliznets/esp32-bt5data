/*!
	\file
	\brief Локация по BLE.
	\authors Близнец Р.А.(r.bliznets@gmail.com)
	\version 0.1.0.0
	\date 16.09.2025
*/
#pragma once

#include "sdkconfig.h"
#include <list>
#include <array>

#include "CBTTask.h"

class CMacStore
{
protected:
    bool mBeaconEnable;
    bool mMacEnable;
    std::list<std::array<uint8_t, 6>>* mWhiteList;

    std::list<SBeacon>* mOldBeacons;
    std::list<SBeacon>* mNewBeacons;
    std::list<SMac>* mOldMacs;
    std::list<SMac>* mNewMacs;
public:
    CMacStore(bool beacon = true, bool mac = false, std::list<std::array<uint8_t, 6>>* white = nullptr);
    ~CMacStore();

    void addBeacon(SBeacon *data);
    void addMac(SMac *mac);
    bool calculate();

    uint8_t* getData(uint16_t& size);
#if CONFIG_LOG_DEFAULT_LEVEL > 2
    void debug();
#else
    inline void debug() {};
#endif
}; 