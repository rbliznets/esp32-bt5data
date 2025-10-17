/*!
    \file
    \brief BLE Location.
    \authors Bliznets R.A.(r.bliznets@gmail.com)
    \version 1.0.0.0
    \date 16.09.2025
*/
#pragma once

#include "sdkconfig.h"
#include <list>
#include <array>

#include "CBTTask.h" // Includes definitions for SBeacon and SMac

#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * @brief Class for storing and analyzing BLE device data
 *
 * Implements functionality for tracking MAC addresses and iBeacon beacons,
 * including comparing data between scans and generating reports.
 */
class CMacStore
{
protected:
    bool mBeaconEnable;                            ///< Flag to enable iBeacon tracking
    bool mMacEnable;                               ///< Flag to enable MAC address tracking
    std::list<std::array<uint8_t, 6>> *mWhiteList; ///< Whitelist of allowed MAC addresses

    std::list<SBeacon> *mOldBeacons; ///< List of iBeacons from the previous scan
    std::list<SBeacon> *mNewBeacons; ///< List of iBeacons from the current scan
    std::list<SMac> *mOldMacs;       ///< List of MAC addresses from the previous scan
    std::list<SMac> *mNewMacs;       ///< List of MAC addresses from the current scan

public:
    /**
     * @brief Constructor for the CMacStore class
     *
     * Creates a new instance of the BLE device data storage.
     *
     * @param[in] beacon Flag to enable iBeacon tracking (default true)
     * @param[in] mac Flag to enable MAC address tracking (default false)
     * @param[in] white Pointer to the MAC address whitelist (default nullptr)
     */
    CMacStore(bool beacon = true, bool mac = false, std::list<std::array<uint8_t, 6>> *white = nullptr);

    /**
     * @brief Destructor for the CMacStore class
     *
     * Frees the memory allocated for the data lists.
     */
    ~CMacStore();

    /**
     * @brief Add iBeacon data
     *
     * Adds information about a found iBeacon to the current scan list.
     *
     * @param[in] data Pointer to the iBeacon data structure to add
     */
    void addBeacon(SBeacon *data);

    /**
     * @brief Add MAC address
     *
     * Adds a MAC address to the current scan list with a check against the whitelist.
     *
     * @param[in] mac Pointer to the MAC address structure to add
     */
    void addMac(SMac *mac);

    /**
     * @brief Calculate changes between scans
     *
     * Compares the current and previous scan data to detect changes.
     *
     * @return true if changes are detected, false if the data remains the same
     */
    bool calculate();

    /**
     * @brief Clear the stored 'old' data lists.
     *
     * Empties the lists containing data from the previous scan (mOldBeacons and mOldMacs).
     */
    void clear();

    /**
     * @brief Get serialized data
     *
     * Forms a binary buffer with data about found devices.
     *
     * @param[out] size Reference to store the size of the formed data buffer
     * @return Pointer to the data buffer (needs to be freed with delete[] by the caller)
     */
    uint8_t *getData(uint16_t &size);

    /**
     * @brief Get stored data as a JSON array
     *
     * Converts the data stored in the 'old' lists (mOldMacs, mOldBeacons) into a JSON array.
     *
     * @return A nlohmann::json object containing an array of device data.
     */
    json getJSON();

    /**
     * @brief Parse binary data buffer into a JSON array
     *
     * Takes a binary buffer previously created by getData() and converts it back into
     * a nlohmann::json array, reconstructing the MAC address and iBeacon information.
     *
     * @param data Pointer to the binary buffer (format: 3 byte header + device data).
     * @return A nlohmann::json object containing an array of device data parsed from the buffer.
     */
    static json data2json(uint8_t *data);

#if CONFIG_LOG_DEFAULT_LEVEL > 2
    /**
     * @brief Debug output of data
     *
     * Prints information about found MAC addresses and iBeacons to the log for debugging.
     */
    void debug();
#else
    /**
     * @brief Empty debug function (in release build)
     *
     * Stub for debug output in release build.
     */
    inline void debug() {};
#endif
};