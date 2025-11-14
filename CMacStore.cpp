/*!
    \file
    \brief BLE Location.
    \authors Bliznets R.A.(r.bliznets@gmail.com)
    \version 1.0.0.0
    \date 16.09.2025
*/
#include "CMacStore.h"
#include <cstring>
#include <algorithm>
#include "esp_log.h"

#if CONFIG_LOG_DEFAULT_LEVEL > 2
static const char *TAG = "CMacStore"; ///< Tag for logging
#endif

/**
 * @brief Constructor for the CMacStore class
 *
 * Initializes the MAC address and iBeacon data storage with the given parameters.
 * It ensures that at least one of the tracking modes (iBeacon or MAC) is enabled.
 * Allocates memory for four lists: two for storing data from the previous scan (old)
 * and two for storing data from the current scan (new).
 *
 * @param beacon Flag to enable iBeacon tracking
 * @param mac Flag to enable MAC address tracking
 * @param white Pointer to the MAC address whitelist (can be nullptr).
 *              If provided, only MAC addresses in this list will be stored.
 */
CMacStore::CMacStore(bool beacon, bool mac, std::list<std::array<uint8_t, 6>> *white) : mBeaconEnable(beacon), mMacEnable(mac), mWhiteList(white)
{
    // Check that at least one mode is enabled. This is a critical requirement.
    assert(beacon || mac);

    // Create lists to store data from old and new scans
    mOldBeacons = new std::list<SBeacon>; ///< List of iBeacons from the previous scan
    mNewBeacons = new std::list<SBeacon>; ///< List of iBeacons from the current scan
    mOldMacs = new std::list<SMac>;       ///< List of MAC addresses from the previous scan
    mNewMacs = new std::list<SMac>;       ///< List of MAC addresses from the current scan
}

/**
 * @brief Destructor for the CMacStore class
 *
 * Frees the memory allocated for the data lists (old/new for both MACs and Beacons).
 * Also frees the memory for the white list if it was provided during construction.
 */
CMacStore::~CMacStore()
{
    // Free the memory of all dynamically allocated lists
    delete mNewMacs;    // Delete the list storing current scan MACs
    delete mOldMacs;    // Delete the list storing previous scan MACs
    delete mNewBeacons; // Delete the list storing current scan Beacons
    delete mOldBeacons; // Delete the list storing previous scan Beacons
    // Free the whitelist if it was created and stored
    if (mWhiteList != nullptr)
        delete mWhiteList;
}

/**
 * @brief Add iBeacon data to the storage
 *
 * Adds information about a found iBeacon to the list of new data (mNewBeacons),
 * if iBeacon tracking is enabled (mBeaconEnable is true).
 * The data is copied into the list.
 *
 * @param[in] data Pointer to the SBeacon structure containing the iBeacon information.
 *                 This pointer is expected to be valid and pointing to a complete SBeacon object.
 */
void CMacStore::addBeacon(SBeacon *data)
{
    if (mBeaconEnable)
    {
        // Add the received data to the new iBeacon list
        mNewBeacons->push_back(*data);
    }
    // If mBeaconEnable is false, the data is simply ignored.
}

/**
 * @brief Add MAC address to the storage
 *
 * Adds a MAC address and its RSSI value to the list of new data (mNewMacs),
 * if MAC address tracking is enabled (mMacEnable is true).
 * If a whitelist (mWhiteList) is provided, the MAC address is only added if it exists in the whitelist.
 * The data is copied into the list.
 *
 * @param[in] mac Pointer to the SMac structure containing the MAC address and RSSI.
 *                This pointer is expected to be valid and pointing to a complete SMac object.
 */
void CMacStore::addMac(SMac *mac)
{
    if (mMacEnable)
    {
        // Check if there is a whitelist configured
        if (mWhiteList != nullptr)
        {
            // Search for the MAC address in the whitelist
            // std::find returns an iterator to the end if the element is not found
            if (std::find(mWhiteList->begin(), mWhiteList->end(), mac->mac) != mWhiteList->end())
            {
                // If the address is found in the whitelist, add it to the new list
                mNewMacs->push_back(*mac);
            }
            // If the address is not in the whitelist, it is ignored.
        }
        else
        {
            // If no whitelist is configured, add all MAC addresses to the new list
            mNewMacs->push_back(*mac);
        }
    }
    // If mMacEnable is false, the data is simply ignored.
}

/**
 * @brief Calculate changes in scan data
 *
 * Compares the data from the current scan (mNewMacs, mNewBeacons) with the data from
 * the previous scan (mOldMacs, mOldBeacons). It checks for additions and removals.
 * If any changes are detected, it swaps the old and new lists (so the new data becomes the old data
 * for the next comparison) and returns true. The new lists are cleared ready for the next scan.
 *
 * @return true if changes (additions or removals) are detected between the old and new scans,
 *         false if the sets of MAC addresses and iBeacons are identical.
 */
bool CMacStore::calculate()
{
    bool res = false; // Flag indicating if any changes were found

    // Check changes in iBeacon data if iBeacon tracking is enabled
    if (mBeaconEnable)
    {
        bool exit = false; // Flag for changes detected in iBeacon data

        // Check if any iBeacons from the previous scan are missing in the current scan (disappeared)
        for (auto &var : *mOldBeacons)
        {
            // If 'var' from old list is not found in the new list, it means it disappeared
            if (std::find(mNewBeacons->begin(), mNewBeacons->end(), var) == mNewBeacons->end())
            {
                exit = true; // A change (disappearance) was found
                break;       // No need to check further
            }
        }

        // If no disappearances were found, check if any *new* iBeacons appeared
        if (!exit)
        {
            for (auto &var : *mNewBeacons)
            {
                // If 'var' from new list is not found in the old list, it means it appeared
                if (std::find(mOldBeacons->begin(), mOldBeacons->end(), var) == mOldBeacons->end())
                {
                    exit = true; // A change (appearance) was found
                    break;       // No need to check further
                }
            }
        }

        // If changes were detected in the iBeacon data
        if (exit)
        {
            res = true; // Indicate that overall changes were found
            // Swap the old and new data lists for iBeacons
            std::list<SBeacon> *tmp = mOldBeacons; // Temporary pointer
            mOldBeacons = mNewBeacons;             // Old list now points to new data
            mNewBeacons = tmp;                     // New list now points to old data (for reuse)
        }
        // Clear the new data list (which now holds the old data after swap or not) for the next scan cycle
        mNewBeacons->clear();
    }

    // Check changes in MAC address data if MAC tracking is enabled
    if (mMacEnable)
    {
        bool exit = false; // Flag for changes detected in MAC data

        // Check if any MAC addresses from the previous scan are missing in the current scan (disappeared)
        for (auto &var : *mOldMacs)
        {
            if (std::find(mNewMacs->begin(), mNewMacs->end(), var) == mNewMacs->end())
            {
                exit = true; // A change (disappearance) was found
                break;       // No need to check further
            }
        }

        // If no disappearances were found, check if any *new* MAC addresses appeared
        if (!exit)
        {
            for (auto &var : *mNewMacs)
            {
                if (std::find(mOldMacs->begin(), mOldMacs->end(), var) == mOldMacs->end())
                {
                    exit = true; // A change (appearance) was found
                    break;       // No need to check further
                }
            }
        }

        // If changes were detected in the MAC address data
        if (exit)
        {
            res = true; // Indicate that overall changes were found
            // Swap the old and new data lists for MACs
            std::list<SMac> *tmp = mOldMacs; // Temporary pointer
            mOldMacs = mNewMacs;             // Old list now points to new data
            mNewMacs = tmp;                  // New list now points to old data (for reuse)
        }
        // Clear the new data list (which now holds the old data after swap or not) for the next scan cycle
        mNewMacs->clear();
    }

    return res; // Return true if any changes were found in either MAC or Beacon data
}

#if CONFIG_LOG_DEFAULT_LEVEL > 2
/**
 * @brief Debug output of data
 *
 * Prints information about the MAC addresses and iBeacons stored in the 'old' lists
 * (mOldMacs, mOldBeacons) to the ESP log. This is useful for debugging and verifying
 * the data collection and comparison logic.
 */
void CMacStore::debug()
{
    ESP_LOGI(TAG, "== olds ==");
    // Print information about MAC addresses from the previous scan
    for (auto &var : *mOldMacs)
    {
        ESP_LOGW(TAG, "mac rssi:%d dBm", var.rssi); // Log RSSI value
        ESP_LOG_BUFFER_HEX(TAG, var.mac.data(), 6); // Log the 6-byte MAC address in hex
    }

    // Print information about iBeacons from the previous scan
    for (auto &var : *mOldBeacons)
    {
        ESP_LOGE(TAG, "iBeacon %d:%d rssi:%d dBm, pwr: %d dBm", var.major, var.minor, var.rssi, var.power);
        ESP_LOG_BUFFER_HEX(TAG, var.uuid.data(), 16); // Log the 16-byte UUID in hex
    }
}
#endif

/**
 * @brief Get serialized data
 *
 * Forms a binary buffer containing data about the devices found in the *previous* scan
 * (stored in mOldMacs and mOldBeacons). The buffer format is:
 * [Header: 3 bytes][MAC data: N * 7 bytes][iBeacon data: M * 22 bytes].
 * The caller is responsible for freeing the returned buffer using delete[].
 *
 * @param[out] size Reference to a uint16_t where the size of the allocated buffer will be stored.
 * @return Pointer to the allocated data buffer (needs to be freed with delete[] by the caller),
 *         or nullptr if no data is available (size would be 3 in that case).
 */
uint8_t *CMacStore::getData(uint16_t &size)
{
    // Calculate the total size of the output buffer:
    // 3 bytes header + (number of MACs in old list * 7 bytes per MAC [6 for addr + 1 for RSSI])
    //               + (number of iBeacons in old list * 22 bytes per Beacon [16 UUID + 2 Major + 2 Minor + 1 Pwr + 1 RSSI])
    size = 2 + mOldMacs->size() * 7 + mOldBeacons->size() * 22;
    if (mOldBeacons->size() != 0)
        size++;

    // If there is no actual device data (only the 3-byte header), return nullptr
    if (size == 2)
        return nullptr;

    // Allocate memory for the data buffer based on the calculated size
    uint8_t *data = new uint8_t[size];

    // Form the header at the beginning of the buffer
    data[0] = 0x08;             // Data format identifier (arbitrary, defined by application protocol)
    data[1] = mOldMacs->size(); // Number of MAC addresses included in the buffer

    uint16_t index = 2; // Index for writing data into the buffer, starting after the 3-byte header

    // Copy MAC address data into the buffer
    for (auto &var : *mOldMacs)
    {
        std::memcpy(&data[index], var.mac.data(), 6); // Copy the 6-byte MAC address
        index += 6;                                   // Move index forward by 6 bytes
        data[index] = (uint8_t)var.rssi;              // Copy the RSSI value (cast to uint8_t)
        index++;                                      // Move index forward by 1 byte
    }

    data[index] = mOldBeacons->size(); // Number of iBeacons included in the buffer
    index++;

    // Copy iBeacon data into the buffer
    for (auto &var : *mOldBeacons)
    {
        std::memcpy(&data[index], var.uuid.data(), 16); // Copy the 16-byte UUID
        index += 16;                                    // Move index forward by 16 bytes
        std::memcpy(&data[index], &var.major, 2);       // Copy the 2-byte Major number (assumes little-endian storage in buffer)
        index += 2;                                     // Move index forward by 2 bytes
        std::memcpy(&data[index], &var.minor, 2);       // Copy the 2-byte Minor number (assumes little-endian storage in buffer)
        index += 2;                                     // Move index forward by 2 bytes
        data[index] = (uint8_t)var.power;               // Copy the power value (cast to uint8_t)
        index++;                                        // Move index forward by 1 byte
        data[index] = (uint8_t)var.rssi;                // Copy the RSSI value (cast to uint8_t)
        index++;                                        // Move index forward by 1 byte
    }

    return data; // Return the pointer to the allocated buffer
}

/**
 * @brief Get stored data as a JSON array
 *
 * Converts the data stored in the 'old' lists (mOldMacs, mOldBeacons) into a JSON array.
 * Each MAC address and iBeacon is represented as an individual JSON object within the array.
 *
 * @return A nlohmann::json object containing an array of device data.
 */
json CMacStore::getJSON()
{
    json beacon = json::array(); // Initialize the root JSON array

    // Process MAC addresses from the old list
    for (auto &var : *mOldMacs)
    {
        json j;               // Create a JSON object for this MAC entry
        std::string str = ""; // String to hold the MAC address in hex format
        char tmp[4];          // Temporary buffer for sprintf (e.g., "AA\0")
        // Convert each byte of the MAC address to a two-digit hex string
        for (auto &x : var.mac)
        {
            std::sprintf(tmp, "%02x", x); // Format byte as two-digit hex (e.g., 255 -> "ff")
            str += tmp;                   // Append to the full MAC string
        }
        j["mac"] = str;       // Add the formatted MAC string to the JSON object
        j["rssi"] = var.rssi; // Add the RSSI value
        beacon.push_back(j);  // Add this MAC's JSON object to the main array
    }

    // Process iBeacons from the old list
    for (auto &var : *mOldBeacons)
    {
        json j;               // Create a JSON object for this iBeacon entry
        std::string str = ""; // String to hold the UUID in hex format
        char tmp[4];          // Temporary buffer for sprintf
        // Convert each byte of the UUID to a two-digit hex string
        for (auto &x : var.uuid)
        {
            std::sprintf(tmp, "%02x", x); // Format byte as two-digit hex
            str += tmp;                   // Append to the full UUID string
        }
        j["uuid"] = str;        // Add the formatted UUID string to the JSON object
        j["major"] = var.major; // Add the Major number
        j["minor"] = var.minor; // Add the Minor number
        j["pwr"] = var.power;   // Add the power value
        j["rssi"] = var.rssi;   // Add the RSSI value
        beacon.push_back(j);    // Add this iBeacon's JSON object to the main array
    }

    return beacon; // Return the complete JSON array
}

/**
 * @brief Parse binary data buffer into a JSON array
 *
 * Takes a binary buffer previously created by getData() and converts it back into
 * a nlohmann::json array, reconstructing the MAC address and iBeacon information.
 *
 * @param data Pointer to the binary buffer (format: 3 byte header + device data).
 * @return A nlohmann::json object containing an array of device data parsed from the buffer.
 */
json CMacStore::data2json(uint8_t *data)
{
    json beacon = json::array();  // Initialize the root JSON array
    uint16_t szmac = data[1];     // Read the number of MAC addresses from the header (byte 1)
    uint16_t szgbeacon = data[2]; // Read the number of iBeacons from the header (byte 2)
    uint16_t index = 3;           // Start reading device data after the 3-byte header

    // Parse MAC address entries
    for (uint16_t i = 0; i < szmac; i++)
    {
        json j;               // Create a JSON object for this MAC entry
        std::string str = ""; // String to hold the MAC address in hex format
        char tmp[4];          // Temporary buffer for sprintf
        // Convert the next 6 bytes (the MAC address) to a hex string
        for (size_t i = 0; i < 6; i++)
        {
            std::sprintf(tmp, "%02x", data[index + i]); // Format byte as two-digit hex
            str += tmp;                                 // Append to the full MAC string
        }
        j["mac"] = str;                        // Add the formatted MAC string to the JSON object
        j["rssi"] = (int8_t)(data[index + 6]); // Read the RSSI value (byte after MAC address) and cast back to int8_t
        index += 7;                            // Move index forward by 7 bytes (6 for MAC + 1 for RSSI)
        beacon.push_back(j);                   // Add this MAC's JSON object to the main array
    }

    // Parse iBeacon entries
    for (uint16_t i = 0; i < szgbeacon; i++)
    {
        json j;               // Create a JSON object for this iBeacon entry
        std::string str = ""; // String to hold the UUID in hex format
        char tmp[4];          // Temporary buffer for sprintf
        // Convert the next 16 bytes (the UUID) to a hex string
        for (size_t i = 0; i < 16; i++)
        {
            std::sprintf(tmp, "%02x", data[index + i]); // Format byte as two-digit hex
            str += tmp;                                 // Append to the full UUID string
        }
        j["uuid"] = str; // Add the formatted UUID string to the JSON object
        // Read the Major number (2 bytes, little-endian format in buffer)
        j["major"] = (data[index + 16]) + (data[index + 17] << 8);
        // Read the Minor number (2 bytes, little-endian format in buffer)
        j["minor"] = (data[index + 18]) + (data[index + 19] << 8);
        j["pwr"] = (int8_t)(data[index + 20]);  // Read the power value and cast back to int8_t
        j["rssi"] = (int8_t)(data[index + 21]); // Read the RSSI value and cast back to int8_t
        index += 22;                            // Move index forward by 22 bytes (16 UUID + 2 Major + 2 Minor + 1 Pwr + 1 RSSI)
        beacon.push_back(j);                    // Add this iBeacon's JSON object to the main array
    }

    return beacon; // Return the complete JSON array reconstructed from the buffer
}

/**
 * @brief Clear the stored 'old' data lists.
 *
 * Empties the lists containing data from the previous scan (mOldBeacons and mOldMacs).
 * This is useful if you want to reset the stored state, for example, when starting a new tracking session.
 */
void CMacStore::clear()
{
    mOldBeacons->clear(); // Remove all elements from the old iBeacon list
    mOldMacs->clear();    // Remove all elements from the old MAC list
}