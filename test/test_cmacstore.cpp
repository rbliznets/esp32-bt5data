/*!
	\file
	\brief Модульные тесты.
	\authors Близнец Р.А.
	\version 0.0.0.1
	\date 05.07.2026
*/

#include <cstring>
#include "unity.h"
#include "unity_test_utils_memory.h"
#include "CMacStore.h"
#include "sdkconfig.h"
#include "esp_heap_caps.h"

static SMac makeMac(uint8_t last, int8_t rssi)
{
	SMac m;
	m.mac = {0x01, 0x02, 0x03, 0x04, 0x05, last};
	m.rssi = rssi;
	return m;
}

static SBeacon makeBeacon(uint16_t major, uint16_t minor, int8_t rssi)
{
	SBeacon b;
	b.uuid.fill(0xAA);
	b.major = major;
	b.minor = minor;
	b.power = -59;
	b.rssi = rssi;
	return b;
}

// Regression test for a heap buffer overflow: getData() used to allocate a
// buffer that was 1 byte too small whenever the beacon list was empty but the
// MAC list was not, while still unconditionally writing the beacon-count byte.
TEST_CASE("CMacStore_getData_macsOnly", "[ble]")
{
	unity_utils_record_free_mem();

	CMacStore *store = new CMacStore(true, true);
	SMac m = makeMac(0x10, -40);
	store->addMac(&m);
	// First calculate(): MAC appeared -> old/new lists swap, returns true.
	TEST_ASSERT_TRUE(store->calculate());

	uint16_t size = 0;
	uint8_t *data = store->getData(size);
	TEST_ASSERT_NOT_NULL(data);
	TEST_ASSERT_EQUAL_UINT16(3 + 7, size); // header(3) + 1 MAC(7)
	TEST_ASSERT_EQUAL_UINT8(0x08, data[0]);
	TEST_ASSERT_EQUAL_UINT8(1, data[1]); // MAC count
	TEST_ASSERT_EQUAL_UINT8(0, data[9]); // Beacon count, right after the 1 MAC entry (2 + 7)
	delete[] data;
	delete store;

#ifdef CONFIG_HEAP_POISONING_COMPREHENSIVE
	// Would previously fail: getData() wrote one byte past the allocated buffer.
	TEST_ASSERT_TRUE(heap_caps_check_integrity_all(true));
#endif

	unity_utils_evaluate_leaks_direct(0);
}

TEST_CASE("CMacStore_getData_beaconsOnly", "[ble]")
{
	unity_utils_record_free_mem();

	CMacStore *store = new CMacStore(true, true);
	SBeacon b = makeBeacon(1, 2, -50);
	store->addBeacon(&b);
	TEST_ASSERT_TRUE(store->calculate());

	uint16_t size = 0;
	uint8_t *data = store->getData(size);
	TEST_ASSERT_NOT_NULL(data);
	TEST_ASSERT_EQUAL_UINT16(3 + 22, size); // header(3) + 1 beacon(22)
	TEST_ASSERT_EQUAL_UINT8(0, data[1]);    // MAC count
	TEST_ASSERT_EQUAL_UINT8(1, data[2]);    // Beacon count
	delete[] data;
	delete store;

#ifdef CONFIG_HEAP_POISONING_COMPREHENSIVE
	TEST_ASSERT_TRUE(heap_caps_check_integrity_all(true));
#endif

	unity_utils_evaluate_leaks_direct(0);
}

TEST_CASE("CMacStore_getData_empty", "[ble]")
{
	unity_utils_record_free_mem();

	CMacStore *store = new CMacStore(true, true);
	// No devices ever added -> calculate() reports no changes.
	TEST_ASSERT_FALSE(store->calculate());

	uint16_t size = 0xffff;
	uint8_t *data = store->getData(size);
	TEST_ASSERT_NULL(data);
	delete store;

	unity_utils_evaluate_leaks_direct(0);
}

TEST_CASE("CMacStore_getData_macAndBeacon_roundtrip", "[ble]")
{
	unity_utils_record_free_mem();

	CMacStore *store = new CMacStore(true, true);
	SMac m = makeMac(0x20, -33);
	store->addMac(&m);
	SBeacon b = makeBeacon(7, 8, -44);
	store->addBeacon(&b);
	TEST_ASSERT_TRUE(store->calculate());

	uint16_t size = 0;
	uint8_t *data = store->getData(size);
	TEST_ASSERT_NOT_NULL(data);
	TEST_ASSERT_EQUAL_UINT16(3 + 7 + 22, size);

	{
		json parsed = CMacStore::data2json(data);
		TEST_ASSERT_EQUAL_INT(2, parsed.size());
	}
	delete[] data;
	delete store;

#ifdef CONFIG_HEAP_POISONING_COMPREHENSIVE
	TEST_ASSERT_TRUE(heap_caps_check_integrity_all(true));
#endif

	unity_utils_evaluate_leaks_direct(0);
}
