/*!
	\file
	\brief Класс задачи под BT5 LE.
	\authors Близнец Р.А.(r.bliznets@gmail.com)
	\version 0.1.0.0
	\date 17.11.2023

	Один объект на приложение.
*/

#pragma once

#include "sdkconfig.h"
#ifdef CONFIG_BT_NIMBLE_ENABLED

#include "CBaseTask.h"

#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#ifdef CONFIG_ESP_TASK_WDT
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY
#endif

#define MSG_END_TASK (0) ///< Команда завершения задачи.
#ifdef CONFIG_BLE_DATA_IBEACON
#define MSG_INIT_BEACON_TX (10) ///< Команда инициализации режима iBeacon.
#define MSG_INIT_BEACON_RX (11) ///< Команда инициализации режима iBeacon.
#endif
#define MSG_INIT_DATA (2)  ///< Команда инициализации режима потоковых каналов.
#define MSG_OFF (3)		   ///< Команда отключения BT.
#define MSG_WRITE_DATA (4) ///< Сообщение для записи данных в основной канал.
#define MSG_READ_DATA (5)  ///< Сообщение для чтения данных из основного канала.

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
#define MSG_READ_DATA2 (16)	 ///< Сообщение для чтения данных из второго канала.
#define MSG_INIT_DATA2 (17)	 ///< Команда установки callback функции на прием данных из второго канала.
#define MSG_WRITE_DATA2 (18) ///< Сообщение для записи данных во второй канал.
#define MSG_SKIP_WRITE (19)	 ///< Команда отмены записи во второй канал.
#endif

#define BTTASK_NAME "bt"			///< Имя задачи для отладки.
#define BTTASK_STACKSIZE (4 * 1024) ///< Размер стека задачи.
#define BTTASK_PRIOR (2)			///< Приоритет задачи.
#define BTTASK_LENGTH (30)			///< Длина приемной очереди задачи.
#ifdef CONFIG_BLE_DATA_TASK0
#define BTTASK_CPU (0) ///< Номер ядра процессора.
#else
#define BTTASK_CPU (1) ///< Номер ядра процессора.
#endif

/// Режимы работы BT.
enum class EBTMode
{
	Off, ///< Выключено.
#ifdef CONFIG_BLE_DATA_IBEACON
	iBeaconTx, ///< iBeacon.
	iBeaconRx, ///< iBeacon.
#endif
	Data ///< Обмен данными.
};

/// Функция события приема данных.
/*!
 * \param[in] data данные.
 * \param[in] size размер данных.
 */
typedef void onBLEDataRx(uint8_t *data, size_t size);
typedef void onBeaconRx(uint8_t *id, uint16_t major, uint16_t minor, int8_t power, int8_t rssi);

/// Класс логики работы канала данных по BLE.
class CBTTask : public CBaseTask
{
private:
	static CBTTask *theSingleInstance;	///< Единственный объект.
	static const ble_uuid16_t svs_uuid; ///< uuid сервиса.
	static const ble_uuid16_t chr_uuid; ///< uuid основного канала.
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	static const ble_uuid16_t chr_uuid2; ///< uuid второго канала.
#endif

	static const struct ble_gatt_svc_def gatt_svr_svcs[]; ///< Настройка сервиса.
	static const struct ble_gatt_chr_def gatt_svr_chrs[]; ///< Настройка характеристик сервиса.

protected:
	EBTMode mMode = EBTMode::Off; ///< Текущий режим работы.
	bool mConnect = false;		  ///< Флаг соединения.
	onBLEDataRx *mOnRx = nullptr; ///< Callback функция события приема данных на основном канале.
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	onBLEDataRx *mOnRx2 = nullptr; ///< Callback функция события приема данных на втором канале.
#endif
	uint8_t own_addr_type; ///< Тип адреса BLE.

	/// Установить режим работы.
	/*!
		Инициализирует Nimble
	  \param[in] mode режим работы.
	  \return текущий режим работы
	*/
	EBTMode init_bt(EBTMode mode);
	/// Выключить.
	/*!
		Отключает Nimble
	*/
	void deinit_bt();

#ifdef CONFIG_BLE_DATA_IBEACON
	uint8_t mBeaconID[16];	   ///< Поле ID. Берется из nvs "beacon" (blob) или случайный.
	uint16_t mBeaconMajor = 0; ///< Поле Major. Задается при инициализации режима.
	uint16_t mBeaconMinor = 0; ///< Поле Minor. Задается при инициализации режима.
	uint8_t mBeaconTx = 0;	   ///< Поле мощности на 1м. Берется из nvs "btx" (u8).

	onBeaconRx *mOnBeacon = nullptr; ///< Callback функция события приема данных от маяка.

	static void ble_on_sync_rx();
	static void ble_scan();
	static int ble_rx_gap_event(struct ble_gap_event *event, void *arg);

	/// Stack sync callback (iBeacon).
	static void ble_on_sync_beacon();
	/// Begin advertising (iBeacon).
	static void ble_advertise_beacon();
	/// Установка случайного адреса (iBeacon).
	static void ble_app_set_addr();
#endif

	/// Запуск Nimble.
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
	static void ble_on_sync_data();
	/// Begin advertising.
	static void ble_advertise_data();
	/// Callback от GAP.
	static int ble_server_gap_event(struct ble_gap_event *event, void *arg);
	/// Инициализация сервиса.
	static int gatt_svr_init();
	/// Callback function для сервиса основного канала.
	static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	/// Callback function для сервиса второго канала.
	static int ble_svc_gatt_handler2(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
#endif
	/// Callback function для чтения данных из канала.
	/*!
	  \param[in] om данные.
	  \param[in] chn номер канала.
	  \return 0 если без ошибки.
	*/
	static int gatt_svr_chr_write(struct os_mbuf *om, uint16_t chn = 1);

	/// Конструктор.
	CBTTask();
	/// Деструктор.
	virtual ~CBTTask();

	/// Функция задачи.
	virtual void run() override;

	using CBaseTask::sendCmd;

public:
	/// Единственный экземпляр класса.
	/*!
	  \return Указатель на CBTTask
	*/
	static CBTTask *Instance();
	/// Освобождение ресурсов.
	static void free();

#ifdef CONFIG_BLE_DATA_IBEACON
	/// Включить режим iBeacon.
	/*!
	  \param[in] major поле major.
	  \param[in] minor поле minor.
	  \return true если без ошибки.
	*/
	inline bool setBeacon(uint16_t major, uint16_t minor)
	{
		return sendCmd(MSG_INIT_BEACON_TX, major, minor);
	};
	inline bool setBeacon(onBeaconRx *onBeacon)
	{
		return sendCmd(MSG_INIT_BEACON_RX, 0, (uint32_t)onBeacon);
	};
#endif
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	/// Включить режим каналов данных.
	/*!
	  \param[in] onRx callback на прием данных основного канала.
	  \param[in] onRx2 callback на прием данных второго канала.
	  \return true если без ошибки.
	*/
	inline bool setData(onBLEDataRx *onRx, onBLEDataRx *onRx2)
	{
		sendCmd(MSG_INIT_DATA2, 0, (uint32_t)onRx2);
		return sendCmd(MSG_INIT_DATA, 0, (uint32_t)onRx);
	};
	/// Посылка данных во второй канал.
	/*!
	  \param[in] data данные.
	  \param[in] size размер данных.
	  \param[in] index номер пакета данных.
	  \param[in] xTicksToWait время таймаута очереди сообщений.
	  \return true если без ошибки.
	*/
	bool sendData2(uint8_t *data, size_t size, uint16_t index, TickType_t xTicksToWait = portMAX_DELAY);
#else
	/// Включить режим канала данных.
	/*!
	  \param[in] onRx callback на прием данных основного канала.
	  \return true если без ошибки.
	*/
	inline bool setData(onBLEDataRx *onRx)
	{
		return sendCmd(MSG_INIT_DATA, 0, (uint32_t)onRx);
	};
#endif
	/// Посылка данных в основной канал.
	/*!
	  \param[in] data данные.
	  \param[in] size размер данных.
	  \param[in] xTicksToWait время таймаута очереди сообщений.
	  \return true если без ошибки.
	*/
	bool sendData(uint8_t *data, size_t size, TickType_t xTicksToWait = portMAX_DELAY);
};
#endif
