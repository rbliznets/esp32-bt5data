/*!
	\file
	\brief Класс задачи под BT5 LE.
	\authors Близнец Р.А.(r.bliznets@gmail.com)
	\version 0.2.0.0
	\date 17.11.2023

	Один объект на приложение.
*/

#pragma once

#include "sdkconfig.h"
#ifdef CONFIG_BT_NIMBLE_ENABLED

#include "CBaseTask.h"
#include "CLock.h"
#include "CSoftwareTimer.h"

#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include <array>

#ifdef CONFIG_ESP_TASK_WDT
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY
#endif

#define MSG_END_TASK (0) ///< Команда завершения задачи.
#ifdef CONFIG_BLE_DATA_IBEACON_TX
#define MSG_INIT_BEACON_TX (10) ///< Команда инициализации режима iBeacon.
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
#define MSG_INIT_BEACON_RX (11) ///< Команда инициализации режима сканирования iBeacon.
#define MSG_BEACON_DATA (12)	///< Сообщение с данными iBeacon.
#define MSG_BEACON_TIMER (13)	///< Сообщение таймера iBeacon.
#define MSG_MAC_DATA (14)		///< Сообщение с MAC адресом устройства.
#endif
#define MSG_INIT_DATA (2)	 ///< Команда инициализации режима потоковых каналов.
#define MSG_OFF (3)			 ///< Команда отключения BT.
#define MSG_WRITE_DATA (4)	 ///< Сообщение для записи данных в основной канал.
#define MSG_READ_DATA (5)	 ///< Сообщение для чтения данных из основного канала.
#define MSG_SET_ADV_DATA (6) ///< Установить данные для поля Manufacturer specific data в advertizing.

#define MSG_INIT_DATA3 (15) ///< Команда установки callback функции на соединение.
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
#ifdef CONFIG_BLE_DATA_IBEACON_TX
	iBeaconTx, ///< Режим передатчика iBeacon.
#endif
#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
	iBeaconRx, ///< Режим приемника iBeacon.
#endif
	Data ///< Режим обмена данными.
};

/**
 * @brief Структура данных iBeacon
 *
 * Содержит информацию о маяке iBeacon, включая UUID, major, minor номера,
 * мощность сигнала и уровень принимаемого сигнала (RSSI).
 */
struct SBeacon
{
	std::array<uint8_t, 16> uuid; ///< Уникальный идентификатор маяка (16 байт)
	uint16_t major;				  ///< Major номер (группа маяков)
	uint16_t minor;				  ///< Minor номер (конкретный маяк в группе)
	int8_t power;				  ///< Мощность передатчика на расстоянии 1 метр
	int8_t rssi;				  ///< Уровень принимаемого сигнала

	/**
	 * @brief Оператор сравнения двух маяков
	 * @param other Второй маяк для сравнения
	 * @return true если UUID совпадают, false в противном случае
	 */
	bool operator==(const SBeacon &other) const
	{
		return this->uuid == other.uuid;
	}
};

/**
 * @brief Структура MAC адреса Bluetooth устройства
 *
 * Содержит MAC адрес устройства и уровень принимаемого сигнала.
 */
struct SMac
{
	std::array<uint8_t, 6> mac; ///< MAC адрес устройства (6 байт)
	int8_t rssi;				///< Уровень принимаемого сигнала

	/**
	 * @brief Оператор сравнения двух MAC адресов
	 * @param other Второй MAC адрес для сравнения
	 * @return true если MAC адреса совпадают, false в противном случае
	 */
	bool operator==(const SMac &other) const
	{
		return this->mac == other.mac;
	}
};

/// Функция события приема данных.
/*!
 * \param[in] data данные.
 * \param[in] size размер данных.
 */
typedef void onBLEDataRx(uint8_t *data, size_t size);

/**
 * @brief Callback функция для обработки данных от iBeacon
 *
 * Вызывается при обнаружении iBeacon или MAC адреса Bluetooth устройства.
 *
 * @param[in] data Указатель на структуру данных iBeacon (может быть nullptr)
 * @param[in] mac Указатель на структуру MAC адреса (может быть nullptr)
 */
typedef void onBeaconRx(SBeacon *data, SMac *mac);

/**
 * @brief Callback функция для обработки событий соединения
 *
 * Вызывается при установке или разрыве соединения Bluetooth.
 *
 * @param[in] connected true если соединение установлено, false если разорвано
 */
typedef void onBLEConnect(bool connected);

/// Класс логики работы канала данных по BLE.
class CBTTask : public CBaseTask, CLock
{
private:
	static CBTTask *theSingleInstance;	///< Единственный объект.
	static const ble_uuid16_t svs_uuid; ///< UUID сервиса.
	static const ble_uuid16_t chr_uuid; ///< UUID основного канала.
#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	static const ble_uuid16_t chr_uuid2; ///< UUID второго канала.
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
	onBLEConnect *mOnConnect = nullptr; ///< Callback функция события соединения.

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

#ifdef CONFIG_BT_NIMBLE_EXT_ADV
	ble_addr_t mAddr = {0, {0, 0, 0, 0, 0, 0}}; ///< Адрес устройства для расширенной рекламы
#endif

#ifdef CONFIG_BLE_DATA_IBEACON_TX
	uint8_t mBeaconID[16];	   ///< Поле ID. Берется из nvs "beacon" (blob) или случайный.
	uint16_t mBeaconMajor = 0; ///< Поле Major. Задается при инициализации режима.
	uint16_t mBeaconMinor = 0; ///< Поле Minor. Задается при инициализации режима.
	uint8_t mBeaconTx = 0;	   ///< Поле мощности на 1м. Берется из nvs "btx" (u8).

	/// Stack sync callback (iBeacon).
	/*!
	 * @brief Callback синхронизации стека для режима iBeacon передатчика
	 *
	 * Вызывается при синхронизации стека BLE в режиме передатчика iBeacon.
	 */
	static void ble_on_sync_beacon();

	/// Begin advertising (iBeacon).
	/*!
	 * @brief Начало рекламы в режиме iBeacon
	 *
	 * Запускает передачу рекламных пакетов в формате iBeacon.
	 */
	static void ble_advertise_beacon();

	/// Установка случайного адреса (iBeacon).
	/*!
	 * @brief Установка случайного адреса устройства для iBeacon
	 *
	 * Генерирует и устанавливает случайный неразрешаемый частный адрес.
	 */
	static void ble_app_set_addr();
#endif

#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
	onBeaconRx *mOnBeacon = nullptr;		///< Callback функция события приема данных от маяка.
	uint32_t mBeaconSleepTime = 5000;		///< Время сна между сканированиями (мс)
	CSoftwareTimer *mBeaconTimer = nullptr; ///< Таймер для управления сканированием
	bool mBeaconSleep = false;				///< Флаг режима сна сканера

	/**
	 * @brief Callback синхронизации стека для режима iBeacon приемника
	 *
	 * Вызывается при синхронизации стека BLE в режиме приемника iBeacon.
	 */
	static void ble_on_sync_rx();

	/**
	 * @brief Запуск сканирования BLE устройств
	 *
	 * Инициирует процесс сканирования окружающих BLE устройств.
	 */
	static void ble_scan();

	/**
	 * @brief Обработчик событий GAP для сканирования
	 *
	 * Обрабатывает события обнаружения BLE устройств и iBeacon маяков.
	 *
	 * @param[in] event Указатель на структуру события GAP
	 * @param[in] arg Дополнительные аргументы (не используются)
	 * @return Код возврата (0 при успешной обработке)
	 */
	static int ble_rx_gap_event(struct ble_gap_event *event, void *arg);
#endif

	uint8_t *mManufacturerData = nullptr; ///< Данные производителя для рекламы
	uint8_t mManufacturerDataSize = 0;	  ///< Размер данных производителя

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
	/*!
	 * @brief Callback синхронизации стека для режима передачи данных
	 *
	 * Вызывается при синхронизации стека BLE в режиме передачи данных.
	 */
	static void ble_on_sync_data();

	/// Begin advertising.
	/*!
	 * @brief Начало рекламы в режиме передачи данных
	 *
	 * Запускает передачу рекламных пакетов в режиме передачи данных.
	 */
	static void ble_advertise_data();

	/// Callback от GAP.
	/*!
	 * @brief Обработчик событий GAP для сервера данных
	 *
	 * Обрабатывает события подключения, отключения и других событий GAP.
	 *
	 * @param[in] event Указатель на структуру события GAP
	 * @param[in] arg Дополнительные аргументы (не используются)
	 * @return Код возврата (0 при успешной обработке)
	 */
	static int ble_server_gap_event(struct ble_gap_event *event, void *arg);

	/// Инициализация сервиса.
	/*!
	 * @brief Инициализация GATT сервиса для передачи данных
	 *
	 * Регистрирует пользовательский GATT сервис и его характеристики.
	 *
	 * @return Код возврата (0 при успешной инициализации)
	 */
	static int gatt_svr_init();

	/// Callback function для сервиса основного канала.
	/*!
	 * @brief Обработчик доступа к характеристике основного канала
	 *
	 * Обрабатывает операции чтения/записи для основного канала данных.
	 *
	 * @param[in] conn_handle Идентификатор соединения
	 * @param[in] attr_handle Идентификатор атрибута
	 * @param[in] ctxt Контекст доступа к GATT
	 * @param[in] arg Дополнительные аргументы
	 * @return Код возврата (0 при успешной обработке)
	 */
	static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	/// Callback function для сервиса второго канала.
	/*!
	 * @brief Обработчик доступа к характеристике второго канала
	 *
	 * Обрабатывает операции чтения/записи для второго канала данных.
	 *
	 * @param[in] conn_handle Идентификатор соединения
	 * @param[in] attr_handle Идентификатор атрибута
	 * @param[in] ctxt Контекст доступа к GATT
	 * @param[in] arg Дополнительные аргументы
	 * @return Код возврата (0 при успешной обработке)
	 */
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
	/*!
	 * @brief Конструктор класса CBTTask
	 *
	 * Создает новый экземпляр класса Bluetooth задачи.
	 */
	CBTTask();

	/// Деструктор.
	/*!
	 * @brief Деструктор класса CBTTask
	 *
	 * Освобождает ресурсы, используемые задачей Bluetooth.
	 */
	virtual ~CBTTask();

	/// Функция задачи.
	/*!
	 * @brief Основная функция выполнения задачи Bluetooth
	 *
	 * Обрабатывает все сообщения и события Bluetooth в цикле задачи.
	 */
	virtual void run() override;

	using CBaseTask::sendCmd;

public:
	static const char *device_name; ///< Имя устройства Bluetooth

	inline EBTMode getMode(){return mMode;};

	/// Единственный экземпляр класса.
	/*!
	  \return Указатель на CBTTask
	*/
	static CBTTask *Instance();

	/// Освобождение ресурсов.
	/*!
	 * @brief Освобождение ресурсов и удаление экземпляра класса
	 *
	 * Завершает задачу Bluetooth и освобождает все связанные ресурсы.
	 */
	static void free();

	/**
	 * @brief Проверка, запущена ли задача Bluetooth
	 *
	 * @return true если задача запущена, false если нет
	 */
	static inline bool isRun() { return (theSingleInstance != nullptr); };

#ifdef CONFIG_BLE_DATA_IBEACON_TX
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
#endif

#ifdef CONFIG_BLE_DATA_IBEACON_SCAN
	/**
	 * @brief Установка режима сканирования iBeacon
	 *
	 * @param[in] onBeacon Callback функция для обработки обнаруженных маяков
	 * @param[in] sleep Время сна между сканированиями в секундах
	 * @return true если команда отправлена успешно
	 */
	inline bool setBeacon(onBeaconRx *onBeacon, uint16_t sleep = 5)
	{
		return sendCmd(MSG_INIT_BEACON_RX, sleep, (uint32_t)onBeacon);
	};
#endif

#ifdef CONFIG_BLE_DATA_SECOND_CHANNEL
	/// Включить режим каналов данных.
	/*!
	  \param[in] onRx callback на прием данных основного канала.
	  \param[in] onRx2 callback на прием данных второго канала.
	  \return true если без ошибки.
	*/
	inline bool setData(onBLEDataRx *onRx, onBLEDataRx *onRx2, onBLEConnect *onConnect = nullptr)
	{
		sendCmd(MSG_INIT_DATA3, 0, (uint32_t)onConnect);
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
	inline bool setData(onBLEDataRx *onRx, onBLEConnect *onConnect = nullptr)
	{
		sendCmd(MSG_INIT_DATA3, 0, (uint32_t)onConnect);
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

	/// Установить данные для advertizing.
	/*!
	  \param[in] data данные.
	  \param[in] size размер данных.
	  \param[in] xTicksToWait время таймаута очереди сообщений.
	  \return true если без ошибки.
	*/
	bool setManufacturerData(uint8_t *data, size_t size, TickType_t xTicksToWait = portMAX_DELAY);
};
#endif