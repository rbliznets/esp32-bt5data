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

/**
 * @brief Класс для хранения и анализа данных BLE устройств
 *
 * Реализует функциональность отслеживания MAC адресов и iBeacon маяков,
 * включая сравнение данных между сканированиями и формирование отчетов.
 */
class CMacStore
{
protected:
    bool mBeaconEnable;                            ///< Флаг включения отслеживания iBeacon
    bool mMacEnable;                               ///< Флаг включения отслеживания MAC адресов
    std::list<std::array<uint8_t, 6>> *mWhiteList; ///< Белый список разрешенных MAC адресов

    std::list<SBeacon> *mOldBeacons; ///< Список iBeacon с предыдущего сканирования
    std::list<SBeacon> *mNewBeacons; ///< Список iBeacon с текущего сканирования
    std::list<SMac> *mOldMacs;       ///< Список MAC адресов с предыдущего сканирования
    std::list<SMac> *mNewMacs;       ///< Список MAC адресов с текущего сканирования

public:
    /**
     * @brief Конструктор класса CMacStore
     *
     * Создает новый экземпляр хранилища данных BLE устройств.
     *
     * @param[in] beacon Флаг включения отслеживания iBeacon (по умолчанию true)
     * @param[in] mac Флаг включения отслеживания MAC адресов (по умолчанию false)
     * @param[in] white Указатель на белый список MAC адресов (по умолчанию nullptr)
     */
    CMacStore(bool beacon = true, bool mac = false, std::list<std::array<uint8_t, 6>> *white = nullptr);

    /**
     * @brief Деструктор класса CMacStore
     *
     * Освобождает память, выделенную под списки данных.
     */
    ~CMacStore();

    /**
     * @brief Добавление данных iBeacon
     *
     * Добавляет информацию о найденном iBeacon в список текущего сканирования.
     *
     * @param[in] data Указатель на структуру данных iBeacon для добавления
     */
    void addBeacon(SBeacon *data);

    /**
     * @brief Добавление MAC адреса
     *
     * Добавляет MAC адрес в список текущего сканирования с проверкой по белому списку.
     *
     * @param[in] mac Указатель на структуру MAC адреса для добавления
     */
    void addMac(SMac *mac);

    /**
     * @brief Расчет изменений между сканированиями
     *
     * Сравнивает данные текущего и предыдущего сканирования для обнаружения изменений.
     *
     * @return true если обнаружены изменения, false если данные остались прежними
     */
    bool calculate();

    /**
     * @brief Получение сериализованных данных
     *
     * Формирует бинарный буфер с данными о найденных устройствах.
     *
     * @param[out] size Размер сформированного буфера данных
     * @return Указатель на буфер с данными (необходимо освободить delete[])
     */
    uint8_t *getData(uint16_t &size);

#if CONFIG_LOG_DEFAULT_LEVEL > 2
    /**
     * @brief Отладочный вывод данных
     *
     * Выводит в лог информацию о найденных MAC адресах и iBeacon для отладки.
     */
    void debug();
#else
    /**
     * @brief Пустая функция отладочного вывода (в release сборке)
     *
     * Заглушка для отладочного вывода в release сборке.
     */
    inline void debug() {};
#endif
};