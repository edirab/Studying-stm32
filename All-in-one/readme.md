### Домашние часы-термометр

Проект по объединению всех предыдущих наработок:

- Записи данных на SD-карту
- Опроса DHT11
- Вывода на LCD I2C 16x2
- Опроса часов реального времени DS3231 
- Опрос датчика даввления и температуры BMP280

**Сложности отладки**
![](./debug.PNG)

    main.c
        find_volume() at ff.c 2 317
            disk_initialize() at diskio.c 99
                USER_initialize() at user_diskio.c 85
                    SD_disk_initialize() at fatfs_sd.c 280
                        SD_PowerOn() at fatfs_sd.c 90