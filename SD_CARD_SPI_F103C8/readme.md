### Проект SD_CARD_SPI_F103C8. Об оптимизации компилятора

> Project -> Properties -> C/C++ Build -> Settings
> Вкладка Tool Setings -> MCU GCC Compiler -> (Debugging && Optimization)


1. Исходный проект (Debug Maximum (-g3), Optimization: None (-O0))
	
        RAM   4,24 KB -  21,21%
        FLASH 30,16 KB - 47,13%

2. Просто переключаем на конфигурацию **Release**

	    RAM   4,24 KB -  21,21%
	    FLASH 18,67 KB - 29,17%
	
3. Убрали удаление файлов: 

	    FLASH 18,24 KB - 28,50%
	
4. Убрано измерение объёма карты

	    FLASH 13,62 KB - 21,29%
	
	<details>
	
		f_getfree("", &fre_clust, &pfs);
		total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		sprintf (buffer, "SD CARD Total Size: \t%lu\n",total);
		send_uart(buffer);
		clear_buffer();
		free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
		sprintf (buffer, "SD CARD Free Space: \t%lu\n\n",free_space);
		send_uart(buffer);
		clear_buffer();
	
	</details>

5. Убраны все вызовы f_read & f_gets

	    FLASH 12,96 KB - 20,26%
	
После всех сокращений: 

|    Проект           |     Конфигурация  |    Объём Flash         | ОБъём RAM     |
|---                  | :--------------:  | -----                  | --------      |
| SD_CARD_SPI_F103C8  | Release           | **12,96 KB (20,26 %)**  | 4,11 KB (20,55%)
|                     | Debug             | 23,34 KB (36,47 %)      | 
| I2C_DS3231          | Release           | **9,94 KB (15,03 %)**   | 1,99 KB (9,94%)
|                     | Debug             | 9,66 KB (15,09 %)       | 
| DHT11_v1            | Release           | **7,67 KB (5,99 %)**    | 2,18 KB (10,92%)
|                     | Debug             | 16,96 KB (13,25 %)      | 
| Итого               | Release           | **30,57 KB (41,28 %)**  | 8,28 KB (41,4%)
|                     | Debug             |                         |
| adc_to_pwm          | Release           | 10,01 KB (15,64 %)      | 1,9 KB (9,49 %)  
|                     | Debug             |  KB ( %)                | 
| LCD_I2C             | Release           | 5,1 KB (7,97 %)         | 1,63 KB (8,16%) 
|                     | Debug             | 7,42 KB (11,59 %)       |  

NB: Каждый из этих проектов содержит модуль UART, а также строки в виде констант и буфер для их пересылки





