# Домашняя метеостанция-часы

Проект по объединению всех предыдущих наработок:

- Опрос датчика давления и температуры [BMP280](../literature/sensors/bst-bmp280-ds001.pdf)   
- Опрос DHT11 (раз в 3 секунды, только влажность)
- Опрос часов реального времени [DS3231](../literature/sensors/DS3231.pdf)
- Вывод на LCD I2C 16x2
- АЦП для измерения двух фоторезисторов и датчика влажности почвы
- Отладка по USART
- Запись данных на SD-карту раз в 10 секунд с помощью библиотеки-прослойки FatFS в формате

```
Время    День Дата    Температура Влажность Давление     Вл. почвы Освещённость 1 и 2

21:56:00 MON 26/01/21 T = 26.96*C H = 40% P = 98539.83 Pa S = 4016 L1 = 1203 L2 = 1894
21:56:01 MON 26/01/21 T = 26.97*C H = 40% P = 98545.28 Pa S = 4025 L1 = 1209 L2 = 1903
21:56:02 MON 26/01/21 T = 26.95*C H = 40% P = 98540.83 Pa S = 4011 L1 = 1205 L2 = 1899
...
```

| |   |
| ---   | --- |
| **Release** |   |
| RAM         | 28,52%  |
| Flash       | 57,18%  |
| **Debug**   |   |
| RAM         | 28,52%  |
| Flash       | 84,20%  |

- Нормальный цикл измерения занимает 25 мс, из которых 20 мс МК ждёт ответа от DHT 11. 50 мс на отправку 880 байт по SPI

### Описание подключения

| STM32F103C8T6 | Название   | Комментарий |
| :-----:       | :------:   | -------     |
| PA4			| SPI1_NSS   | SPI1 для SD-карты  |
| PA5			| SPI1_SCK   | Аппаратное управление выводом ~CS|
| PA6			| SPI1_MISO  ||
| PA7			| SPI1_MOSI  ||
| PA0           | ADC0       |  Датчик влажности почвы |
| PA1           | ADC1       |  Фоторезисторы №1 и №2           |
| PA2           | ADC2       |   |
| PA13          | SWDIO      |  Внутрисхемный отладчик |
| PA14          | SWCLK      ||
| PB0           | Debug LED  | Светодиод для отладки   |
| PB3           | Set        | Кнопки настройки часов  |
| PB4           | Minus      ||
| PB5           | Plus       ||
| PB6           | I2C1_SCL   | Часы реального вмемени и  |
| PB7           | I2C1_SDA   | LCD 16x2 через расширитель портов|
| PB8           | DHT 11     ||
| PB10          | USART3_TX  | В целях отладки |
| PB11          | USART3_RX  ||


- Подключение BMP280

| BPM 280       | Назначение | Комментарий |
| :-----:       | :--------: | --- |
| SDO			| GND        | Мл. бит адреса 0x76 при работе с инт. I2C |
| CSB			| VCC        | Выбор интерфейса I2C |
| SDA			| PB7        ||
| SCL			| PB6        ||
| GND           | GND        ||
| VCC           | VCC        | +3.3V |

- Шкала аналоговых значений датчика влажности почвы

| Значение      | Комментарий | 
| :-----:       | :-------- | 
| `> 4000	    | Разрыв                                   |
| `>3700			| Если взяться пальцами                    |
| 1400 ... 300  | Стакан с водой                           |
| < 400         | Короткое замыкание проводом по выводам   |


<style TYPE="text/css">
code.has-jax {font: inherit; font-size: 100%; background: inherit; border: inherit;}
</style>
<script type="text/x-mathjax-config">
MathJax.Hub.Config({
    tex2jax: {
        inlineMath: [['$','$'], ['\\(','\\)']],
        skipTags: ['script', 'noscript', 'style', 'textarea', 'pre'] // removed 'code' entry
    }
});
MathJax.Hub.Queue(function() {
    var all = MathJax.Hub.getAllJax(), i;
    for(i = 0; i < all.length; i += 1) {
        all[i].SourceElement().parentNode.className += ' has-jax';
    }
});
</script>
<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.4/MathJax.js?config=TeX-AMS_HTML-full"></script>


- Вывод формул на `LATEX` в Markdown
    ```text
        $$ p_мм.рт.ст. = p_Паскали / 133.3224
    ```
    + <img src="https://render.githubusercontent.com/render/math?math=p_{mm} = \frac{ p_{Pascals} }{ 133.3224 }">
    + <img src="https://render.githubusercontent.com/render/math?math=e^{i \pi} = -1">



**Сложности отладки**
![](./debug.PNG)

    main.c
        find_volume() at ff.c 2 317
            disk_initialize() at diskio.c 99
                USER_initialize() at user_diskio.c 85
                    SD_disk_initialize() at fatfs_sd.c 280
                        SD_PowerOn() at fatfs_sd.c 90