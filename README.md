[![AlexGyver YouTube](http://alexgyver.ru/git_banner.jpg)](https://www.youtube.com/channel/UCgtAOyEQdAyjvm9ATCi_Aig?sub_confirmation=1)
# Диммер переменного тока на Arduino
* [Описание проекта](#chapter-0)
* [Папки проекта](#chapter-1)
* [Схемы подключения](#chapter-2)
* [Материалы и компоненты](#chapter-3)
* [Настройка и использование](#chapter-4)
* [FAQ](#chapter-5)
* [Полезная информация](#chapter-6)

<a id="chapter-0"></a>
## Описание проекта
Диммер переменки на Arduino, управляем симистором, опираясь на таймер timer1 и детектор нуля. Вся высоковольтная часть развязана с логической, все подробности смотрите на схемах.
- Подробности в видео: https://youtu.be/jPbptVGZisc

<a id="chapter-1"></a>
## Папки
- **Library** - библиотеки для дисплея и прочего, скопировать в  
`C:\Program Files (x86)\Arduino\libraries\` (Windows x64)  
`C:\Program Files\Arduino\libraries\` (Windows x86)
- **Sketches** - прошивки для Arduino, файлы в папках открыть в Arduino IDE (читай [FAQ](#chapter-5))
  + **dimmer_timer** - показанная в видео прошивка для 1 канала, с таймеромм и потенциометром
  + **dimmer_6ch** - прошивка для 6ти канального диммера с таймером, для платы из видео
  + **dimmer_delay** - версия, часто встречающаяся в интернете, с задержками. Чисто для ознакомления
- **Schematics&PCB** - схемы и печатки. Весь проект полностью находится здесь https://easyeda.com/beragumbo/AC_Dimmer-76ae9ae002a64ab28c81e22fb88a56ab

<a id="chapter-2"></a>
## Схема диммера
![СХЕМА](https://github.com/AlexGyver/AC_Dimmer/blob/master/Schematics%26PCB/dimmer_sch.png)

## Подключаем к Ардуино
![СХЕМА](https://github.com/AlexGyver/AC_Dimmer/blob/master/Schematics%26PCB/dimmer_sch_ard.png)

## Вариант печатки с подтяжкой на плате
![СХЕМА](https://github.com/AlexGyver/AC_Dimmer/blob/master/Schematics%26PCB/pcb+5V.png)

<a id="chapter-3"></a>
## Материалы и компоненты
* Arduino NANO http://ali.pub/1qqtjx
* Макетка http://ali.pub/im4fk
* Линейный потенциометр http://ali.pub/1rcv5g

РАССЫПУХА
* Симистор - любой на нужный ток, корпус TO-220AB
  + 4 Ампера https://www.chipdip.ru/product/bt136-600d
  + 8 Ампер https://www.chipdip.ru/product/bt137-600e
  + 16 Ампер https://www.chipdip.ru/product/bt139-600e
* Оптопара симистора MOC3020, MOC3021, MOC3022, MOC3023
  + MOC3021 https://www.chipdip.ru/product/moc3021m
* Оптопара детектора нуля PC814, FOD814
  + PC814 https://www.chipdip.ru/product/pc814-fairchild
* Резистор 51 кОм, 0.5 или 1 Вт
* Резисторы 220 Ом, 10 кОм, 1 кОм
* Клеммники 5 мм http://ali.ski/UCZN8


## Вам скорее всего пригодится
* [Всё для пайки (паяльники и примочки)](http://alexgyver.ru/all-for-soldering/)
* [Недорогие инструменты](http://alexgyver.ru/my_instruments/)
* [Все существующие модули и сенсоры Arduino](http://alexgyver.ru/arduino_shop/)
* [Электронные компоненты](http://alexgyver.ru/electronics/)
* [Аккумуляторы и зарядные модули](http://alexgyver.ru/18650/)

<a id="chapter-4"></a>
## Настройка и использование
* [Загрузка прошивки](http://alexgyver.ru/arduino-first/) - ультра подробная статья по началу работы с Ардуино
* Переменная Dimmer - величина диммирования, от 0 до 255. В этом коде на пин А0 подключен потенциометр для управления яркостью. Также можно вводить число для переменной Dimmer через монитор порта, для этого в лупе надо раскомментировать код

## Настройки в коде


<a id="chapter-5"></a>
## FAQ
### Основные вопросы
В: Как скачать с этого грёбаного сайта?  
О: На главной странице проекта (где ты читаешь этот текст) вверху справа зелёная кнопка **Clone or download**, вот её жми, там будет **Download ZIP**

В: Скачался какой то файл .zip, куда его теперь?  
О: Это архив. Можно открыть стандартными средствами Windows, но думаю у всех на компьютере установлен WinRAR, архив нужно правой кнопкой и извлечь.

В: Я совсем новичок! Что мне делать с Ардуиной, где взять все программы?  
О: Читай и смотри видос http://alexgyver.ru/arduino-first/

В: Компьютер никак не реагирует на подключение Ардуины!  
О: Возможно у тебя зарядный USB кабель, а нужен именно data-кабель, по которому можно данные передавать

В: Ошибка! Скетч не компилируется!  
О: Путь к скетчу не должен содержать кириллицу. Положи его в корень диска.

В: Сколько стоит?  
О: Ничего не продаю.

### Вопросы по этому проекту
В: Работает нестабильно, мерцает!  
О: Пайка приветствуется, соединение джамперами очень ненадёжно

<a id="chapter-6"></a>
## Полезная информация
* [Мои видеоуроки по пайке](https://www.youtube.com/playlist?list=PLOT_HeyBraBuMIwfSYu7kCKXxQGsUKcqR)
* [Мои видеоуроки по Arduino](http://alexgyver.ru/arduino_lessons/)