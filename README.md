# NahalOS v0.2.1α 💢

Агрессивная ОС для самодельного боксмода [NaVal](https://github.com/users/ruthenium-44/projects/4).

### Дисклеймер💀

Внимание!
Я ни в коем случае не пропагандирую парение и все остальное, что вас убьет. Если вы собрали "все по схеме", а дудка загорелась, подорвалась или спровоцировала ядерный взрыв, то я не виноват!

### Комплектующие🔌

• Arduino Nano👾
• Две китайские банки 18650🔋🔋
• Резисторы на 100 и 10К ОМ🪄
• Транзистор >= 50А🪄
• 510 коннектор (куда без него)💨
• Держалка аккумов➿
• Дисплей 0'98 i2c🖥
• Плата зарядки🔌
• Китайские провода🌀
• Руки из плеч (или чудо)🤷‍♂️

## Обновление v0.2.1ALPHA🔄

⚠️ **Теперь доступна только версия без дисплея**

<details><summary>Вот почему</summary>
Версия с дисплеем работала мягко говоря так себе, а твёрдо говоря не работала. Вторая причина: я уничтожил дисплей. Для вас это
хорошая новость, будет версия и на этот, и на большой цветной tft)</details>

✅ Создали документацию<br>
❌ Научились играть на скрипке

# Документация

## Версия без дисплея

Эта версия не имеет дисплея, и урезана в функционале.
Также есть изменения в навигации:

- Доступен только режим вариватт, где изменения производятся нажатиями кнопок UP и DOWN,
  добавляющие/убирающие по 10Вт.
- Навигация происходит в слепую, т.к. дисплей отсутствует. (Ватты регулируйте по вкусу ¯\_(ツ)\_/¯ )
- Приветственный экран отсутствует, вместо него сразу запускается режим вариватт.
- Хранение тяг остается, но как их достать - неизвестно.🤷‍
- Убраны лишние настройки
- Кнопка SET вам больше не нужна.

#### Схема

![Схема](https://alexgyver.ru/wp-content/uploads/2021/03/scheme2-1536x864.jpg)
Внимание! Я ленивая жопа, и не стал переделывать схему. Просто не подключайте дисплей, и все будет хорошо👍.

#### Сборка

Паяем все по схеме и пихаем в лаконичный корпус. Закрываем ноутбук, готовим чашечку кофе,
выходим на балкон... И идем обратно, т.к. не прочитали инструкцию.

#### Инструкция

1. После сборки, подключите банки к зарядке, и зарядите их до 4.2В.
2. Подключите все это дело к компьютеру, и загрузите прошивку.
3. После загрузки проведите калибровку. Для этого:
<details><summary>*Клик*</summary>

4. Подготовить вольтметр
5. Убедиться, что #define initial_calibration 1 стоит единичка (16 строка кода прошивки)
6. Нажать “Загрузить”, дождаться появления надписи “Загрузка завершена”
7. Открыть монитор порта (правый верхний угол, иконка лупы), дождаться появления строки с real vcc
8. Измерить напряжение на пинах 5V и GND
9. Ввести напряжение в порт В МИЛЛИВОЛЬТАХ. Т.е. если вольтметр показывает 4.67, то ввести 4670.
10. Нажать кнопку “Отправить” или нажать Enter на клавиатуре
11. Дождаться строку с результатом расчёта константы
12. Вернуться к 16 строке прошивки, изменить #define initial_calibration 0
13. Снова загрузить прошивку в Arduino
</details>
После калибровки учимся пользоваться устройством.

| Кнопка |                                            Что делает                                            |
| ------ | :----------------------------------------------------------------------------------------------: |
| FIRE   | Нажать 5 раз: Включится<br/>Удержание: Парение<br/>Нажать 3 раза: Сон<br/>Двойное нажатие: TURBO |
| SET    |                                               🤷‍                                                |
| UP     |                                              +10Вт                                               |
| DOWN   |                                              -10Вт                                               |

Простите за подробность, объяснил чайникам.

#### Настройки

В начале скетча есть настройки, которые можно менять под себя. Но вообще, не трогайте их, если не знаете, что делаете.
А вот и они:
![Image: Settings](displayIcons/setting.png)

> Некоторых настроек со скриншота нет на версии без дисплея.

- **initial_calibration**, если нужно калибровать, 0 если нет. После калибровки, ставьте 0.
  Калибровка описана выше, в инструкции.
- **sleep_timer** Таймер сна. По умолчанию 45 секунд.
- **vape_threshold** Таймер отсечки. Стоит 5 секунд, чтобы ничего не сгорело. Если вы бессмертный, то можете поставить больше.
- **turbo_mode** Турбо режим. Честно говоря, даже я не уверен как это работает. Но если нажать FIRE дважды, мосфет подаст максимальный ток на коил (жалко его). По умолчанию выключен.
- **battery_low** Порог низкого заряда батареи. Прямо в конец, когда она выключится. По умолчанию 3V на всякий случай.

---

## Версия с дисплеем 0.98"

Это версия с дисплеем и кучей менюшек. Готов интерфейс только к варивату. Вот так он выглядит:
![Image: Interface](displayIcons/vavadis.png)
Таки не додумался что сделать с пустым местом внизу.

#### Инструкция

1. После сборки, подключите банки к зарядке, и зарядите их.
2. Подключите все это дело к компьютеру, и загрузите прошивку.
3. После загрузки проведите калибровку. Для этого:
<details><summary>*Клик*</summary>

4. Подготовить вольтметр
5. Убедиться, что #define initial_calibration 1 стоит единичка (16 строка кода прошивки)
6. Нажать “Загрузить”, дождаться появления надписи “Загрузка завершена”
7. Открыть монитор порта (правый верхний угол, иконка лупы), дождаться появления строки с real vcc
8. Измерить напряжение на пинах 5V и GND
9. Ввести напряжение в порт В МИЛЛИВОЛЬТАХ. Т.е. если вольтметр показывает 4.67, то ввести 4670.
10. Нажать кнопку “Отправить” или нажать Enter на клавиатуре
11. Дождаться строку с результатом расчёта константы
12. Вернуться к 16 строке прошивки, изменить #define initial_calibration 0
13. Снова загрузить прошивку в Arduino
</details>
После калибровки учимся пользоваться устройством.

| Кнопка |                                              Что делает                                               |
| ------ | :---------------------------------------------------------------------------------------------------: |
| FIRE   | Нажать 5 раз: Включится<br/>Удержание: Парение<br/>Нажать 3 раза: Сон<br/>Двойное нажатие: TURBO<br/> |
| SET    |                   Удеражание: Показать инфу об аккамуляторах<br/>Нажатие: НЕ НАДО!                    |
| UP     |                                                 +1Вт                                                  |
| DOWN   |                                                 -1Вт                                                  |

Простите за подробность, объяснил чайникам.

#### Экраны

Для разблокировки нужно 5 раз нажать кнопку FIRE. Это сопровождается звуком и включением дисплея:

![Image: Unlock](displayIcons/fiveClicksUnlock.png)

После включения (если в настройках не выключено) показывается доброе приветствие:
![Image: Welcome](displayIcons/welcome.png)

А может и не включится. Если видите это:

![Image: Locked](displayIcons/calibration_lock.png)
то нужно снова прошить с initial_calibration = 0.

Отобразится главный экран:

Если battery_info = 1, то отобразится:

![Image: Battery](displayIcons/battery.png)

И наконец главное меню:

![Image: Interface](displayIcons/vavadis.png)

Если сядет батарея:

![Image: Battery](displayIcons/discharges.png)

И если сработала отсечка:

![Image: Battery](displayIcons/overtime.png)

Кстати выход из нее жестокий, нужно доставать батарейку. Возможно, это я такой лох.
Короче, лучше до этого не доводить.

При выключении:

![Image: Battery](displayIcons/goodbue.png)

---
