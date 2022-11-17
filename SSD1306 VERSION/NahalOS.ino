/*
!Во избежании проблем положите папку в C:
При первой прошивке выполните калибровку.

Код от Alex Gyver, модифицировал ruthenium44

https://github.com/ruthenium-44/NahalOS

███╗░░██╗░█████╗░██╗░░░██╗░█████╗░██╗░░░░░ 
████╗░██║██╔══██╗██║░░░██║██╔══██╗██║░░░░░
██╔██╗██║███████║╚██╗░██╔╝███████║██║░░░░░ 
██║╚████║██╔══██║░╚████╔╝░██╔══██║██║░░░░░
██║░╚███║██║░░██║░░╚██╔╝░░██║░░██║███████╗
╚═╝░░╚══╝╚═╝░░╚═╝░░░╚═╝░░░╚═╝░░╚═╝╚══════╝

*/
//-----------------------------------НАСТРОЙКИ------------------------------------
#define initial_calibration 0  // калибровка вольтметра 1 - включить, 0 - выключить
#define welcome 1              // приветствие
#define battery_info 0         // отображение напряжения аккумулятора при запуске, 1 - включить, 0 - выключить
#define battery_info_time 1000 // время отображения напряжения аккумулятора при запуске в миллисекундах
#define sleep_timer 45         // таймер сна в секундах
#define vape_threshold 5       // отсечка затяжки, в секундах
#define turbo_mode 0           // турбо режим 1 - включить, 0 - выключить (не рекомендовано!)
#define battery_low 3          // нижний порог срабатывания защиты от переразрядки аккумулятора, в Вольтах!
//-----------------------------------НАСТРОЙКИ------------------------------------

#include <EEPROMex.h>  // библиотека для работы со внутренней памятью ардуино
#include <LowPower.h>  // библиотека сна


//-----------кнопки-----------
#define butt_up 5    // кнопка вверх
#define butt_down 4  // кнопка вниз
#define butt_set 3   // кнопка выбора
#define butt_vape 2  // кнопка fire
//-----------кнопки-----------

//-----------флажки-----------
boolean up_state, down_state, set_state, vape_state;
boolean up_flag, down_flag, set_flag, set_flag_hold, set_hold, vape_btt, vape_btt_f;
volatile boolean wake_up_flag, vape_flag;
boolean change_v_flag, change_w_flag, change_o_flag;
volatile byte mode, mode_flag = 1;
boolean flag;  // флаг разрешения подачи тока на коил
//-----------флажки-----------

//-----------пины-------------
#define mosfet 10  // пин мосфета (нагрев спирали)
#define battery 3  // пин измерения напряжения акума
//-----------пины-------------

//-----------дисплей-----------
#include <TimerOne.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#define OLED_RESET -1
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define SCLK 6
#define RCLK 7
#define DIO 8
//-----------дисплей-----------

int bat_vol, bat_volt_f;  // хранит напряжение на акуме
int PWM, PWM_f;           // хранит PWM сигнал
int pafs;                 // хранит тяги

//-------переменные и коэффициенты для фильтра-------
int bat_old, PWM_old = 800;
float filter_k = 0.04;
float PWM_filter_k = 0.1;
//-------переменные и коэффициенты для фильтра-------

unsigned long last_time, vape_press, set_press, last_vape, wake_timer, timerIsr;  // таймеры
int volts, watts;                                                                 // храним вольты и ватты
float ohms;                                                                       // храним омы
float my_vcc_const;                                                               // константа вольтметра
volatile byte vape_mode, vape_release_count;

void setup() {
    Serial.begin(9600);
    if (initial_calibration) calibration();  // калибровка, если разрешена

    //----читаем из памяти-----
    volts = EEPROM.readInt(0);
    watts = EEPROM.readInt(2);
    ohms = EEPROM.readFloat(4);
    my_vcc_const = EEPROM.readFloat(8);
    pafs = EEPROM.readInt(5);
    //----читаем из памяти-----

    Timer1.initialize(1500);  // таймер
    Timer1.attachInterrupt(timerIsr);

    //---настройка кнопок и выходов-----
    pinMode(butt_up, INPUT_PULLUP);
    pinMode(butt_down, INPUT_PULLUP);
    pinMode(butt_set, INPUT_PULLUP);
    pinMode(butt_vape, INPUT_PULLUP);
    pinMode(mosfet, OUTPUT);
    Timer1.disablePwm(mosfet);  // принудительно отключить койл
    digitalWrite(mosfet, LOW);  // принудительно отключить койл
    //---настройка кнопок и выходов-----

    //------приветствие-----
    if (welcome) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(40, 25);
        display.println("SOSI");
        display.display();
        delay(5000);
        display.clearDisplay();
    }
    //------приветствие-----

    // измерить напряжение аккумулятора
    bat_vol = readVcc();
    bat_old = bat_vol;

    // проверка заряда акума, если разряжен то прекратить работу
    if (bat_vol < battery_low * 1000) {
        flag = 0;
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(5, 15);
        display.println("IDI NA HUY");
        display.setCursor(10, 40);
        display.println("OR CHARGE");
        display.display();
        delay(3000);
        display.clearDisplay();
        Timer1.disablePwm(mosfet);  // принудительно отключить койл
        digitalWrite(mosfet, LOW);  // принудительно отключить койл
    } else {
        flag = 1;
    }

    if (battery_info) {  // отобразить заряд аккумулятора при первом включении
        display.clearDisplay();
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(15, 25);
        display.print("BATTERY");
        display.setCursor(60, 20);
        display.println((float)bat_vol / 8400 * 100);
        display.setCursor(100, 20);
        display.print("%");
        display.setCursor(60, 30);
        display.println((float)bat_vol / 1000);
        display.setCursor(100, 30);
        display.println("V");
        display.display();
        delay(battery_info_time);
        display.clearDisplay();
    }
}

void loop() {
    if (millis() - last_time > 50) {  // 20 раз в секунду измеряем напряжение
        last_time = millis();
        bat_vol = readVcc();                                         // измерить напряжение аккумулятора в миллиВольтах
        bat_volt_f = filter_k * bat_vol + (1 - filter_k) * bat_old;  // фильтруем
        bat_old = bat_volt_f;                                        // фильтруем
        if (bat_volt_f < battery_low * 1000) {                       // если напряжение меньше минимального
            flag = 0;                                                  // прекратить работу
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(WHITE);
            display.setCursor(5, 15);
            display.println("IDI NA HUY");
            display.setCursor(10, 40);
            display.println("OR CHARGE");
            display.display();
            delay(3000);
            display.clearDisplay();
            Timer1.disablePwm(mosfet);  // принудительно отключить койл
            digitalWrite(mosfet, LOW);  // принудительно отключить койл
        }
    }

    //-----------опрос кнопок-----------
    up_state = !digitalRead(butt_up);
    down_state = !digitalRead(butt_down);
    set_state = !digitalRead(butt_set);
    vape_state = !digitalRead(butt_vape);

    // если нажата любая кнопка, "продлить" таймер ухода в сон
    if (up_state || down_state || set_state || vape_state) wake_timer = millis();
    //-----------опрос кнопок-----------

    //service_mode();  // раскомментировать для отладки кнопок
    // показывает, какие кнопки нажаты или отпущены
    // использовать для проерки правильности подключения

    //---------------------отработка нажатия SET и изменение режимов---------------------
    if (flag) {                      // если акум заряжен
        if (set_state && !set_hold) {  // если кнпока нажата
            set_hold = 1;
            set_press = millis();  // начинаем отсчёт
            while (millis() - set_press < 300) {
                if (digitalRead(butt_set)) {  // если кнопка отпущена до 300 мс
                    set_hold = 0;
                    set_flag = 1;
                    break;
                }
            }
        }
        if (set_hold && set_state) {  // если кнопка всё ещё удерживается
            if (!set_flag_hold) {
                display.clearDisplay();
                set_flag_hold = 1;
            }
            if (round(millis() / 150) % 2 == 0) {
                    display.clearDisplay();
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(WHITE);
                    display.setCursor(15, 25);
                    display.print("BATTERY");
                    display.setCursor(60, 20);
                    display.println((float)bat_vol / 8400 * 100);
                    display.setCursor(100, 20);
                    display.print("%");
                    display.setCursor(60, 30);
                    display.println((float)bat_vol / 1000);
                    display.setCursor(100, 30);
                    display.println("V");
                    display.display();
                    delay(500);
                    delay(1000);
                    display.clearDisplay();  // показать заряд акума в вольтах
            }
        }
        if (set_hold && !set_state && set_flag_hold) {  // если удерживалась и была отпущена
            set_hold = 0;
            set_flag_hold = 0;
            mode_flag = 1;
        }

        if (!set_state && set_flag) {  // если нажали-отпустили
            set_hold = 0;
            set_flag = 0;
            mode++;  // сменить режим
            mode_flag = 1;
            if (mode > 2) mode = 0;  // ограничение на 3 режима
        }
        // ----------------------отработка нажатия SET и изменение режимов---------------------------

        // ------------------режим ВАРИВОЛЬТ-------------------
        if (mode == 0 && !vape_state && !set_hold) {
            if (mode_flag) {  // приветствие
                mode_flag = 0;
                vavoDis();
            }
            //---------кнопка ВВЕРХ--------
            if (up_state && !up_flag) {
                volts += 10;
                volts = min(volts, bat_volt_f);  // ограничение сверху на текущий заряд акума
                up_flag = 1;
                display.clearDisplay();
            }
            if (!up_state && up_flag) {
                up_flag = 0;
                change_v_flag = 1;
            }
            //---------кнопка ВВЕРХ--------

            //---------кнопка ВНИЗ--------
            if (down_state && !down_flag) {
                volts -= 10;
                volts = max(volts, 0);
                down_flag = 1;
                display.clearDisplay();
            }
            if (!down_state && down_flag) {
                down_flag = 0;
                change_v_flag = 1;
            }
            //---------кнопка ВНИЗ--------
              vavoDis();  // отобразить на дисплее
        }
        // ------------------режим ВАРИВОЛЬТ-------------------


        // ------------------режим ВАРИВАТТ-------------------

        if (mode == 1 && !vape_state && !set_hold) {
            if (mode_flag) {  // приветствие
                mode_flag = 0;
                vavaDis();
            }
            //---------кнопка ВВЕРХ--------
            if (up_state && !up_flag) {
                watts += 1;
                byte maxW = (sq((float)bat_volt_f / 1000)) / ohms;
                watts = min(watts, maxW);  // ограничение сверху на текущий заряд акума и сопротивление
                up_flag = 1;
                display.clearDisplay();
            }
            if (!up_state && up_flag) {
                up_flag = 0;
                change_w_flag = 1;
            }
            //---------кнопка ВВЕРХ--------

            //---------кнопка ВНИЗ--------
            if (down_state && !down_flag) {
                watts -= 1;
                watts = max(watts, 0);
                down_flag = 1;
                display.clearDisplay();
            }
            if (!down_state && down_flag) {
                down_flag = 0;
                change_w_flag = 1;
            }
            //---------кнопка ВНИЗ--------
            vavaDis();  // отобразить на дисплее
        }
        // ------------------режим ВАРИВАТТ--------------

        // ----------режим установки сопротивления-----------
        if (mode == 2 && !vape_state && !set_hold) {
            if (mode_flag) {  // приветствие
                mode_flag = 0;
                omSet();

            }
            //---------кнопка ВВЕРХ--------
            if (up_state && !up_flag) {
                ohms += 0.05;
                ohms = min(ohms, 3);
                up_flag = 1;
                display.clearDisplay();
            }
            if (!up_state && up_flag) {
                up_flag = 0;
                change_o_flag = 1;
            }
            //---------кнопка ВВЕРХ--------

            //---------кнопка ВНИЗ--------
            if (down_state && !down_flag) {
                ohms -= 0.05;
                ohms = max(ohms, 0);
                down_flag = 1;
                display.clearDisplay();
            }
            if (!down_state && down_flag) {
                down_flag = 0;
                change_o_flag = 1;
            }
            //---------кнопка ВНИЗ--------
            vavoDis();  // отобразить на дисплее
        }
        // ----------режим установки сопротивления-----------

        //---------отработка нажатия кнопки парения-----------
        if (vape_state && flag && !wake_up_flag) {

            if (!vape_flag) {
                vape_flag = 1;
                vape_mode = 1;          // первичное нажатие
                delay(20);              // анти дребезг (сделал по-тупому, лень)
                vape_press = millis();  // первичное нажатие
            }

            if (vape_release_count == 1) {
                vape_mode = 2;  // двойное нажатие
                delay(20);      // анти дребезг (сделал по-тупому, лень)
            }
            if (vape_release_count == 2) {
                vape_mode = 3;  // тройное нажатие
            }

            if (millis() - vape_press > vape_threshold * 1000) {  // "таймер затяжки"
                vape_mode = 0;
                digitalWrite(mosfet, 0);
                display.clearDisplay();
                display.setTextSize(2);
                display.setTextColor(WHITE);
                display.setCursor(13, 25);
                display.print("OVERTIME!");
                display.display();
            }

            if (vape_mode == 1) {  // обычный режим парения
                if (round(millis() / 150) % 2 == 0)
                    //   disp_send(vape1);
                    // else disp_send(vape2);                                        // мигать медленно
                    if (mode == 0) {                                              // если ВАРИВОЛЬТ
                        PWM = (float)volts / bat_volt_f * 1024;                     // считаем значение для ШИМ сигнала
                        if (PWM > 1023) PWM = 1023;                                 // ограничил PWM "по-тупому", потому что constrain сука не работает!
                        PWM_f = PWM_filter_k * PWM + (1 - PWM_filter_k) * PWM_old;  // фильтруем
                        PWM_old = PWM_f;                                            // фильтруем
                    }
                Timer1.pwm(mosfet, PWM_f);  // управление мосфетом
                pafs += 1;                  // + тяга
                EEPROM.writeInt(5, pafs);
            }
            if (vape_mode == 2 && turbo_mode) {  // турбо режим парения (если включен)
                if (round(millis() / 50) % 2 == 0)
                    //   disp_send(vape1);
                    // else disp_send(vape2);    // мигать быстро
                    digitalWrite(mosfet, 1);  // херачить на полную мощность
            }
            if (vape_mode == 3) {  // тройное нажатие
                vape_release_count = 0;
                vape_mode = 1;
                vape_flag = 0;
                good_night();  // вызвать функцию сна
            }
            vape_btt = 1;
        }

        if (!vape_state && vape_btt) {  // если кнопка ПАРИТЬ отпущена
            if (millis() - vape_press > 180) {
                vape_release_count = 0;
                vape_mode = 0;
                vape_flag = 0;
            }
            vape_btt = 0;
            if (vape_mode == 1) {
                vape_release_count = 1;
                vape_press = millis();
            }
            if (vape_mode == 2) vape_release_count = 2;

            digitalWrite(mosfet, 0);
            display.clearDisplay();
            mode_flag = 0;

            // если есть изменения в настройках, записать в память
            if (change_v_flag) {
                EEPROM.writeInt(0, volts);
                EEPROM.writeInt(5, pafs);
                change_v_flag = 0;
            }
            if (change_w_flag) {
                EEPROM.writeInt(2, watts);
                EEPROM.writeInt(5, pafs);
                change_w_flag = 0;
            }
            if (change_o_flag) {
                EEPROM.writeFloat(4, ohms);
                EEPROM.writeInt(5, pafs);
                change_o_flag = 0;
            }
            // если есть изменения в настройках, записать в память
        }
        if (vape_state && !flag) {  // если акум сел, а мы хотим подымить
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(WHITE);
            display.setCursor(5, 15);
            display.println("IDI NA HUY");
            display.setCursor(10, 40);
            display.println("OR CHARGE");
            display.display();
            delay(3000);
            display.clearDisplay();
            vape_flag = 1;
        }
        //---------отработка нажатия кнопки парения-----------
    }

    if (wake_up_flag) wake_puzzle();  // вызвать функцию 5 нажатий для пробудки

    if (millis() - wake_timer > sleep_timer * 1000) {  // если кнопки не нажимались дольше чем sleep_timer секунд
        good_night();
    }
}  // конец loop

//------функция, вызываемая при выходе из сна (прерывание)------
void wake_up() {
    Timer1.disablePwm(mosfet);     // принудительно отключить койл
    digitalWrite(mosfet, LOW);     // принудительно отключить койл
    wake_timer = millis();         // запомнить время пробуждения
    wake_up_flag = 1;
    vape_release_count = 0;
    vape_mode = 0;
    vape_flag = 0;
    mode_flag = 1;
}
//------функция, вызываемая при выходе из сна (прерывание)------

//------функция 5 нажатий для полного пробуждения------
void wake_puzzle() {
    detachInterrupt(0);  // отключить прерывание
    vape_btt_f = 0;
    boolean wake_status = 0;
    byte click_count = 0;
    while (1) {
        vape_state = !digitalRead(butt_vape);

        if (vape_state && !vape_btt_f) {
            vape_btt_f = 1;
            click_count++;
            switch (click_count) {
                case 1:
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(WHITE);
                    display.setCursor(15, 25);
                    display.println("4 CLICKS TO WAKE");
                    display.display();
                    break;
                case 2:
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(WHITE);
                    display.setCursor(15, 25);
                    display.println("3 CLICKS TO WAKE");
                    display.fillRect(10, 40, 45, 10, WHITE);
                    display.display();
                    break;
                case 3:
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(WHITE);
                    display.setCursor(15, 25);
                    display.println("2 CLICKS TO WAKE");
                    display.fillRect(10, 40, 85, 10, WHITE);
                    display.display();
                    break;
                case 4:
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(WHITE);
                    display.setCursor(25, 25);
                    display.println("CLICK TO WAKE");
                    display.fillRect(10, 40, 105, 10, WHITE);
                    display.display();
                    break;
            }
            if (click_count > 4) {  // если 5 нажатий сделаны за 3 секунды
                wake_status = 1;      // флаг "проснуться"
                break;
            }
        }
        if (!vape_state && vape_btt_f) {
            vape_btt_f = 0;
            delay(70);
        }
        if (millis() - wake_timer > 3000) {// если 5 нажатий не сделаны за 3 секунды
            display.clearDisplay();
            wake_status = 0;                   // флаг "спать"
            break;
        }
    }
    if (wake_status) {
        wake_up_flag = 0;
        display.clearDisplay();
        delay(100);
    } else {
        display.clearDisplay();
        good_night();  // спать
    }
}
//------функция 5 нажатий для полного пробуждения------

//-------------функция ухода в сон----------------
void good_night() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(40, 25);
    display.println("PROSCHAI!");
    display.display();
    delay(2000);
    display.clearDisplay();
    Timer1.disablePwm(mosfet);  // принудительно отключить койл
    digitalWrite(mosfet, LOW);  // принудительно отключить койл
    delay(50);
    attachInterrupt(0, wake_up, FALLING);  // подключить прерывание для пробуждения
    delay(50);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);  // спать. mode POWER_OFF, АЦП выкл
}
//-------------функция ухода в сон----------------

//----------режим теста кнопок----------
void service_mode() {
    if (set_state && !set_flag) {
        set_flag = 1;
        Serial.println("SET pressed");
    }
    if (!set_state && set_flag) {
        set_flag = 0;
        Serial.println("SET released");
    }
    if (up_state && !up_flag) {
        up_flag = 1;
        Serial.println("UP pressed");
    }
    if (!up_state && up_flag) {
        up_flag = 0;
        Serial.println("UP released");
    }
    if (down_state && !down_flag) {
        down_flag = 1;
        Serial.println("DOWN pressed");
    }
    if (!down_state && down_flag) {
        down_flag = 0;
        Serial.println("DOWN released");
    }
    if (vape_state && !vape_flag) {
        vape_flag = 1;
        Serial.println("VAPE pressed");
    }
    if (!vape_state && vape_flag) {
        vape_flag = 0;
        Serial.println("VAPE released");
    }
}
//----------режим теста кнопок----------
void vavaDis() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.drawLine(0, 10, 128, 10, WHITE);
    display.setCursor(45, 0);
    display.print("VarWatt");
    display.drawLine(0, 50, 128, 50, WHITE);
    display.setCursor(12, 55);
    display.print("BATTERY:");
    display.setCursor(65, 55);
    display.println((float) bat_vol / 8400 * 100);
    display.setCursor(110, 55);
    display.print("%");
    display.setTextSize(4);
    display.setCursor(1, 15);
    display.print(watts);
    display.setTextSize(1);
    display.setCursor(50, 35);
    display.print("W");
    display.setCursor(72, 12);
    display.print("PAFF:");
    display.setCursor(100, 12);
    display.print(pafs);
    display.setCursor(72, 20);
    display.print("OM:");
    display.setCursor(100, 20);
    display.print((float)ohms);
    display.setCursor(72, 36);
    display.print("VOLT:");
    display.setCursor(100, 36);
    display.print((float)volts / 1000);
    display.drawLine(68, 40, 128, 40, INVERSE);
    display.drawLine(0, 50, 128, 50, WHITE);
    display.drawLine(68, 30, 128, 30, WHITE);
    display.drawLine(68, 40, 128, 40, INVERSE);
    display.drawLine(68, 10, 68, 50, WHITE);
    display.drawLine(127, 30, 127, 50, WHITE);
    display.display();
}

void vavoDis() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.drawLine(0, 10, 128, 10, WHITE);
    display.setCursor(45, 0);
    display.print("VarVolt");
    display.drawLine(0, 50, 128, 50, WHITE);
    display.setCursor(12, 55);
    display.print("BATTERY:");
    display.setCursor(65, 55);
    display.println((float) bat_vol / 8400 * 100);
    display.setCursor(110, 55);
    display.print("%");
    display.setTextSize(2);
    display.setCursor(5, 25);
    display.print((float)volts / 1000);
    display.setTextSize(1);
    display.setCursor(55, 35);
    display.print("V");
    display.setCursor(72, 12);
    display.print("PAFF:");
    display.setCursor(100, 12);
    display.print(pafs);
    display.setCursor(72, 20);
    display.print("OM:");
    display.setCursor(100, 20);
    display.print((float)ohms);
    display.setCursor(72, 37);
    display.print("WATT:");
    display.setCursor(100, 37);
    display.print(watts);
    display.drawLine(68, 40, 128, 40, INVERSE);
    display.drawLine(0, 50, 128, 50, WHITE);
    display.drawLine(68, 30, 128, 30, WHITE);
    display.drawLine(68, 40, 128, 40, INVERSE);
    display.drawLine(68, 10, 68, 50, WHITE);
    display.drawLine(127, 30, 127, 50, WHITE);
    display.display();
}

void omSet() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.drawLine(0, 10, 128, 10, WHITE);
    display.setCursor(50, 0);
    display.print("SET OM");
    display.drawLine(0, 50, 128, 50, WHITE);
    display.setCursor(12, 55);
    display.print("BATTERY:");
    display.setCursor(65, 55);
    display.println((float) bat_vol / 8400 * 100);
    display.setCursor(110, 55);
    display.print("%");
    display.setTextSize(3);
    display.setCursor(5, 20);
    display.print("OM:");
    display.setCursor(55, 20);
    display.print((float)ohms);
    display.setCursor(72, 36);
    display.display();
}

void calibration() {
    //--------калибровка----------
    for (byte i = 0; i < 7; i++) EEPROM.writeInt(i, 0);  // чистим EEPROM для своих нужд
    my_vcc_const = 1.1;
    Serial.print("Real VCC is: ");
    Serial.println(readVcc());  // общаемся с пользователем
    Serial.println("Write your VCC (in millivolts)");
    while (Serial.available() == 0)
        ;
    int Vcc = Serial.parseInt();                      // напряжение от пользователя
    float real_const = (float)1.1 * Vcc / readVcc();  // расчёт константы
    Serial.print("New voltage constant: ");
    Serial.println(real_const, 3);
    EEPROM.writeFloat(8, real_const);  // запись в EEPROM
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(15, 28);
    display.println("CALIBRATION DONE");
    display.display();
    while (1)
        ;  // уйти в бесконечный цикл
    //------конец калибровки-------




}

long readVcc() {  //функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
    delay(2);             // Wait for Vref to settle
    ADCSRA |= _BV(ADSC);  // Start conversion
    while (bit_is_set(ADCSRA, ADSC))
        ;                   // measuring
    uint8_t low = ADCL;   // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH;  // unlocks both
    long result = (high << 8) | low;

    result = my_vcc_const * 1023 * 1000 / result;  // расчёт реального VCC
    return result;                                 // возвращает VCC
}
