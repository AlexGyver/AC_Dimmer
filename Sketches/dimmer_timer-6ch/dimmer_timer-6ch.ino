/*
   Диммер переменки на Arduino. 6 каналов для полной прожарки
*/

// подключаем кучу выходов на симисторы
#define dimPin1 4
#define dimPin2 5
#define dimPin3 6
#define dimPin4 7
#define dimPin5 8
#define dimPin6 9

#define zeroPin 2
#include <CyberLib.h> // шустрая библиотека для таймера
volatile byte tic, Dimmer1 = 20, Dimmer2 = 50, Dimmer3 = 200, Dimmer4 = 140, Dimmer5 = 100, Dimmer6 = 90;

void setup() {
  Serial.begin(9600);

  pinMode(dimPin1, OUTPUT);
  digitalWrite(dimPin1, 0);
  pinMode(dimPin2, OUTPUT);
  digitalWrite(dimPin2, 0);
  pinMode(dimPin3, OUTPUT);
  digitalWrite(dimPin3, 0);
  pinMode(dimPin4, OUTPUT);
  digitalWrite(dimPin4, 0);
  pinMode(dimPin5, OUTPUT);
  digitalWrite(dimPin5, 0);
  pinMode(dimPin6, OUTPUT);
  digitalWrite(dimPin6, 0);

  pinMode(zeroPin, INPUT);                 // настраиваем порт на вход для отслеживания прохождения сигнала через ноль
  attachInterrupt(0, detect_up, FALLING);  // настроить срабатывание прерывания interrupt0 на pin 2 на низкий уровень

  StartTimer1(timer_interrupt, 40);        // время для одного разряда ШИМ
  StopTimer1();                            // остановить таймер
}

void loop() {
  /*
    раскомментировать для ввода числа диммирования чеерез монитор порта
    while (Serial.available()) {
    Dimmer11 = Serial.parseInt();
    Serial.println(Dimmer11);
    }
  */
}

//----------------------ОБРАБОТЧИКИ ПРЕРЫВАНИЙ--------------------------
void timer_interrupt() {       // прерывания таймера срабатывают каждые 40 мкс
  tic++;                       // счетчик
  if (tic > Dimmer1)            // если настало время включать ток
    digitalWrite(dimPin1, 1);   // врубить ток
  if (tic > Dimmer2)            // если настало время включать ток
    digitalWrite(dimPin2, 1);   // врубить ток
  if (tic > Dimmer3)            // если настало время включать ток
    digitalWrite(dimPin3, 1);   // врубить ток
  if (tic > Dimmer4)            // если настало время включать ток
    digitalWrite(dimPin4, 1);   // врубить ток
  if (tic > Dimmer5)            // если настало время включать ток
    digitalWrite(dimPin5, 1);   // врубить ток
  if (tic > Dimmer6)            // если настало время включать ток
    digitalWrite(dimPin6, 1);   // врубить ток
}

void  detect_up() {    // обработка внешнего прерывания на пересекание нуля снизу
  tic = 0;                                  // обнулить счетчик
  ResumeTimer1();                           // перезапустить таймер
  attachInterrupt(0, detect_down, RISING);  // перенастроить прерывание
}

void  detect_down() {  // обработка внешнего прерывания на пересекание нуля сверху
  tic = 0;                                  // обнулить счетчик
  StopTimer1();                             // остановить таймер
  digitalWrite(dimPin1, 0);                  // вырубить ток
  digitalWrite(dimPin2, 0);                  // вырубить ток
  digitalWrite(dimPin3, 0);                  // вырубить ток
  digitalWrite(dimPin4, 0);                  // вырубить ток
  digitalWrite(dimPin5, 0);                  // вырубить ток
  digitalWrite(dimPin6, 0);                  // вырубить ток
  attachInterrupt(0, detect_up, FALLING);   // перенастроить прерывание
}
//----------------------ОБРАБОТЧИКИ ПРЕРЫВАНИЙ--------------------------
