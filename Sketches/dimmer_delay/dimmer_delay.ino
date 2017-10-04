int dimpin = 4;      // Выход диммера на симистор
char dim = 50;       // Начальный уровень диммирования от 0 до 255
 
void setup() {
  pinMode(dimpin , OUTPUT);               // установим выход диммера
  attachInterrupt(0, light, FALLING);     // прерывание для детектора нуля (2 пин!!!)
}

void light() {                        // функция управления яркостью
  if (dim > 0 && dim < 255) {         // утановим диммирование, если dim не5 равен 0 или 255
    delayMicroseconds(33*(255-dim));  // вместо 33 былло 34, так между прочим. но если dim = 5 лампа припадошно мигает
    digitalWrite(dimpin , HIGH);
    delayMicroseconds(500);
    digitalWrite(dimpin , LOW);
  }
}


void loop() {
  
}
