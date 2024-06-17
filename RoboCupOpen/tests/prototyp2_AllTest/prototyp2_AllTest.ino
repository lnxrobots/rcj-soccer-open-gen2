// motordrivre na doske
const int pwm1 = 8;
const int inb1 = 9;
const int ina1 = 10;

const int pwm2 = 5;
const int inb2 = 6;
const int ina2 = 7;

const int pwm3 = 2;
const int inb3 = 3;
const int ina3 = 4;

const int pwm4 = 28;
const int inb4 = 29;
const int ina4 = 30;

const int speed = 0;

void setup() {
  pinMode(pwm1, OUTPUT);  
  pinMode(inb1, OUTPUT);  
  pinMode(ina1, OUTPUT);
  
  pinMode(pwm2, OUTPUT);  
  pinMode(inb2, OUTPUT);  
  pinMode(ina2, OUTPUT);

  pinMode(pwm3, OUTPUT);  
  pinMode(inb3, OUTPUT);  
  pinMode(ina3, OUTPUT);

  pinMode(pwm4, OUTPUT);  
  pinMode(inb4, OUTPUT);  
  pinMode(ina4, OUTPUT);

  delay(1000);
}

void loop() {
  // vpred
  digitalWrite(ina1, LOW);
  digitalWrite(inb1, HIGH);
  analogWrite(pwm1,speed);

  digitalWrite(ina2, LOW);
  digitalWrite(inb2, HIGH);
  analogWrite(pwm2,speed);

  digitalWrite(ina3, HIGH);
  digitalWrite(inb3, LOW);
  analogWrite(pwm3,speed);

  digitalWrite(ina4, HIGH);
  digitalWrite(inb4, LOW);
  analogWrite(pwm4,speed);

  delay(1000);

  // vzad
  digitalWrite(ina1, HIGH);
  digitalWrite(inb1, LOW);
  analogWrite(pwm1,speed);

  digitalWrite(ina2, 1);
  digitalWrite(inb2, 0);
  analogWrite(pwm2,speed);

  digitalWrite(ina3, LOW);
  digitalWrite(inb3, HIGH);
  analogWrite(pwm3,speed);

  digitalWrite(ina4, 0);
  digitalWrite(inb4, 1);
  analogWrite(pwm4,speed);

  delay(1000);
}