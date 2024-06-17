// motordriver na doske
const int pwm1 = 5;
const int inb1 = 6;
const int ina1 = 7;
// motordriver external
const int pwm2 = 28;
const int inb2 = 29;
const int ina2 = 30;
//ir senzory z prava do lava ak sa pozerame zo zadu
const int ir1 = A8;
const int ir2 = A9;
const int ir3 = A10;
const int ir4 = A12;

int ir1_value = 0;
int ir2_value = 0;
int ir3_value = 0;
int ir4_value = 0;

const int speed = 90;
const int test = 1; // test mode

void setup() {
  pinMode(pwm1, OUTPUT);  
  pinMode(inb1, OUTPUT);  
  pinMode(ina1, OUTPUT); 
  pinMode(pwm2, OUTPUT);  
  pinMode(inb2, OUTPUT);  
  pinMode(ina2, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(115200);
}

void loop() {
  ir1_value = analogRead(ir1);
  ir2_value = analogRead(ir2);
  ir3_value = analogRead(ir3);
  ir4_value = analogRead(ir4);
  if (test == 1){
    Serial1.print("ir1 = ");
    Serial1.println(ir1_value);
    Serial1.print(",ir2 = ");
    Serial1.println(ir2_value);
    Serial1.print(",ir3 = ");
    Serial1.println(ir3_value);
    Serial1.print(",ir4 = ");
    Serial1.println(ir4_value);
    delay(100);    
  }
  /*
  // go backwards
  digitalWrite(ina1, LOW);
  digitalWrite(inb1, HIGH);
  digitalWrite(ina2, LOW);
  digitalWrite(inb2, HIGH);
  analogWrite(pwm1, 100);
  analogWrite(pwm2, 100);  
  */
  
  // go forward
//   digitalWrite(ina1, HIGH);
//   digitalWrite(inb1, LOW);
//   digitalWrite(ina2, HIGH);
//   digitalWrite(inb2, LOW);
//   analogWrite(pwm1, 120);
//   analogWrite(pwm2, 120);

  if (test == 0){
    //ir1_value < 500 || ir2_value < 500 || ir3_value < 500 || 
    if(ir4_value < 500){
      digitalWrite(ina1, LOW);
      digitalWrite(inb1, HIGH);
      digitalWrite(ina2, LOW);
      digitalWrite(inb2, HIGH);
      analogWrite(pwm1,speed);
      analogWrite(pwm2, speed); 
      delay(500);
    } else {
      digitalWrite(ina1, HIGH);
      digitalWrite(inb1, LOW);
      digitalWrite(ina2, HIGH);
      digitalWrite(inb2, LOW);
      analogWrite(pwm1, speed);
      analogWrite(pwm2, speed);
         
    }
  }
  /*
  for (int i = 0; i < 240; i++){
    analogWrite(pwm1, i);  
    delay(10);
  }
  for (int i = 255; i < 10; i--){
    analogWrite(pwm1, i);  
    delay(10);
  }
  for (int i = 0; i < 240; i++){
    analogWrite(pwm2, i);  
    delay(10);
  }
  for (int i = 255; i < 10; i--){
    analogWrite(pwm2, i);  
    delay(10);
  }*/
}
