
#include <XSpaceV10.h>
#include <Arduino.h>

#define ServoMin 4
#define ServoMax 25


volatile double servoD_pos = ServoMin; // en cuentas - entre 7 y 26
volatile double servoI_pos = ServoMin;
volatile double servoC_pos = ServoMin;
volatile double servo_counter = 0; // Para alternar entre los servos en la ISR


ISR(TIMER2_COMPA_vect) {
  
  if(servo_counter>=servoD_pos)digitalWrite(SERVO_D,LOW);
  if(servo_counter>=servoI_pos)digitalWrite(SERVO_I,LOW);
  if(servo_counter>=servoC_pos)digitalWrite(SERVO_C,LOW);

  servo_counter++;
  if(servo_counter>=230){
    servo_counter = 0;
    digitalWrite(SERVO_D,HIGH);
    digitalWrite(SERVO_I,HIGH);
    digitalWrite(SERVO_C,HIGH);
  } 
}


xTB6612FNG TB6612FNG;

void XSpaceV10Board::init(){
  
  TB6612FNG.xIN1[MOTOR_B] = 6;
  TB6612FNG.xIN1[MOTOR_A] = 7;
  TB6612FNG.PWMx[MOTOR_A] = 10;
  TB6612FNG.PWMx[MOTOR_B] = 9;
  TB6612FNG.STBY = 8;


  pinMode(TB6612FNG.xIN1[MOTOR_B],OUTPUT);
  pinMode(TB6612FNG.xIN1[MOTOR_A],OUTPUT);

  pinMode(TB6612FNG.PWMx[MOTOR_A],OUTPUT);
  pinMode(TB6612FNG.PWMx[MOTOR_B],OUTPUT);
  pinMode(TB6612FNG.STBY,OUTPUT);

  digitalWrite(TB6612FNG.STBY,LOW);

  // Configuración del Timer1 en Fast PWM con TOP en OCR1A
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  // PWM en pines 9 y 10
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11);     // Prescaler 8
  ICR1 = 99;  // Establecer TOP para 10 kHz

  pinMode(SERVO_D,OUTPUT);
  pinMode(SERVO_I,OUTPUT);
  pinMode(SERVO_C,OUTPUT);

  digitalWrite(SERVO_D,LOW);
  digitalWrite(SERVO_I,LOW);
  digitalWrite(SERVO_C,LOW);


  // Configuración del Timer2 para interrupción de 20ms
  TCCR2A = (1 << WGM21);  // Modo CTC
  TCCR2B = (1 << CS21) ;  // Prescaler 8
  OCR2A = 100;
  TIMSK2 |= (1 << OCIE2A);  // Habilitar interrupción en OCR2A

}


void XSpaceV10Board::TB6612FNG_Sleep(){
	digitalWrite(TB6612FNG.STBY,LOW);
}
void XSpaceV10Board::TB6612FNG_Wake(){
	digitalWrite(TB6612FNG.STBY,HIGH);
}
void XSpaceV10Board::TB6612FNG_Voltage(int MotorX, double Vp){

	if(Vp>5) Vp = 5;
	if(Vp<-5) Vp = -5;
	uint8_t Duty = (uint8_t) ( (abs(Vp)/5) * 100);

	if(Vp<0){
		digitalWrite(TB6612FNG.xIN1[MotorX],HIGH);
	}else{
		digitalWrite(TB6612FNG.xIN1[MotorX],LOW);
	}
  if(MotorX == MOTOR_A) OCR1B = (Duty * ICR1) / 100;
  else OCR1A = (Duty * ICR1) / 100;
}

void WalleRobot::init(){
  XSBoard.init();
  XSBoard.TB6612FNG_Wake();
}


void WalleRobot::Avanzar(double power){
  if(power>100)power=100;
  if(power<0)power=0;
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,power);
}

void WalleRobot::Retroceder(double power){
  if(power>100)power=100;
  if(power<0)power=0;
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,-power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,-power);
}


void WalleRobot::GirarIzquierda(double power){
  if(power>100)power=100;
  if(power<0)power=0;
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,-power); 
}

void WalleRobot::GirarDerecha(double power){
  if(power>100)power=100;
  if(power<0)power=0;
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,-power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,power); 
}

void WalleRobot::BrazoDerecho(double power){
  if(power>100)power=100;
  if(power<0)power=0;
  servoD_pos = power/100*(ServoMax-ServoMin) + ServoMin;
}
void WalleRobot::BrazoIzquierdo(double power){
  if(power>100)power=100;
  if(power<0)power=0;
  servoI_pos = (100-power)/100*(ServoMax-ServoMin) + ServoMin;
}
void WalleRobot::Cabeza(double power){
  if(power>100)power=100;
  if(power<0)power=0;
  servoC_pos = power/100*(ServoMax-ServoMin) + ServoMin;
}
