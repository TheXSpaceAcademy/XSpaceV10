
#include <XSpaceV10.h>
#include <Arduino.h>

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


void WalleRobot::avanzar(double power){
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,power);
}

void WalleRobot::retroceder(double power){
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,-power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,-power);
}


void WalleRobot::rotarDerecha(double power){
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,-power); 
}

void WalleRobot::rotarIzquierda(double power){
  power = power/100*5;
  XSBoard.TB6612FNG_Voltage(MOTOR_A,-power);
  XSBoard.TB6612FNG_Voltage(MOTOR_B,power); 
}
