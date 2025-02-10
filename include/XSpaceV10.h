#ifndef XSPACEV10_H
#define XSPACEV10_H

#define MOTOR_A 0
#define MOTOR_B 1

#define SERVO_D 2  
#define SERVO_I 3
#define SERVO_C 11

extern volatile double servoD_pos;
extern volatile double servoI_pos;
extern volatile double servoC_pos;

struct xTB6612FNG{
  int STBY;
	int xIN1[2];
  int PWMx[2];
	double Vmx;
};

class XSpaceV10Board{
  public:
    void init();
    void TB6612FNG_Sleep();
    void TB6612FNG_Wake();
    void TB6612FNG_Voltage(int MotorX, double Vp);
    
};

class WalleRobot{
  private:
    XSpaceV10Board XSBoard;
  public:
    void init();
    void Avanzar(double power);
    void Retroceder(double power);
    void GirarDerecha(double power);
    void GirarIzquierda(double power);

    void BrazoDerecho(double power);
    void BrazoIzquierdo(double power);
    void Cabeza(double power);
};


#endif