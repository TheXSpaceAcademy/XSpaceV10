#ifndef XSPACEV10_H
#define XSPACEV10_H

#define MOTOR_A 0
#define MOTOR_B 1

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
    void avanzar(double power);
    void retroceder(double power);
    void rotarDerecha(double power);
    void rotarIzquierda(double power);
};


#endif