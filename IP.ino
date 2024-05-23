
#include <Servo.h>

const int servoPin1 = 5;
const int servoPin2 = 6;
const int servoPin3 = 7;
const int servoPin4 = 8;
const int servoPin5 = 9;
const int servoPin6 = 10;
const int servoPin7 = 11;
const int servoPin8 = 12;
//const int potPin = A0;

double setpoint1 = -250.0;
double setpoint2 = 90;
double setpoint3 = -300;
double setpoint4 = 90;
double setpoint5 = -90.0;
double setpoint6 = 180;
double setpoint7 = -120.0;
double setpoint8 = 200;
double input1, input2, input3, input4, input5, input6, input7, input8, output1, output2, output3, output4, output5, output6, output7, output8;
double kp = 1.0, ki = 5.0, kd = 1.0;
double integral1 = 0.0, integral2 = 0.0, integral3 = 0.0, integral4 = 0.0, integral5 = 0.0, integral6 = 0.0, integral7 = 0.0, integral8 = 0.0, lastInput1 = 0.0, lastInput2 = 0.0, lastInput3 = 0.0, lastInput4 = 0.0, lastInput5 = 0.0, lastInput6 = 0.0, lastInput7 = 0.0, lastInput8 = 0.0;
unsigned long lastTime;
double dt = 0.1;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

void setup()
{
    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    servo3.attach(servoPin3);
    servo4.attach(servoPin4);
    servo5.attach(servoPin5);
    servo6.attach(servoPin6);
    servo7.attach(servoPin7);
    servo8.attach(servoPin8);
    lastTime = millis();
}

void loop()
{
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime) / 1000.0;

    input1 = servo1.read();
    input1 = map(input1, 0, 1023, 0, 180);
    input2 = servo2.read();
    input2 = map(input2, 0, 1023, 0, 180);
    input3 = servo3.read();
    input3 = map(input3, 0, 1023, 0, 180);
    input4 = servo4.read();
    input4 = map(input4, 0, 1023, 0, 180);
    input5 = servo5.read();
    input5 = map(input5, 0, 1023, 0, 180);
    input6 = servo6.read();
    input6 = map(input6, 0, 1023, 0, 180);
    input7 = servo7.read();
    input7 = map(input7, 0, 1023, 0, 180);
    input8 = servo8.read();
    input8 = map(input8, 0, 1023, 0, 180);

    double error1 = setpoint1 - input1;
    double error2 = setpoint2 - input2;
    double error3 = setpoint3 - input3;
    double error4 = setpoint4 - input4;
    double error5 = setpoint5 - input5;
    double error6 = setpoint6 - input6;
    double error7 = setpoint7 - input7;
    double error8 = setpoint8 - input8;

    integral1 += (error1 * timeChange);
    integral2 += (error2 * timeChange);
    integral3 += (error3 * timeChange);
    integral4 += (error4 * timeChange);
    integral5 += (error5 * timeChange);
    integral6 += (error6 * timeChange);
    integral7 += (error7 * timeChange);
    integral8 += (error8 * timeChange);
    
    double derivative1 = (error1 - lastInput1) / timeChange;
    double derivative2 = (error2 - lastInput2) / timeChange;
    double derivative3 = (error3 - lastInput3) / timeChange;
    double derivative4 = (error4 - lastInput4) / timeChange;
    double derivative5 = (error5 - lastInput5) / timeChange;
    double derivative6 = (error6 - lastInput6) / timeChange;
    double derivative7 = (error7 - lastInput7) / timeChange;
    double derivative8 = (error8 - lastInput8) / timeChange;

    output1 = kp * error1 + ki * integral1 + kd * derivative1;
    output2 = kp * error2 + ki * integral2 + kd * derivative2;
    output3 = kp * error3 + ki * integral3 + kd * derivative3;
    output4 = kp * error4 + ki * integral4 + kd * derivative4;
    output5 = kp * error5 + ki * integral5 + kd * derivative5;
    output6 = kp * error6 + ki * integral6 + kd * derivative6;
    output7 = kp * error7 + ki * integral7 + kd * derivative7;
    output8 = kp * error8 + ki * integral8 + kd * derivative8;

    if (output1 > 200)
        output1 = 200;
    if (output1 < 70)
        output1 = 70;
    if (output2 > 70)
        output2 = 70;
    if (output2 < 50)
        output2 = 50;
    if (output3 > 70)
        output3 = 70;
    if (output3 < 70)
        output3 = 70;
    if (output4 > 70)
        output4 = 70;
    if (output4 < 40)
        output4 = 40;
    if (output5 > 100)
        output5 = 100;
    if (output5 < 100)
        output5 = 100;
    if (output6 > 110)
        output6 = 110;
    if (output6 < 50)
        output6 = 50;
    if (output7 > 50)
        output7 = 50;
    if (output7 < 30)
        output7 = 30;
    if (output8 > 50)
        output8 = 50;
    if (output8 < 0)
        output8 = 0;



    servo1.write(output1);
    servo2.write(output2);
    servo3.write(output3);
    servo4.write(output4);
    servo5.write(output5);
    servo6.write(output6);
    servo7.write(output7);
    servo8.write(output8);

    lastInput1 = error1;
    lastInput2 = error2;
    lastInput3 = error3;
    lastInput4 = error4;
    lastInput5 = error5;
    lastInput6 = error6;
    lastInput7 = error7;
    lastInput8 = error8;
    lastTime = now;

    delay(dt * 1000);
}
