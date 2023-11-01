// #include <Arduino.h>
#include <TB6612_ESP32.h>
#include <Wire.h>
#include <AS5600.h>


//#define Encoder_output_A 33 // pin2 of the Arduino
//#define Encoder_output_B 32 // pin 3 of the Arduino
#define Encoder_output_A 4
#define Encoder_output_B 2
AS5600 ams5600;
#define DIR_PIN 23

float angle_in = 0;
unsigned long previousMicros = 0;
unsigned long currentMicros = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int Count_pulses = 0;
arreglos
//
float referencias[]= {108.4310942,109.1519976,109.9892438,110.9187191,111.9142363,113.5200224,115.0242864,116.3815808,117.5525283,118.5062361,119.2216465,119.68779,119.9031225,119.8742447,119.6143163,119.1414226,118.4770673,117.6448843,116.6695967,115.5762125,114.389424,113.1331722,111.830336,110.5025172,109.1698934,107.8511238,106.5632976,105.3219181,104.1409219,103.3917667,102.5918851,101.7758104,100.9768754,100.2256423,99.54884419,98.96881961,98.50333947,97.77364476,96.89662491,95.89386389,94.78821925,93.60324618,92.36270443,91.090129,89.808446,88.53961966,87.3043239,86.12164044,85.00879399,83.98094047,83.05102673,82.22973679,81.52553253,80.94478596,80.49198894,80.17001702,79.98041827,79.92369589,79.99955615,80.20709692,80.54491699,80.71320874,80.71368579,80.54577721,80.21704422,79.74280958,79.14517679,78.45152815,77.69268377,76.45623829,75.27067197,74.15078824,73.10964232,72.15856001,71.30720752,70.56370964,69.93480856,69.42605242,69.04199828,68.7864125,68.66245024,68.67279522,68.81974073,69.10519278,69.53057591,70.09662265,70.80302933,71.64796677,72.62744587,73.73455927,74.95865161,76.28451277,77.69173274,78.56522315,79.37496365,80.0884022,80.67396234,81.10259578,81.34964031,81.3968935,81.23471783,80.87814688,80.67694346,80.6425429,80.78718574,81.12369839,81.66499992,82.42322709,83.40836178,84.626271,86.07615671,87.747589,89.61756388,91.64832575,93.78687524,95.96694626,98.11367108,100.150286,102.0054699,103.6196667,104.949158,105.9674631,106.6644164,107.0437111,107.1197593,107.1766599,107.4245743,107.848618,108.4310942};

float referencias2[]={-131.5510222,-130.9830457,-130.5697264,-130.3239467,-130.2532839,-130.1548867,-129.8045714,-129.212074,-128.3933924,-127.3697211,-126.1661973,-124.8106334,-123.3323709,-121.7613338,-120.1273034,-118.4594024,-116.7857459,-115.1332169,-113.5273186,-111.992067,-110.5498945,-109.2215446,-108.0259471,-106.9800706,-106.0987548,-105.3945326,-104.8774536,-104.5549243,-104.4315784,-104.3003636,-103.9404449,-103.3666671,-102.6024696,-101.6786722,-100.6319879,-99.50339262,-98.33645857,-96.43465321,-94.52001546,-92.62318055,-90.77435697,-89.00310433,-87.33807578,-85.80670053,-84.43479156,-83.24607686,-82.26166703,-81.49948946,-80.97373233,-80.69435212,-80.66669883,-80.8913043,-81.36386079,-82.07539254,-83.01259826,-84.15832241,-85.49210189,-86.99073416,-88.62881962,-90.37924649,-92.21359975,-93.31920423,-94.28890827,-95.08520812,-95.67702514,-96.04106658,-96.16291208,-96.03771839,-95.67046174,-94.99151152,-94.52001547,-94.26356853,-94.22632537,-94.40889076,-94.80830348,-95.41811498,-96.22855482,-97.22676537,-98.39708303,-99.72134079,-101.1791692,-102.748277,-104.4047001,-106.1230165,-107.8765319,-109.6374526,-111.3770662,-113.0659632,-114.6743329,-116.172371,-117.5308311,-118.7217401,-119.7192703,-120.3503702,-121.1188863,-121.9983605,-122.9578672,-123.9627747,-124.9756791,-125.9575444,-126.869084,-128.3011142,-129.8045714,-131.3586315,-132.9405506,-134.5256026,-136.0870501,-137.5961976,-139.0226,-140.3345139,-141.499689,-142.4865748,-143.2659579,-143.8129279,-144.1089267,-144.1435133,-143.9154485,-143.4328288,-142.7122325,-141.7770972,-140.6557067,-139.3791729,-137.9796974,-136.4892491,-134.9386695,-133.9788652,-133.0740557,-132.2556489,-131.5510222};


// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;
int motor2Pin1 = 18;
int motor2Pin2 = 19;
int enable1Pin2 = 25;
//int motor2 = 2 ;



// Setting PWM properties para control de los motores
const int freq = 30000;
const int pwmChannel = 12;
const int pwmChannel2 = 1;
const int resolution = 8;


//variables controlador
//tiempo de muestreo en micro segundos
int Ts = 1000;
//tiempo en microsegundos entre muestras
int Tr = 620000; //vel=80s
//int Tr = 465000; //vel=60s
int U1pwm=0;
float U1=0;
float Up1=0;
float Upp1=0;
float Uppp1=0;
float E1=0;

//float R1=80.0;
int U2pwm=0;
float U2=0;

float E2=0;
float Ep2=0;
//float R2=-90.0;
float R1=referencias[0];
float R2=referencias2[0];
int cont=0;


//funcion creada por nosotros para mapear el angulo ya que el map de arduino no funciona para decimales
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//funcion para medir el encoder digital 
float Angle() {

  digitalWrite(DIR_PIN, LOW);
  float in;
  /*
  in = mapf(float(ams5600.rawAngle()), 0, 4095, -93, 267);
  if (in > 200) {
    in = -93 - (267 - in);
  }
  */
  //calibracion encoder para obtener los angulos con el 0 en le linea prolongada del eslabon 1
  in = mapf(float(ams5600.rawAngle()), 0, 4095, 0, 360);
  //Serial.println("angulo");
  in-=30.0;
  if(in>150){
    in= -30-(330-in);
  }
  return in;
}


//funcion que se ejecuta con la interrupcion
void DC_Motor_Encoder() {
  int b = digitalRead(Encoder_output_B);
  if (b > 0) {
    Count_pulses++;
  }
  else {
    Count_pulses--;
  }
  //Serial.println("interrupcion");
  //Count_pulses=
}

void setup() {
  //direccion del pin del encoder digital
  pinMode(DIR_PIN, OUTPUT);
  //Serial.begin(115200);
  Wire.begin();
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  //motor 2
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable1Pin2, OUTPUT);

  
  //se codifican los pines del encoder 1 y se
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input
  //se define la interrupcion
  attachInterrupt(digitalPinToInterrupt(Encoder_output_A), DC_Motor_Encoder, RISING);

  // configure LED PWM functionalitites
  //m1
  ledcSetup(pwmChannel, freq, resolution);
  //m2
  ledcSetup(pwmChannel2, freq, resolution);
  // attach the channel to the GPIO to be controlled
  //m1
  ledcAttachPin(enable1Pin, pwmChannel);
  //m2
  ledcAttachPin(enable1Pin2, pwmChannel2);

  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
  digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, 200);
  
  delay(150);
}


void loop()
{
  

  currentMicros = micros();

  currentMillis=micros();
 //este if es el que va cambiando las referencias
  if(currentMillis-previousMillis>=Tr){
    previousMillis=currentMillis;
    R2=referencias2[cont];
    R1=referencias[cont];
    cont++;
    if(cont>=129){
      cont=0;
    }
  }
  //este if es para poder controlar el tiempo de muestreo
  if(currentMicros-previousMicros>=Ts){
    previousMicros=currentMicros;
    //se mide el angulo del encoder digital
    if (ams5600.detectMagnet() == 1 ) {
        angle_in = Angle();
        //Serial.println(angle_in);
    }
    
    float angulo2 = mapf(float(Count_pulses), 0, 341.2, 0, 360) * 90.0 / 60.0;
    // se calculan los errores
    E1= R1-angulo2;
    E2= R2-angle_in;
    
    
    U1=0.05*E1;
    U2=0.03*E2;
    //se define el sentido de giro
    if(U1<0){
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
    }else{
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
    }
    if(U2<0){
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
    }else{
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
    }
    //se define la saturacion
    if(U1>1){
      U1=1;
    }else if(U1<-1){
      U1=-1;
    }
    if(U2>1){
      U2=1;
    }else if(U2<-1){
      U2=-1;
    }
    //se transforma el resultado de la salida a pwm
    float U1pw=255*min(abs(U1),float(1));
    U1pwm=U1pw;
    //la salida del motor 2 se escala ya que este no puede soportar los 12 volteos que puede soportar el motor q
    float U2pw=min(abs(U2),float(1))*255.0*(9.0/12.0);
    U2pwm=U2pw;
    //se escribe la salida en los pines pwm
    ledcWrite(pwmChannel, U1pwm);
    ledcWrite(pwmChannel2, U2pwm);

    
    //se imprimen algunos valores con el fin de revisar resultados y hacer debugging si es necesario
    //Serial.print("U1:");
    //Serial.print(U1);
    //Serial.print(",");
    Serial.print("U2:");
    Serial.print(U2);
    Serial.print(",");
    Serial.print("R1:");
    Serial.print(R1);
    Serial.print(",");
    Serial.print("ang1:");
    Serial.print(angulo2);
    Serial.print(",");
    Serial.print("U1pwm:");
    Serial.print(U1pwm);
    Serial.print(",");
    Serial.print("U2pwm:");
    Serial.print(U2pwm);
    Serial.print(",");
    Serial.print("U2pwm:");
    Serial.print(U2pw);
    Serial.print(",");
    Serial.print("ang2:");
    Serial.println(angle_in);
    
  }


}
