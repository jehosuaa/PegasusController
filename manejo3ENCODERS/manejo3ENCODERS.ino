#include <Math.h>
//#include <SoftwareSerial.h>

int sentgir = 1; // Variable para el pin 32 
int sentgir1 = 1; // Variable para el pin 33 
int sentgir2 = 1; // Variable para el pin 34 
int sentgir3 = 1; // Variable para el pin 35 
int sentgir4 = 1; // Variable para el pin 36 
int sentgir5 = 1; // Variable para el pin 37 
int pwm;    // Variable para el pin 2, 3, 4, 5, 6, 7 

bool encoder1; //Variable para el pin 22
int encoder2 = 24; //Variable para el pin 24
int encoder3 = 26; //Variable para el pin 26
int encoder4x = 28; //Variable para el pin 28
int encoder4z = 31; //Variable para el pin 31
int encoder5 = 12; //SIN INFORMACION DE UN DATO

int fincarrera1 = 23; //Variable para el pin 23
int fincarrera2 = 25; //Variable para el pin 25
int fincarrera3 = 27; //Variable para el pin 27
int fincarrera4 = 29; //Variable para el pin 29
int fincarrera5 = 30; //Variable para el pin 30

//VARIABLES
const float pi = 3.1415;
const float deg2rad = pi/180;
const float vel2pwm = 255/100;
const float eje1 = 245698;
const float eje2 = 139260;
const float eje3 = 115988;
const float eje4x = 37260;
const float eje4z = 74590;
const float eje5 = 68238;
float L1;
float L2;
float L3;
float L4;
float px;
float py;
float pz;
int elegido;
int rango;
int giro;
int contarflancos1 = 0;  // Contadores para los encoders
int contarflancos2 = 0;
int contarflancos3 = 0;
int contarflancos4x = 0;
int contarflancos4z = 0;
int contarflancos5 = 0;
String msg;

float* posicion(float* q){
  px = cos(deg2rad*q[0]) * (L2*cos(deg2rad * q[1]) + L3*cos((deg2rad * q[1]) + (deg2rad * q[2])) + L4*cos((deg2rad * q[1]) + (deg2rad * q[2]) + (deg2rad * q[3])));
  py = sin(deg2rad*q[0]) * (L2*cos(deg2rad * q[1]) + L3*cos((deg2rad * q[1]) + (deg2rad * q[2])) + L4*cos((deg2rad * q[1]) + (deg2rad * q[2]) + (deg2rad * q[3])));
  pz = L1 + L2*sin(deg2rad * q[1]) + L3*sin((deg2rad * q[1]) + (deg2rad * q[2])) + L4*sin((deg2rad * q[1]) + (deg2rad * q[2]) + (deg2rad * q[3]));
  float pos[3] = {px, py, pz};
  return pos;
}
float* fincarrera(){
 int finc1 = digitalRead(fincarrera1);
 int finc2 = digitalRead(fincarrera2);
 int finc3 = digitalRead(fincarrera3);
 int finc4 = digitalRead(fincarrera4);
 int finc5 = digitalRead(fincarrera5);
 float finales[5] = {finc1, finc2, finc3, finc4, finc5};
 return finales;
}

void mover1(int sg, int vel){
  digitalWrite(32, sg);
  analogWrite(2, vel);}
void mover2(int sg, int vel){
  digitalWrite(33, sg);
  analogWrite(3, vel);}
void mover3(int sg, int vel){
  digitalWrite(34, sg);
  analogWrite(4, vel);}
void mover4x(int sg, int vel){
  digitalWrite(35, sg);
  analogWrite(5, vel);}
void mover4z(int sg, int vel){
  digitalWrite(36, sg);
  analogWrite(6, vel);}
void mover5(int sg, int vel){
  digitalWrite(37, sg);
  analogWrite(7, vel);}
int enco1(int q){
  int value = round(q*eje1/(2*pi));
  return value;}
int enco2(int q){
  int value = round(q*eje2/pi);
  return value;}
int enco3(int q){
  int value = round(q*eje3/pi);
  return value;}
int enco4x(int q){
  int value = round(q*eje4x/pi);
  return value;}
int enco4z(int q){
  int value = round(q*eje4z/(2*pi));
  return value;}
int enco5(int q){
  int value = round(q*eje4z/(2*pi));  //CAMBIA POR SER LA PINZA 
  return value;}

void homePos(){
  float* fincarrer;
  fincarrer = fincarrera();
  float finc1 = fincarrer[0];
  while(fincarrer[0] == finc1){ //MOTOR 1
    if (fincarrer[0] == 0){
      //Mover motores para que lamb sea <180
      mover1(0, pwm);
    }
    else {
      //Mover para que lamb sea >180
      mover1(1, pwm);
    }
    fincarrer = fincarrera(); 
  }
  contarflancos1 = eje1/2;
  float finc2 = fincarrer[1];
  while(fincarrer[1] == finc2){ //MOTOR 2
    if (fincarrer[1] == 0){
      //Mover motores para que lamb sea <90
      mover2(0, pwm);
    }
    else {
      //Mover para que lamb sea >180
      mover2(1, pwm);
    }
    fincarrer = fincarrera(); 
  }
  contarflancos2 = eje2/2;
  float finc3 = fincarrer[2];
  while(fincarrer[2] == finc3){ //MOTOR 3
    if (fincarrer[2] == 0){
      //Mover motores para que lamb sea 0
      mover3(0, pwm);
    }
    else {
      //Ya esta en poscion
      mover3(0, 0);
    }
    fincarrer = fincarrera(); 
  }
  contarflancos3 = eje3/2;  //aqui va 90° o 0°?
  
  while(fincarrer[4] == 0){
      //Mover motores para que lamb sea 0
    mover4z(0, pwm);
    fincarrer = fincarrera(); 
  }
  contarflancos5 = 0;
  float finc4x = fincarrer[3];  //MOTOR 4 EJE Z
  while(fincarrer[3] == finc4x){
    if (fincarrer[3] == 0){
      //Mover motores para que lamb sea <90
      mover4x(0, pwm);
    }
    else {
      //Mover para que lamb sea sea >90
      mover4x(1, pwm);
    }
    fincarrer = fincarrera(); 
  }
  contarflancos4x = eje4x/2;  
  //return contarflancos1, contarflancos2, contarflancos3, contarflancos4, contarflancos4;
}

void accion1(int sg, int mot,int q, int pw){
  int contar = 0;
  giro = sg;
  if (mot == 0){
    rango = enco1(q);
    while(rango >= contar){
      encoder1 = digitalRead(22);
      if (encoder1 == HIGH){
        contar++;}
      mover1(giro, pw);}
  mover1(giro, 0);
  if (giro == 0){
    contarflancos1 += rango;}
  else{
    contarflancos1 -= rango;}
  contar = 0;
  }
  if (mot == 1){
    rango = enco2(q);
    while(rango >= contar){
      encoder2 = digitalRead(24);
      if (encoder2 == HIGH){
        contar++;}
      mover2(giro, pw);}
  mover2(giro, 0);
  if (giro == 0){
    contarflancos2 += rango;}
  else{
    contarflancos2 -= rango;}
  contar = 0;
  }
  if (mot == 2){
    rango = enco3(q);
    while(rango >= contar){
      encoder3 = digitalRead(12); //NO EXISTE INFORMACION
      if (encoder3 == HIGH){
        contar++;}
      mover3(giro, pw);}
  mover3(giro, 0);
  if (giro == 0){
    contarflancos3 += rango;}
  else{
    contarflancos3 -= rango;}
  contar = 0;
  }
  if (mot == 3){
    rango = enco4x(q);
    while(rango >= contar){
      encoder4x = digitalRead(28);
      if (encoder4x == HIGH){
        contar++;}
      mover4x(giro, pw);}
  mover4x(giro, 0);
  if (giro == 0){
    contarflancos4x += rango;}
  else{
    contarflancos4x -= rango;}
  contar = 0;
  }
  if (mot == 4){
    rango = enco4z(q);
    while(rango >= contar){
      encoder4z = digitalRead(31);
      if (encoder4z == HIGH){
        contar++;}
      mover4z(giro, pw);}
  mover4z(giro, 0);
  if (giro == 0){
    contarflancos4z += rango;}
  else{
    contarflancos4z -= rango;}
  contar = 0;
  }
  if (mot == 5){
    rango = enco5(q);
    while(rango >= contar){
      encoder5 = digitalRead(26);
      if (encoder5 == HIGH){
        contar++;}
      mover5(giro, pw);}
  mover5(giro, 0);
  if (giro == 0){
    contarflancos5 += rango;}
  else{
    contarflancos5 -= rango;}
  contar = 0;
  }
}

void setup() {  
  //SENSOR FIN DE CARRERA
  pinMode(fincarrera1, INPUT);
  pinMode(fincarrera2, INPUT);
  pinMode(fincarrera3, INPUT);
  pinMode(fincarrera4, INPUT);
  pinMode(fincarrera5, INPUT);

 //ENCODERS
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  pinMode(encoder3, INPUT);
  pinMode(encoder4x, INPUT);
  pinMode(encoder4z, INPUT);
  pinMode(encoder5, INPUT);

  homePos();
}

void accion2() {;
  
}

void loop() {
  if(Serial.available()){
    msg = Serial.readString();
    delay(15);
  }
  String cinematica = msg.substring(0, 1);        // Primer carácter
  String veloc = msg.substring(1, 4);
  elegido = cinematica.toInt();
  pwm = round(veloc.toInt()*vel2pwm);
  if (elegido == 0){
    String sengi = msg.substring(4, 5);
    String motsel = msg.substring(5, 6);
    String q = msg.substring(6, 7);
    int sg = sengi.toInt();
    int mot = motsel.toInt();
    int grad = deg2rad*q.toInt();
    accion1(sg, mot, grad, pwm);}
  else {
    accion2();
  }
  
}
