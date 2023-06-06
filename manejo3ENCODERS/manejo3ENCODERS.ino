//INPUTS - OUTPUTS
int m1 = 34;
int m2 = 33;
int m3 = 32;
int m4x = 35;
int m4z = 36;
int mpinza = 37;

int m1v = 4;
int m2v = 3;
int m3v = 2;
int m4xv = 5;
int m4zv = 6;
int mpinzav = 7;

int encoder1 = 22;
int encoder2 = 24;
int encoder3 = 26;
int encoder4x = 28;
int encoder4z = 31;

int fincarrera1 = 23;
int fincarrera2 = 25;
int fincarrera3 = 27;
int fincarrera4x = 30;
int fincarrera4z = 29;

//CONSTANTES
const float pi = 3.1415;
const float deg2rad = pi/180;
const float vel2pwm = 255/100;
const int eje1 = 245698;
const int eje2 = 139260;
const int eje3 = 115988;
const int eje4x = 37260;
const int eje4z = 74590;
const int eje5 = 68238;
const float kp1= 1.44;
const float kp2= 1.11;
const float kp3= 0.54;
const float kp4x= 0.22;
const float kp4z= 0.27;
const int k1 = 15;
const int k2 = 20;
const int k3 = 8;
const float L1 = 23;
const float L2 = 23;
const float L3 = 23;
const float L4 = 7;

int sentgir = 1; // Variable para el pin 32 
int sentgir1 = 1; // Variable para el pin 33 
int sentgir2 = 1; // Variable para el pin 34 
int sentgir3 = 1; // Variable para el pin 35 
int sentgir4 = 1; // Variable para el pin 36 
int sentgir5 = 1; // Variable para el pin 37 
int pwm;    // Variable para el pin 2, 3, 4, 5, 6, 7 

//VARIABLES
String msg;
float px;
float py;
float pz;
int elegido;
int rango;
int giro;
double v1;
volatile int contarflancos1 = 0;  // Contadores para los encoders
volatile int contarflancos2 = 0;
volatile int contarflancos3 = 0;
volatile int contarflancos4x = 0;
volatile int contarflancos4z = 0;
volatile int contarflancos5 = 0;
int decision1;
int decision2;
int decision3;
int decision4x;
int decision4z;
volatile int contarf1 = 0; // Contadores para los CONTROLES
volatile int contarf2 = 0;
volatile int contarf3 = 0;
volatile int contarf4x= 0;
volatile int contarf4z= 0;
volatile int contarf5= 0;
volatile int finca1 = 0;
volatile int finca2 = 0;
volatile int finca3 = 0;
volatile int finca4x = 0;
volatile int finca4z = 0;
volatile int enp = 0;
volatile float ep = 0;


float* posicion(int q1, int q2, int q3, int q4){
  px = cos(deg2rad*q1) * (L2*cos(deg2rad * q2) + L3*cos((deg2rad * q2) + (deg2rad * q3)) + L4*cos((deg2rad * q2) + (deg2rad * q3) + (deg2rad * q4)));
  py = sin(deg2rad*q1) * (L2*cos(deg2rad * q2) + L3*cos((deg2rad * q2) + (deg2rad * q3)) + L4*cos((deg2rad * q2) + (deg2rad * q3) + (deg2rad * q4)));
  pz = L1 + L2*sin(deg2rad * q2) + L3*sin((deg2rad * q2) + (deg2rad * q3)) + L4*sin((deg2rad * q2) + (deg2rad * q3) + (deg2rad * q4));
  float pos[3] = {px, py, pz};
  return pos;}
  
void finc1(){
  finca1 = digitalRead(fincarrera1); }
void finc2(){
  finca2 = digitalRead(fincarrera2);  }
void finc3(){
  finca3 = digitalRead(fincarrera3);  }
void finc4x(){
  finca4x = digitalRead(fincarrera4x);  }
void finc4z(){
  finca4z = digitalRead(fincarrera4z);  }

void mover1(int sg, int vel){
  digitalWrite(34, sg);
  analogWrite(4, vel);}
void mover2(int sg, int vel){
  digitalWrite(33, sg);
  analogWrite(3, vel);}
void mover3(int sg, int vel){
  digitalWrite(32, sg);
  analogWrite(2, vel);}
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

void inc1(){
  contarf1++;}
void inc2(){
  contarf2++;}
void inc3(){
  contarf3++;}
void inc4x(){
  contarf4x++;}
void inc4z(){
  contarf4z++;}

double control1(float Oe, float e){
  double v1 = kp1*(Oe-e); 
  return v1;}
double control2(float Oe, float e, float th1, float th2){
  double v1 = kp2*(Oe-e)+((k1*cos(th1)+k2*cos(th1+th2))*cos(th2)); 
  return v1;}
double control3(float Oe, float e, float th1, float th2){
  double v1 = kp3*(Oe-e)+k3*sin(th1 + th2); 
  return v1;}
double control4x(float Oe, float e){
  double v1 = kp4x*(Oe-e); 
  return v1;}
double control4z(float Oe, float e){
  double v1 = kp4z*(Oe-e); 
  return v1;}

void moverse1(int E, int pwm, int gs, bool hom = false){
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder1), inc1, RISING);
    v1 = control1(E, enp);
    if((pwm-round(v1)) < 60){mover1(gs, 70);}
    else if((pwm-round(v1)) < pwm){mover1(gs, (pwm-round(v1)));}
    else{mover1(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    if (gs == 1){contarflancos1 += ep;}
    else{contarflancos1 -= ep;}
    ep = contarf1*360/eje1;}
  enp = 0;
  ep = 0;
  contarf1 = 0;
}
void moverse2(int E, int pwm, int gs, bool hom = false){
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
    v1 = control2(E, enp, ep, contarflancos3);
    if((pwm-round(v1)) < 60){mover2(gs, 70);}
    else if((pwm-round(v1)) < pwm){mover2(gs, (pwm-round(v1)));}
    else{mover2(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    if (gs == 1){contarflancos2 += ep;}
    else{contarflancos2 -= ep;}
    ep = contarf2*180/eje2;}
  enp = 0;
  ep = 0;
  contarf2 = 0;
}
void moverse3(int E, int pwm, int gs, bool hom = false){
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
    v1 = control3(E, enp, contarflancos2, ep);
    if((pwm-round(v1)) < 60){mover3(gs, 70);}
    else if((pwm-round(v1)) < pwm){mover3(gs, (pwm-round(v1)));}
    else{mover3(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    if (gs == 1){contarflancos3 += ep;}
    else{contarflancos3 -= ep;}
    ep = contarf3*180/eje3;}
  enp = 0;
  ep = 0;
  contarf3 = 0;
}
void moverse4x(int E, int pwm, int gs, bool hom = false){
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder4x), inc4x, RISING);
    v1 = control4x(E, enp);
    if((pwm-round(v1)) < 60){mover4x(gs, 70);}
    else if((pwm-round(v1)) < pwm){mover4x(gs, (pwm-round(v1)));}
    else{mover4x(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    if (gs == 1){contarflancos4x += ep;}
    else{contarflancos4x -= ep;}
    ep = contarf4x*180/eje4x;}
  enp = 0;
  ep = 0;
  contarf4x = 0;
}
void moverse4z(int E, int pwm, int gs, bool hom = false){
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder4z), inc4z, RISING);
    v1 = control4z(E, enp);
    if((pwm-round(v1)) < 60){mover4z(gs, 70);}
    else if((pwm-round(v1)) < pwm){mover4z(gs, (pwm-round(v1)));}
    else{mover4z(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    if (gs == 1){contarflancos4z += ep;}
    else{contarflancos4z -= ep;}
    ep = contarf4z*360/eje4z;}
  enp = 0;
  ep = 0;
  contarf4z = 0;
}

void moverpinza(int swi, int pwm){
  
}

void homePos(){
  float E = 90;
  float v1;
  while(finca1 == decision1 ){
    if(finca1 == 1){
      attachInterrupt(digitalPinToInterrupt(encoder1), inc1, RISING);
      v1 = control1(E, enp);
      if((pwm-round(v1)) < 60){mover1(1, 70);}
      else if((pwm-round(v1)) < pwm){mover1(1, (pwm-round(v1)));}
      else{mover1(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf1*2*180/eje1;}
    else{
      attachInterrupt(digitalPinToInterrupt(encoder1), inc1, RISING);
      v1 = control1(E, enp);
      if((pwm-round(v1)) < 60){mover1(0, 70);}
      else if((pwm-round(v1)) < pwm){mover1(0, (pwm-round(v1)));}
      else{mover1(0, pwm);}
      if (ep < E/2){
        ep = contarf1*2*180/eje1;
        enp = ep;}
      else{
        ep = contarf1*360/eje1;
        enp = E - ep;}}
  attachInterrupt(digitalPinToInterrupt(fincarrera1), finc1, CHANGE);}
  mover1(0, 0);
  contarflancos1 = 180;
  enp = 0;
  ep = 0;
  contarf1 = 0;
  
  while(finca2 == decision2 ){
    if(finca2 == 0){
      attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
      v1 = control2(E, enp, ep, contarflancos3);
      if((pwm-round(v1)) < 60){mover2(1, 60);}
      else if((pwm-round(v1)) < pwm){mover2(1, (pwm-round(v1)));}
      else{mover2(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf2*180/eje2;}
    else{
      attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
      v1 = control2(E, enp, ep, contarflancos3);
      if((pwm-round(v1)) < 60){mover2(0, 60);}
      else if((pwm-round(v1)) < pwm){mover2(0, (pwm-round(v1)));}
      else{mover2(0, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf2*180/eje2;}
  attachInterrupt(digitalPinToInterrupt(fincarrera2), finc2, CHANGE);
  }
  mover2(0, 0);
  contarflancos2 = 90;
  enp = 0;
  ep = 0;
  contarf2 = 0;
  
  while(finca3 == decision3 ){
    if(finca3 == 0){
      v1 = control3(E, enp, contarflancos2, ep);
      attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
      if((pwm-round(v1)) < 60){mover3(0, 60);}
      else if((pwm-round(v1)) < pwm){mover3(0, (pwm-round(v1)));}
      else{mover3(0, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf3*180/eje3;}
    else{
      attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
      v1 = control3(E, enp,contarflancos2,ep);
      if((pwm-round(v1)) < 0){mover3(1, 60);}
      else if((pwm-round(v1)) < pwm){mover3(1, (pwm-round(v1)));}
      else{mover3(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf3*180/eje3;}
  attachInterrupt(digitalPinToInterrupt(fincarrera3), finc3, CHANGE);
  }
  contarflancos3 = 90;
  mover3(0, v1);
  enp = 0;
  ep = 0;
  contarf3 = 0;

  while(finca4z == decision4z ){

    Serial.println(decision4z);
    if(finca4z == 1){
      v1 = control1(E, enp);
      attachInterrupt(digitalPinToInterrupt(encoder4x), inc4x, RISING);
      //Serial.println(finca4x);
      if((pwm-round(v1)) < 60){
        mover4x(0, 50);
        mover4z(1, 50);}
      else if((pwm-round(v1)) < pwm){
        mover4x(0, (pwm-round(v1)));
        mover4z(1, (pwm-round(v1)));}
      else{
        mover4x(0, pwm);
        mover4z(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf4z*360/eje4z;
    }
  else{
    attachInterrupt(digitalPinToInterrupt(encoder4x), inc4z, RISING);
    v1 = control4z(E, enp);
    if((pwm-round(v1)) < 60){
      mover4x(1, 40);
      mover4z(0, 40);}
    else if((pwm-round(v1)) < pwm){
      mover4x(1, (pwm-round(v1)));
      mover4z(0, (pwm-round(v1)));}
    else{
      mover4x(1, pwm);
      mover4z(0, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    ep = contarf4z*360/eje4z;
    }
  attachInterrupt(digitalPinToInterrupt(fincarrera4x), finc4x, CHANGE);
  }
  contarflancos4z = 0;
  mover4x(0, 0);  
  mover4z(0, 0);
  enp = 0;
  ep = 0;
  contarf4z = 0;
  finc4x();
  decision4x = finca4x;

  while(finca4x == decision4x ){
    if(finca4x == 1){
      v1 = control1(E, enp);
      attachInterrupt(digitalPinToInterrupt(encoder4x), inc4x, RISING);
      if((pwm-round(v1)) < 60){
        mover4x(1, 70);
        mover4z(1, 70);}
      else if((pwm-round(v1)) < pwm){
        mover4x(1, (pwm-round(v1)));
        mover4z(1, (pwm-round(v1)));}
      else{
        mover4x(1, pwm);
        mover4z(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf4x*180/eje4x;
    }
    else{
      v1 = control1(E, enp);
      attachInterrupt(digitalPinToInterrupt(encoder4x), inc4x, RISING);
      if((pwm-round(v1)) < 60){
        mover4x(0, 70);
        mover4z(0, 70);}
      else if((pwm-round(v1)) < pwm){
        mover4x(0, (pwm-round(v1)));
        mover4z(0, (pwm-round(v1)));}
      else{
        mover4x(0, pwm);
        mover4z(0, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf4x*180/eje4x;
    }
  attachInterrupt(digitalPinToInterrupt(fincarrera4x), finc4x, CHANGE);
  }
  mover4x(0, 0);  
  mover4z(0, 0);
  contarflancos4x = 90;
  enp = 0;
  ep = 0;
  contarf4x = 0;
}

void accion1(int ang1, int ang2,int ang3, int ang4x, int ang4z, int pinza, int pwm){
  int dif1 = ang1 - contarflancos1;
  if (dif1 < 0){moverse1(abs(dif1), pwm, 0);}
  else{moverse1(abs(dif1), pwm, 1);}
  int dif2 = ang2 - contarflancos2;
  if (dif2 < 0){moverse2(abs(dif2), pwm, 0);}
  else{moverse2(abs(dif2), pwm, 1);}
  int dif3 = ang3 - contarflancos3;
  if (dif3 < 0){moverse3(abs(dif3), pwm, 0);}
  else{moverse3(abs(dif3), pwm, 1);}
  int dif4x = ang4x - contarflancos4x;
  if (dif4x < 0){moverse4x(abs(dif4x), pwm, 0);}
  else{moverse4x(abs(dif4x), pwm, 1);}
  int dif4z = ang4z - contarflancos4z;
  if (dif4z < 0){moverse4z(abs(dif4z), pwm, 0);}
  else{moverse4z(abs(dif4z), pwm, 1);}
  if (pinza == 1){moverpinza(1, pwm);}
}

void demo(){
  homePos();
  moverse1(30, pwm, 1);
  moverse2(35, pwm, 1);
  moverse3(35, pwm, 1);
  delay(2000);
  moverse1(30, pwm, 1);
  moverse2(90, pwm, 1);
  moverse3(90, pwm, 1); 
  moverpinza(1, pwm);
  delay(2000); 
  moverse1(330, pwm, 1);
  moverse2(90, pwm, 1);
  moverse3(90, pwm, 1); 
  delay(2000); 
  moverse1(330, pwm, 1);
  moverse2(35, pwm, 1);
  moverse3(35, pwm, 1);
  delay(2000); 
  homePos();
}

void setup() {  
  pinMode(fincarrera1, INPUT);
  pinMode(encoder1, INPUT_PULLUP);
  pinMode(34, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(fincarrera2, INPUT);
  pinMode(encoder2, INPUT_PULLUP);
  pinMode(33, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(fincarrera3, INPUT);
  pinMode(encoder3, INPUT_PULLUP);
  pinMode(36, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(fincarrera4x, INPUT);
  pinMode(encoder4x, INPUT_PULLUP);
  pinMode(35, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(fincarrera4z, INPUT);
  pinMode(encoder4z, INPUT_PULLUP);

  finc1();
  decision1 = finca1;
  finc2();
  decision2 = finca2;
  finc3();
  decision3 = finca3;
  finc4z();
  decision4x = finca4x;
  finc4z();
  decision4z = finca4z;

  attachInterrupt(digitalPinToInterrupt(encoder1), inc1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder4x), inc4x, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder4x), inc4z, RISING);

  Serial.begin(9600);
  homePos();
}

void loop() {
  if(Serial.available()){
    msg = Serial.readString();
    String comando = msg.substring(0, 1);        // Primer carácter
    elegido = comando.toInt();
    if (elegido == 0){
      String veloc = msg.substring(1, 4);
      String m1 = msg.substring(4, 7);
      String m2 = msg.substring(7, 10);
      String m3 = msg.substring(10, 13);
      String m4x = msg.substring(13, 16);
      String m4z = msg.substring(16, 19);
      String pinz = msg.substring(19, 20);
      pwm = round(veloc.toInt()*vel2pwm);
      int ang1 = m1.toInt();
      int ang2 = m2.toInt();
      int ang3 = m3.toInt();
      int ang4x = m4x.toInt();
      int ang4z = m4z.toInt();
      int pinza = pinz.toInt();
      accion1(ang1, ang2, ang3, ang4x, ang4z, pinza, pwm);
    }
    else{
      demo();
    }
    delay(1000);}
}
