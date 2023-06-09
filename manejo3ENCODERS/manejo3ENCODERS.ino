//INPUTS - OUTPUTS
int m1 = 34;
int m2 = 33;
int m3 = 32;
int m4x = 35;
int m4z = 36;
int mpinza = 37;  //PINES DE SALIDA PARA EL SENTIDO DE GIRO

int m1v = 4;
int m2v = 3;
int m3v = 2;
int m4xv = 5;
int m4zv = 6;
int mpinzav = 7; //PINES DE SALIDA PARA EL PWM

int encoder1 = 22;
int encoder2 = 24;
int encoder3 = 26;
int encoder4x = 28;
int encoder4z = 31;  //PINES DE ENTRADA PARA ENCODERS

int fincarrera1 = 23;
int fincarrera2 = 25;
int fincarrera3 = 27;
int fincarrera4x = 30;
int fincarrera4z = 29;  //PINES DE ENTRADA PARA FINALES DE CARRERA

//CONSTANTES
const float pi = 3.1415;
const float deg2rad = pi/180;
const float vel2pwm = 255/100;
const int eje1 = 245698;
const int eje2 = 139260;
const int eje3 = 115988;
const int eje4x = 37260;
const int eje4z = 74590;
const int eje5 = 68238;       //TOTAL DE FLANCOS CAPTURADOS EN UN GIRO COMPLETO
const float kp1= 1.44;
const float kp2= 1.11;
const float kp3= 0.54;
const float kp4x= 0.22;
const float kp4z= 0.27;   //CONSTANTES PARA EL CONTROL P DE LOS MOTORES
const int k1 = 15;
const int k2 = 20;
const int k3 = 8;
const float L1 = 23;
const float L2 = 23;
const float L3 = 23;
const float L4 = 7;   //TAMAÑO EN CM DE LOS BRAZOS

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
int decision1;    //PARA SABER LA POSICION DEL FIN DE CARRERA
int decision2;
int decision3;
int decision4x;
int decision4z;
volatile int contarflancos1 = 0;  //CONTADORES PARA ENCODERS 
volatile int contarflancos2 = 0;
volatile int contarflancos3 = 0;
volatile int contarflancos4x = 0;
volatile int contarflancos4z = 0;
volatile int contarflancos5 = 0;
volatile int contarf1 = 0;        //CONTADORES PARA CONTROL
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


float* posicion(int q1, int q2, int q3, int q4){  //CINEMÁTICA DIRECTA 
  px = cos(deg2rad*q1) * (L2*cos(deg2rad * q2) + L3*cos((deg2rad * q2) + (deg2rad * q3)) + L4*cos((deg2rad * q2) + (deg2rad * q3) + (deg2rad * q4)));
  py = sin(deg2rad*q1) * (L2*cos(deg2rad * q2) + L3*cos((deg2rad * q2) + (deg2rad * q3)) + L4*cos((deg2rad * q2) + (deg2rad * q3) + (deg2rad * q4)));
  pz = L1 + L2*sin(deg2rad * q2) + L3*sin((deg2rad * q2) + (deg2rad * q3)) + L4*sin((deg2rad * q2) + (deg2rad * q3) + (deg2rad * q4));
  float pos[3] = {px, py, pz};
  return pos;}
  
void finc1(){                               //FUNCIONES PARA LEER EL ESTADO DE LOS FINALES DE CARRERA
  finca1 = digitalRead(fincarrera1); }
void finc2(){
  finca2 = digitalRead(fincarrera2);  }
void finc3(){
  finca3 = digitalRead(fincarrera3);  }
void finc4x(){
  finca4x = digitalRead(fincarrera4z);  }
void finc4z(){
  finca4z = digitalRead(fincarrera4z);  }

void mover1(int sg, int vel){   //FUNCIONES QUE ENVIAN LOS DATOS DE MOVIMIENTO
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

void inc1(){    //FUNCIONES PARA CONTAR LA CANTIDAD DE FLANCOS
  contarf1++;}
void inc2(){
  contarf2++;}
void inc3(){
  contarf3++;}
void inc4x(){
  contarf4x++;}
void inc4z(){
  contarf4z++;}

double control1(int Oe, float e){   //FUNCIONES DE CONTROL
  double v1 = kp1*(Oe-e); 
  return v1;}
double control2(float Oe, float e, float th1, float th2){
  double v1 = kp2*(Oe-e)+((k1*cos(deg2rad*th1)+k2*cos(deg2rad*th1+deg2rad*th2))*cos(deg2rad*th2)); 
  return v1;}
double control3(float Oe, float e, float th1, float th2){
  double v1 = kp3*(Oe-e)+k3*sin(deg2rad*th1 + deg2rad*th2); 
  return v1;}
double control4x(float Oe, float e){
  double v1 = kp4x*(Oe-e); 
  return v1;}
double control4z(float Oe, float e){
  double v1 = kp4z*(Oe-e); 
  return v1;}

void moverse1(int E, int pwm, int gs){    //TIENE LO NECESARIO PARA MOVER EL MOTOR DE UN ANGULO a A UNO b

  contarf1 = 0;
  while(E-1 >= ep){   //ep CRECE DE 0 HASTA E EN UN RECORRIDO, TIENE SU VALOR EN GRADOS AL IGUAL QUE "E"
    attachInterrupt(digitalPinToInterrupt(encoder1), inc1, RISING); //PARA SUMAR LOS CAMBIOS DEL ENCODER Y SABER CUANTO HEMOS RECORRIDO 
    v1 = control1(E, enp);                      //enp PERTENECE AL CONTROL. CRECERÁ HASTA LA MITAD, DE AHI SE DEVUELVE CON LA MISMA PENDIENTE GENERANDO UNA RAMPA INVERTIDA
    if((pwm-round(v1)) < 60){mover1(gs, 60);}   //VEMOS SI LA DIFERENCIA ENTRE EL PWM Y EL CONTROL ES MENOR QUE LA VELOCIDAD MÍNIMA DE ARRANQUE
    else if((pwm-round(v1)) < pwm){mover1(gs, (pwm-round(v1)));}  //VELOCIDAD CAMBIANTE SIEMPRE RESTANDO AL PWM, ASI TAMBIEN PODREMOS FRENAR SUAVEMENTE EL MOTOR CON LA RAMPA INVERTIDA
    else{mover1(gs, pwm);}    //VELOCIDAD MÁXIMA ESTABLECIDA
    if (ep <= E/2){enp = ep;} //enp SE ENCARGA DE GENERAR UNA PENDIENTE CRECIENTE/DECRECIENTE EN EL RECORRIDO
    else{enp = E - ep;}       //CUANDO ep LLEGA A LA MITAD DE SU RECORRIDO ES NECESARIO QUE enp EMPIECE A REDUCIRSE Y CON ESTO LA VELOCIDAD
    ep = contarf1*360/eje1;}  //SE GUARDA EL VALOR EN ÁNGULOS DEL RECORRIDO
  if (gs == 1){contarflancos1 += ep;}   //SI EL SG ES 1 QUIERE DECIR QUE ESTAMOS EN LA REGION 180°+
  else{contarflancos1 -= ep;}           //POR EL CONTRARIO ACÁ ESTAMOS ENTRE 0 Y 180
  mover1(gs, 0);                    //CUANDO EL RECORRIDO TERMINE EL CONTROL YA HABRÁ ACTUADO Y SE PUEDE DEFINIR LA VEL. COMO 0
  enp = 0;
  ep = 0;
}
void moverse2(int E, int pwm, int gs){    //TODAS LAS FUNCIONES MOVERSE FUNCIONAN DE LA MISMA MANERA. A EXCEPCION DE LOS SENTIDOS DE GIRO.
  contarf2 = 0;
  while(E > ep){
    attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
    v1 = control2(E, enp, ep, contarflancos2);
    if((pwm-round(v1)) < 60){mover2(gs, 60);}
    else if((pwm-round(v1)) < pwm){mover2(gs, (pwm-round(v1)));}
    else{mover2(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    ep = contarf2*180/eje2;}
  enp = 0;
  ep = 0;
  if (gs == 1){contarflancos2 += ep;}
  else{contarflancos2 -= ep;}
  mover2(gs, 0);
}
void moverse3(int E, int pwm, int gs, bool hom = false){
  contarf3 = 0;
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
    v1 = control3(E, enp, contarflancos3, ep);
    if((pwm-round(v1)) < 60){mover3(gs, 70);}
    else if((pwm-round(v1)) < pwm){mover3(gs, (pwm-round(v1)));}
    else{mover3(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    ep = contarf3*180/eje3;}
  if (gs == 1){contarflancos3 -= ep;}
  else{contarflancos3 += ep;}
  enp = 0;
  ep = 0;
  mover3(0, v1);
}
/*void moverse4x(int E, int pwm, int gs, bool hom = false){   //FUNCIONALES PERO BAJO CIERTAS RESTRICCIONES. LEER README
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder4x), inc4x, RISING);
    v1 = control4x(E, enp);
    if((pwm-round(v1)) < 60){mover4x(gs, 70);}
    else if((pwm-round(v1)) < pwm){mover4x(gs, (pwm-round(v1)));}
    else{mover4x(gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    ep = contarf4z*180/eje4x;}
  enp = 0;
  ep = 0;
  if (gs == 1){contarflancos4x += ep;}
  else{contarflancos4x -= ep;}
  contarf4x = 0;
  mover4x(gs, 0);
  mover4z(gs, 0);
}
void moverse4z(int E, int pwm, int gs, bool hom = false){
  while(E > ep && hom == false){
    attachInterrupt(digitalPinToInterrupt(encoder4z), inc4z, RISING);
    v1 = control4z(E, enp);
    if((pwm-round(v1)) < 60){
      mover4z(gs, 70);
      mover4x(!gs, 70);}
    else if((pwm-round(v1)) < pwm){
      mover4z(gs, (pwm-round(v1)));
      mover4x(!gs, (pwm-round(v1)));}
    else{
      mover4z(gs, pwm);
      mover4x(!gs, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    if (gs == 1){contarflancos4z += ep;}
    else{contarflancos4z -= ep;}
    ep = contarf4z*360/eje4z;}
  enp = 0;
  ep = 0;
  contarf4z = 0;
  mover4z(gs, 0);
  mover4x(gs, 0);
}*/

void homePos(){
  float E = 90;   //SE DEFINE 90 COMO EL ANGULO MAXIMO DE GIRO. SI NO DETECTA CAMBIO DEL FIN DE CARRERA VUELVE A INGRESARLOS.
  float v1;
  while(finca1 == decision1 ){  //decision1 YA ESTÁ INICIALIZADA, SE ESPERA EL CAMBIO DE FLANCO.
    if(finca1 == 1){                                                  //LO UNICO QUE CAMBIA ES EL SENTIDO DE GIRO
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
  attachInterrupt(digitalPinToInterrupt(fincarrera1), finc1, CHANGE);}  //ESPERAMOS EL CAMBIO EN EL FIN DE CARRERA
  mover1(0, 0);
  contarflancos1 = 180;   //VALOR POR DEFECTO DE LA POSICION INICIAL
  enp = 0;
  ep = 0;
  contarf1 = 0;
  
  while(finca2 == decision2 ){  //FUNCIONA DE LA MISMA MANERA
    if(finca2 == 0){
      attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
      v1 = control2(E, enp, ep, contarflancos2);
      if((pwm-round(v1)) < 60){mover2(1, 60);}
      else if((pwm-round(v1)) < pwm){mover2(1, (pwm-round(v1)));}
      else{mover2(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf2*180/eje2;}
    else{
      attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
      v1 = control2(E, enp, ep, contarflancos2);
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
      v1 = control3(E, 90, contarflancos3, ep);
      attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
      if((pwm-round(v1)) < 60){mover3(0, 60);}
      else if((pwm-round(v1)) < pwm){mover3(0, (pwm-round(v1)));}
      else{mover3(0, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf3*180/eje3;}
    else{
      attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
      v1 = control3(E, enp,contarflancos3,ep);
      if((pwm-round(v1)) < 0){mover3(1, 60);}
      else if((pwm-round(v1)) < pwm){mover3(1, (pwm-round(v1)));}
      else{mover3(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf3*180/eje3;}
  attachInterrupt(digitalPinToInterrupt(fincarrera3), finc3, CHANGE);
  }
  contarflancos3 = 90;
  mover3(0,v1);
  enp = 0;
  ep = 0;
  contarf3 = 0;
 
/*  while(finca4z == decision4z ){    //PARA MOVER LA MUÑECA EN EL EJE Z AMBOS MOTORES GIRAN AL MISMO LADO, PARA MOVER EN X GIRAN EN SENTIDOS CONTRARIOS.
    //int conta=contarf4z;
    //Serial.println(contarf4z);
    if(finca4z == 1){
      v1 = control1(E, enp);
      attachInterrupt(digitalPinToInterrupt(encoder4x), inc4z, RISING);
      //Serial.println(finca4x);
      if((pwm-round(v1)) < 60){
        mover4x(1, 60);
        mover4z(1, 60);}
      else if((pwm-round(v1)) < pwm){
        mover4x(1, (pwm-round(v1)));
        mover4z(1, (pwm-round(v1)));}
      else{
        mover4x(1, pwm);
        mover4z(1, pwm);}
      if (ep < E/2){enp = ep;}
      else{enp = E - ep;}
      ep = contarf4z*360/eje4z;
    }
  else{
    attachInterrupt(digitalPinToInterrupt(encoder4x), inc4z, RISING);
    v1 = control4z(E, enp);
    if((pwm-round(v1)) < 60){
      mover4x(0, 60);
      mover4z(0, 60);}
    else if((pwm-round(v1)) < pwm){
      mover4x(0, (pwm-round(v1)));
      mover4z(0, (pwm-round(v1)));}
    else{
      mover4x(0, pwm);
      mover4z(0, pwm);}
    if (ep < E/2){enp = ep;}
    else{enp = E - ep;}
    
    ep = contarf4z*360/eje4z;
    }
  
  attachInterrupt(digitalPinToInterrupt(fincarrera4z), finc4z, CHANGE);
  }
  contarflancos4z = 0;
  mover4x(0, 0);  
  mover4z(0, 0);
  enp = 0;
  ep = 0;
  contarf4z = 0;
  finc4x();
  decision4x = finca4x;*/
}

void accion1(int ang1, int ang2,int ang3, int ang4x, int ang4z, int pinza, int pwm){  //SE ENCARGA DE RECIBIR LOS ANGULOS DESEADOS DESDE MATLAB
  int dif1 = ang1 - contarflancos1;           //AQUI SE DEFINE CUAL ES EL SENTIDO DE GIRO PARA LLEGAR AL ANGULO DESEADO
  if (dif1 < 0){moverse1(abs(dif1), pwm, 0);}   //SE EJECUTA EL MOVIMIENTO DEPENDIENDO EL SENTIDO DE GIRO
  else{moverse1(abs(dif1), pwm, 1);}
  int dif2 = ang2 - contarflancos2;
  if (dif2 < 0){moverse2(abs(dif2), pwm, 0);}
  else{moverse2(abs(dif2), pwm, 1);}
  int dif3 = ang3 - contarflancos3;
  if (dif3 < 0){moverse3(abs(dif3), pwm, 0);}
  else{moverse3(abs(dif3), pwm, 1);}
  int dif4x = ang4x - contarflancos4x;
  /*if (dif4x < 0){moverse4x(abs(dif4x), pwm, 0);}
  else{moverse4x(abs(dif4x), pwm, 1);}
  int dif4z = ang4z - contarflancos4z;
  if (dif4z < 0){moverse4z(abs(dif4z), pwm, 0);}
  else{moverse4z(abs(dif4z), pwm, 1);}
  if (pinza == 1){moverpinza(1, pwm);}*/
}

void demo(){    //CUMPLE UNA SERIE DE PASOS YA DEFINIDOS
  homePos();
  
  int dif1 = 90 - contarflancos1;   //contarflancosX SE ENCARGA DE LLEVAR REGISTRO DEL ANGULO ACTUAL DEL MOTOR X
  int dif2 = 45 - contarflancos2;
  int dif3 = 60 - contarflancos3;
  moverse1(abs(dif1), pwm, 0);      //DENTRO DE LA FUNCION SE ACTUALIZA EL VALOR DEL ANGULO ACTUAL
  moverse2(abs(dif2), pwm, 0);
  moverse3(abs(dif3), pwm, 0);
  delay(1000);
  dif2 = 90 - contarflancos2;
  dif3 = 90 - contarflancos3;
  moverse2(abs(dif2), pwm, 1);
  moverse3(abs(dif3), pwm, 1);
  delay(1000);
  dif1 = 270 - contarflancos1;
  moverse1(abs(dif1), pwm, 1);
  delay(1000);
  dif2 = 45 - contarflancos2;
  dif3 = 60 - contarflancos3;
  moverse2(abs(dif2), pwm, 0);
  moverse3(abs(dif3), pwm, 0);
  delay(2000);
   
  homePos();    //SIEMPRE FINALIZA EN homePos PARA NO PERDER LA ORIENTACION
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
  decision1 = finca1;   //SE GUARDAN LOS VALORES DE LOS FINALES DE CARRERA ANTES DE EMPEZAR A MOVERSE
  finc2();
  decision2 = finca2;
  finc3();
  decision3 = finca3;
  finc4z();
  decision4x = finca4z;
  finc4z();
  decision4z = finca4z;

  attachInterrupt(digitalPinToInterrupt(encoder1), inc1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), inc2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder3), inc3, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoder4x), inc4x, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoder4x), inc4z, RISING);

  Serial.begin(9600);
  homePos();    //SIEMPRE INICIA EN homePos
}

void loop() {
  if(Serial.available()){   //EL ROBOT SE MOVERÁ SIGUIENDO LAS INSTRUCCIONES QUE LLEGAN DESDE MATLAB
    msg = Serial.readString();
    String comando = msg.substring(0, 1);        // DEFINE QUÉ FUNCION REALIZARÁ
    elegido = comando.toInt();
    if (elegido == 0){                      //0 SE DEFINE PARA LA accion1, DONDE SE RECIBIRÁN LOS ANGULOS ESPECIFICADOS POR LA PERSONA
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
      accion1(ang1, ang2, ang3, ang4x, ang4z, pinza, pwm);}
    else if(elegido == 1){demo();}    //SI EL BOTON DEMO ES CLICKEADO ENTONCES REALIZARÁ LOS MOVIMIENTOS PREDETERMINADOS
    else{homePos();}                  //BOTON AUXILIAR PARA MOVER A homePos
    
    delay(1000);}
}
