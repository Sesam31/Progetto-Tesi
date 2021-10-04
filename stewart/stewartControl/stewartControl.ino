#include <Servo.h>
#include <math.h>
Servo servo[6];

const float pi = 3.141592653;
// range movimento inferiore e superiore
const int inf = 900;
const int sup = 2200;
// microsecondi per avere la manovella dei sevomotori orizzontali
const int oriz[6] = {1500,1550,1500,1560,1560,1500};
// microsecondi per avere la manovella dei servomotori in posizione di calibrazione
//const int cali[6] = {1325,1375,1325,1385,1385,1385};
// relazione tra microsecondi e radianti [us/rad]
const int usrad = (375/45)*(360/(2*pi));                        
// posizione richiesta {x,y,z,roll,pitch,yaw}

// VETTORI DI SISTEMA
// matrice rotazione globale
float Rg[3][3];   
// altezza piattaforma a riposo
float h0 = 106.5;                                                // ALTEZZA OTTENUTA SPERIMENTALMENTE, TEST: ROTAZIONE ASSE X 5°, DEVE ESSERE SIMMETRICO!!!
// vettore posizione {x,y,z} inizializzato con altezza riposo
float T[3][1] = {0,0,h0};
// vettore rotazione {roll,pitch,yaw}
float R[3][1] = {0,0,0};
// coordinate giunto biella/manovella al variare di alfa e beta (inizializzata durante il setup)
float giun[6][3];
// vettori origine base -> giunto piattaforma
float baseVec[6][3];
// vettori asse servo -> giunto piattaforma
float armVec[6][3];

//DESCRIZIONE TOPOLOGICA SISTEMA
const float alfa0[6];
// angolo manovella rispetto al piano xy
//float alfa[6];
float alfa[6] = {-0.34,-0.34,-0.34,-0.34,-0.34,-0.34};
// angolo manovella rispetto all'asse x
const float beta[6] = {-pi/2,pi/6,pi/6,5*pi/6,5*pi/6,pi/2};
// lunghezza manovella
const float lungman = 16;
// lunghezza biella
const float lungbie = 124;

//angolo asse rotazione servo rispetto a origine base
const float gamma = (13.38*2*pi)/360;
// distanza tra centro riferimento e asse rotazione servo [mm]
const float rd = 87.63;    // rotation distance (base)
// coordinate punti base
float base[6][3] = {{-rd*cos(gamma),-rd*sin(gamma),0},
                    {rd*cos(pi/3+gamma),-rd*sin(pi/3+gamma),0},
                    {rd*cos(pi/3-gamma),-rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3-gamma),rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3+gamma),rd*sin(pi/3+gamma),0},
                    {-rd*cos(gamma),rd*sin(gamma),0}};

//angolo asse giunto rispetto origine piattaforma
const float tau = (4.14*2*pi)/360;
const float pd = 80.52;    // pivot distance (piattaforma)
//coordinate punti piattaforma (inizializzato con altezza homing)
float piat[6][3] = {{pd*cos(2*pi/3+tau),-pd*sin(2*pi/3+tau),T[2][0]},
                    {pd*cos(2*pi/3-tau),-pd*sin(2*pi/3-tau),T[2][0]},
                    {pd*cos(tau),-pd*sin(tau),T[2][0]},
                    {pd*cos(tau),pd*sin(tau),T[2][0]},                        
                    {pd*cos(2*pi/3-tau),pd*sin(2*pi/3-tau),T[2][0]},
                    {pd*cos(2*pi/3+tau),pd*sin(2*pi/3+tau),T[2][0]}};

// distanza^2 tra asse e giunto piattaforma
float d2[6];
// distanza^2 manovella
const float m2 = pow(lungman,2);
// distanza^2 manovella
const float b2 = pow(lungbie,2);

// PARAMETRI EQUAZIONE ANGOLO
float L[6],M[6],N[6],angServo[6];


// PARAMETRI PIANO RESISTIVO
#define sensePin A0
// min 100 max 920, piatto 0.34x0.27m
float x, xOld;
float y, yOld;
float xorig,yorig;
#define in1 12  
#define in2 13 

// PARAMETRI PID
float Kp = 22.5;//22.5;   //22.5 con foglio carta
float Ki = 0;//10;
float Kd = 0;//40; //0.1
double oldTime, newTime, deltaTime;
double sumErrX, sumErrY;
double oldDerivataX, oldDerivataY;
float setX = 0.17, setY = 0.135, errX, errY; // setpoint centrato 26.95 21.90
float xVel, yVel, xVelOld, yVelOld;
float xAcc, yAcc;
double pidX, pidY;
bool pause = false;
char command;

void setup() {
  Serial.begin(38400);      // tenere alto per evitare rallentamento processi

  // LED usato in caso di emergenza
  pinMode(LED_BUILTIN, OUTPUT);

  // pin output controllo piano
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  
  servo[0].attach(3, inf, sup);
  servo[1].attach(5, inf, sup);
  servo[2].attach(6, inf, sup);
  servo[3].attach(9, inf, sup);
  servo[4].attach(10, inf, sup);
  servo[5].attach(11, inf, sup);

  
  getSense();
  oldTime = millis();
  xOld = x;
  yOld = y;
  
  /*while(true){
    setPosition(0,0,110,radians(0),radians(-5),radians(0)); 
  }*/
  
  /*while(true){
    for(float t = -3; t < 0; t+=0.1){
      setPosition(0,0,110,radians(t),radians(t+3),radians(0)); 
    }
    for(float t = 0; t < 3; t+=0.1){
      setPosition(0,0,110,radians(t),radians(3-t),radians(0)); 
    }
    for(float t = 3; t > 0; t-=0.1){
      setPosition(0,0,110,radians(t),radians(t-3),radians(0)); 
    }
    for(float t = 0; t > -3; t-=0.1){
      setPosition(0,0,110,radians(t),radians(-3-t),radians(0)); 
    }
  }
  */
  /*while(true){
  for(float t = -8; t < 8; t+=0.25){
    setPosition(0,0,110,radians(0),radians(0),radians(t));
    Serial.println(t);
  }
  for(float t = 8; t > -8; t-=0.25){
    setPosition(0,0,110,radians(0),radians(0),radians(t));
    Serial.println(t);
  }
  }*/
  
  /*while(true){
    for(float t = 0; t < 2*pi; t+=0.07){
      setPosition(10*cos(t),10*sin(t),110+5*sin(t),radians(0),radians(0),radians(0)); 
    }
  }*/
  
}

void loop() {
  if(Serial.available()){
    command = Serial.read();
  }
  
  if(command == 'p'){
    pause = true;
    while(pause){
      setPosition(0,0,110,radians(0),radians(0),radians(0));
      delay(500);
      if(Serial.read() == 'p'){
        pause = false;
      }
    }
  }
  if(command == 'd'){ 
    setX = 0.26;//0.17
    setY = 0.135;
  }
  if(command == 's'){ 
    setX = 0.09;//0.17
    setY = 0.135;
  }
  Serial.print(100*x);
  Serial.print(" ");
  
  
  xOld = x;
  yOld = y;

  xVelOld = xVel;
  yVelOld = yVel;
  
  getSense();
  delay(8);//8

  newTime = millis();

  Serial.print(newTime - oldTime);
  Serial.print(" ");
  
  deltaTime = (newTime - oldTime)/1000;
  oldTime = newTime;

  // velocità pallina in m/s
  
  xVel = (x - xOld)/(deltaTime);
  yVel = (y - yOld)/(deltaTime);
  
  xAcc = (xVel - xVelOld)/(deltaTime);
  yAcc = (yVel - yVelOld)/(deltaTime);


  // filto spazio 
  // aggiungere eq cinematica calcolo max vel
  float maxVel = 0.22;//0.5;//0.25
  if(abs(xVel) > maxVel){
    if(xVel > 0){
      x = xOld + maxVel*(deltaTime);
    }
    else{
      x = xOld - maxVel*(deltaTime);
    }
  }
  if(abs(yVel) > maxVel){
    if(yVel > 0){
      y = yOld + maxVel*(deltaTime);
    }
    else{
      y = yOld - maxVel*(deltaTime);
    }
  }

  // filto velocità (derivata)
  // aggiungere eq cinematica calcolo max acc
  
  float maxAcc = 0.5;//0.45;   // max per 5 gradi
  if(abs(xAcc) > maxAcc){
    if(xAcc > 0){
      xVel = xVelOld + maxAcc*(deltaTime);
    }
    else{
      xVel = xVelOld - maxAcc*(deltaTime);
    }
  }
  if(abs(yAcc) > maxAcc){
    if(yAcc > 0){
      yVel = yVelOld + maxAcc*(deltaTime);
    }
    else{
      yVel = yVelOld - maxAcc*(deltaTime);
    }
  }
  
  Serial.println(100*xVel);
  

  
  //Serial.print(10*xVel);
  //Serial.print(" ");
  //Serial.println(10*x);
  //Serial.print(" ");
  //Serial.print(xVel*100);
  //Serial.print(y*100);
  
  /*Serial.print(xVel*100);
  Serial.print(" ");
  Serial.println(yVel*100);*/
  
  float errXold = errX;
  float errYold = errY;
  errX = setX - x;
  errY = setY - y;

  float antiwindup = 0.02;
  sumErrX += errX * deltaTime;
  sumErrY += errY * deltaTime;
  if(abs(sumErrX) > antiwindup){
    if(sumErrX > 0){
      sumErrX = antiwindup;
    }
    else{
      sumErrX = -antiwindup;
    }
  }
  if(abs(sumErrY) > antiwindup){
    if(sumErrY > 0){
      sumErrY = antiwindup;
    }
    else{
      sumErrY = -antiwindup;
    }
  }

  //float tau = 30;

  float proporzionaleX = Kp * errX;
  float integraleX = Ki * sumErrX;
  float derivativoX = -Kd * xVel; // - per il derivative o measurement
  //float derivativoX = -(2*Kd *(x-xOld)+(2*tau-deltaTime)*oldDerivataX)/(2*tau+deltaTime); // errX-errXold derivata normale x-xOld derivative on measurement

  /*float maxDerivativo = 1;
  if(abs(derivativoX) > maxDerivativo){
    if(derivativoX > 0){
      derivativoX = xoldDerivataX + maxAcc*(deltaTime);
    }
    else{
      xVel = xVelOld - maxAcc*(deltaTime);
    }
  }*/
  
  oldDerivataX = derivativoX;

  float proporzionaleY = Kp * errY;
  float integraleY = Ki * sumErrY;
  float derivativoY = -Kd * yVel;
  //float derivativoY = -(2*Kd *(y-yOld)+(2*tau-deltaTime)*oldDerivataY)/(2*tau+deltaTime);
  oldDerivataY = derivativoY;

  
  
  pidX = proporzionaleX + integraleX + derivativoX;   // errX-errXold derivata normale x-xOld derivative on measurement
  pidY = proporzionaleY + integraleY + derivativoY;


  float maxangle = 5;
  float tiltX;
  float tiltY;
  if(abs(pidY) > maxangle){
    if(-pidY > 0){
      tiltX = maxangle;
    }
    if(-pidY < 0){
      tiltX = -maxangle;
    }
  }
  else{
    tiltX = -pidY;
  }

  if(abs(pidX) > maxangle){
    if(pidX > 0){
      tiltY = maxangle;
    }
    if(pidX < 0){
      tiltY = -maxangle;
    }
  }
  else{
    tiltY = pidX;
  }
  
  setPosition(0,0,110,radians(tiltX),radians(tiltY),radians(0));

}

// costruisci matrice rotazione globale
void setMRot(float rol,float pit,float yaw){
  Rg[0][0] = cos(yaw)*cos(pit);
  Rg[0][1] = -sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol);
  Rg[0][2] = sin(yaw)*sin(rol)+cos(yaw)*sin(pit)*cos(rol);

  Rg[1][0] = sin(yaw)*cos(pit);
  Rg[1][1] = cos(yaw)*cos(rol)+sin(yaw)*sin(pit)*sin(rol);
  Rg[1][2] = -cos(yaw)*sin(rol)+sin(yaw)*sin(pit)*cos(rol);

  Rg[2][0] = -sin(pit);
  Rg[2][1] = cos(pit)*sin(rol);
  Rg[2][2] = cos(pit)*cos(rol);
}

// calcola la posizione del giunto biella/manovella
void setJLoc(float alfa, float beta, int n){      // alfa: angolo rispetto al piano xy. beta: angolo rispetto all'asse x.
  if(n % 2){    //n dispari
    giun[n][0] = lungman*cos(alfa)*cos(pi+beta)+base[n][0];     // per il sistema di angoli impiegato non è necessario indicare pi-alfa
    giun[n][1] = lungman*cos(alfa)*sin(pi+beta)+base[n][1];
    giun[n][2] = lungman*sin(alfa)+base[n][2];
  }
  else{
    giun[n][0] = lungman*cos(alfa)*cos(beta)+base[n][0];
    giun[n][1] = lungman*cos(alfa)*sin(beta)+base[n][1];
    giun[n][2] = lungman*sin(alfa)+base[n][2];
  }
}

// calcola moltiplicazione matrici
void getMult(float a[3][3], float b[3][1], float c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = 0;
    for (int j = 0; j < 3; j++){
      c[i][0] += a[i][j]*b[j][0];
    }
  }
}

// calcola somma matrici
void getSum(float a[3][1], float b[3][1], float c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = a[i][0]+b[i][0];
  }
}

// estrae una riga dalla matrice e la converte in colonna per semplificare moltiplicazione
void getRow(float a[3][1], float b[6][3], int n){
  for(int i = 0; i < 3; i++){
    a[i][0] = b[n][i];
  }
}

// costruisce matrice vettori origine base -> giunto piattaforma
void getBaseVec(){
  float tempMult[3][1];
  float tempSum[3][1];
  float piatVec[3][1];
  for(int i = 0; i < 6; i++){
    getRow(piatVec, piat, i);
    getMult(Rg,piatVec,tempMult);
    getSum(T,tempMult,tempSum);
    baseVec[i][0] = tempSum[0][0];
    baseVec[i][1] = tempSum[1][0];
    baseVec[i][2] = tempSum[2][0]-h0;
    /*Serial.println("altezza");
    Serial.println(tempSum[2][0]);*/
  }
}

// costruisce matrice vettori asse servo -> giunto piattaforma
void getArmVec(){
  for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      armVec[i][j] = baseVec[i][j] - base[i][j];
    }
  }
}

// calcola il valore del quadrato del vettore fornito (come differenza tra inizio e fine)
float quadVec(float inizio[3][1], float fine[3][1]){
  float v2;
  return v2 = pow((inizio[0][0]-fine[0][0]),2)+
              pow((inizio[1][0]-fine[1][0]),2)+
              pow((inizio[2][0]-fine[2][0]),2);
}

// calcola ampiezza impulso per ottenere l'angolo richiesto
float getAmpImp(float angolo, int n){     
  float pw;
  if(n % 2){
    return pw = oriz[n]-(alfa[n]-alfa0[n])*usrad;
  }
  else{
    return pw = oriz[n]+(alfa[n]-alfa0[n])*usrad;
  }
}

// svolge i dovuti calcoli e imposta l'angolo dei servomotori
void setPosition(float x, float y, float z, float rol, float pit, float yaw){
  // imposta posizione desiderata
  T[0][0] = x;
  T[0][1] = y;
  T[0][2] = z;
  R[0][0] = rol;
  R[0][1] = pit;
  R[0][2] = yaw;
  // inizializza matrice posizione giunto biella/manovella
  for(int n = 0; n < 6; n++){
    setJLoc(alfa[n],beta[n],n);
  }
  // inizializza matrice rotazione globale
  setMRot(R[0][0],R[0][1],R[0][2]);
  // inizializza matrice vettori base -> giunto piattaforma
  getBaseVec();
  // inizializza matrice vettori asse -> giunto piattaforma
  getArmVec();
  // calcola distanza^2 tra asse e giunto piattaforma
  for(int i = 0; i < 6; i++){
    float inizio[3][1];
    float fine[3][1];
    getRow(inizio,baseVec,i);
    getRow(fine,base,i);
    d2[i] = quadVec(inizio,fine);
  }
  // calcolo angoli servomotori
  //Serial.println("Angoli servo:");
  for(int i = 0; i < 6; i++){
    L[i] = d2[i]-(b2-m2);
    M[i] = 2*lungman*(piat[i][2]-base[i][2]);
    N[i] = 2*lungman*(cos(beta[i]*(piat[i][0]-base[i][0])+sin(beta[i])*(piat[i][1]-base[i][1])));
    alfa[i]=asin(L[i]/sqrt(pow(M[i],2)+pow(N[i],2)))-atan(N[i]/M[i]);
    //Serial.println(alfa[i]);
  }
  // controllo microsecondi se ho raggiunto il limite
  int emergenza = 0;
  for(int i = 0; i < 6; i++){
    //Serial.println(constrain(getAmpImp(alfa[i],i),inf,sup));
    if(constrain(getAmpImp(alfa[i],i),inf,sup) == sup || constrain(getAmpImp(alfa[i],i),inf,sup) == inf || isnan(constrain(getAmpImp(alfa[i],i),inf,sup))){     // isnan controlla se la computazione è risultata in un valore impossibile.
      emergenza++;
    }
  }
  // assegnazione angolo servomotori
  if(emergenza == 0){
    //Serial.println("Microsecondi:");
    for(int i = 0; i < 6; i++){
        servo[i].writeMicroseconds(constrain(getAmpImp(alfa[i],i),inf,sup));
        //Serial.println(constrain(getAmpImp(alfa[i],i),inf,sup));
    }
  }
  else{
    Serial.println("Limite raggiunto, arresto motori.");
    fermoEmergenza();
  }
  
}

// in caso di emergenza non muoverti, entra in loop infinito lampeggiante allarme
void fermoEmergenza(){
  while(true){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void getSense(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  delay(1);               // RIVEDERE DELAY
  x = (analogRead(sensePin)-100)*(0.34/820);
  //Serial.print("x orig: "); //segnale ingresso non filtrato
  //Serial.print(" ");
  //Serial.println(x*100);

  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  delay(1);               // RIVEDERE DELAY
  y = (analogRead(sensePin)-100)*(0.27/820);
}
