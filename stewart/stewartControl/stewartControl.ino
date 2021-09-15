#include <Servo.h>
#include<stdio.h>
#include<stdlib.h>
Servo servo[6];

const float pi = 3.141592653;
// range movimento inferiore e superiore
const int inf = 1800;
const int sup = 900;
// microsecondi per avere i bracci dei sevomotori orizzontali
int oriz[6] = {1500,1550,1500,1560,1560,1500};
// relazione tra microsecondi e radianti [us/rad]
int usrad = (400/45)*(360/(2*pi));                        //CALCOLARE ANGOLO SPERIMENTALE
// posizione richiesta {x,y,z,roll,pitch,yaw}
float pos[6] = {0,0,0,0,0,0};


float Rg[3][3];   // matrice rotazione globale
float T[3] = {0,0,114.5};       // vettore posizione {x,y,z} inizializzato con altezza riposo

//DESCRIZIONE TOPOLOGICA SISTEMA
// angolo manovella rispetto al piano xy
float alfa[6] = {0,0,0,0,0,0};
// angolo manovella rispetto all'asse x
float beta[6] = {-pi/2,pi/6,pi/6,5*pi/6,5*pi/6,pi/2};
// lunghezza manovella
float lungman = 16;

//angolo asse rotazione rispetto origine base
float gamma = (13.38*2*pi)/360;
// distanza tra centro riferimento e asse rotazione servo [mm]
float rd = 87.63;    // rotation distance (base)
// coordinate punti base
float base[6][3] = {{-rd*cos(gamma),-rd*sin(gamma),0},
                    {rd*cos(pi/3+gamma),-rd*sin(pi/3+gamma),0},
                    {rd*cos(pi/3-gamma),-rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3-gamma),rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3+gamma),rd*sin(pi/3+gamma),0},
                    {-rd*cos(gamma),rd*sin(gamma),0}};

// coordinate giunto biella/manovella al variare di alfa e beta (inizializzata durante il setup)
float giun[6][3] = {{0,0,0},
                    {0,0,0},
                    {0,0,0},
                    {0,0,0},
                    {0,0,0},
                    {0,0,0}};

//angolo asse pivot rispetto origine piattaforma
float tau = (4.14*2*pi)/360;
float pd = 80.52;    // pivot distance (piattaforma)
//coordinate punti piattaforma (inizializzato con altezza homing)
float piat[6][3] = {{pd*cos(2*pi/3+tau),-pd*sin(2*pi/3+tau),T[2]},
                    {pd*cos(2*pi/3-tau),-pd*sin(2*pi/3-tau),T[2]},
                    {pd*cos(tau),-pd*sin(tau),T[2]},
                    {pd*cos(tau),pd*sin(tau),T[2]},                        
                    {pd*cos(2*pi/3-tau),pd*sin(2*pi/3-tau),T[2]},
                    {pd*cos(2*pi/3+tau),pd*sin(2*pi/3+tau),T[2]}};

// vettore origine base -> giunto piattaforma
// CONTINUARE

void setup() {
  Serial.begin(9600);
  
  servo[0].attach(3, inf, sup);
  servo[1].attach(5, inf, sup);
  servo[2].attach(6, inf, sup);
  servo[3].attach(9, inf, sup);
  servo[4].attach(10, inf, sup);
  servo[5].attach(11, inf, sup);

  //inizializza matrice posizione giunto biella/manovella
  for(int n = 0; n < 6; n++){
    setJLoc(alfa[n],beta[n],n);
  }

  
  
  /*for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      Serial.print(giun[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }*/

  /*int a[3][3] = {{2,3,1}, {4,2,1}, {3,1,5} };
  int b[3][1] = {1,2,3};
  int c[3][1];

  mult(a,b,c);
   for(int i=0; i < 3; i++){
    for(int j=0; j < 1; j++){
      Serial.print(c[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }*/
}

void loop() {
  
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
void mult(int a[3][3], int b[3][1], int c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = 0;
    for (int j = 0; j < 3; j++){
      c[i][0] += a[i][j]*b[j][0];
    }
  }
}

// calcola somma matrici
void sum(int a[3][1], int b[3][1], int c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = a[i][0]+b[i][0];
  }
}
