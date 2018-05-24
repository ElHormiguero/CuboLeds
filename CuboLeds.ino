/* Cubo de leds con gravedad utilizando una matriz de 32x32 (16 Matrices 8x8) controlado desde el driver Max7219 y una IMU MPU6050.
   Autor: Javier Vargas. El Hormiguero
   https://creativecommons.org/licenses/by/4.0/
*/

//PINES
//Pin DIN --> MOSI
//Pin CLK --> SCK
#define pinCS 10
#define pinAdd 7
#define pinRes 8


//CONFIGURACION
#define Nmax 15 //Número maximo de particulas
#define Amor 0.6f //Amortiguamiento con paredes
#define Amor2 0.3f //Amortiguamiento entre bolas
#define Muestreo 30 //ms
#define MuestreoS 0.03f //s

//LIBRERIAS
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>
#include <MPU6050_CompFilter.h>
MPU6050_CompFilter mpu(0x68);

const int numberOfHorizontalDisplays = 4;
const int numberOfVerticalDisplays = 4;
Max72xxPanel matrix = Max72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);

//Vectores de posición y velocidad
float xPos[Nmax]; //Posicion actual
float yPos[Nmax];
float xPos0[Nmax]; //Posicion anterior
float yPos0[Nmax];
double xVel[Nmax]; //Velocidad
double yVel[Nmax];

//Acelerometro
float AcX;
float AcY;
int OffsetX = 0;
int OffsetY = 20;

byte Np = 0; //Numero de particulas actual
unsigned long millisactual;

void setup() {

  //Pines botones
  pinMode(pinAdd, INPUT_PULLUP);
  pinMode(pinRes, INPUT_PULLUP);

  //Matriz
  matrix.setIntensity(0); //Brillo bajo
  for (int i = 0; i < 16; i++) {
    matrix.setRotation(i, 1); //Rotacion adecuada para tener las posiciones x e y correctas
  }
  delay(100);
  matrix.fillScreen(LOW); //Apagamos todos los pixeles
  matrix.write();
  matrix.setIntensity(10); //Brillo

  //Iniciamos IMU
  mpu.Iniciar(MuestreoS);
  mpu.setKcompFilter(0.99);

  //Añadimos una particula
  AddParticula();
}
//////////////////////////
//////////LOOP////////////
//////////////////////////

void loop() {
  if (millis() / Muestreo != millisactual) {
    millisactual = millis() / Muestreo;
    BotonAdd();
    BotonRes();
    ObtenerAceleracion();
    ObtenerPosicion();
    Imprimir();
  }
}

//////////////////////////
//////////////////////////
//////////////////////////

void ObtenerPosicion() {

  for (int i = 0; i < Np; i++) {
    //Vector velocidad
    xVel[i] += AcX;
    yVel[i] += AcY;

    //Vector posicion
    xPos[i] += xVel[i];
    yPos[i] += yVel[i];

    //Rebote con bordes
    if (xPos[i] > 31 || xPos[i] < 0) {
      xVel[i] = -xVel[i] * Amor;
    }
    if (yPos[i] > 31 || yPos[i] < 0) {
      yVel[i] = -yVel[i] * Amor;
    }

    //Bordes
    xPos[i] = constrain(xPos[i], 0, 31.99);
    yPos[i] = constrain(yPos[i], 0, 31.99);

    //Rebote entre bolas
    for (int j = 0; j < Np; j++) {
      if (i != j) {
        //Coincidencia
        if ((int)yPos[i] == (int)yPos[j] && (int)xPos[i] == (int)xPos[j]) {
          //Impacto diagonal
          if (xPos0[i] != xPos[i] && yPos0[i] != yPos[i]) {
            xVel[i] = -xVel[i] * Amor2;
            yVel[i] = -yVel[i] * Amor2;
            yPos[i] = yPos0[i];
            xPos[i] = xPos0[i];
          }
          //Impacto horizontal
          else if (xPos0[i] != xPos[i]) {
            xVel[i] = -xVel[i] * Amor2;
            xPos[i] = xPos0[i];
          }
          //Impacto vertical
          else if (yPos0[i] != yPos[i]) {
            yVel[i] = -yVel[i] * Amor2;
            yPos[i] = yPos0[i];
          }
          constrain(xPos[i], 0, 31.99);
          constrain(yPos[i], 0, 31.99);
        }
      }
    }


  }

}

void ObtenerAceleracion() {
  mpu.Lectura(1, 1);
  AcX = sin((mpu.angY() + OffsetY) * 0.01745);
  AcY = sin((mpu.angX() + OffsetX) * 0.01745);
  //      Serial.print(mpu.angX());
  //      Serial.print("/");
  //      Serial.println(mpu.angY());
}

void Imprimir() {

  //Borramos posicion anterior
  for (int i = 0; i < Np; i++) {
    if ((int)xPos0[i] != (int)xPos[i] || (int)yPos0[i] != (int)yPos[i]) {
      matrix.drawPixel((int)xPos0[i], (int)yPos0[i], LOW);
      matrix.write(); // Send bitmap to display
    }
  }

  //Imprimir posicion actual
  for (int i = 0; i < Np; i++) {
    matrix.drawPixel((int)xPos[i], (int)yPos[i], HIGH);
    matrix.write(); // Send bitmap to display
  }

  //Se guarda posicion actual como anterior posición anterior
  for (int i = 0; i < Np; i++) {
    xPos0[i] = xPos[i];
    yPos0[i] = yPos[i];
  }
}

void AddParticula() {
  if (Np < Nmax) {
    Np += 1; //Añadimos una particula
    xPos[Np - 1] = 15;
    yPos[Np - 1] = 15;
  }
  if (Np == 1) {
    CuentaAtras();
  }
}

void ResParticula() {
  if (Np > 0) {
    Np -= 1; //Añadimos una particula
    matrix.drawPixel((int)xPos[Np], (int)yPos[Np], LOW);
    matrix.write();
  }
}

boolean BotonAdd() {
  static boolean s = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinAdd) == 0) {
      s = 1;
      mils = millis();
      AddParticula();
    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinAdd) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }

}

boolean BotonRes() {
  static boolean s = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinRes) == 0) {
      s = 1;
      mils = millis();
      ResParticula();
    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinRes) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }

}

void CuentaAtras() {
  boolean x0 = 0;
  boolean y0 = 3;

  //Tres
  matrix.drawLine(7 + x0, 17 + y0, 7 + x0, 10 + y0, HIGH);
  matrix.drawLine(16 + x0, 15 + y0, 16 + x0, 10 + y0, HIGH);
  matrix.drawLine(25 + x0, 17 + y0, 25 + x0, 10 + y0, HIGH);
  matrix.drawLine(7 + x0, 10 + y0, 25 + x0, 10 + y0, HIGH);
  matrix.write();
  delay(1000);

  //Dos
  matrix.fillScreen(LOW); //Apagamos todos los pixeles
  matrix.write();
  matrix.drawLine(7 + x0, 17 + y0, 7 + x0, 10 + y0, HIGH);
  matrix.drawLine(7 + x0, 10 + y0, 16 + x0, 10 + y0, HIGH);
  matrix.drawLine(16 + x0, 10 + y0, 16 + x0, 17 + y0, HIGH);
  matrix.drawLine(16 + x0, 17 + y0, 25 + x0, 17 + y0, HIGH);
  matrix.drawLine(25 + x0, 17 + y0, 25 + x0, 10 + y0, HIGH);
  matrix.write();
  delay(1000);

  //Uno
  matrix.fillScreen(LOW); //Apagamos todos los pixeles
  matrix.write();
  matrix.drawLine(10 + x0, 17 + y0, 7 + x0, 14 + y0, HIGH);
  matrix.drawLine(7 + x0, 13 + y0, 25 + x0, 13 + y0, HIGH);
  matrix.write();
  delay(1000);

  matrix.fillScreen(LOW); //Apagamos todos los pixeles
  matrix.write();
}

int sign(float s) {
  if (s >= 0) return 1;
  else return -1;
}

