// Sketch para posicionar um protótipo de agulha de braquiterapia em cinco pontos ao longo dos eixos x e y, utilizando um cálculo aproximado de passos e feedback de sensores de distância
// Autor: André Fiúza
// Universidade Federal de Minas Gerais - UFMG. 
// Data da versão: 15-06-2023

#include <HCSR04.h>
#include "Adafruit_VL53L0X.h"

#define deslocamentoPorVoltaX 0.8
#define deslocamentoPorVoltaY 0.3
#define passosPorVolta 200

#define erroMaximoPermitido 0.25

#define janelaDoFiltro 20

#define DIR_H 6
#define STEP_H 7

#define DIR_V 2
#define STEP_V 3

#define p_trigger 4  // Pino digital de trigger do HC-SR04
#define p_echo 5     // Pino digital de echo do HC-SR04

#define numeroDePontos 5

UltraSonicDistanceSensor distanceSensor(p_trigger, p_echo);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float matrixDePontos[][numeroDePontos] = {{6.5, 2.5}, {13, 6}, {3.5, 6.5}, {10, 3}, {13, 5}}; 

void setup() {
  Serial.begin(19200);

  while (! Serial) {
    delay(1);
  }

  Serial.println(F("Teste de distância por passo usando matrix de pontos iniciado..."));

  pinMode(DIR_H, OUTPUT);
  pinMode(STEP_H, OUTPUT);
  pinMode(DIR_V, OUTPUT);
  pinMode(STEP_V, OUTPUT);
}

void loop() {

  for (int j = 0; j < numeroDePontos; j++) 
  {
    int deslocamento = 1; 
    float posicaoDesejadaX = matrixDePontos[0][j];
    float posicaoDesejadaY = matrixDePontos[1][j];
    unsigned long inicioDaExecucao = millis();
    
    float posicaoAtualX = measureDistance('H');
    float posicaoAtualY = measureDistance('V');
    int numDePassosX = calculateNumberOfSteps(posicaoDesejadaX, 'H');  // Calcula o número de passos necessários para chegar à posição X desejada
    int numDePassosY = calculateNumberOfSteps(posicaoDesejadaY, 'V');  // Calcula o número de passos necessários para chegar à posição Y desejada

    Serial.print("Posicao horizontal desejada : ");
    Serial.println(posicaoDesejadaX);
    Serial.print("Posicao vertical desejada : ");
    Serial.println(posicaoDesejadaY);

    Serial.print("Posicao horizontal atual: "); 
    Serial.println(posicaoAtualX);
    Serial.print("Posicao vertical atual: "); 
    Serial.println(posicaoAtualY);

    deslocamento = runThisManySteps(numDePassosX, 'H');
    while(deslocamento)
    { }

    deslocamento = 1; 
    deslocamento = runThisManySteps(numDePassosY, 'V');
    while(deslocamento)
    { }

    float erroX = checkPositioningPrecision(posicaoDesejadaX, 'H');
    float erroY = checkPositioningPrecision(posicaoDesejadaY, 'V');
    
    while ((modulo(erroX) > erroMaximoPermitido) && (modulo(erroY) > erroMaximoPermitido))
    {
      if(modulo(erroX) > erroMaximoPermitido)
      {
        runThisManySteps(calculateNumberOfSteps(posicaoDesejadaX, 'H'), 'H');
        erroX = checkPositioningPrecision(posicaoDesejadaX, 'H');
      }
      if(modulo(erroY) > erroMaximoPermitido)
      {
        runThisManySteps(calculateNumberOfSteps(posicaoDesejadaY, 'V'), 'V');
        erroY = checkPositioningPrecision(posicaoDesejadaY, 'V');
      }
    }
    
    unsigned long fimDaExecucao = millis();

    Serial.print("Tempo de execucao decorrido: ");
    Serial.print((fimDaExecucao - inicioDaExecucao) / 1000);
    Serial.println(" segundos");

    Serial.print("Erro final calculado : ");
    Serial.print(erroX * 10);
    Serial.println(" milímetros para a posicao longitudinal desejada");

    Serial.print("Erro final calculado : ");
    Serial.print(erroY * 10);
    Serial.println(" milímetros para a posicao latitudinal desejada");

    delay(1500);
  }
}

int calculateNumberOfSteps(float posicaoDesejada, char sentido) {

  float deltaDistancia = posicaoDesejada - measureDistance(sentido); 
  float deslocamentoPorPasso; 

  if ((sentido == 'H') || (sentido == 'h')) 
    deslocamentoPorPasso = deslocamentoPorVoltaX / passosPorVolta;
  else if ((sentido == 'V') || (sentido == 'v')) 
    deslocamentoPorPasso = deslocamentoPorVoltaY / passosPorVolta;

  return (deltaDistancia / deslocamentoPorPasso);
}

float checkPositioningPrecision(float posicaoDesejada, char sentido) {
  return posicaoDesejada - measureDistance(sentido);
}

int runThisManySteps(int numDePassos, char sentido) {
  // Realiza o número de passos estipulado pelo usuário

  if ((sentido == 'H') || (sentido == 'h')) {
    if (numDePassos >= 0) {
      digitalWrite(DIR_H, LOW);
    } else {
      digitalWrite(DIR_H, HIGH);
      numDePassos *= -1;
    }

    for (int steps = 0; steps < numDePassos; steps++) {
      digitalWrite(STEP_H, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_H, LOW);
      delayMicroseconds(500);
    }
  } 
  else if ((sentido == 'V') || (sentido == 'v')) {
    if (numDePassos >= 0) {
      digitalWrite(DIR_V, LOW);
    } else {
      digitalWrite(DIR_V, HIGH);
      numDePassos *= -1;
    }

    for (int steps = 0; steps < numDePassos; steps++) {
      digitalWrite(STEP_V, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_V, LOW);
      delayMicroseconds(500);
    }
  }
  return 0;
}

float modulo(float number) {
  return number > 0 ? number : -1 * number;
}

float measureDistanceWithUlstrassonicSensor()
{
  float leiturasDoSensor[janelaDoFiltro]; 
  float posicaoLida = 0;
  int pontosValidos = 0;

  for (int i = 0; i < janelaDoFiltro; i++) {
    leiturasDoSensor[i] = distanceSensor.measureDistanceCm();
    delayMicroseconds(10); 
    if(leiturasDoSensor[i] > 0)
    {
      posicaoLida += leiturasDoSensor[i];
      pontosValidos += 1; 
    }
  }

  posicaoLida = posicaoLida / pontosValidos;
  
  return posicaoLida > 0 ? posicaoLida : leiturasDoSensor[0]; 
}

float measureDistanceWithOpticSensor()
{
  float distanciaLida; 
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  float medicao = 0; 

  for(int i = 0; i < janelaDoFiltro; i++)
  {
    if (measure.RangeStatus != 4) 
    {  
      distanciaLida += measure.RangeMilliMeter / 10.0 - 3.0;
    } 
    delay(50);
  } 

  distanciaLida /= janelaDoFiltro; 

  return distanciaLida; 
}

float measureDistance(char sentido)
{
  return sentido = 'H' ? measureDistanceWithUlstrassonicSensor() : measureDistanceWithOpticSensor(); 
}

