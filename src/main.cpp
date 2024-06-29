#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <PID_v1.h>
#include "math.h"

#define Serial_Debug Serial
#define GPS_Serial Serial1
#define SerialBT Serial2

//////////////////////////////// OBJECTS ////////////////////////////
// The TinyGPS++ object
TinyGPSPlus gps;

//////////////DECLARAÇÕES DE FUNÇÕES////////////////////////
double calc_dist(double lat, double longt, double lat_goal, double long_goal);
double *le_gps(bool escreve_na_port);
void faz_vetores(double x_zero, double y_zero, double x_1, double y_1);
double dot_prod(double x_vetor_0, double y_vetor_0, double x_vetor_1, double y_vetor_1);
double modulo(double x, double y);
double to_rad(double angulo_grau);
void anda_para_frente(int tempo);
void anda_para_tras(int tempo);
void gira_para_direita(int tempo);
void gira_para_esquerda(int tempo);

//////////////Variáveis Declaradas////////////////////////

// In2 1 e In1 0 lado direito para frente
uint32_t in1 = 9;
uint32_t in2 = 8;

// In3 0 e In4 1 lado esquerdo para frente
uint32_t in3 = 6;
uint32_t in4 = 7;

double ponto_goal[] = {-20.310872, -40.319732}; // Dentro da quadra
double rTerra = 6371;                           // Raio da terra em km
bool orientado = false;

void setup()
{
  // Configurações iniciais...
  Serial_Debug.begin(9600); // Inicia Serial com computador
  GPS_Serial.begin(9600);   // Inicia Serial com GPS
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}
unsigned long agora, zero = 0;

double ponto0[] = {0, 0};
double ponto1[] = {0, 0};
bool hasInitialPoint = false;
void loop()
{

  double *posicao_inicial = le_gps(true);
  ponto0[0] = posicao_inicial[0];
  ponto0[1] = posicao_inicial[1];

  delay(1000); // Espera 1 segundo antes de tentar novamente
  anda_para_frente(5000);

  // Pegue o segundo ponto
  double *posicao_final = le_gps(true);
  ponto1[0] = posicao_final[0];
  ponto1[1] = posicao_final[1];

  // Exibir os pontos para verificação
  Serial_Debug.print("Ponto Inicial: Latitude= ");
  Serial_Debug.print(ponto0[0], 6);
  Serial_Debug.print(" Longitude= ");
  Serial_Debug.println(ponto0[1], 6);

  Serial_Debug.print("Ponto Final: Latitude= ");
  Serial_Debug.print(ponto1[0], 6);
  Serial_Debug.print(" Longitude= ");
  Serial_Debug.println(ponto1[1], 6);
}

//////////////FUNÇÕES E DESCRIÇÕES////////////////////////
/*Calcula a distância entre o ponto atual e a o ponto desejado. Resultado em metros*/
double calc_dist(double lat, double longt, double lat_goal, double long_goal)
{
  double Distancia = rTerra * acos(
                                  sin(to_rad(lat)) * sin(to_rad(lat_goal)) +
                                  cos(to_rad(lat)) * cos(to_rad(lat_goal)) *
                                      cos(to_rad(long_goal) - to_rad(to_rad(longt))));

  return Distancia * 1000; // Valor sai em m.
}
/*Função de leitura do módulo GPS. Quando o valor é positivo o valor é impresso na comunicação Serial no pc.*/
double *le_gps(bool escreve_na_port)
{
  static double ponto_local[2]; // Static to keep the values outside the scope of the function

  if (GPS_Serial.available() > 0)
  {
    gps.encode(GPS_Serial.read());
    if (gps.location.isUpdated())
    {
      ponto_local[0] = gps.location.lat();
      ponto_local[1] = gps.location.lng();

      if (escreve_na_port)
      {
        Serial_Debug.print("Latitude= ");
        Serial_Debug.print(gps.location.lat(), 6);
        Serial_Debug.print(" Longitude= ");
        Serial_Debug.println(gps.location.lng(), 6);
      }
    }
  }
  else
  {
    Serial_Debug.println("Falha na leitura do GPS!!");
  }
  return ponto_local;
}

void faz_vetores(double x_zero, double y_zero, double x_1, double y_1)
{
  double x_vec = x_1 - x_zero;
  double y_vec = y_1 - y_zero;
}
/*Retorna angulo entre dois vetores em radianos*/
double dot_prod(double x_vetor_0, double y_vetor_0, double x_vetor_1, double y_vetor_1)
{
  double angulo = asin((x_vetor_0 * x_vetor_1 + y_vetor_0 * y_vetor_1) / (modulo(x_vetor_0, y_vetor_0) * modulo(x_vetor_1, y_vetor_1)));
  return angulo;
}
/*Faz módulo de vetor*/
double modulo(double x, double y)
{
  double modulo = sqrt(pow(x, 2) + pow(y, 2));
  return modulo;
}

/*Conversor de grau para Radiano*/
double to_rad(double angulo_grau)
{
  return (angulo_grau * PI / 180);
}

void anda_para_frente(int tempo)
{
  analogWrite(in1, 0);
  analogWrite(in4, 0);
  analogWrite(in2, 255);
  analogWrite(in3, 255);
  delay(tempo);
  analogWrite(in1, 0);
  analogWrite(in3, 0);
  analogWrite(in2, 0);
  analogWrite(in4, 0);
}

void anda_para_tras(int tempo)
{
  analogWrite(in1, 255);
  analogWrite(in4, 255);
  analogWrite(in2, 0);
  analogWrite(in3, 0);
  delay(tempo);
  analogWrite(in1, 0);
  analogWrite(in3, 0);
  analogWrite(in2, 0);
  analogWrite(in4, 0);
}

void gira_para_esquerda(int tempo)
{
  analogWrite(in1, 0);
  analogWrite(in2, 255);

  analogWrite(in4, 0);
  analogWrite(in3, 120);
  delay(tempo);
  analogWrite(in1, 0);
  analogWrite(in3, 0);
  analogWrite(in2, 0);
  analogWrite(in4, 0);
}

void gira_para_direita(int tempo)
{
  analogWrite(in1, 0);
  analogWrite(in2, 120);

  analogWrite(in4, 0);
  analogWrite(in3, 255);
  delay(tempo);
  analogWrite(in1, 0);
  analogWrite(in3, 0);
  analogWrite(in2, 0);
  analogWrite(in4, 0);
}

////////////////////ROTINA DE ORIENTAÇÃO PARA NORTE///////////////////
