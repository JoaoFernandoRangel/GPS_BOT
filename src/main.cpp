#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "math.h"

#define Serial_Debug Serial
#define GPS_Serial Serial1
#define SerialBT Serial2

//////////////////////////////// OBJECTS ////////////////////////////
// The TinyGPS++ object
TinyGPSPlus gps;

//////////////DECLARAÇÕES DE FUNÇÕES////////////////////////
void pega_pontos();
double calc_dist(double lat, double longt, double lat_goal, double long_goal);
void le_gps(bool escreve_na_port, double ponto_x, double ponto_y);
void faz_vetores(double x_zero, double y_zero, double x_1, double y_1, double x_vec, double y_vec);
double dot_prod(double x_vetor_0, double y_vetor_0, double x_vetor_1, double y_vetor_1, double angulo);
double modulo(double x, double y);
double to_rad(double angulo_grau);
double to_ang(double angulo_rad);
void anda_para_frente(int tempo);
void anda_para_tras(int tempo);
void gira_para_esquerda(int tempo);
void gira_para_direita(int tempo);
void ajusta_para(bool direita, float tempo);
void stop();
void ajusta_angulo(double angulo0, double angulo1);

//////////////Variáveis Declaradas////////////////////////

// Variáveis do tipo uint32_t
uint32_t in1 = 9;
uint32_t in2 = 8;
uint32_t in3 = 6;
uint32_t in4 = 7;

// Variáveis do tipo double
double ponto_goal[] = {-20.310872, -40.319732}; // Dentro da quadra
double rTerra = 6371;                           // Raio da terra em km
double angulo0 = 0;
double angulo1 = 0;
double ponto0[] = {0, 0};
double ponto1[] = {0, 0};
double vec0[] = {0, 0};
double vec_goal[] = {0, 0};

// Variáveis do tipo bool
bool orientado = false;
bool primeiro = true;
bool hasInitialPoint = false;
bool cond = true; // Variável vai ser usada para iniciar a operação de busca uma vez que o ponto objetivo for enviado pelo Bluetooth

// Variáveis do tipo unsigned long
unsigned long agora = 0;
unsigned long zero = 0;

void setup()
{
  // Configurações iniciais...
  Serial_Debug.begin(9600); // Inicia Serial com computador
  GPS_Serial.begin(9600);   // Inicia Serial com GPS
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial_Debug.println("Finish Setup");
}

void loop()
{
  if (cond)
  {
    agora = millis();
    if (agora - zero >= 1000) // Executa a rotina a cada segundo.
    {
      pega_pontos();
      if (calc_dist(ponto1[0], ponto1[1], ponto_goal[0], ponto_goal[1]) <= 2) // verifica se estamos a menos de 2 metros do alvo
      {
        stop();
        cond = false; // Para o loop uma vez que a distância é atingida.
      } // Anda em linha reta por 2,5 segundos
      faz_vetores(ponto0[0], ponto0[1], ponto1[0], ponto1[1], vec0[0], vec0[1]);                 // vetor de movimento
      faz_vetores(ponto1[0], ponto1[1], ponto_goal[0], ponto_goal[1], vec_goal[0], vec_goal[1]); // vetor entre o ponto atual e o objetivo
      dot_prod(vec0[0], vec0[1], vec_goal[0], vec_goal[1], angulo0);                             // retorna o angulo0 entre os dois vetores
      ajusta_angulo(angulo0, angulo1);                                                           // Rotaciona o carrinho de acordo com o angulo adquirido
      angulo1 = angulo0;
      zero = agora;
    }
  }
}

void ajusta_angulo(double angulo0, double angulo1)
{
  double dif = to_ang(angulo1) - to_ang(angulo0);
  if (abs(dif) >= 40)
  {
    if (dif < 0)
    {
      ajusta_para(true, 2.5); // Para direita por 2,5 segundos
    }
    else
    {
      ajusta_para(false, 2.5); // Ajusta para esquerda por 2,5 segundos
    }
  }
  else if (abs(dif) < 40 && abs(dif) >= 30)
  {
    if (dif < 0)
    {
      ajusta_para(true, 1.5);
    }
    else
    {
      ajusta_para(false, 1.5);
    }
  }
  else if (abs(dif) < 30 && abs(dif) >= 20)
  {
    if (dif < 0)
    {
      ajusta_para(true, 1);
    }
    else
    {
      ajusta_para(false, 1);
    }
  }
  else if (abs(dif) < 20 && abs(dif) >= 10)
  {
    if (dif < 0)
    {
      ajusta_para(true, 0.5); // Para direita por 2,5 segundos
    }
    else
    {
      ajusta_para(false, 0.5); // Ajusta para esquerda por 2,5 segundos
    }
  }
  else if (abs(dif) < 10)
  {
    if (dif < 0)
    {
      ajusta_para(true, 0.2); // Para direita por 2,5 segundos
    }
    else
    {
      ajusta_para(false, 0.2); // Ajusta para esquerda por 2,5 segundos
    }
  }
}

//////////////FUNÇÕES E DESCRIÇÕES////////////////////////
/*Função para pegar dois pontos*/
void pega_pontos()
{
  le_gps(true, ponto0[0], ponto0[1]);
  anda_para_frente(3000);
  delay(200);
  le_gps(true, ponto1[0], ponto1[1]);
}
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
void le_gps(bool escreve_na_port, double ponto_x, double ponto_y)
{
  if (GPS_Serial.available() > 0)
  {
    gps.encode(GPS_Serial.read());
    ponto_x = gps.location.lat();
    ponto_y = gps.location.lng();
    Serial_Debug.print("Latitude= ");
    Serial_Debug.print(gps.location.lat(), 8);
    Serial_Debug.print(" Longitude= ");
    Serial_Debug.println(gps.location.lng(), 8);
  }
  else
  {
    Serial_Debug.println("Falha na leitura do GPS!!");
  }
}
/*Calcula um vetor entre dois pontos e retorna o valor para as ultimas variáveis passadas para a função*/
void faz_vetores(double x_zero, double y_zero, double x_1, double y_1, double x_vec, double y_vec)
{
  x_vec = x_1 - x_zero;
  y_vec = y_1 - y_zero;
}
/*Retorna angulo entre dois vetores em radianos*/
double dot_prod(double x_vetor_0, double y_vetor_0, double x_vetor_1, double y_vetor_1, double angulo)
{
  angulo = asin((x_vetor_0 * x_vetor_1 + y_vetor_0 * y_vetor_1) / (modulo(x_vetor_0, y_vetor_0) * modulo(x_vetor_1, y_vetor_1)));
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
/*Conversor de Radiano para grau*/
double to_ang(double angulo_rad)
{
  return angulo_rad * 90 / PI;
}

//////////////FUNÇÕES DE MOVIMENTAÇÃO////////////////////////
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
/*Se verdadeiro vai para a direita. Tempo colocado em segundos*/
void ajusta_para(bool direita, float tempo)
{
  if (direita)
  {
    gira_para_direita(1000 * tempo);
    delay(200);
    anda_para_frente(3000);
  }
  else
  {
    gira_para_esquerda(1000 * tempo);
    delay(200);
    anda_para_frente(3000);
  }
  Serial_Debug.println("Posição ajustada");
}
/*Para todos os motores*/
void stop()
{
  analogWrite(in1, 0);
  analogWrite(in4, 0);
  analogWrite(in2, 0);
  analogWrite(in3, 0);
}
////////////////////ROTINA DE ORIENTAÇÃO PARA NORTE///////////////////
