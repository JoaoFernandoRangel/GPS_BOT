#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "math.h"
#include "string.h"
#include "stdlib.h"

#define Serial_Debug Serial
#define GPS_Serial Serial1
#define SerialBT Serial3

/*
Centro do campo
Latitude= -20.309399 Longitude= -40.319274

Gol longe
Latitude= -20.309140 Longitude= -40.319346

Gol perto
Latitude= -20.309714 Longitude= -40.319204
*/

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
void escreve_Serial(bool debug, bool debug_BT, String mensagem);
void envia_BT(String mensagem0, String mensagem1);
bool filtro_msg(String msg, bool comando);
//////////////Variáveis Declaradas////////////////////////

// Portas de controle da ponte H
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

// Variáveis do tipo String
String ponto_goal_lat = "-20.310872556";
String ponto_goal_lng = "-40.319732665"; // Strings de local
String mensagem = ";";
String mensagem_envio1, mensagem_envio2, mensagem_comando;

// Verificadores
bool orientado = false;
bool primeiro = true;
bool hasInitialPoint = false;
bool cond = true;       // Variável vai ser usada para iniciar a operação de busca uma vez que o ponto objetivo for enviado pelo Bluetooth
bool Connected = false; // Variável para conexão bluetooth
bool comando = false;

// Temporizadores
unsigned long agora = 0;
unsigned long zero = 0;
unsigned long zero2 = 0;

void setup()
{
  // Configurações iniciais...
  Serial_Debug.begin(9600); // Inicia Serial com computador
  GPS_Serial.begin(9600);   // Inicia Serial com GPS
  SerialBT.begin(9600);     // Inicia Serial com módulo Bluetooth

  Serial_Debug.println("Finish Setup");
}

void loop()
{
  agora = millis();
  if (comando)
  {
    Serial_Debug.println("Comando recebido");
  }
  while (SerialBT.available())
  {
    mensagem += SerialBT.readStringUntil('/');
    delay(5); // Introduce a short delay inside the loop
  }

  if (agora - zero2 >= 1000)
  {
    /* envia_BT("Lat: " + String(ponto_goal[0]) +" _ "+" Lng: "+String(ponto_goal[1])); */

    if (filtro_msg(mensagem, comando))
    {
      mensagem_comando = mensagem;
      mensagem_comando.replace(";", " ");
      mensagem_comando.trim();
      Serial_Debug.println("Filtro passado");
    }
    escreve_Serial(true, true, mensagem_comando);
    // Serial_Debug.println("Envio_BT");
    zero2 = agora;
    mensagem = "";
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
    Serial_Debug.print(gps.location.lat(), 6);
    Serial_Debug.print(" Longitude= ");
    Serial_Debug.println(gps.location.lng(), 6);
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

//////////////FUNÇÕES BLUETOOTH////////////////////////

/// @brief Escreve algo na comunicação Serial
/// @param debug Comunicação Serial com o PC
/// @param debug_BT Comunicação Serial para o Bluetooth
/// @param mensagem  Mensagem a ser enviada
void escreve_Serial(bool debug, bool debug_BT, String mensagem)
{
  if (debug)
  {
    Serial_Debug.println(mensagem_comando);
  }
  if (debug_BT)
  {
    SerialBT.println(mensagem_comando);
  }
}

void envia_BT(String mensagem0, String mensagem1)
{
  SerialBT.print(mensagem0);
  SerialBT.print("_");
  SerialBT.println(mensagem1);

  Serial_Debug.print(mensagem0);
  Serial_Debug.print("_");
  Serial_Debug.println(mensagem1);
}

bool filtro_msg(String msg, bool comando)
{
  bool ver1 = false; // valida os marcadores
  bool ver2 = false; // valida o tamanho da mensagem
  if (msg.indexOf("#") != -1)
  {
    comando = true;
    return true;
  }
  if (msg.endsWith(";;") && msg.startsWith(";"))
  { // verifica inicio e fim da mensagem
    ver1 = true;
  }
  if (msg.length() >= 23)
  { // Duas coordenadas + sinais de menos + marcadores de inicio e fim + separador
    ver2 = true;
  }
  if (ver1 && ver2)
  {
    return true;
  }else{
    return false;
  }
}

//////////////FUNÇÕES DE MOVIMENTAÇÃO////////////////////////
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
