/*------------------Librarys------------------*/
#include <Wire.h>
#include <Kalman.h>
#include <ArduinoOTA.h>
#ifdef ESP8266//Se estiver usando ESP8266, automáticamente irá adicionar as bibliotecas do ESP8266.
#include <ESP8266WiFi.h>
#include <WiFiServer.h>
#elif defined ESP32//Se estiver usando ESP32, fara a mesma operaçao.
#include <WiFi.h>
#endif

/*------------------Defines-------------------*/

WiFiServer sv(555);//Cria o objeto servidor na porta 555
WiFiClient cl;//Cria o objeto cliente.

String mensagem;
String constante;
/*-----------------Variables------------------*/
/*-*-*-*-*-*-*-*-*-*MPU6050-*-*-*-*-*-*-*-*-*-*/
double accX, accY, accZ, gyroX, gyroY, gyroZ;
uint8_t i2c_data[14];
/*-*-*-*-*-*-*-*-*-*KALMAN-*-*-*-*-*-*-*-*-*-*-*/
double KalAngleX, KalAngleY, KalAngleZ;
double gyroXangle, gyroYangle, gyroZangle;
/*-*-*-*-*-*-*-*-*-*-*PID-*-*-*-*-*-*-*-*-*-*-*/
int valor_max=1024;
int valor_min=-1024;
double tempoAnterior;
float PosicaoAtualx = 0.0;
float PosicaoAnteriorx = 0.0;
float spx = 0.0;
float Kpx = 2.5; //VALUE 1.9 2.5
float Kix = 0.07; //VALUE 0.03 0.04
float Kdx = 0.0065; //VALUE 0.008 0.0065
float Pfloatx = 0.0;
float Ifloatx = 0.0;
float Dfloatx = 0.0;
float PIDfloatx = 0.0;
float erro_antx = 0.0;
float erro_atualx = 0.0;
float derrox = 0.0;
float dt = 0.001;
int Px = 0;
int Ix = 0;
int Dx = 0;
int PIDx = 0;
int contagemInicialx = 0;
int tempo_anteriorx = 0;
int tempo_atualx = 0;
int posicao_anteriorx = 0;
bool primeiraleiturax = 0;
int Speed = 500,Vel_PID=50;
float spy = 0.0;
float Kpy = 2.5; //VALUE 1.9 2.5
float Kiy = 0.07; //VALUE 0.03 0.04
float Kdy = 0.0065; //VALUE 0.008 0.0065
float Pfloaty = 0.0;
float Ifloaty = 0.0;
float Dfloaty = 0.0;
float PIDfloaty = 0.0;
float erro_anty = 0.0;
float erro_atualy = 0.0;
float derroy = 0.0;

int Py = 0;
int Iy = 0;
int Dy = 0;
int PIDy = 0;
int contagemInicialy = 0;
int tempo_anteriory = 0;
int tempo_atualy = 0;
int posicao_anteriory = 0;
bool primeiraleituray = 0;
float PosicaoAtualy = 0.0;

float PosicaoAnteriory = 0.0;
int velocidade_min = 750; //VALUE 200
//int VelMotor1 = 0;
//int VelMotor2 = 0;
//int VelMotor3 = 0;
//int VelMotor4 = 0;

int velocidadeMotor1=0;
int velocidadeMotor2=0;
int velocidadeMotor3=0;
int velocidadeMotor4=0;
int dt1 = 0,ct1=0,pt1=0;
int motor1 = 0, motor2 = 1, motor3 = 2, motor4 = 3;



bool flagmsgspx = 0, flagmsgspy = 0,flagmsgkp = 0, flagmsgki = 0, flagmsgkd = 0,PIDonOff=1,FlagDesligado=0;
/*-----------------Instâncias-----------------*/
Kalman KalmanX, KalmanY, KalmanZ;

void setup()
{
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);


  WiFi.mode(WIFI_AP);//Define o WiFi como Acess_Point.
  WiFi.softAP("DroneThiago", "");//Cria a rede de Acess_Point.
  sv.begin();//Inicia o servidor TCP na porta declarada no começo.
  

 ledcAttachPin(13, 0);//Atribuimos o pino 5 ao canal 0.
  ledcSetup(0, 1000, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.

  
  ledcAttachPin(15, 1);//Atribuimos o pino 4 ao canal 1.
  ledcSetup(1, 1000, 10);//Atribuimos ao canal 1 a frequencia de 1000Hz com resolucao de 10bits.

  ledcAttachPin(23, 2);//Atribuimos o pino 18 ao canal 2.
  ledcSetup(2, 1000, 10);//Atribuimos ao canal 2 a frequencia de 1000Hz com resolucao de 10bits.

  ledcAttachPin(32, 3);//Atribuimos o pino 19 ao canal 3.
  ledcSetup(3, 1000, 10);//Atribuimos ao canal 3 a frequencia de 1000Hz com resolucao de 10bits.
  
  //WifiOTASetup(); //Init OTA Config
  Serial.begin(115200);
  configMPU(); //Init MPU
  initKalman(); //Init Kalman
  tempoAnterior = millis();
}


/*-----------------MAIN------------------*/
void loop()
{
  //ArduinoOTA.handle();
  Get_Values_MPU6050();
  KalmanAngles();
  if(millis()>=6000)
  {
    if(PIDonOff==1)
      {FlagDesligado=0;
       exePIDx();
        exePIDy();
        SaidaPID();
      }
        
    }
   
  ct1 = millis();
  dt1 = ct1 - pt1;
  
   if (dt1 >= Vel_PID)
  {
    tcp();
    tratarString(mensagem);

    pt1 = ct1;
    //Serial.print("SetPoint"); Serial.print(SetPoint); Serial.print("°"); Serial.print(" kP"); Serial.print(kProporcional,4); Serial.print(" kI"); Serial.print(kIntegral, 4); Serial.print(" kd"); Serial.println(kDiferencial, 4);
    
  
    if (millis() >= 8000)
    
    { if(PIDonOff==1)
      {FlagDesligado=0;
       
      }
      
      
      else{
          if(FlagDesligado==0)
            {
         
           ledcWrite(motor1, 0);
           ledcWrite(motor2,0);
           ledcWrite(motor3, 0);
           ledcWrite(motor4, 0); 
           FlagDesligado=1;
           velocidade_min = 100;
             }
                
        }
    }
    
    
  }


  
}


void initKalman()
{
  while (i2cRead(0x3B, i2c_data, 14)); //Leitura dos dados de Acc XYZ
  /* 2 - XYZ Data Organization */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])
  /* 3 - Calculating Roll, Pitch, Yaw */
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan(accX / sqrt(accZ * accZ + accY * accY)) * RAD_TO_DEG;
  /* 4 - Kalman XYZ Filter Initialized*/
  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);
  KalmanZ.setAngle(yaw);

  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
}

/*--------------Get MPU6050 Values-------------------*/
void Get_Values_MPU6050()
{
  /*---------------Reed Registers---------------*/
  while (i2cRead(0x3B, i2c_data, 14));
  /*---------------Acc Values Treatment---------*/
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MostSB ] [ LeastSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MostSB ] [ LeastSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MostSB ] [ LeastSB ])
  /*--------------Gyro Values Treatment---------*/
  gyroX = (int16_t)((i2c_data[8] << 8) | i2c_data[9]);   // ([ MostSB ] [ LeastSB ])
  gyroY = (int16_t)((i2c_data[10] << 8) | i2c_data[11]); // ([ MostSB ] [ LeastSB ])
  gyroZ = (int16_t)((i2c_data[12] << 8) | i2c_data[13]); // ([ MostSB ] [ LeastSB ])
}



/*--------------Calculating PID-------------------*/
void exePIDx()
{
  PosicaoAtualx = KalAngleY;
  erro_antx = erro_atualx;
  erro_atualx = spx - PosicaoAtualx;
  derrox = erro_atualx - erro_antx;
  Pfloatx = Kpx * erro_atualx;
  Ifloatx += Kix * erro_atualx * dt;
  if (primeiraleiturax == 0)
  {
   Dfloatx = 0;
 }
 else
 {   Dfloatx = Kdx * (PosicaoAtualx - PosicaoAnteriorx) / dt;
 }
  Dfloatx = Kdx * (PosicaoAtualx - PosicaoAnteriorx) / dt;
  PIDfloatx = Pfloatx + Ifloatx - Dfloatx;
  PIDx = PIDfloatx;
  //primeiraleitura = 1;
  if (PIDx >= 1024)
  {
    PIDx = 1024;
  }
  if (PIDx <= -1024)
  {
    PIDx = -1024;
  }
  
  
//  Serial.print("     VelMotor1     ");Serial.println(VelMotor1);
//  Serial.print("     VelMotor2     ");Serial.println(VelMotor2);
  Serial.print("     KalAngleX     ");Serial.println(KalAngleX);
}

void exePIDy()
{
  PosicaoAtualy = KalAngleX;
  erro_anty = erro_atualy;
  erro_atualy = spy - PosicaoAtualy;
  derroy = erro_atualy - erro_anty;
  Pfloaty = Kpy * erro_atualy;
  Ifloaty += Kiy * erro_atualy * dt;
//  if (primeiraleitura == 0)
//  {
//    Dfloat = 0;
//  }
//  else
//  {
//    Dfloat = Kd * (PosicaoAtual - PosicaoAnterior) / dt;
//  }
  Dfloaty = Kdy * (PosicaoAtualy - PosicaoAnteriory) / dt;
  PIDfloaty = Pfloaty + Ifloaty - Dfloaty;
  PIDy = PIDfloaty;
  //primeiraleitura = 1;
  if (PIDy >= 1024)
  {
    PIDy = 1024;
  }
  if (PIDy <= -1024)
  {
    PIDy = -1024;
  }
 
//  Serial.print("     VelMotor1     ");Serial.println(VelMotor1);
//  Serial.print("     VelMotor2     ");Serial.println(VelMotor2);
  Serial.print("     KalAngleY     ");Serial.println(KalAngleY);
}

void SaidaPID()
{ 
  if(velocidade_min<=250)
  {
    velocidade_min=velocidade_min+10;
    }
    
  velocidadeMotor1 = velocidade_min - PIDx + PIDy;
  velocidadeMotor2 = velocidade_min - PIDx - PIDy;
  velocidadeMotor3 = velocidade_min + PIDx - PIDy;
  velocidadeMotor4 = velocidade_min + PIDx + PIDy;
  if (velocidadeMotor1 >= valor_max)
  {
    velocidadeMotor1 = valor_max;
  }

  if (velocidadeMotor1 <= 0)
  {
    velocidadeMotor1 = 0;
  }

  if (velocidadeMotor2 >= valor_max)
  {
    velocidadeMotor2 = valor_max;
  }

  if (velocidadeMotor2 <= 0)
  {
    velocidadeMotor2 = 0;
  }

  if (velocidadeMotor3 >= valor_max)
  {
    velocidadeMotor3 = valor_max;
  }

  if (velocidadeMotor3 <= 0)
  {
    velocidadeMotor3 = 0;
  }

  if (velocidadeMotor4 >= valor_max)
  {
    velocidadeMotor4 = valor_max;
  }

  if (velocidadeMotor4 <= 0)
  {
    velocidadeMotor4 = 0;
  }
 Serial.print("velocidadeMotor1"); Serial.print(velocidadeMotor1);
  Serial.print("velocidadeMotor2"); Serial.print(velocidadeMotor2);
  Serial.print("velocidadeMotor3"); Serial.print(velocidadeMotor3);
 Serial.print("velocidadeMotor4"); Serial.println(velocidadeMotor4);

  ledcWrite(motor1, velocidadeMotor1);
  ledcWrite(motor2, velocidadeMotor2);
  ledcWrite(motor3, velocidadeMotor3);
  ledcWrite(motor4, velocidadeMotor4);
  //ledcWrite(motor1, 1024);
  //ledcWrite(motor2, 1024);
  //ledcWrite(motor3, 1024);
   //ledcWrite(motor4, 1024);
}

/*--------------GET Kalman Angles-------------------*/
void KalmanAngles()
{
  double dt = (double)(millis() - tempoAnterior);
  tempoAnterior = millis();
  
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  gyroXangle = gyroX / 131.0; //deg/s
  gyroYangle = gyroY / 131.0;
  gyroZangle = gyroZ / 131.0;
  
  KalAngleX = KalmanX.getAngle(roll, gyroXangle, dt / 1000);
  KalAngleY = KalmanY.getAngle(pitch, gyroYangle, dt / 1000);
  KalAngleZ = KalmanZ.getAngle(yaw, gyroZangle, dt / 1000);
  
  //Serial.print(KalAngleX);Serial.print("\t");
  //Serial.print(KalAngleY);Serial.print("\t");
  //Serial.print(KalAngleZ);Serial.print("\n");
}






//--------------------------------WIFI-------------------------------------------------------------------------------------
void tratarString(String stringRecebida)
{
  String constante1, constante2, constante3, constante4;
  //Serial.print(stringRecebida[1]);

  if (stringRecebida[0] == 'o' && stringRecebida[1] == 'n')
  {
    PIDonOff = 1;

    
  }

  if (stringRecebida[0] == 'o' && stringRecebida[1] == 'f' && stringRecebida[2] == 'f')
  {
    PIDonOff = 0;


  }

  if (stringRecebida[0] == 'v' && stringRecebida[1] == 'm')
  {
    String constante0;
    for (int i = 2; i <= 7; i++)

    {
      constante0 = constante0 + stringRecebida[i];
      
      
    }
    velocidade_min = constante0.toInt();

    Serial.print("Você enviou veocidade minima com valor "); Serial.println(velocidade_min);
  }
  if(stringRecebida[2]=='x')
  {
    if ((stringRecebida[0] == 'k' || stringRecebida[0] == 's') && (stringRecebida[1] == 'p' || stringRecebida[1] == 'i' || stringRecebida[1] == 'd' || stringRecebida[1] == 'p'))
    {
      if (stringRecebida[0] == 'k' && stringRecebida[1] == 'p')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante1 = constante1 + stringRecebida[i];
          
          Kpx = constante1.toFloat();
          

        }

        Serial.print("Você enviou kpx com valor "); Serial.println(constante1);
        

      }

      if (stringRecebida[1] == 'i')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante2 = constante2 + stringRecebida[i];
          Kix = constante2.toFloat();
          
        }

        Serial.print("Você enviou kix com valor "); Serial.println(constante2);
        


      }

      if (stringRecebida[1] == 'd')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante3 = constante3 + stringRecebida[i];
          Kdx = constante3.toFloat();
          
        }

        Serial.print("Você enviou kdx com valor"); Serial.println(constante3);
        

      }
      if (stringRecebida[0] == 's' && stringRecebida[1] == 'p')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante4 = constante4 + stringRecebida[i];
          spx = constante4.toFloat();
          
        }

        Serial.print("Você enviou set point em x com valor"); Serial.println(constante4);
        


      }
    }
    
  }

  if (stringRecebida[2] == 'y')
  {
    if ((stringRecebida[0] == 'k' || stringRecebida[0] == 's') && (stringRecebida[1] == 'p' || stringRecebida[1] == 'i' || stringRecebida[1] == 'd' || stringRecebida[1] == 'p'))
    {
      if (stringRecebida[0] == 'k' && stringRecebida[1] == 'p')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante1 = constante1 + stringRecebida[i];
          
          

        }
        Kpy = constante1.toFloat();
        
        Serial.print("Você enviou kpy com valor "); Serial.println(constante1);
        

      }

      if (stringRecebida[1] == 'i')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante2 = constante2 + stringRecebida[i];
          
        }
        Kiy = constante2.toFloat();
        
        Serial.print("Você enviou kiy com valor "); Serial.println(constante2);
        


      }

      if (stringRecebida[1] == 'd')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante3 = constante3 + stringRecebida[i];
          
        }
        Kdy = constante3.toFloat();
        
        Serial.print("Você enviou kdy com valor"); Serial.println(constante3);
        

      }
      if (stringRecebida[0] == 's' && stringRecebida[1] == 'p')
      {
        for (int i = 3; i <= 7; i++)

        {
          constante4 = constante4 + stringRecebida[i];
          
        }
        spy = constante4.toFloat();
        
        Serial.print("Você enviou set point em y com valor"); Serial.println(constante4);
        


      }
    }

  }
  
}

void tcp()
{
  if (cl.connected())//Detecta se há clientes conectados no servidor.
  
  {
    
    String angulox= String(KalAngleY);
    cl.print("\n...Agulo x" + angulox + "\n");
    if (cl.available() > 0)//Verifica se o cliente conectado tem dados para serem lidos.
    {
      String req = "";
      while (cl.available() > 0)//Armazena cada Byte (letra/char) na String para formar a mensagem recebida.
      {
        char z = cl.read();
        req += z;
      }

      mensagem = req;

      //Mostra a mensagem recebida do cliente no Serial Monitor.
      Serial.print("\nUm cliente enviou uma mensagem");
      Serial.print("\n...IP do cliente: ");
      Serial.print(cl.remoteIP());
      Serial.print("\n...IP do servidor: ");
      Serial.print(WiFi.softAPIP());
      Serial.print("\n...Mensagem do cliente: " + req + "\n");

      //Envia uma resposta para o cliente
      cl.print("\nO servidor recebeu sua mensagem");
      cl.print("\n...Seu IP: ");
      cl.print(cl.remoteIP());
      cl.print("\n...IP do Servidor: ");
      cl.print(WiFi.softAPIP());
      cl.print("\n...Sua mensagem: " + req + "\n");
      cl.print("\n...Agulo x" + angulox + "\n");

    }
  }

  else//Se nao houver cliente conectado,
  {
    cl = sv.available();//Disponabiliza o servidor para o cliente se conectar.
    delay(1);
  }



}
