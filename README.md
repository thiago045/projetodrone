 
Instruções para desenvolvimento de drone quadrotor.

Thiago Lima Soares


1-	Código Fonte:

O primeiro passo é realizar a leitura do sensor através do pacote kalman do arduino, ele é capaz de realizar a leitura MPU6050 e filtrar os ruidos da vibração dos motores como mostrado no trecho de código abaixo. Esse trecho de código mostra como é feita parte da obtenção dos angulos finais que serão utilizados para medição da angulação. Os angulos KalAngleX,KalAngleY e KalAngleZ já recebem os valores da fusão do acelerômetro e giroscópio juntamente com filtro de kalman, por isso são as variáveis de referência de posição para os PIDs.

```
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

```
Logo após realizar a leitura dos dados é preciso que seja calculado os dois PIDs. Isso é feito utilizando as funções "exePIDx" e "exePIDy". Elas calculam os PIDs individuais para os dois eixos de referência, pitch e roll. O valor calculado pelo PID é armazenado na variável PIDx para o controle do eixo x e PIDy na função "exePIDy" para controle do eixo Y.

```

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


```

Após o calculo dos PIDs é preciso realizar a soma dos dois controladores distribuidos nos 4 motores. Isso é feito pela função "Saida PID". Essa função raliza a soma ou subtração dos PIDs para compor a velocidade dos 4 motores como mostrado no treco de código abaixo e realizar a escrita no PWM da esp 32 através do comando "ledcWrite(canal, velocidade);".

```
 velocidadeMotor1 = velocidade_min - PIDx + PIDy;
  velocidadeMotor2 = velocidade_min - PIDx - PIDy;
  velocidadeMotor3 = velocidade_min + PIDx - PIDy;
  velocidadeMotor4 = velocidade_min + PIDx + PIDy;

```


O código também utiliza a conectividade WIFI da ESP 32 com a conexão feita de forma direta com smartphone android utilizando o aplicativo TCP cliente. Através do aplicativo é possível enviar strings para a ESP32 que são tratados e transformados em comandos. Os comandos estão na tabela abaixo. A função responsável pela conexão Wi-FI é a "tcp" e a responsável pela filtragem e decodificação da string recebida é a "tratarString".

<table border="1">
<tr>
<td>Função</td> <td>Sintaxe</td> <td>Comentário</td>
</tr>

<tr>		
<td>Set point x</td>	<td>spxnum</td>	<td>"num" será o set point em graus de x</td>
</tr>

<tr>		
<td>Set point y</td>	<td>spynum</td>	<td>"num" será o set point em graus de y</td>
</tr>

<tr>		
<td>Ligar PID e partida</td>	<td>on</td>	<td>liga o PID e parte os motores de forma gradativa.</td>
</tr>
		
<tr>		
<td>Desliga PID e motores</td>	<td>off</td>	<td>Desliga o PID e os motores</td>
</tr>	

<tr>		
<td>Constante proporcional de x </td>	<td>kpxnum</td>	<td>"num" será o valor da constante</td>
</tr>	

<tr>		
<td>Constante integral de x  </td>		<td>kixnum</td><td>"num" será o valor da constante</td>
</tr>	

<tr>		
<td>Constante derivativa de x   </td><td>kdxnum</td><td>"num" será o valor da constante</td>
</tr>		

<tr>		
<td>Constante proporcional de y </td>	<td>kpynum</td>	<td>"num" será o valor da constante</td>
</tr>	

<tr>		
<td>Constante integral de y  </td>		<td>kiynum</td><td>"num" será o valor da constante</td>
</tr>	

<tr>		
<td>Constante derivativa de y   </td><td>spynum</td><td>"num" será o valor da constante</td>
</tr>	
				

</table>

2-	Projeto mecânico e placa de controle.
O projeto 3D foi feito no software solid edge e pode ser implementado em uma impressora 3D. A placa de controle foi feita no software altium, impressa e formatada utilizando uma fresa. Com o link para Donwload abaixo.

https://drive.google.com/drive/folders/1SbIifeUDNtIDwXhCFXAo1O3NE0bMHk1v

3-Componentes.
Foram utilizados motores de 20x7mm com alimentação de 3.7V com caixa de redução e helices fabricados para ser utilizados no drone Syma X5. Para o acionamento foram utilizados mosfets do tipo SMD com resistências de \textit{pull-down} de 10k e diodos ligados em paralelo aos motores para evitar fuga de corrente na parada. O conjunto todo foi alimentado por uma bateria de íons de lítio de 850 mAh 3.7 VCC. A figura abaixo mostra o diagrama esquemático de ligação dos componentes.

![Diagrama esquemático de coneções do projeto](https://github.com/thiago045/projetodrone/blob/master/esquematicoCircuito1819.png)

