#include <Arduino.h>
#include "heltec.h"

struct __attribute__((packed)) SensorData { // Struct de dados para os pacotes de informações que será transmitido pelo LoRa.
  float radiacaoSolar; // Variáveis meterológicas
  float velocidadeVento; // Essa struct a partir da função packed é usada para garantir que os dados sejam enviados de forma compacta, sem bytes de preenchimento entre os campos. Pois o emissor e receptor precisam ter a mesma estrutura de dados para interpretar corretamente os valores transmitidos.
  float direcaoVento;
  float umidadeAr;
  float tempPlaca;
  float tempAmbiente;
  float inclinacao;
  double gpsLat;
  double gpsLong;
  uint16_t ano; // uint16_t é usado para armazenar valores inteiros de 16 bits, adequado para representar anos.
  uint8_t mes;
  uint8_t dia;
  uint8_t hora;
  uint8_t minuto;
  uint8_t segundo;
}; // Aqui termina-se a definição da struct SensorData, que será usada para armazenar e transmitir os dados meteorológicos coletados pelo emissor LoRa.

void setup() {  // A função setup() é executada uma única vez, quando o ESP32 é ligado ou reiniciado. Ela é utilizada para inicializar os periféricos e configurar o sistema.
  Heltec.begin(true, true, true, true, 470E6); // inicializa os recursos da placa Heltec. 470E6 é a frequência de operação do LoRa, que é 470 MHz. O primeiro parâmetro true indica que o display OLED deve ser inicializado, o segundo true indica que o LoRa deve ser inicializado, o terceiro true indica que o Wi-Fi deve ser inicializado e o quarto true indica que o Bluetooth deve ser inicializado.
  LoRa.setSyncWord(0x12); // Configura a palavra de sincronização do rádio LoRa.
  Serial.begin(9600); // Inicializa a comunicação serial do ESP32 com o computador a uma taxa de 9600 bits por segundo. Isso permite que o ESP32 envie mensagens de depuração e status para o monitor serial.
}

void loop() {
  static unsigned long lastTx = 0; // Essa variável armazena o instante da última transmissão. O Static é usado para que a variável mantenha seu valor entre as chamadas da função loop(), permitindo que o código verifique o tempo decorrido desde a última transmissão.
  if (millis() - lastTx > 3000) { // Verifica se se passaram mais de 3 segundos desde a última transmissão. A função millis() retorna o tempo em milissegundos desde que o ESP32 foi ligado.
    SensorData data; // Aqui é criado um objeto chamado data do tipo SensorData. Ele representa o pacote que será preenchido e posteriormente transmitido.
    // INICIALIZA TODOS OS CAMPOS PARA NÃO ENVIAR LIXO
    data.radiacaoSolar = 800.0 + (random(-10, 11) / 1.0);
    data.velocidadeVento = 5.0 + (random(0, 100) / 100.0);
    data.direcaoVento = 180.0;
    data.umidadeAr = 60.0;
    data.tempPlaca = 45.0;
    data.tempAmbiente = 25.0;
    data.inclinacao = 23.5;
    data.gpsLat = -23.550;
    data.gpsLong = -46.633;
    data.ano = 2026;
    data.mes = 5;
    data.dia = 14;
    data.hora = 14;
    data.minuto = random(0, 60);
    data.segundo = random(0, 60);

    LoRa.beginPacket(); // Inicia a preparação de um novo pacote LoRa. A partir daqui, os dados que forem escritos serão colocados no pacote que será transmitido.
    LoRa.write((uint8_t*)&data, sizeof(SensorData)); // Nesta linha o método write() é usado para escrever os dados do objeto data no pacote LoRa. O operador & é usado para obter o endereço de memória do objeto data, e o sizeof(SensorData) retorna o tamanho em bytes da struct SensorData. Isso garante que todos os campos da struct sejam incluídos no pacote.
    LoRa.endPacket(); // Finaliza a preparação do pacote e envia os dados pelo rádio LoRa. Após essa chamada, o pacote é transmitido para qualquer receptor LoRa na mesma frequência e com a mesma palavra de sincronização.
    
    Serial.printf(">>> ENVIANDO LORA: Rad=%.1f | Tam=%d bytes\n", data.radiacaoSolar, sizeof(SensorData));
    
    Heltec.display->clear();// Limpa o conteúdo do display OLED antes de desenhar novas informações.
    Heltec.display->drawString(0, 0, "EMISSOR OK");
    Heltec.display->drawString(0, 20, "Enviando Rad: " + String(data.radiacaoSolar, 1)); // Desenha a mensagem "Enviando Rad: <valor>" na posição (0, 20) do display OLED. O valor da radiação solar é formatado com uma casa decimal.
    Heltec.display->display(); // Atualiza o display OLED para mostrar as informações desenhadas. Sem essa chamada, as alterações feitas com drawString() não seriam visíveis no display.
    
    lastTx = millis(); // Atualiza a variável lastTx com o tempo atual em milissegundos, marcando o instante da última transmissão. Isso é usado para controlar o intervalo entre as transmissões subsequentes.
  }
}
