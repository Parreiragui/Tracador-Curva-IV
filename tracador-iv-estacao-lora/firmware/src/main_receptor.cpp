// --- RECEPTOR (ESTABILIZADO + MULTI-CORE) ---
#include <Arduino.h>
#include <SPI.h> // Biblioteca SPI para comunicação com o módulo LoRa. (Serial Peripheral Interface)
#include <LoRa.h>
#include <Wire.h> // Biblioteca Wire para comunicação I2C com o display OLED.
#include <U8g2lib.h> // Biblioteca U8g2 para controle do display OLED. Ela fornece funções para desenhar texto, gráficos e imagens no display.
#include <Preferences.h> // Biblioteca Preferences para armazenar dados de configuração na memória flash do ESP32. Ela permite salvar e recuperar valores entre reinicializações do dispositivo.
#include <ModbusMaster.h> // Biblioteca ModbusMaster para comunicação Modbus RTU com o medidor de energia PZEM-017. Ela permite ler e escrever registros do dispositivo Modbus.
#include <RTClib.h> // Biblioteca RTClib para comunicação com o módulo RTC (Real Time Clock) DS3231. Ela fornece funções para ler e ajustar a data e hora do RTC.
#include <SD.h> // Biblioteca SD para comunicação com o cartão SD. Ela permite ler e escrever arquivos no cartão SD.
#include <WiFi.h>
#include <ArduinoOTA.h> // Biblioteca ArduinoOTA para atualização de firmware Over-The-Air (OTA). Ela permite atualizar o firmware do ESP32 sem a necessidade de conexão física via USB.
#include "secrets.h" // Arquivo de cabeçalho "secrets.h" que contém informações sensíveis, como credenciais de Wi-Fi e senhas. Ele é incluído para manter essas informações separadas do código principal e facilitar a manutenção.
#include <ESPAsyncWebServer.h> // Biblioteca ESPAsyncWebServer para criar um servidor web assíncrono no ESP32. Ela permite lidar com requisições HTTP de forma eficiente, sem bloquear a execução do código principal.
#include <Update.h> // Biblioteca Update para gerenciar o processo de atualização de firmware OTA. Ela fornece funções para iniciar, escrever e finalizar a atualização do firmware no ESP32.
#include "web_dashboard.h" // Arquivo de cabeçalho "web_dashboard.h" que contém o código HTML e JavaScript para a interface web do dashboard. Ele é incluído para fornecer a interface de usuário para monitoramento e controle do sistema.
#include "web_teste.h" // Arquivo de cabeçalho "web_teste.h" que contém o código HTML e JavaScript para a interface web de teste. Ele é incluído para fornecer a interface de usuário para iniciar e monitorar os testes do sistema.

// --- PINOS HELTEC V2 ---
#define LORA_SCK 5 // Pino de clock do SPI para o módulo LoRa. Ele é usado para sincronizar a transferência de dados entre o ESP32 e o módulo LoRa.
#define LORA_MISO 19 // Pino de entrada do SPI para o módulo LoRa. Ele é usado para receber dados do módulo LoRa para o ESP32.
#define LORA_MOSI 27 // Pino de saída do SPI para o módulo LoRa. Ele é usado para enviar dados do ESP32 para o módulo LoRa.
#define LORA_SS 18 // Pino de seleção do chip (SS) do SPI para o módulo LoRa. Ele é usado para ativar ou desativar a comunicação com o módulo LoRa.
#define LORA_RST 14 // Pino de reset do módulo LoRa. Ele é usado para reiniciar o módulo LoRa quando necessário.
#define LORA_DI0 26 // Pino de interrupção do módulo LoRa. Ele é usado para sinalizar eventos importantes, como a recepção de um pacote de dados.
#define LORA_BAND 915E6 // Frequência de operação do módulo LoRa. 915E6 representa 915 MHz, que é uma das frequências permitidas para comunicação LoRa em algumas regiões.

#define SD_SCK 17 // Pino de clock do SPI para o cartão SD. Ele é usado para sincronizar a transferência de dados entre o ESP32 e o cartão SD.
#define SD_MISO 13 // Pino de entrada do SPI para o cartão SD. Ele é usado para receber dados do cartão SD para o ESP32.
#define SD_MOSI 23 // Pino de saída do SPI para o cartão SD. Ele é usado para enviar dados do ESP32 para o cartão SD.
#define SD_CS 12 // Pino de seleção do chip (CS) do SPI para o cartão SD. Ele é usado para ativar ou desativar a comunicação com o cartão SD.

struct SensorData {
  float radiacaoSolar;
  float velocidadeVento;
  float direcaoVento;
  float umidadeAr;
  float tempPlaca;
  float tempAmbiente;
  float inclinacao;
  double gpsLat;
  double gpsLong;
  uint16_t ano;
  uint8_t mes;
  uint8_t dia;
  uint8_t hora;
  uint8_t minuto;
  uint8_t segundo;
} __attribute__((packed)); // Atributo packed é usado para garantir que a struct SensorData seja armazenada na memória de forma compacta, sem bytes de preenchimento entre os campos. Isso é importante para a transmissão de dados via LoRa, pois o emissor e o receptor precisam ter a mesma estrutura de dados para interpretar corretamente os valores transmitidos.

struct IVPoint { // Struct para armazenar os pontos de dados I-V (tensão-corrente) coletados durante o teste.
  float v; 
  float i;
  int pwm;
};

#define MAX_BUFFER_STEPS 1024 // Define o tamanho máximo do buffer de pontos I-V que podem ser armazenados durante um teste. Isso limita a quantidade de dados que podem ser coletados e salvos no cartão SD.

int pwmStartVal = 1000; // Valor inicial do PWM (Pulse Width Modulation) usado para controlar a carga IGBT durante o teste. Esse valor é ajustado dinamicamente com base no perfil de varredura definido pelo usuário.
int pwmEndVal = 400; // Valor final do PWM usado para controlar a carga IGBT durante o teste. Esse valor é ajustado dinamicamente com base no perfil de varredura definido pelo usuário.   
int pwmStepSize = 4; // (Não está sendo utilizado) Tamanho do passo do PWM usado para controlar a carga IGBT durante o teste. Esse valor determina a resolução da varredura de tensão-corrente, ou seja, quantos pontos serão coletados entre o valor inicial e final do PWM.
int currentTotalSteps = 20; // Número total de passos que serão realizados durante o teste I-V. Esse valor é ajustado dinamicamente com base no perfil de varredura definido pelo usuário e determina quantos pontos de dados serão coletados e armazenados no buffer ivBuffer.
int pwmSweepProfile[21]; // Array que armazena o perfil de varredura do PWM usado para controlar a carga IGBT durante o teste. Esse perfil é calculado com base nos valores de início e fim do PWM definidos pelo usuário e determina como o PWM será ajustado ao longo do teste.

IVPoint ivBuffer[MAX_BUFFER_STEPS]; // Array que armazena os pontos de dados I-V coletados durante o teste. Cada ponto contém a tensão, corrente e valor do PWM correspondente. O tamanho do array é definido pela constante MAX_BUFFER_STEPS, limitando a quantidade de dados que podem ser armazenados durante o teste. 

// IDLE, PRE_CONDITIONING, SWEEPING, SAVING, OTA_MODE, significam respectivamente: o sistema está aguardando para iniciar um teste ou operação, o sistema está realizando a pré-condicionamento da carga IGBT antes do teste, o sistema está realizando a varredura de tensão-corrente durante o teste, o sistema está salvando os dados coletados no cartão SD após o teste, e o sistema está em modo de atualização de firmware Over-The-Air (OTA).
enum SystemState { IDLE, PRE_CONDITIONING, SWEEPING, SAVING, OTA_MODE }; // Enumeração que define os estados do sistema. Cada estado representa uma fase diferente do processo de teste I-V ou do modo de operação do sistema. O estado atual é armazenado na variável currentState.
volatile SystemState currentState = IDLE; // Variável que armazena o estado atual do sistema. Ela é declarada como volatile para indicar ao compilador que seu valor pode ser alterado por interrupções ou outras threads, garantindo que o valor mais recente seja sempre lido. Nesse caso, começando no estado IDLE, que indica que o sistema está aguardando para iniciar um teste ou operação.

ModbusMaster node; // Instância da classe ModbusMaster usada para comunicação Modbus RTU com o medidor de energia PZEM-017. Ela permite ler e escrever registros do dispositivo Modbus, como tensão, corrente e potência.
RTC_DS3231 rtc; // Instância da classe RTC_DS3231 usada para comunicação com o módulo RTC (Real Time Clock) DS3231. Ela fornece funções para ler e ajustar a data e hora do RTC, permitindo que o sistema mantenha a hora correta mesmo após reinicializações ou falhas de energia.
SPIClass spiSD(HSPI); // Instância da classe SPIClass usada para comunicação SPI com o cartão SD. Ela permite ler e escrever arquivos no cartão SD, armazenando os dados coletados durante os testes I-V. O parâmetro HSPI indica que a comunicação SPI será realizada usando o barramento HSPI do ESP32, que é um dos dois barramentos SPI disponíveis no microcontrolador.
Preferences prefs; // Instância da classe Preferences usada para armazenar dados de configuração na memória flash do ESP32. Ela permite salvar e recuperar valores entre reinicializações do dispositivo, como índices de teste, escalas de tensão e corrente, e outras configurações importantes para o funcionamento do sistema.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, 16, 15, 4); // Instância da classe U8G2_SSD1306_128X64_NONAME_F_HW_I2C usada para controlar o display OLED. Ela fornece funções para desenhar texto, gráficos e imagens no display. O parâmetro U8G2_R0 indica a rotação do display, enquanto os números 16, 15 e 4 representam os pinos de reset, clock e dados do barramento I2C, respectivamente.

SensorData lastReceivedData; // Variável que armazena os últimos dados recebidos via LoRa. Ela é usada para manter uma cópia dos dados mais recentes transmitidos pelo emissor, permitindo que o sistema acesse essas informações para exibição no display OLED, salvamento no cartão SD ou outras operações relacionadas ao teste I-V.
SensorData snapshotData; // Variável que armazena uma cópia dos dados recebidos via LoRa no momento em que um teste I-V é iniciado. Ela é usada para manter um registro dos dados ambientais e de sensores no início do teste, permitindo que essas informações sejam associadas aos pontos de dados coletados durante o teste e salvos no cartão SD. 

float VOLTAGE_SCALE = 1.0f, CURRENT_SCALE = 1.0f; // Variáveis que armazenam os fatores de escala para tensão e corrente, respectivamente. Esses fatores são usados para ajustar os valores lidos do medidor de energia PZEM-017, permitindo que o sistema converta os valores brutos em unidades reais (volts e amperes) com base em calibração ou ajustes específicos do hardware.
float raw_v = 0, raw_i = 0; // Variáveis que armazenam os valores brutos de tensão e corrente lidos do medidor de energia PZEM-017. Esses valores são obtidos diretamente do dispositivo Modbus e são posteriormente ajustados pelos fatores de escala VOLTAGE_SCALE e CURRENT_SCALE para obter os valores reais de tensão e corrente. 
int testIndex = 0, currentStep = 0, packetCount = 0; // Variáveis que armazenam o índice do teste atual, o passo atual do teste I-V e a contagem de pacotes recebidos via LoRa, respectivamente. O testIndex é usado para identificar e salvar os arquivos de teste no cartão SD, o currentStep indica qual ponto de dados está sendo coletado durante o teste, e o packetCount mantém o controle do número de pacotes de dados recebidos pelo receptor.

AsyncWebServer server(80); // Instância da classe AsyncWebServer usada para criar um servidor web assíncrono no ESP32. Ela permite lidar com requisições HTTP de forma eficiente, sem bloquear a execução do código principal. O parâmetro 80 indica que o servidor estará escutando na porta padrão HTTP (porta 80) para receber conexões de clientes, como navegadores web ou aplicativos móveis.
uint32_t lastRtcSyncDay = 0; // Variável que armazena o último dia em que o relógio RTC foi sincronizado com os dados recebidos via LoRa. Ela é usada para evitar sincronizações repetidas no mesmo dia, garantindo que a hora do sistema seja atualizada apenas uma vez por dia com base nos dados recebidos do emissor.
int currentPwmFreq = 15000; // Variável que armazena a frequência atual do PWM (Pulse Width Modulation) usado para controlar a carga IGBT durante o teste. Essa frequência é ajustável via interface web e determina a taxa de comutação do sinal PWM, afetando o desempenho da carga e a precisão dos testes I-V.
unsigned long stateTimer = 0, lastPacketMillis = 0; // Variáveis que armazenam o tempo em milissegundos desde a última mudança de estado do sistema (stateTimer) e o tempo desde o último pacote de dados recebido via LoRa (lastPacketMillis). Elas são usadas para controlar temporizações e intervalos de operação, como a duração de cada fase do teste I-V e a detecção de perda de pacotes.
bool sdCardOK = false, rtcOK = false; // Variáveis booleanas que indicam o status do cartão SD (sdCardOK) e do módulo RTC (rtcOK). Elas são usadas para verificar se o cartão SD foi inicializado corretamente e se o módulo RTC está funcionando, permitindo que o sistema tome decisões apropriadas com base na disponibilidade desses recursos.
float PZEMVoltage = 0, PZEMCurrent = 0; // Variáveis que armazenam os valores de tensão e corrente ajustados lidos do medidor de energia PZEM-017. Esses valores são calculados a partir dos valores brutos raw_v e raw_i, aplicando os fatores de escala VOLTAGE_SCALE e CURRENT_SCALE, respectivamente. Eles representam as medições reais de tensão (em volts) e corrente (em amperes) que podem ser exibidas no display OLED, enviadas via interface web ou usadas para cálculos adicionais durante o teste I-V.

void preTransmission() { digitalWrite(21, HIGH); delayMicroseconds(500); } // Função de pré-transmissão usada para preparar o medidor de energia PZEM-017 antes de enviar comandos Modbus. Ela ativa o pino 21 (usado como controle de direção para comunicação RS-485) e aguarda 500 microssegundos para garantir que o dispositivo esteja pronto para receber os dados. Isso é necessário para evitar problemas de comunicação e garantir que os comandos Modbus sejam enviados corretamente.
void postTransmission() { Serial2.flush(); delay(5); digitalWrite(21, LOW); } // Função de pós-transmissão usada para finalizar a comunicação com o medidor de energia PZEM-017 após enviar comandos Modbus. Ela aguarda 5 milissegundos para garantir que todos os dados tenham sido transmitidos e, em seguida, desativa o pino 21 (usado como controle de direção para comunicação RS-485). Isso é necessário para evitar problemas de comunicação e garantir que o dispositivo esteja pronto para receber novos comandos ou enviar respostas.

void taskPZEM(void *pvParameters) { // Função de tarefa (task) que é executada em um núcleo separado do ESP32 para ler continuamente os valores de tensão e corrente do medidor de energia PZEM-017 usando a comunicação Modbus RTU. Essa tarefa é responsável por atualizar as variáveis raw_v, raw_i, PZEMVoltage e PZEMCurrent com os valores mais recentes lidos do dispositivo, permitindo que o sistema tenha acesso a medições atualizadas mesmo durante a execução de outras operações, como testes I-V ou comunicação LoRa.
  for(;;) {
    if (currentState == IDLE) {
      if (node.readInputRegisters(0x0000, 2) == node.ku8MBSuccess) {
        raw_v = node.getResponseBuffer(0) / 100.0f;
        raw_i = node.getResponseBuffer(1) / 100.0f;
        PZEMVoltage = raw_v * VOLTAGE_SCALE;
        PZEMCurrent = raw_i * CURRENT_SCALE;
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Aguarda 2 segundos antes de ler novamente os valores do medidor PZEM-017. Isso evita sobrecarregar o dispositivo com leituras contínuas e permite que o sistema tenha tempo suficiente para processar outras tarefas, como comunicação LoRa, atualização do display OLED ou execução de testes I-V. A função vTaskDelay é usada para suspender a execução da tarefa por um período específico, permitindo que outras tarefas sejam executadas no núcleo do ESP32.
  }
}

void receiveLoRaData() { // Função que recebe os dados transmitidos via LoRa. Ela verifica se há pacotes de dados disponíveis, lê os dados recebidos e atualiza a variável lastReceivedData com os valores mais recentes. Além disso, ela mantém o controle da contagem de pacotes recebidos e sincroniza o relógio RTC com os dados recebidos, se necessário.
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    if (packetSize == sizeof(SensorData)) {
      LoRa.readBytes((uint8_t*)&lastReceivedData, sizeof(lastReceivedData));
      packetCount++;
      lastPacketMillis = millis();
      Serial.printf(">>> Pacote #%d Recebido!\n", packetCount);
      
      if (currentState == IDLE && lastReceivedData.ano >= 2024 && lastRtcSyncDay != lastReceivedData.dia) {
        if (rtcOK) {
          rtc.adjust(DateTime(lastReceivedData.ano, lastReceivedData.mes, lastReceivedData.dia, 
                              lastReceivedData.hora, lastReceivedData.minuto, lastReceivedData.segundo));
        }
        lastRtcSyncDay = lastReceivedData.dia;
      }
    } else {
      Serial.printf("!!! Erro Tam: %d\n", packetSize);
      while (LoRa.available()) LoRa.read();
    }
  }
}

void saveToSD() { // Função que salva os dados coletados durante o teste I-V no cartão SD. Ela cria um arquivo CSV com os valores de tensão, corrente, potência e outros parâmetros ambientais, permitindo que os resultados do teste sejam analisados posteriormente.
  testIndex++;
  prefs.putInt("test_idx", testIndex);
  File f = SD.open("/iv_test_" + String(testIndex) + ".csv", FILE_WRITE);
  if (f) {
    f.println("PWM;Data_Hora;Tensao_V;Corrente_A;Potencia_W;Radiacao_Solar_Wm2;Velocidade_Vento_ms;Direcao_Vento_graus;Umidade_Ar_Perc;Temp_Placa_C;Temp_Ambiente_C;Inclinacao_graus;GPS_Lat;GPS_Long");
    for (int i = 0; i < currentTotalSteps; i++) {
      if (ivBuffer[i].v > 0 || ivBuffer[i].i > 0) {
        char dt[20];
        sprintf(dt, "%02d/%02d/%04d %02d:%02d", lastReceivedData.dia, lastReceivedData.mes, lastReceivedData.ano, lastReceivedData.hora, lastReceivedData.minuto);
        f.printf("%d;%s;%.3f;%.3f;%.3f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.3f;%.3f\n",
               ivBuffer[i].pwm, dt, ivBuffer[i].v, ivBuffer[i].i, ivBuffer[i].v * ivBuffer[i].i,
               snapshotData.radiacaoSolar, snapshotData.velocidadeVento, snapshotData.direcaoVento,
               snapshotData.umidadeAr, snapshotData.tempPlaca, snapshotData.tempAmbiente,
               snapshotData.inclinacao, snapshotData.gpsLat, snapshotData.gpsLong);
      }
    }
    f.close();
    Serial.println(">>> SD Salvo.");
  }
}

// Função isolada para calcular o Perfil Não-Linear (Zoom Dinâmico)
void calculateSweepProfile(int startPct, int endPct) { // Função que calcula o perfil de varredura do PWM usado para controlar a carga IGBT durante o teste I-V. Ela gera um array de valores PWM que representam uma transição suave entre os valores inicial e final definidos pelo usuário, permitindo que o teste seja realizado com maior precisão e controle.
    currentTotalSteps = 20;
    pwmStartVal = (startPct * 1023) / 100; // Calcula o valor inicial do PWM com base na porcentagem fornecida pelo usuário (startPct). O valor é convertido para uma escala de 0 a 1023, que corresponde à resolução de 10 bits do PWM no ESP32. Isso permite que o teste comece com um nível de tensão/corrente específico, conforme definido pelo usuário.
    pwmEndVal = (endPct * 1023) / 100; // Calcula o valor final do PWM com base na porcentagem fornecida pelo usuário (endPct). O valor é convertido para uma escala de 0 a 1023, que corresponde à resolução de 10 bits do PWM no ESP32. Isso permite que o teste termine com um nível de tensão/corrente específico, conforme definido pelo usuário.
    
    int pwmKneeStart = (98.0 / 100.0) * 1023; // Calcula o valor do "joelho" inicial do PWM, que representa o ponto de transição entre a parte linear e a parte não-linear do perfil de varredura. O valor é definido como 98% da escala máxima de 1023, permitindo que a varredura comece com uma tensão/corrente relativamente alta antes de entrar na região não-linear.
    int pwmKneeEnd = (94.1 / 100.0) * 1023; // Calcula o valor do "joelho" final do PWM, que representa o ponto de transição entre a parte não-linear e a parte linear do perfil de varredura. O valor é definido como 94,1% da escala máxima de 1023, permitindo que a varredura termine com uma tensão/corrente relativamente baixa após passar pela região não-linear.
    // Essas quatro linhas precisam de mais atenção!
    int kneePoints = 15; // Define o número de pontos que serão usados para a transição não-linear entre os valores de joelho inicial e final do PWM. Esses pontos representam a parte curva do perfil de varredura, permitindo uma transição suave entre os níveis de tensão/corrente.
    int tailPoints = currentTotalSteps - kneePoints; // Calcula o número de pontos restantes que serão usados para a transição linear entre o valor final do joelho e o valor final do PWM. Esses pontos representam a parte reta do perfil de varredura, permitindo que a varredura termine com uma tensão/corrente estável após passar pela região não-linear.
    
    for(int i = 0; i < kneePoints; i++) { // Loop que preenche o array pwmSweepProfile com os valores PWM correspondentes à transição não-linear entre os valores de joelho inicial e final. A fórmula usada calcula cada valor PWM com base na posição do ponto atual (i) em relação ao total de pontos de joelho, garantindo uma variação suave e gradual do PWM.
      pwmSweepProfile[i] = pwmKneeStart - i * ((pwmKneeStart - pwmKneeEnd) / (kneePoints - 1));
    }
    for(int i = 0; i < tailPoints; i++) { // Loop que preenche o array pwmSweepProfile com os valores PWM correspondentes à transição linear entre o valor final do joelho e o valor final do PWM. A fórmula usada calcula cada valor PWM com base na posição do ponto atual (i) em relação ao total de pontos restantes, garantindo uma variação suave e gradual do PWM até atingir o valor final definido pelo usuário.
      pwmSweepProfile[kneePoints + i] = pwmKneeEnd - (i + 1) * ((pwmKneeEnd - pwmEndVal) / tailPoints);
    }
    pwmStartVal = pwmSweepProfile[0]; // Atualiza o valor inicial do PWM com base no primeiro valor calculado no array pwmSweepProfile. Isso garante que o teste comece com o nível de tensão/corrente correto, conforme definido pelo perfil de varredura.
}

void handleFSM() { // Função que gerencia a máquina de estados finita (FSM) do sistema. Ela verifica o estado atual do sistema e executa as ações apropriadas com base nesse estado, como aguardar para iniciar um teste, realizar pré-condicionamento, coletar dados durante a varredura I-V, salvar os resultados no cartão SD ou entrar no modo de atualização de firmware OTA.
  static unsigned long btnPressTime = 0;// Variável estática que armazena o tempo em milissegundos desde que o botão físico foi pressionado. Ela é usada para determinar a duração da pressão do botão e decidir se o sistema deve iniciar um teste I-V, entrar no modo OTA ou ignorar a ação do botão. A variável é declarada como estática para manter seu valor entre chamadas consecutivas da função handleFSM, permitindo que o sistema acompanhe o tempo de pressão do botão mesmo quando a função é chamada repetidamente. 
  static bool isBtnPressed = false; // Variável estática que indica se o botão físico está atualmente pressionado. Ela é usada para detectar mudanças no estado do botão e determinar se o sistema deve iniciar um teste I-V, entrar no modo OTA ou ignorar a ação do botão. A variável é declarada como estática para manter seu valor entre chamadas consecutivas da função handleFSM, permitindo que o sistema acompanhe o estado do botão mesmo quando a função é chamada repetidamente.
  
  bool currentBtn = (digitalRead(0) == LOW); // Lê o estado atual do botão físico conectado ao pino 0. O botão é configurado como INPUT_PULLUP, então ele estará em nível lógico baixo (LOW) quando pressionado e em nível lógico alto (HIGH) quando não pressionado. A variável currentBtn armazena o estado atual do botão, permitindo que o sistema detecte mudanças no estado do botão e execute ações apropriadas com base na duração da pressão do botão.
  
  if (currentBtn && !isBtnPressed) { // Verifica se o botão físico foi pressionado (currentBtn é verdadeiro) e se ele não estava pressionado anteriormente (isBtnPressed é falso). Se essa condição for verdadeira, significa que o botão acabou de ser pressionado, e o sistema deve registrar o tempo da pressão do botão e atualizar o estado do botão para indicar que ele está atualmente pressionado.
    isBtnPressed = true;
    btnPressTime = millis();
    delay(50); 
  } else if (!currentBtn && isBtnPressed) { // Verifica se o botão físico foi liberado (currentBtn é falso) e se ele estava pressionado anteriormente (isBtnPressed é verdadeiro). Se essa condição for verdadeira, significa que o botão acabou de ser liberado, e o sistema deve calcular a duração da pressão do botão e executar ações apropriadas com base nessa duração, como iniciar um teste I-V, entrar no modo OTA ou ignorar a ação do botão.
    isBtnPressed = false;
    unsigned long duration = millis() - btnPressTime; // Calcula a duração da pressão do botão em milissegundos, subtraindo o tempo registrado quando o botão foi pressionado (btnPressTime) do tempo atual (millis()). A variável duration armazena o tempo total que o botão permaneceu pressionado, permitindo que o sistema determine se a ação do botão deve iniciar um teste I-V, entrar no modo OTA ou ser ignorada com base na duração da pressão.
    
    if (currentState == OTA_MODE) {
    } else if (duration >= 3000) {
      currentState = OTA_MODE;
      ledcWrite(22, 0); 
      Serial.println(">>> Entrando em modo OTA...");
      
      if (WiFi.getMode() != WIFI_AP) { // Verifica se o modo atual do WiFi não é ponto de acesso (WIFI_AP). Se essa condição for verdadeira, significa que o sistema não está atualmente em modo ponto de acesso, e ele deve desconectar-se da rede WiFi existente e configurar o ESP32 para operar como um ponto de acesso (AP) com o SSID "Tracador-IV" e a senha "admin_password". Isso permite que o usuário se conecte diretamente ao ESP32 para realizar a atualização de firmware OTA.
        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP); // Configura o ESP32 para operar como um ponto de acesso (AP), permitindo que outros dispositivos se conectem diretamente a ele. Isso é necessário para realizar a atualização de firmware OTA, pois o usuário precisa se conectar ao ESP32 para enviar o novo firmware.
        WiFi.softAP("Tracador-IV", "admin_password"); // Configura o ponto de acesso (AP) do ESP32 com o SSID "Tracador-IV" e a senha "admin_password". Isso permite que o usuário se conecte diretamente ao ESP32 usando essas credenciais para realizar a atualização de firmware OTA.
      }
      
      ArduinoOTA.setHostname(OTA_HOSTNAME); // Define o nome do host para a atualização de firmware OTA. O nome do host é usado para identificar o dispositivo na rede durante o processo de atualização, permitindo que o usuário selecione o dispositivo correto ao enviar o novo firmware.
      ArduinoOTA.setPassword(OTA_PASSWORD); // Define a senha para a atualização de firmware OTA. A senha é usada para autenticar o usuário durante o processo de atualização, garantindo que apenas usuários autorizados possam enviar o novo firmware para o dispositivo.
      ArduinoOTA.begin(); // Inicia o serviço de atualização de firmware OTA. Isso permite que o ESP32 comece a escutar por conexões de clientes que desejam enviar um novo firmware para o dispositivo, permitindo que o usuário atualize o firmware sem a necessidade de conexão física via USB.
      
      u8g2.clearBuffer();
      u8g2.drawStr(0, 15, "MODO OTA ATIVO");
      u8g2.drawStr(0, 30, "Rede: Tracador-IV");
      u8g2.drawStr(0, 45, WiFi.softAPIP().toString().c_str()); // Exibe o endereço IP do ponto de acesso (AP) do ESP32 na tela OLED. Isso permite que o usuário saiba qual endereço IP usar para se conectar ao ESP32 durante a atualização de firmware OTA, facilitando o processo de atualização.
      u8g2.sendBuffer(); // Envia o conteúdo do buffer para o display OLED, atualizando a tela com as informações sobre o modo OTA ativo, o SSID da rede e o endereço IP do ponto de acesso. Isso fornece feedback visual ao usuário sobre o estado atual do sistema e as informações necessárias para se conectar ao ESP32 durante a atualização de firmware OTA.
    } else if (duration > 50) {
      if (currentState == IDLE) { // Verifica se o estado atual do sistema é IDLE, indicando que o sistema está aguardando para iniciar um teste I-V. Se essa condição for verdadeira, significa que o botão físico foi pressionado por um período curto (mais de 50 ms, mas menos de 3 segundos), e o sistema deve iniciar um teste I-V usando os valores de zoom dinâmico definidos pelo usuário.
        // Correção Crítica: Inicializa os valores matemáticos se acionado pelo botão físico!
        calculateSweepProfile(98, 40); // Chama a função calculateSweepProfile com os valores de zoom dinâmico 98% e 40%, calculando o perfil de varredura do PWM usado para controlar a carga IGBT durante o teste I-V. Isso garante que o teste seja realizado com os níveis de tensão/corrente corretos, conforme definido pelo usuário.

        memcpy(&snapshotData, &lastReceivedData, sizeof(SensorData)); // Copia os dados recebidos via LoRa (lastReceivedData) para a variável snapshotData, criando uma cópia dos dados ambientais e de sensores no momento em que o teste I-V é iniciado. Isso permite que essas informações sejam associadas aos pontos de dados coletados durante o teste e salvos no cartão SD, garantindo que os resultados do teste possam ser analisados com base nas condições ambientais no início do teste.
        memset(ivBuffer, 0, sizeof(ivBuffer)); // Limpa o buffer ivBuffer, que armazena os pontos de dados I-V coletados durante o teste. Isso garante que o buffer esteja vazio antes de iniciar a coleta de novos dados, evitando que valores antigos interfiram nos resultados do teste atual.
        currentState = PRE_CONDITIONING; // Altera o estado atual do sistema para PRE_CONDITIONING, indicando que o sistema está prestes a iniciar a fase de pré-condicionamento da carga IGBT antes de coletar os dados de varredura I-V. Isso prepara o sistema para iniciar o teste I-V, garantindo que a carga esteja pronta para receber os sinais PWM e que os dados coletados sejam precisos e confiáveis.
        currentStep = 0; // Inicializa o passo atual do teste I-V (currentStep) para 0, indicando que o sistema está prestes a iniciar a coleta de dados a partir do primeiro ponto do perfil de varredura. Isso garante que os dados coletados durante o teste sejam armazenados na ordem correta no buffer ivBuffer, permitindo uma análise precisa dos resultados do teste.
        stateTimer = millis(); // Registra o tempo atual em milissegundos (millis()) na variável stateTimer, que é usada para controlar a duração da fase de pré-condicionamento e a coleta de dados durante o teste I-V. Isso permite que o sistema execute ações com base no tempo decorrido desde o início do teste, garantindo que cada fase do teste seja realizada pelo período correto.
        ledcWrite(22, pwmStartVal);  // Define o valor inicial do PWM (pwmStartVal) no canal 22, que é usado para controlar a carga IGBT durante o teste I-V. Isso garante que a carga receba o nível de tensão/corrente correto no início do teste, conforme definido pelo perfil de varredura calculado anteriormente.
        Serial.printf(">>> Teste I-V INICIADO via Botão (Zoom Dinamico: %d a %d)\n", pwmStartVal, pwmEndVal);
      }
    }
  }

  switch (currentState) { // Estrutura de controle switch que verifica o estado atual do sistema (currentState) e executa ações apropriadas com base nesse estado. Cada caso representa uma fase diferente do processo de teste I-V ou do modo de operação do sistema, permitindo que o sistema gerencie a execução do teste, a coleta de dados e a atualização de firmware OTA de forma organizada e eficiente.
    case IDLE:
      break;
      
    case PRE_CONDITIONING:
      if (millis() - stateTimer >= 1500) { // Verifica se 1,5 segundos se passaram desde o início da fase de pré-condicionamento. Se essa condição for verdadeira, significa que a carga IGBT foi pré-condicionada com sucesso e o sistema está pronto para iniciar a coleta de dados durante a varredura I-V.
        currentState = SWEEPING; // Altera o estado atual do sistema para SWEEPING, indicando que o sistema está prestes a iniciar a fase de coleta de dados durante a varredura I-V. Isso prepara o sistema para ler os valores de tensão e corrente do medidor PZEM-017 e armazená-los no buffer ivBuffer, permitindo que os resultados do teste sejam analisados posteriormente.
        stateTimer = millis() - 500; // Ajusta o temporizador stateTimer para iniciar a coleta de dados imediatamente, subtraindo 500 milissegundos do tempo atual (millis()). Isso garante que a primeira leitura de dados seja realizada sem atraso, permitindo que o sistema colete os valores de tensão e corrente do medidor PZEM-017 no início da fase de varredura I-V.
      }
      break;
      
    case SWEEPING:
      if (millis() - stateTimer >= 500) {
        uint8_t res = node.readInputRegisters(0x0000, 2); // Lê os registros de entrada do medidor de energia PZEM-017 usando a comunicação Modbus RTU. A função readInputRegisters envia um comando para o dispositivo Modbus para ler dois registros de entrada (endereços 0x0000 e 0x0001), que contêm os valores brutos de tensão e corrente. O resultado da leitura é armazenado na variável res, que indica se a operação foi bem-sucedida (node.ku8MBSuccess) ou se ocorreu um erro.
        ivBuffer[currentStep].v = (res == node.ku8MBSuccess) ? (node.getResponseBuffer(0) / 100.0f) * VOLTAGE_SCALE : 0;
        ivBuffer[currentStep].i = (res == node.ku8MBSuccess) ? (node.getResponseBuffer(1) / 100.0f) * CURRENT_SCALE : 0;
        ivBuffer[currentStep].pwm = pwmSweepProfile[currentStep]; // Armazena o valor PWM correspondente ao passo atual (currentStep) no buffer ivBuffer, permitindo que os dados coletados durante a varredura I-V sejam associados ao nível de tensão/corrente aplicado à carga IGBT. Isso garante que os resultados do teste possam ser analisados com base nos valores de PWM usados durante a coleta de dados.
        
        currentStep++; 
        
        if (currentStep < currentTotalSteps) { 
          ledcWrite(22, (uint32_t)pwmSweepProfile[currentStep]); // Define o valor do PWM para o próximo passo da varredura I-V, ajustando a carga IGBT de acordo com o perfil de varredura calculado anteriormente. Isso permite que o sistema colete dados de tensão e corrente em diferentes níveis de PWM, garantindo uma análise completa do comportamento da carga durante o teste I-V.
          stateTimer = millis(); 
        } else {
          ledcWrite(22, 0); 
          currentState = SAVING; 
        }
      }
      break;
      
    case SAVING: 
      saveToSD(); 
      currentState = IDLE; 
      break;
  }
}

void setup() { // Função de configuração do sistema, executada uma vez durante a inicialização do ESP32. Ela configura os pinos, inicializa a comunicação serial, o display OLED, o módulo LoRa, a conexão WiFi, o cartão SD, o módulo RTC, o PWM e a comunicação Modbus RTU com o medidor de energia PZEM-017. Além disso, ela define as rotas do servidor web e inicia a tarefa de leitura contínua dos valores do medidor PZEM-017.
  Serial.begin(115200); // Inicia a comunicação serial com o computador a uma taxa de 115200 bps, permitindo que o ESP32 envie mensagens de depuração e informações de status para o monitor serial. Isso é útil para monitorar o funcionamento do sistema, identificar problemas e verificar os resultados dos testes I-V durante o desenvolvimento e a operação do dispositivo.
  
  pinMode(21, OUTPUT); // Configura o pino 21 como saída digital, que é usado como controle de direção para a comunicação RS-485 com o medidor de energia PZEM-017. Isso permite que o ESP32 ative ou desative a transmissão de dados para o dispositivo Modbus, garantindo uma comunicação correta e evitando conflitos na linha de dados.
  digitalWrite(21, LOW); // Inicializa o pino 21 em nível lógico baixo (LOW), desativando a transmissão de dados para o medidor de energia PZEM-017. Isso garante que o dispositivo esteja pronto para receber comandos Modbus e evita problemas de comunicação durante a inicialização do sistema.
  delay(200); // Aumento de delay para estabilizar o Vext do OLED

  Wire.begin(4, 15); // Inicia a comunicação I2C no ESP32, configurando o pino 4 como SDA (Serial Data Line) e o pino 15 como SCL (Serial Clock Line). Isso permite que o ESP32 se comunique com dispositivos I2C, como o display OLED, enviando e recebendo dados de forma serializada. A função Wire.begin() inicializa a biblioteca Wire, que é usada para gerenciar a comunicação I2C no ESP32.
  u8g2.begin(); // Inicia a biblioteca U8g2, que é usada para controlar o display OLED conectado ao ESP32. A função u8g2.begin() inicializa o display, configurando os parâmetros necessários para a comunicação e preparando-o para exibir gráficos e texto. Isso permite que o sistema forneça feedback visual ao usuário sobre o estado atual do sistema, os resultados dos testes I-V e outras informações relevantes.
  u8g2.setFont(u8g2_font_6x10_tf); // Define a fonte usada pelo display OLED para exibir texto. A função u8g2.setFont() seleciona a fonte "6x10_tf", que é uma fonte de tamanho 6x10 pixels, permitindo que o sistema exiba informações legíveis e compactas na tela. Isso garante que o usuário possa visualizar facilmente os dados e mensagens exibidos no display durante a operação do sistema.
  u8g2.clearBuffer(); // Limpa o buffer de memória do display OLED, removendo qualquer conteúdo anterior que possa estar armazenado. A função u8g2.clearBuffer() prepara o display para exibir novas informações, garantindo que o conteúdo exibido seja atualizado corretamente e evitando sobreposição de texto ou gráficos. Isso é importante para fornecer feedback visual claro e preciso ao usuário durante a operação do sistema.
  u8g2.drawStr(10, 30, "Iniciando Sistema..."); // Desenha a string "Iniciando Sistema..." na posição (10, 30) do display OLED. A função u8g2.drawStr() permite que o sistema exiba mensagens de status e informações relevantes para o usuário, fornecendo feedback visual sobre o progresso da inicialização do sistema e garantindo que o usuário esteja ciente do estado atual do dispositivo.
  u8g2.sendBuffer();// Envia o conteúdo do buffer de memória do display OLED para o display físico, atualizando a tela com as informações desenhadas anteriormente. A função u8g2.sendBuffer() garante que o conteúdo exibido no display seja atualizado corretamente, permitindo que o usuário visualize as mensagens de status e informações relevantes durante a inicialização do sistema.

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS); // Inicia a comunicação SPI no ESP32, configurando os pinos LORA_SCK, LORA_MISO, LORA_MOSI e LORA_SS para se comunicar com o módulo LoRa. A função SPI.begin() inicializa a biblioteca SPI, que é usada para gerenciar a comunicação serial síncrona entre o ESP32 e o módulo LoRa, permitindo que o sistema envie e receba dados de forma eficiente.
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0); // Configura os pinos usados pelo módulo LoRa para comunicação com o ESP32. A função LoRa.setPins() define os pinos LORA_SS (Slave Select), LORA_RST (Reset) e LORA_DI0 (Interrupt) para controlar o módulo LoRa, permitindo que o sistema envie e receba dados de forma eficiente e gerencie eventos de interrupção relacionados à comunicação LoRa.
  if (LoRa.begin(LORA_BAND)) { // Inicia o módulo LoRa com a frequência especificada em LORA_BAND. A função LoRa.begin() inicializa a biblioteca LoRa, configurando os parâmetros necessários para a comunicação sem fio. Se a inicialização for bem-sucedida, o sistema imprime ">>> LoRa OK." no monitor serial, indicando que o módulo LoRa está pronto para enviar e receber dados. Caso contrário, o sistema não imprime nada, e o usuário deve verificar a conexão e a configuração do módulo LoRa.
    LoRa.setSyncWord(0x12); // Define a palavra de sincronização usada pelo módulo LoRa para identificar pacotes de dados válidos. A função LoRa.setSyncWord() configura o valor 0x12 como a palavra de sincronização, permitindo que o sistema filtre pacotes de dados recebidos e ignore pacotes inválidos ou de outros dispositivos LoRa na mesma frequência. Isso garante uma comunicação mais confiável e eficiente entre o ESP32 e outros dispositivos LoRa.
    Serial.println(">>> LoRa OK.");
  }

  u8g2.clearBuffer(); // Limpa o buffer de memória do display OLED, removendo qualquer conteúdo anterior que possa estar armazenado. A função u8g2.clearBuffer() prepara o display para exibir novas informações, garantindo que o conteúdo exibido seja atualizado corretamente e evitando sobreposição de texto ou gráficos. Isso é importante para fornecer feedback visual claro e preciso ao usuário durante a operação do sistema.
  u8g2.drawStr(0, 15, "Conectando WiFi..."); 
  u8g2.sendBuffer();

  WiFi.disconnect(true); // Desconecta o ESP32 de qualquer rede WiFi existente, garantindo que ele esteja pronto para se conectar a uma nova rede. A função WiFi.disconnect(true) remove todas as redes salvas e limpa as configurações de rede anteriores, permitindo que o sistema inicie uma nova conexão WiFi sem interferência de configurações antigas.
  WiFi.mode(WIFI_STA); // Configura o ESP32 para operar no modo estação (WIFI_STA), permitindo que ele se conecte a uma rede WiFi existente. A função WiFi.mode(WIFI_STA) prepara o sistema para se conectar a um ponto de acesso (AP) e obter um endereço IP, permitindo que o ESP32 se comunique com outros dispositivos na mesma rede e acesse recursos da Internet, como atualizações de firmware OTA.
  IPAddress local_IP(192, 168, 137, 80); // Define o endereço IP estático que o ESP32 usará ao se conectar à rede WiFi. A variável local_IP armazena o endereço IP
  IPAddress gateway(192, 168, 137, 1); // Define o endereço IP do gateway da rede WiFi, que é usado para rotear o tráfego de rede entre o ESP32 e outros dispositivos na mesma rede ou na Internet. A variável gateway armazena o endereço IP do roteador ou ponto de acesso que gerencia a rede, permitindo que o ESP32 se comunique com outros dispositivos e acesse recursos externos.
  IPAddress subnet(255, 255, 255, 0); // Define a máscara de sub-rede usada pela rede WiFi, que determina quais endereços IP pertencem à mesma rede local. A variável subnet armazena a máscara de sub-rede, permitindo que o ESP32 identifique quais dispositivos estão na mesma rede e se comunique com eles de forma eficiente. A máscara de sub-rede
  WiFi.config(local_IP, gateway, subnet); // Configura o endereço IP estático, o gateway e a máscara de sub-rede para o ESP32 ao se conectar à rede WiFi. A função WiFi.config() aplica essas configurações, garantindo que o ESP32 use o endereço IP especificado e possa se comunicar corretamente com outros dispositivos na mesma rede e acessar recursos externos, como atualizações de firmware OTA.
  
  WiFi.begin("JOÃO-VICTOR 3128", "seilansei");  // Inicia a conexão do ESP32 à rede WiFi com o SSID "JOÃO-VICTOR 3128" e a senha "seilansei". A função WiFi.begin() tenta estabelecer uma conexão com o ponto de acesso especificado, permitindo que o ESP32 se comunique com outros dispositivos na mesma rede e acesse recursos da Internet, como atualizações de firmware OTA.
  
  int attempts = 0; // Inicializa a variável attempts com o valor 0, que será usada para contar o número de tentativas de conexão à rede WiFi. A variável attempts permite que o sistema limite o tempo de espera para a conexão WiFi, evitando que o ESP32 fique preso em um loop infinito caso a conexão falhe. Isso garante que o sistema possa continuar a inicialização e fornecer feedback ao usuário sobre o status da conexão WiFi.
  // Limitando espera do WiFi a 5s para não parecer que a placa morreu
  while (WiFi.status() != WL_CONNECTED && attempts < 10) { // Loop que aguarda a conexão do ESP32 à rede WiFi. A condição do loop verifica se o status da conexão WiFi não é WL_CONNECTED (indicando que o ESP32 ainda não está conectado) e se o número de tentativas de conexão (attempts) é menor que 10. Se ambas as condições forem verdadeiras, o loop continua, permitindo que o sistema tente se conectar à rede WiFi por um período limitado de tempo (aproximadamente 5 segundos, considerando o delay de 500 ms entre as tentativas). Isso garante que o sistema não fique preso em um loop infinito caso a conexão falhe, permitindo que ele continue a inicialização e forneça feedback ao usuário sobre o status da conexão WiFi.
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  u8g2.clearBuffer(); 
  if (WiFi.status() == WL_CONNECTED) { // Verifica se o ESP32 conseguiu se conectar à rede WiFi. A condição do if verifica se o status da conexão WiFi é WL_CONNECTED, indicando que o ESP32 está conectado com sucesso à rede. Se essa condição for verdadeira, o sistema imprime ">>> WiFi Conectado!" no monitor serial e exibe a mensagem "WiFi Conectado!" e o endereço IP local na tela OLED, fornecendo feedback visual ao usuário sobre o status da conexão WiFi e permitindo que ele saiba que o sistema está pronto para se comunicar com outros dispositivos na mesma rede.
    Serial.println("\n>>> WiFi Conectado!");
    u8g2.drawStr(0, 15, "WiFi Conectado!");
    u8g2.drawStr(0, 30, WiFi.localIP().toString().c_str()); 
    ArduinoOTA.setHostname("esp32-tracador"); // Define o nome do host para a atualização de firmware OTA. O nome do host é usado para identificar o dispositivo na rede durante o processo de atualização, permitindo que o usuário selecione o dispositivo correto ao enviar o novo firmware.
    ArduinoOTA.setPassword("admin_password"); // Define a senha para a atualização de firmware OTA. A senha é usada para autenticar o usuário durante o processo de atualização, garantindo que apenas usuários autorizados possam enviar o novo firmware para o dispositivo.
    ArduinoOTA.begin(); // Inicia o serviço de atualização de firmware OTA. Isso permite que o ESP32 comece a escutar por conexões de clientes que desejam enviar um novo firmware para o dispositivo, permitindo que o usuário atualize o firmware sem a necessidade de conexão física via USB.
  } else {
    Serial.println("\n>>> Falha no WiFi.");
    u8g2.drawStr(0, 15, "Falha no WiFi");
  }
  u8g2.sendBuffer();
  delay(1000);

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); // Inicia a comunicação SPI com o cartão SD, configurando os pinos SD_SCK, SD_MISO, SD_MOSI e SD_CS para se comunicar com o cartão SD. A função spiSD.begin() inicializa a biblioteca SPI, que é usada para gerenciar a comunicação serial síncrona entre o ESP32 e o cartão SD, permitindo que o sistema leia e grave dados de forma eficiente.
  sdCardOK = SD.begin(SD_CS, spiSD); // Inicializa o cartão SD usando o pino SD_CS e a interface SPI configurada anteriormente (spiSD). A função SD.begin() verifica se o cartão SD está presente e acessível, retornando true se a inicialização for bem-sucedida e false caso contrário. O resultado da inicialização é armazenado na variável sdCardOK, permitindo que o sistema saiba se o cartão SD está pronto para uso e forneça feedback ao usuário sobre o status do cartão SD.

  rtcOK = rtc.begin(); // Inicia o módulo RTC (Real Time Clock) usando a biblioteca RTC. A função rtc.begin() verifica se o módulo RTC está presente e acessível, retornando true se a inicialização for bem-sucedida e false caso contrário. O resultado da inicialização é armazenado na variável rtcOK, permitindo que o sistema saiba se o módulo RTC está pronto para uso e forneça feedback ao usuário sobre o status do módulo RTC.
  pinMode(0, INPUT_PULLUP); // Configura o pino 0 como entrada digital com resistor pull-up interno ativado. Isso permite que o ESP32 detecte o estado do botão físico conectado ao pino 0, garantindo que ele esteja em nível lógico alto (HIGH) quando não pressionado e em nível lógico baixo (LOW) quando pressionado. A função pinMode() define o modo do pino, permitindo que o sistema leia corretamente o estado do botão físico durante a operação.
  
  ledcAttach(22, currentPwmFreq, 10); // Anexa o pino 22 ao canal PWM configurado com a frequência atual (currentPwmFreq) e uma resolução de 10 bits. A função ledcAttach() permite que o ESP32 gere sinais PWM no pino especificado, controlando a carga IGBT durante os testes I-V. A resolução de 10 bits permite que o sistema ajuste o nível de tensão/corrente aplicado à carga com precisão, garantindo resultados de teste confiáveis e consistentes.
  ledcWrite(22, 0); // Define o valor inicial do PWM no canal 22 como 0, garantindo que a carga IGBT não receba nenhum sinal PWM no início da operação. A função ledcWrite() envia o valor especificado para o canal PWM, permitindo que o sistema controle a carga IGBT de acordo com o perfil de varredura calculado durante os testes I-V.
  
  Serial2.begin(9600, SERIAL_8N1, 32, 33); // Inicia a comunicação serial no ESP32 usando a porta Serial2, configurando a taxa de transmissão para 9600 bps, o formato de dados SERIAL_8N1 (8 bits de dados, sem paridade e 1 bit de parada) e os pinos 32 (RX) e 33 (TX) para se comunicar com o medidor de energia PZEM-017. A função Serial2.begin() inicializa a porta serial, permitindo que o ESP32 envie e receba dados do dispositivo Modbus RTU, garantindo uma comunicação confiável durante os testes I-V.
  node.begin(0x01, Serial2); // Inicia a comunicação Modbus RTU com o medidor de energia PZEM-017, configurando o endereço do dispositivo como 0x01 e usando a porta Serial2 para enviar e receber dados. A função node.begin() inicializa a biblioteca ModbusMaster, permitindo que o ESP32 envie comandos Modbus para o medidor de energia e leia os valores de tensão e corrente durante os testes I-V.
  node.preTransmission(preTransmission); // Define a função preTransmission como a função de callback que será chamada antes de cada transmissão Modbus. A função preTransmission é usada para ativar o pino de direção RS-485 (pino 21) antes de enviar dados para o medidor de energia PZEM-017, garantindo que o ESP32 esteja pronto para transmitir dados e evitando conflitos na linha de comunicação. A função node.preTransmission() registra a função de callback, permitindo que o sistema gerencie corretamente a comunicação Modbus durante os testes I-V.
  node.postTransmission(postTransmission); // Define a função postTransmission como a função de callback que será chamada após cada transmissão Modbus. A função postTransmission é usada para desativar o pino de direção RS-485 (pino 21) após enviar dados para o medidor de energia PZEM-017, garantindo que o ESP32 esteja pronto para receber dados e evitando conflitos na linha de comunicação. A função node.postTransmission() registra a função de callback, permitindo que o sistema gerencie corretamente a comunicação Modbus durante os testes I-V.

  prefs.begin("pzem17", false); // Inicia a biblioteca Preferences para armazenar e recuperar dados persistentes no ESP32, usando o namespace "pzem17" e configurando o modo de leitura/gravação como false (somente leitura). A função prefs.begin() inicializa a biblioteca Preferences, permitindo que o sistema armazene e recupere valores de configuração, como escalas de tensão e corrente, entre reinicializações do dispositivo. Isso garante que as configurações do usuário sejam mantidas mesmo após desligar ou reiniciar o ESP32.
  VOLTAGE_SCALE = prefs.getFloat("v_scale", 1.0f); // Recupera o valor da escala de tensão armazenado nas preferências do ESP32, usando a chave "v_scale". Se o valor não estiver presente, a função retorna 1.0f como valor padrão. A variável VOLTAGE_SCALE armazena o fator de escala usado para ajustar os valores de tensão lidos do medidor de energia PZEM-017, garantindo que os resultados dos testes I-V sejam precisos e consistentes com as condições reais do sistema.
  CURRENT_SCALE = prefs.getFloat("i_scale", 1.0f); // Recupera o valor da escala de corrente armazenado nas preferências do ESP32, usando a chave "i_scale". Se o valor não estiver presente, a função retorna 1.0f como valor padrão. A variável CURRENT_SCALE armazena o fator de escala usado para ajustar os valores de corrente lidos do medidor de energia PZEM-017, garantindo que os resultados dos testes I-V sejam precisos e consistentes com as condições reais do sistema.
  testIndex = prefs.getInt("test_idx", 0); // Recupera o índice do teste armazenado nas preferências do ESP32, usando a chave "test_idx". Se o valor não estiver presente, a função retorna 0 como valor padrão. A variável testIndex armazena o número do teste atual, permitindo que o sistema identifique e salve os resultados dos testes I-V em arquivos separados no cartão SD, garantindo que os dados de cada teste sejam organizados e acessíveis para análise posterior.
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ // Define a rota raiz ("/") do servidor web, que responde a solicitações HTTP GET. Quando um cliente acessa a página inicial do servidor web, a função lambda é chamada, enviando uma resposta HTTP com o código de status 200 (OK), o tipo de conteúdo "text/html" e o conteúdo da página HTML definido na constante WEB_TESTE_HTML. Isso permite que os usuários acessem a interface web do sistema para visualizar informações e interagir com os testes I-V.
    request->send(200, "text/html", WEB_TESTE_HTML);
  });
  
  server.on("/ajustes", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", WEB_DASHBOARD_HTML);
  });

  server.on("/live", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{\"v\":" + String(PZEMVoltage) + ",\"i\":" + String(PZEMCurrent) + 
                  ",\"raw_v\":" + String(raw_v) + ",\"raw_i\":" + String(raw_i) + "}";
    request->send(200, "application/json", json);
  });

  server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("real_v", true)) {
      float real = request->getParam("real_v", true)->value().toFloat();
      if (raw_v > 0) {
        VOLTAGE_SCALE = real / raw_v;
        prefs.putFloat("v_scale", VOLTAGE_SCALE);
      }
    }
    if (request->hasParam("real_i", true)) {
      float real = request->getParam("real_i", true)->value().toFloat();
      if (raw_i > 0) {
        CURRENT_SCALE = real / raw_i;
        prefs.putFloat("i_scale", CURRENT_SCALE);
      }
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/set_pwm", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("val", true) && currentState == IDLE) {
      int val_pct = request->getParam("val", true)->value().toInt();
      val_pct = max(0, min(100, val_pct));
      int pwm_val = (val_pct / 100.0) * 1023; // Ajustado para 10-bits (1023) em vez de 255
      ledcWrite(22, pwm_val);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Teste em curso ou erro");
    }
  });

  server.on("/set_freq", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("val")) {
      currentPwmFreq = request->getParam("val")->value().toInt();
      ledcAttach(22, currentPwmFreq, 10); 
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing val");
    }
  });

  server.on("/start_test", HTTP_POST, [](AsyncWebServerRequest *request){
    if (currentState == IDLE) {
      int startPct = request->hasParam("start") ? request->getParam("start")->value().toInt() : 98;
      int endPct = request->hasParam("end") ? request->getParam("end")->value().toInt() : 40;
      
      calculateSweepProfile(startPct, endPct);

      memcpy(&snapshotData, &lastReceivedData, sizeof(SensorData));
      memset(ivBuffer, 0, sizeof(ivBuffer)); 
      currentState = PRE_CONDITIONING;
      currentStep = 0;
      stateTimer = millis();
      ledcWrite(22, pwmStartVal);
      Serial.printf(">>> Teste I-V INICIADO via Web Dashboard (Zoom Dinamico: %d a %d)\n", pwmStartVal, pwmEndVal);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Ocupado");
    }
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String stateStr = "IDLE";
    if (currentState == PRE_CONDITIONING || currentState == SWEEPING) stateStr = "SWEEPING";
    else if (currentState == SAVING) stateStr = "SAVING";
    else if (currentState == OTA_MODE) stateStr = "OTA";
    
    String json = "{\"state\":\"" + stateStr + "\",\"step\":" + String(currentStep) + "}";
    request->send(200, "application/json", json);
  });

  server.on("/last_test", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "[";
    for(int i=0; i<currentTotalSteps; i++) {
      json += "{\"v\":" + String(ivBuffer[i].v, 2) + ",\"i\":" + String(ivBuffer[i].i, 2) + "}";
      if(i < currentTotalSteps - 1) json += ",";
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  server.on("/download_csv", HTTP_GET, [](AsyncWebServerRequest *request){
    if (sdCardOK) {
      String fileName = "/iv_test_" + String(testIndex) + ".csv";
      if (SD.exists(fileName)) {
        request->send(SD, fileName, "text/csv", true);
      } else {
        request->send(404, "text/plain", "Nenhum teste encontrado no SD ainda.");
      }
    } else {
      request->send(500, "text/plain", "Erro no Cartao SD");
    }
  });

  server.begin();
  
  // Só inicia a task Modbus DEPOIS de todo o Setup e WiFi (evita o pin 21 desligar o OLED no boot)
  xTaskCreatePinnedToCore(taskPZEM, "TaskPZEM", 4096, NULL, 1, NULL, 0);
  Serial.println(">>> Setup concluído!");
}

void loop() {
  ArduinoOTA.handle(); 

  if (currentState == OTA_MODE) {
    return; 
  }

  receiveLoRaData(); 
  handleFSM();       
  
  static unsigned long lastOled = 0;
  if (millis() - lastOled > 500) {
    lastOled = millis();
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, currentState == IDLE ? "RECEPTOR: IDLE" : "TESTE EM CURSO");
    
    char buf[40];
    u8g2.drawStr(0, 25, "IP do Tracador:");
    u8g2.drawStr(0, 40, WiFi.localIP().toString().c_str());
    
    int currentPwm = ledcRead(22);
    int pct = (currentPwm * 100) / 1023; // Ajustado para 10-bits
    sprintf(buf, "Carga IGBT: %d%%", pct);
    u8g2.drawStr(0, 55, buf);
    
    u8g2.sendBuffer();
  }
}
