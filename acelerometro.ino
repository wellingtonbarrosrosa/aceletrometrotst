/*
  ------------------------------------------------------------
  Projeto: Sistema de Monitoramento de Vibração com ADXL345 ESP32
  Autor: Wellington Barros Rosa
  Data de Criação: 01/10/2024
  Versão: 2.2
  Descrição:
  Este projeto implementa um sistema de monitoramento de vibração utilizando o sensor ADXL e
  um microcontrolador ESP32 para a leitura de dados de aceleração e, além de uma interface
  web para visualização e ajuste de parâmetros de medição. 

  Funcionalidades:
  - Leitura e processamento de dados de aceleração do MPU6050.
  - Interface web para visualização dos dados de vibração em tempo real.
  - Ajuste automático de offsets e calibração inicial do giroscópio.
  - Ajuste configurável do tempo de exposição (Texp) pela interface web.
  - Cálculos de parâmetros de vibração como ARE, AREN, AM, AMR, AREP, VDVR, CF, AMJ, PICO e VDVEXP.
  - Filtros de ponderação WB, WC, WD, WE, WF, WH, WJ, WK e WM para análise de vibração em corpo inteiro e mão/braço.(em desenvolvimento)
  - Configuração de unidade de medida entre m/s².
  - Realizada o corte da influencia da gravidade nos eixos

  Bibliotecas Utilizadas:
  - Wire.h: Para comunicação I2C.
  - Adafruit_ADXL345_U.h: Para controle do sensor ADXL345.
  - WiFi.h: Para configuração da rede Wi-Fi.
  - WebServer.h: Para criação do servidor web.

  Agradecimentos:
  Gostaria de expressar minha profunda gratidão aos professores Fábio Miranda e Mauro de Mendonça Costa 
  pelo apoio e orientação durante o desenvolvimento deste projeto.
*/

#include <Wire.h>
#include <Adafruit_ADXL345_U.h>  // Biblioteca para o ADXL345
#include <WiFi.h>
#include <WebServer.h>
#include <cmath> // Para usar as funções seno e cosseno
#include <arduinoFFT.h>  // Biblioteca para FFT
#include <vector> // Para usar vetores


// --------------------- CONSTANTES E DEFINIÇÕES PARA FFT ---------------------
#define SAMPLES 128                 // Número de amostras para a FFT
#define SAMPLING_FREQUENCY 1000     // Frequência de amostragem em Hz para coleta de dados

double vReal[SAMPLES];
double vImag[SAMPLES];

// Especificação explícita do tipo `double` para ArduinoFFT
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, (double)SAMPLING_FREQUENCY);

// --------------------- DEFINIÇÕES GLOBAIS ---------------------
float offsetX = 0.0; // Valor inicial para offsetX

#define UNIDADE_MS2 true // true para m/s², false para g
#define BANDA_OITAVA 1   // 1 para 1/1 oitava, 3 para 1/3 oitava
const float cutoffFreq = 0.1; // Frequência de corte (em Hz) ajustável conforme necessário
float alpha = 0.0;            // Fator de suavização do filtro, calculado com base na frequência de corte e na taxa de amostragem
float previousX = 0, previousY = 0, previousZ = 0; // Armazenar as leituras anteriores
float filteredX = 0, filteredY = 0, filteredZ = 0; // Leituras filtradas
// Definição dos fatores de multiplicação
const float f_x = 1.4;
const float f_y = 1.4;
const float f_z = 1.0;
const int NUM_LEITURAS = 100; // Número total de leituras para calcular a média
float leiturasX[NUM_LEITURAS]; // Array para armazenar as leituras de X
float leiturasY[NUM_LEITURAS]; // Array para armazenar as leituras de Y
float leiturasZ[NUM_LEITURAS]; // Array para armazenar as leituras de Z
int contadorLeituras = 0; // Contador de leituras
// Variável Texp
float Texp = 1.0;  // Valor inicial de Texp em segundos
std::vector<float> leiturasAceleracao; // Para armazenar as leituras de aceleração

// Filtros de Ponderação
enum FiltroPonderacao {WB, WC, WD, WE, WF, WH, WJ, WK, WM};
FiltroPonderacao filtroAtual = WB;

// Estruturas para Envio de Dados
struct DadosVibracao {
    float am;
    float amr;
    float are;
    float aren;
    float arep;
    float vdvr;
    float cf;
    float amj;
    float pico;
    float vdvexp;
    float frequencia;
};

DadosVibracao dados;

// --------------------- OBJETOS E INSTÂNCIAS ---------------------
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
WebServer server(80);

// --------------------- PROTÓTIPOS DE FUNÇÕES ---------------------
float calcularAREN(float are, float texp);
float calcularAM(float x, float y, float z);
float aplicarFiltroPonderacao(float valor, FiltroPonderacao filtro);
float calcularAMR(float x, float y, float z);
float calcularARE(float x, float y, float z);
float calcularAREP(float x, float y, float z);
float calcularVDVR(float x, float offset);
float calcularCF(float am, float are);
float calcularAMJ(float x, float y, float z);
float calcularPICO(float x, float y, float z);
float calcularVDVEXP(float x, float vdvr);
float calcularFrequencia(float x, float y, float z);
float calcularFrequenciaFFT();

// --------------------- CONFIGURAÇÃO ---------------------
void setup() {
    Serial.begin(115200);
    inicializarSensor();
    configurarWiFi();
    configurarServidor();
    alpha = 2 * M_PI * cutoffFreq / (2 * M_PI * cutoffFreq + SAMPLING_FREQUENCY);
    Serial.println("Servidor HTTP iniciado.");
}

// Função para inicializar o sensor
void inicializarSensor() {
    if (!adxl.begin()) {
        Serial.println("Erro ao inicializar ADXL345");
        while (1);
    }
    adxl.setRange(ADXL345_RANGE_16_G);
}

// Função para configurar Wi-Fi
void configurarWiFi() {
    const char* ssid_ap = "Acelerometro";
    const char* password_ap = "";

    const char* ssid_sta = "Suarede";
    const char* password_sta = "Suarede";

    // Inicializa o Wi-Fi no modo AP
    WiFi.softAP(ssid_ap, password_ap);
    Serial.println("Ponto de acesso iniciado.");

    // Conecta à rede Wi-Fi (station mode)
    WiFi.begin(ssid_sta, password_sta);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Conectado à rede Wi-Fi.");
}

// Função para configurar o servidor



// --------------------- Função para configurar o servidor ---------------------
void configurarServidor() {
    server.on("/", []() {
        server.send(200, "text/html", 
            "<html><head><meta charset='utf-8'><title>Monitor de Vibração</title>"
            "<style>"
            ".tab { display: none; }"
            ".tab-button { cursor: pointer; padding: 10px; background-color: #f1f1f1; border: 1px solid #ccc; display: inline-block; }"
            ".active { background-color: #ccc; }"
            "</style>"
            "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js'></script>"
            "<script>"
            "function showTab(tabId) {"
            "  $('.tab').hide();"
            "  $('#' + tabId).show();"
            "  $('.tab-button').removeClass('active');"
            "  $('#btn-' + tabId).addClass('active');"
            "} "
            "function updateData() {"
            "$.getJSON('/data', function(data) {"
            "$('#am').text(data.am.toFixed(2));"
            "$('#amr').text(data.amr.toFixed(2));"
            "$('#are').text(data.are.toFixed(2));"
            "$('#aren').text(data.aren.toFixed(2));"
            "$('#arep').text(data.arep.toFixed(2));"
            "$('#vdvr').text(data.vdvr.toFixed(2));"
            "$('#cf').text(data.cf.toFixed(2));"
            "$('#amj').text(data.amj.toFixed(2));"
            "$('#pico').text(data.pico.toFixed(2));"
            "$('#vdvexp').text(data.vdvexp.toFixed(2));"
            "$('#frequencia').text(data.frequencia.toFixed(2));"
            "});"
            "} "
            "function setTexp() {"
            "  var texp = parseFloat(document.getElementById('texp').value);"
            "  $.post('/setTexp', { texp: texp });"
            "} "
            "setInterval(updateData, 1000);"
            "showTab('dados');" // Mostra a aba de dados por padrão
            "</script></head>"
            "<body><h1>Dados do Acelerômetro</h1>"
            "<div>"
            "<div class='tab-button' id='btn-dados' onclick='showTab(\"dados\")'>Dados de Vibração</div>"
            "<div class='tab-button' id='btn-config' onclick='showTab(\"config\")'>Configurações</div>"
            "</div>"
            "<div id='dados' class='tab'>"
            "<p>Aceleração Média (AM): <span id='am'>0.00</span> m/s²</p>"
            "<p>Aceleração Média Retificada (AMR): <span id='amr'>0.00</span> m/s²</p>"
            "<p>Aceleração Resultante Efetiva (ARE): <span id='are'>0.00</span> m/s²</p>"
            "<p>Aceleração Resultante Normalizada (AREN): <span id='aren'>0.00</span> m/s²</p>"
            "<p>Aceleração Resultante de Pico (AREP): <span id='arep'>0.00</span> m/s²</p>"
            "<p>Variação Dinâmica da Velocidade Resultante (VDVR): <span id='vdvr'>0.00</span> m/s²</p>"
            "<p>Fator de Crista (CF): <span id='cf'>0.00</span></p>"
            "<p>Aceleração Máxima Instantânea (AMJ): <span id='amj'>0.00</span> m/s²</p>"
            "<p>Pico Absoluto (PICO): <span id='pico'>0.00</span> m/s²</p>"
            "<p>Valor Dose de Vibração Exponencial (VDVEXP): <span id='vdvexp'>0.00</span></p>"
            "<p>Frequência Média: <span id='frequencia'>0.00</span> Hz</p>"
            "</div>"
            "<div id='config' class='tab'>"
            "<p>Tempo de Exposição (Texp): <input type='number' id='texp' value='1.0' step='0.1'>"
            "<button onclick='setTexp()'>Atualizar</button></p>"
            "</div>"
            "</body></html>");
    });

    server.on("/data", []() {
        String json = "{\"am\":" + String(dados.am) + 
                      ",\"amr\":" + String(dados.amr) + 
                      ",\"are\":" + String(dados.are) + 
                      ",\"aren\":" + String(dados.aren) + 
                      ",\"arep\":" + String(dados.arep) + 
                      ",\"vdvr\":" + String(dados.vdvr) + 
                      ",\"cf\":" + String(dados.cf) + 
                      ",\"amj\":" + String(dados.amj) + 
                      ",\"pico\":" + String(dados.pico) + 
                      ",\"vdvexp\":" + String(dados.vdvexp) + 
                      ",\"frequencia\":" + String(dados.frequencia) + "}";
        server.send(200, "application/json", json);
    });

    server.on("/setTexp", HTTP_POST, []() {
        if (server.hasArg("texp")) {
            Texp = server.arg("texp").toFloat();
        }
        server.send(200, "text/plain", "Texp atualizado");
    });

    server.begin();
}


// --------------------- LOOP PRINCIPAL ---------------------
void loop() {
    server.handleClient();
    sensors_event_t event;
    adxl.getEvent(&event);

    // Aplicar filtro passa-alta nas leituras de aceleração
    filteredX = alpha * (filteredX + event.acceleration.x - previousX);
    filteredY = alpha * (filteredY + event.acceleration.y - previousY);
    filteredZ = alpha * (filteredZ + event.acceleration.z - previousZ);

    // Atualizar as leituras anteriores
    previousX = event.acceleration.x;
    previousY = event.acceleration.y;
    previousZ = event.acceleration.z;

    // Adicionar a leitura de aceleração ao vetor
    leiturasAceleracao.push_back(filteredX); // Você pode escolher x, y ou z conforme necessário

    // Limitar o tamanho do vetor para evitar uso excessivo de memória
    if (leiturasAceleracao.size() > 100) { // Mantenha as últimas 100 leituras
        leiturasAceleracao.erase(leiturasAceleracao.begin());
    }

    // Calcule os dados de vibração usando as leituras filtradas
    dados.am = aplicarFiltroPonderacao(calcularAM(filteredX, filteredY, filteredZ), filtroAtual);
    dados.amr = aplicarFiltroPonderacao(calcularAMR(filteredX, filteredY, filteredZ), filtroAtual);
    dados.are = aplicarFiltroPonderacao(calcularARE(filteredX, filteredY, filteredZ), filtroAtual);
    dados.aren = calcularAREN(dados.are, Texp);
    dados.arep = aplicarFiltroPonderacao(calcularAREP(filteredX, filteredY, filteredZ), filtroAtual);
    dados.vdvr = calcularVDVR(filteredX, offsetX);
    dados.amj = aplicarFiltroPonderacao(calcularAMJ(filteredX, filteredY, filteredZ), filtroAtual);
    dados.pico = calcularPICO(filteredX, filteredY, filteredZ);
    dados.cf = calcularCF(dados.pico, dados.are);
    dados.vdvexp = calcularVDVEXP(dados.vdvr); // Atualize para usar o novo cálculo


    // Calcula a frequência média usando FFT
    dados.frequencia = calcularFrequenciaFFT();
}

// --------------------- FUNÇÕES NOVAS ---------------------
float calcularFrequenciaFFT() {
    // Preenche o vetor com amostras
    for (int i = 0; i < SAMPLES; i++) {
        sensors_event_t event;
        adxl.getEvent(&event);
        
        // Use a magnitude da aceleração para a FFT
        vReal[i] = calcularARE(event.acceleration.x, event.acceleration.y, event.acceleration.z);
        vImag[i] = 0;
        delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
    }

    // Aplica a janela Hamming
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    // Encontra o pico
    double peakFrequency = FFT.majorPeak();

    // Adicione uma verificação para evitar valores de frequência inválidos
    if (peakFrequency < 0 || peakFrequency > SAMPLING_FREQUENCY / 2) {
        return 0.0; // Retorna zero se a frequência estiver fora do esperado
    }

    return peakFrequency;
}


float calcularAREN(float are, float texp) {
    return are * sqrt(texp / 8.0);
}


float calcularAM(float x, float y, float z) {
    // Armazena as leituras
    if (contadorLeituras < NUM_LEITURAS) {
        leiturasX[contadorLeituras] = x;
        leiturasY[contadorLeituras] = y;
        leiturasZ[contadorLeituras] = z;
        contadorLeituras++;
    }

    // Se já temos o número máximo de leituras, calcule a média
    if (contadorLeituras == NUM_LEITURAS) {
        float somaX = 0, somaY = 0, somaZ = 0;

        for (int i = 0; i < NUM_LEITURAS; i++) {
            somaX += leiturasX[i];
            somaY += leiturasY[i];
            somaZ += leiturasZ[i];
        }

        // Calcula a média
        float am = sqrt((somaX / NUM_LEITURAS) * (somaX / NUM_LEITURAS) +
                        (somaY / NUM_LEITURAS) * (somaY / NUM_LEITURAS) +
                        (somaZ / NUM_LEITURAS) * (somaZ / NUM_LEITURAS));

        // Reinicia o contador para a próxima série de leituras
        contadorLeituras = 0;
        return am;
    }

    // Retorna 0 se não houver leituras suficientes
    return 0.0;
}


float aplicarFiltroPonderacao(float valor, FiltroPonderacao filtro) {
    // Implementar o filtro de ponderação
    return valor; // Retorna o valor sem filtro, como exemplo
}

float calcularAMR(float x, float y, float z) {
    static float leiturasX[NUM_LEITURAS]; // Array para armazenar as leituras de X
    static float leiturasY[NUM_LEITURAS]; // Array para armazenar as leituras de Y
    static float leiturasZ[NUM_LEITURAS]; // Array para armazenar as leituras de Z
    static int contadorLeituras = 0; // Contador de leituras

    // Armazena as leituras
    if (contadorLeituras < NUM_LEITURAS) {
        leiturasX[contadorLeituras] = fabs(x); // Usa o valor absoluto
        leiturasY[contadorLeituras] = fabs(y); // Usa o valor absoluto
        leiturasZ[contadorLeituras] = fabs(z); // Usa o valor absoluto
        contadorLeituras++;
    }

    // Se já temos o número máximo de leituras, calcule a média
    if (contadorLeituras == NUM_LEITURAS) {
        float somaX = 0, somaY = 0, somaZ = 0;

        for (int i = 0; i < NUM_LEITURAS; i++) {
            somaX += leiturasX[i];
            somaY += leiturasY[i];
            somaZ += leiturasZ[i];
        }

        // Calcula a AMR
        float amr = (somaX + somaY + somaZ) / (3 * NUM_LEITURAS); // Dividido por 3 para considerar os três eixos

        // Reinicia o contador para a próxima série de leituras
        contadorLeituras = 0;
        return amr;
    }

    // Retorna 0 se não houver leituras suficientes
    return 0.0;
}

float calcularARE(float x, float y, float z) {
    // Cálculo correto da ARE
    return sqrt(x * x + y * y + z * z);
}

float calcularAREP(float x, float y, float z) {
    // Definindo os fatores de multiplicação
    const float fx = 1.4;
    const float fy = 1.4;
    const float fz = 1.0;

    // Calculando a aceleração média para os eixos
    float amx = calcularAM(x, 0, 0); // Aceleração média no eixo x
    float amy = calcularAM(0, y, 0); // Aceleração média no eixo y
    float amz = calcularAM(0, 0, z); // Aceleração média no eixo z

    // Calculando a AREP
    float arep = sqrt(pow(fx * amx, 2) + pow(fy * amy, 2) + pow(fz * amz, 2));
    return arep;
}

float calcularVDVR(float x, float offset) {
    // Implementar o cálculo de VDVR
    return x - offset;
}

float calcularCF(float pico, float are) {
    if (are != 0) { // Para evitar divisão por zero
        return pico / are;
    }
    return 0; // Retorna 0 se ARE for 0
}


float calcularAMJ(float x, float y, float z) {
    // Implementar o cálculo de AMJ
    return fmax(fabs(x), fmax(fabs(y), fabs(z)));
}

float calcularPICO(float x, float y, float z) {
    // Implementar o cálculo de PICO
    return fmax(fabs(x), fmax(fabs(y), fabs(z)));
}

float calcularVDVEXP(float vdvr) {
    float somaQuartasPotencias = 0.0;

    // Calcular a soma das quartas potências das leituras
    for (float leitura : leiturasAceleracao) {
        somaQuartasPotencias += pow(fabs(leitura), 4); // Usar a magnitude da leitura
    }

    // Se houver leituras, calcular e retornar a raiz quarta
    if (!leiturasAceleracao.empty()) {
        return pow(somaQuartasPotencias, 0.25); // Raiz quarta da soma
    }

    return 0; // Retorna 0 se não houver leituras
}
