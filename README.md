Sistema de Monitoramento de Vibração com ADXL345 e ESP32
Descrição
Este projeto implementa um sistema de monitoramento de vibração utilizando o sensor ADXL345 e um microcontrolador ESP32. O sistema lê dados de aceleração e giroscópio e oferece uma interface web para visualização e ajuste de parâmetros de medição. O sistema permite a configuração de filtros de ponderação e oferece funcionalidades para ajuste de tempo de exposição (Texp), que influencia o cálculo de parâmetros como ARE, AREN e Dy, baseados nas normas de vibração.

Funcionalidades
Leitura e processamento de dados de aceleração do ADXL345.
Interface web para visualização dos dados de vibração em tempo real.
Ajuste automático de offsets e calibração inicial do giroscópio.
Ajuste configurável do tempo de exposição (Texp) pela interface web.
Cálculos de parâmetros de vibração como ARE, AREN, AM, AMR, AREP, VDVR, CF, AMJ, PICO e VDVEXP.
Filtros de ponderação WB, WC, WD, WE, WF, WH, WJ, WK e WM para análise de vibração em corpo inteiro e mão/braço.
Configuração de unidade de medida entre m/s² e g.
Esquema de Ligação
Materiais Necessários
ESP32
Sensor ADXL345
Fios de conexão
Conexões
ADXL345 VCC → ESP32 3V3
ADXL345 GND → ESP32 GND
ADXL345 SDA → ESP32 GPIO 21 (SDA)
ADXL345 SCL → ESP32 GPIO 22 (SCL)
Diagrama
plaintext

             +----------------+
             |    ESP32       |
             |                |
             |   GPIO 21 (SDA)----> SDA (ADXL345)
             |   GPIO 22 (SCL)----> SCL (ADXL345)
             |      3V3--------> VCC (ADXL345)
             |      GND--------> GND (ADXL345)
             +----------------+
Bibliotecas Utilizadas
Wire.h: Para comunicação I2C.
Adafruit_ADXL345_U.h: Biblioteca para controle do sensor ADXL345.
WiFi.h: Para configuração da rede Wi-Fi.
WebServer.h: Para criação do servidor web.
Agradecimentos
Agradeço aos professores Fábio Miranda e Mauro de Mendonça Costa pelo apoio e orientação durante o desenvolvimento deste projeto.

Contato
Para mais informações, consulte o repositório do projeto no GitHub https://github.com/wellingtonbarrosrosa/aceletrometrotst/ ou entre em contato.

Sinta-se à vontade para modificar qualquer parte ou adicionar informações adicionais!

