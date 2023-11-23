#include <SoftwareSerial.h>

// Define os pinos RX e TX do Arduino
const int rxPin = 4;  // Pino conectado ao TX do HC-12
const int txPin = 5;  // Pino conectado ao RX do HC-12

// Configura o objeto SoftwareSerial
SoftwareSerial hc12(rxPin, txPin);

void setup() {
  // Inicializa a comunicação serial com o PC
  Serial.begin(9600);

  // Inicializa a comunicação serial com o módulo HC-12
  hc12.begin(9600);

  // Aguarda alguns segundos para garantir que o módulo HC-12 esteja pronto
  delay(3000);

  hc12.println("AT"); 
  delay(100);
  hc12.println("AT+DEFAULT"); // Restaurar configurações padrão
  delay(100);
  hc12.println("AT+B9600"); // Configurar taxa de baud para 9600 bps
  delay(100);
  hc12.println("AT+C001");  // Configurar canal 001
  delay(100);
  hc12.println("AT+FU3"); // Configurar modo de transmissão FU3 (transmissão rápida)
  delay(100);

  // Aguarda a resposta do módulo HC-12
  delay(1000);

  // Verifica se há dados disponíveis para leitura
  if (hc12.available() > 0) {
    // Lê a resposta do módulo HC-12 e imprime no monitor serial
    String response = hc12.readString();
    Serial.println(response);
  } else {
    Serial.println("Nenhuma resposta recebida do módulo HC-12.");
  }
}

void loop() {
  
}
