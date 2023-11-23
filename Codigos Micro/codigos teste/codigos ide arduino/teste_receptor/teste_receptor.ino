#include <LiquidCrystal.h>
#include <SoftwareSerial.h>


// Pinos do módulo HC-12
const int hc12Tx = 10;
const int hc12Rx = 11;

// Configuração da comunicação serial com o módulo HC-12
SoftwareSerial hc12(hc12Rx, hc12Tx);

// Configuração do display LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // Pinos RS, EN, D4, D5, D6, D7

void setup() {
  Serial.begin(9600);
  hc12.begin(9600);

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
  
  lcd.begin(16, 2);

  lcd.print("Recebendo dados");
}

void loop() {
  if (hc12.available() > 0) {
    // Se há dados disponíveis no módulo HC-12
    String receivedData = hc12.readStringUntil('\n');

    // Limpa o display LCD
    lcd.clear();

    // Exibe os dados recebidos no display LCD
    lcd.setCursor(0, 0);
    lcd.print("Dados Recebidos:");
    lcd.setCursor(0, 1);
    lcd.print(receivedData);

    // Aguarda um pouco antes de limpar o display novamente
    delay(5000);
  }
  lcd.setCursor(0, 0);
  lcd.print("Sem dados");
  delay(1000);
  lcd.clear();
}
