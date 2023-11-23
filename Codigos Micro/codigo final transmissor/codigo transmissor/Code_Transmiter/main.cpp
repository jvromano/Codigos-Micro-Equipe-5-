/*
 * Code_Transmiter.cpp
 *
 * Created: 08/11/2023 23:01:21
 * Author : jvrom
 */ #include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#define F_CPU 16000000
#define BAUD 9600

#define HC12_TX PD1
#define HC12_RX PD0
#define ENCODER_PIN PD3

volatile unsigned long pulsos = 0;
unsigned long timeold = 0;
unsigned long rpm = 0;
const int pulsos_por_volta = 20;

unsigned long millis()
{
	static unsigned long millis_value = 0;
	millis_value += 1; // aumenta o tempo em 1 milissegundo a cada chamada
	_delay_ms(1);      // aguarda 1 milissegundo
	return millis_value;
}

void USART_Iniciar(uint32_t baud, uint32_t freq_cpu)
{ 
	uint16_t myubrr = freq_cpu/16/baud-1; //calcula valor do registrador UBRR
	UBRR0H = (uint8_t)(myubrr>>8); //ajusta a taxa de transmissão
	UBRR0L = (uint8_t)myubrr;
	UCSR0A = 0; //desabilitar velocidade dupla (no Arduino é habilitado por padrão)
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //habilita a transmissão e recepção. Sem interrupcao
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); //assíncrono, 8 bits, 1 bit de parada, sem paridade
}

void USART_Transmitir(unsigned char dado) {
	while (!(UCSR0A & (1 << UDRE0))); // aguardar até que o buffer de transmissão esteja vazio
	UDR0 = dado;                       // colocar dados no buffer de transmissão
}

unsigned char USART_Receber() {
	while (!(UCSR0A & (1 << RXC0))); // aguardar até que dados estejam disponíveis
	return UDR0;                      // retornar os dados recebidos
}

void HC12_Comando(const char* comando) {
	for (int i = 0; comando[i] != '\0'; ++i) {
		USART_Transmitir(comando[i]);
	}
	USART_Transmitir('\r');
}

void HC12_Configucao() {
	// configurar pinos TX e RX
	DDRD |= (1 << HC12_TX);
	DDRD &= ~(1 << HC12_RX);

	USART_Iniciar(BAUD, F_CPU); // configurar a taxa de baud do HC-12

	_delay_ms(1000); // aguardar a inicialização do HC-12

	// configurar o módulo HC-12 (ajuste os parâmetros conforme necessário)
	HC12_Comando("AT+DEFAULT"); // restaurar configurações padrão
	_delay_ms(100);
	HC12_Comando("AT+B9600");   // configurar taxa de baud para 9600 bps
	_delay_ms(100);
	HC12_Comando("AT+C001");    // configurar canal 001
	_delay_ms(100);
	HC12_Comando("AT+FU3");     // configurar modo de transmissão FU3 (transmissão rápida)
	_delay_ms(100);
}

void ENCODER_Iniciar() 
{
	DDRC &= ~(1<<ENCODER_PIN);//configura PD3 como entrada
	
	//configura interrupção extena INT1 (PD2)
	EICRA |= (1<<ISC01) | (1<<ISC00);
	EIMSK |= (1<<INT1);
}

ISR(INT1_vect)
{
	pulsos++;
}


int main(void)
{
    HC12_Configucao();
	ENCODER_Iniciar();
	
	sei(); // habilita interrupções globais
	
    while (1) 
    {
		// atualiza contador a cada segundo
		if (millis() - timeold >= 1000)
		{
			// desabilita interrupção durante o cálculo
			
			cli(); // desabilita interrupções globais

			rpm = (60 * 1000 / pulsos_por_volta) / (millis() - timeold) * pulsos;
			timeold = millis();
			pulsos = 0;

			// Envia o valor de RPM via HC-12 para o outro microcontrolador
			char buffer[10];
			// Converte o valor de rpm para uma string usando snprintf
			snprintf(buffer, sizeof(buffer), "%lu", rpm);
			HC12_Comando(buffer);
			
			sei(); // habilita interrupções globais
		}
    }
}

