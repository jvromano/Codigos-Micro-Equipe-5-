/*
 * teste comando at.cpp
 *
 * Created: 20/11/2023 17:25:27
 * Author : jvrom
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define F_CPU 16000000
#define BAUD 9600

#define HC12_TX PD5
#define HC12_RX PD4

void USART_Iniciar(uint32_t baud, uint32_t freq_cpu) {
	uint16_t myubrr = freq_cpu / 16 / baud - 1;
	UBRR0H = (uint8_t)(myubrr >> 8);
	UBRR0L = (uint8_t)myubrr;
	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmitir(unsigned char dado) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = dado;
}

void HC12_Comando(const char* comando) {
	for (int i = 0; comando[i] != '\0'; ++i) {
		USART_Transmitir(comando[i]);
	}
	USART_Transmitir('\r');
}

void HC12_Configuracao() {
	DDRD |= (1 << HC12_TX);
	DDRD &= ~(1 << HC12_RX);

	USART_Iniciar(BAUD, F_CPU);

	_delay_ms(1000);

	HC12_Comando("AT+DEFAULT");
	_delay_ms(100);
	HC12_Comando("AT+B9600");
	_delay_ms(100);
	HC12_Comando("AT+C001");
	_delay_ms(100);
	HC12_Comando("AT+FU3");
	_delay_ms(100);
}

int main(void) {
	HC12_Configuracao();
	sei();
	while (1) {
	}
}


