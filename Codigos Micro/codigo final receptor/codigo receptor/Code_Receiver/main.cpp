/*
 * Code_Receiver.cpp
 *
 * Created: 10/11/2023 18:10:45
 * Author : jvrom
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h> 
#include "lcd_16X2_ATMEGA328.h"

#define F_CPU 16000000
#define BAUD 9600

#define HC12_TX PD1
#define HC12_RX PD0

#define RS 1
#define E  0

char dadosRecebidos[20];
int posicaoDados = 0;

void USART_Iniciar(uint32_t baud, uint32_t freq_cpu) {
	uint16_t myubrr = freq_cpu/16/baud-1; //calcula valor do registrador UBRR
	UBRR0H = (uint8_t)(myubrr>>8); //ajusta a taxa de transmissão
	UBRR0L = (uint8_t)myubrr;
	UCSR0A = 0; //desabilitar velocidade dupla (no Arduino é habilitado por padrão)
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //habilita a transmissão e recepção. Sem interrupcao
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); //assíncrono, 8 bits, 1 bit de parada, sem paridade
}

void USART_Transmitir(unsigned char dado) {
	while (!(UCSR0A & (1 << UDRE0))); // Aguardar até que o buffer de transmissão esteja vazio
	UDR0 = dado;                       // Colocar dados no buffer de transmissão
}

unsigned char USART_Receber() {
	while (!(UCSR0A & (1 << RXC0))); // Aguardar até que dados estejam disponíveis
	return UDR0;                      // Retornar os dados recebidos
}

void HC12_Comando(const char* comando) {
	for (int i = 0; comando[i] != '\0'; ++i) {
		USART_Transmitir(comando[i]);
	}
	USART_Transmitir('\r');
}

void HC12_Configucao() {
	// Configurar pinos TX e RX
	DDRD |= (1 << HC12_TX);
	DDRD &= ~(1 << HC12_RX);

	USART_Iniciar(BAUD, F_CPU); // Configurar a taxa de baud do HC-12

	_delay_ms(1000); // Aguardar a inicialização do HC-12

	// Configurar o módulo HC-12 (ajuste os parâmetros conforme necessário)
	HC12_Comando("AT+DEFAULT"); // Restaurar configurações padrão
	_delay_ms(100);
	HC12_Comando("AT+B9600");   // Configurar taxa de baud para 9600 bps
	_delay_ms(100);
	HC12_Comando("AT+C001");    // Configurar canal 001
	_delay_ms(100);
	HC12_Comando("AT+FU3");     // Configurar modo de transmissão FU3 (transmissão rápida)
	_delay_ms(100);
}

void ProcessarDadosRecebidos(unsigned char dado) {
	if (posicaoDados < sizeof(dadosRecebidos)-1)
	{
		dadosRecebidos[posicaoDados++] = dado;
		dadosRecebidos[posicaoDados] = '\0';	
	}
}

// Interrupção de recepção
ISR(USART_RX_vect) {
	unsigned char dadoRecebido = USART_Receber();
	ProcessarDadosRecebidos(dadoRecebido);
}

void  pulso_E() {
	PORTB &= ~(1<<E);
	PORTB |= (1<<E);
	PORTB &= ~(1<<E);
	return;
}

void envia_dados(unsigned char comando) {
	PORTD = ((comando & 0xF0) | (PORTD & 0x0F));
	pulso_E();
	
	PORTD = (((comando << 4) & 0xF0) | (PORTD & 0x0F));
	pulso_E();
	return;
}

void Lcd_cmd(unsigned char comando) {
	_delay_ms(1);               // Temporizar 1ms
	PORTB &= ~(1<<RS);          // RS = 0
	envia_dados(comando);
	return;
}


void Lcd_out(char linha_lcd, char coluna_lcd, char *ponteiro) {
	//Seta endereço da DDRAM
	if((linha_lcd > 0) && (linha_lcd < 3))  // verifica se a linha é valida
	if((coluna_lcd > 0) && (coluna_lcd < 41))  //verifica se a coluna é valida
	{
		Lcd_cmd(128 + (coluna_lcd - 1) + ((linha_lcd - 1) * 64));
		//Envia o texto
		while (*ponteiro)
		{
			_delay_ms(1);               // Temporizar 1ms
			PORTB |= (1<<RS);           // RS = 1
			envia_dados(*ponteiro);
			ponteiro++;
		}
	}
	return;
}


void  Lcd_init() {
	//configura portas de saida
	DDRB = DDRB | ((1<<E) + (1<<RS));   // PB1 e PB0 como saida (RS/E)
	DDRD = DDRD | ((1<<PD4)+(1<<PD5)+(1<<PD6)+(1<<PD7));  // PD7 a PD4 Como saida (D7...D4)
	
	
	//inicialiaza o lcd
	PORTB &= ~(1<<RS);                                   // RS = 0
	_delay_ms(15);                                       // Temporizar 15 ms
	PORTD = ((0x30 & 0xF0) | (PORTD & 0x0F));            // Enviar 0x30 ao LCD
	pulso_E();                                           // Pulso no pino E
	
	_delay_ms(5);                                        // Temporizar 5 ms
	pulso_E();                                           // Pulso no pino E
	
	_delay_ms(1);                                        // Temporizar 1 ms
	pulso_E();                                           // Pulso no pino E
	
	_delay_ms(1);                                        // Temporizar 1 ms
	PORTD = ((0x20 & 0xF0) | (PORTD & 0x0F));            // Enviar 0x20 ao LCD
	pulso_E();                                           // Pulso no pino E
	
	Lcd_cmd(0x28);                                      //Especifica a interface de comunicação 4 bits, 2 linhas 5X8 pixels
	Lcd_cmd(0x08);                                      // Desliga o display
	Lcd_cmd(0x01);                                      // Limpa o diplay
	Lcd_cmd(0x06);                                      // Não desloca o display, incrementa o AC (posição do cursor)
	Lcd_cmd(DISPLAY_ON);
	
	return;

}

void AtualizarLCD() {
	 if (posicaoDados > 0) {
		 int valor = atoi(dadosRecebidos);  // Converter string para inteiro
		 char mensagem[20];
		 snprintf(mensagem, sizeof(mensagem), "Valor: %d", valor);

		 // Imprimir a mensagem no LCD
		 Lcd_out(2, 1, mensagem);

		 // Limpar os dados recebidos para a próxima iteração
		 posicaoDados = 0;
	 }
}

int main(void)
{
	HC12_Configucao();
	Lcd_init();
	sei();
	
	while (1)
	{
		AtualizarLCD();
	}
}



