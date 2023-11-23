//****************** Biblioteca LCD 16X2 ATMEGA328 **********************
//    Autor: Bruno Pohlot Ricobom
//    Pinos utilizados:
//    D7...D4:	PD7...PD4
//    RS:	PB1
//    E:	PB0
//********************************************************************
//    DATA      |   Descrição
//********************************************************************
// 29/05/2021   |   Criação da biblioteca   
//              |
//	            |
//              | 
//              | 	
//              |
//              | 
//----------------------------------------------------------------------



enum{ LCD_CLEAR     = 1,
      RETURN_HOME   = 2,
      DISPLAY_ON    = 12,
      DISPLAY_OFF   = 8,
      CURSOR_ON     = 10,
      CURSOR_OFF    = 8,
      CURSOR_BLK_ON = 9,
      CURSOR_BLK_OFF= 8};
      
void pulso_E();      
void Lcd_cmd(unsigned char comando);
void envia_dados(unsigned char comando);
void Lcd_out(char linha_lcd, char coluna_lcd, char *ponteiro);
void Lcd_init();
