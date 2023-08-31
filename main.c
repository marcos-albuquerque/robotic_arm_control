#include <avr/io.h>
#include <avr/eeprom.h> // possui várias funções para o trabalho com a EEPROM.
#include <util/delay.h>

// Definições de macros
#define set_bit(address, bit) (address |= (1 << bit))   // ativa o bit da variável address (coloca em 1)
#define clr_bit(address, bit) (address &= ~(1 << bit))  // limpo o bit da variável address (coloca em 0)
#define cpl_bit(address, bit) (address ^= (1<<bit))     // troca o estado do bit da variável address
#define tst_bit(address, bit) (address & (1<<bit))      // testa o bit da variável address (retorna 0 ou 1)

#define TOP 39999 

#define LED PB7
#define BOTAO PB4

// Botões
#define buttonRecordPositionPin0 PC0    // arduino: 37
#define buttonRecordPositionPin1 PC1    // arduino: 36
#define buttonRecordPositionPin2 PC2    // arduino: 35
#define buttonRecordPositionPin3 PC3    // arduino: 34
#define buttonRecordPositionPin4 PC4    // arduino: 33  
#define buttonRecordPositionPin5 PC5    // arduino: 32
#define buttonManualPin          PC6    // arduino: 31
#define buttonExecuteModePin     PC7    // arduino: 30


// Enumerações para identificar os servos
enum ServoIndex {
    SERVO_1, // 0
    SERVO_2,
    SERVO_3,
    SERVO_4,
    SERVO_5,
    SERVO_6
};


uint8_t currentMode = 1; // Modo atual 


// Função para iniciar o modo de leitura analógica
void ADC_Init() {
    // Configura a referência de tensão para AVCC com capacitor externo em AREF
    ADMUX = (1 << REFS0);
    
    // Habilita o ADC e definir um fator de prescaler de 128 para o clock
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Função para realizar a leitura analógica de um determinado pino
uint16_t ADC_Read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    
    ADCSRA |= (1 << ADSC);
    
    while (ADCSRA & (1 << ADSC)); // Aguarda o término da conversão
    
    return ADC;
}

// Função para configurar um timer para PWM
void setupPWMTimer(uint8_t timerNumber, uint16_t pulseWidth) {
    switch (timerNumber) {
        case 3:
            TCCR3A = (1 << WGM31);
            TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
            set_bit(TCCR3A, COM3A1);
            set_bit(TCCR3A, COM3B1);
            ICR3 = TOP;
            OCR3A = pulseWidth;
            OCR3B = pulseWidth;
            break;
        case 4:
            TCCR4A = (1 << WGM41);
            TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);
            set_bit(TCCR4A, COM4A1);
            set_bit(TCCR4A, COM4B1);
            ICR4 = TOP;
            OCR4A = pulseWidth;
            OCR4B = pulseWidth;
            break;
        case 5:
            TCCR5A = (1 << WGM51);
            TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS51);
            set_bit(TCCR5A, COM5A1);
            set_bit(TCCR5A, COM5B1);
            ICR5 = TOP;
            OCR5A = pulseWidth;
            OCR5B = pulseWidth;
            break;
        default:
            // Valor inválido
            break;
    }
}

// Função que mapeia valores de 0 a 1023 para valores de 1000 a 5000
uint16_t map(uint16_t input) {
  float mapped = ((float)input / 1023.0) * 4000.0 + 1000.0;
  return (uint16_t)mapped;
}
// --------------------------------------------------------------

// Função que define uma posição para um determinado servo
void set_servo_position(uint8_t servo, uint16_t position) {
  // Verifica se a posição está dentro da faixa permitida
  if (position < 1000) {
      position = 1000;
  } else if (position > 5000) {
      position = 5000;
  }

  // Seleciona a saída PWM correspondente ao servo
  switch (servo) {
    case 0: OCR3A = position; break;
    case 1: OCR3B = position; break;
    case 2: OCR4A = position; break;
    case 3: OCR4B = position; break;
    case 4: OCR5A = position; break;
    case 5: OCR5B = position; break;
    default: break;
  }
}

int main() { 
    uint16_t potValue1, potValue2, potValue3, potValue4, potValue5, potValue6;
    uint16_t mappedValue1, mappedValue2, mappedValue3, mappedValue4, mappedValue5, mappedValue6;

    uint16_t lastAddress = 0;           // inicializa último enderoço disponível
    uint16_t lastPosition1 = 3000;         // inicializa a última posição do servo1
    
    uint16_t data_array_s[2] = {0, 0}; // array genérico para leitura
    uint16_t data_array_s1[2] = {0, 0};
    uint16_t data_array_s2[2] = {0, 0};
    // uint16_t data_array_s3[2] = {0, 0};
    // uint16_t data_array_s4[2] = {0, 0};
    // uint16_t data_array_s5[2] = {0, 0};
    // uint16_t data_array_s6[2] = {0, 0};


    // ---------- Analog Read Init -------------
    ADC_Init();   
    
    // Configurar o pino ADC0 como entrada
    clr_bit(DDRF, PF0);
    
    // Habilitar resistor pull-up interno (PORTF para o ADC0)
    set_bit(PORTF, PF0);

    // -----------------------------------------

    // ------- Configuração dos pinos de PWM ----------
    DDRE  = 0b00011000; // Habilita os pinos OC3A e OC3B (PE3, PE4) como saídas |arduino: 5, 2
    PORTE = 0b11100111;
    DDRH  = 0b00011000; // Habilita os pinos OC4A e OC4B (PH3 e PH4) como saídas |arduino: 6, 7
    PORTH = 0b11100111;
    DDRL  = 0b00011000; // Habilita os pinos OC5A e OC5B (PL3 e PL4) como saídas |arduino: 46, 45
    PORTL = 0b11100111;

    // Teste com o led PB7 - pino 13
    DDRB  = 0b10010000; // habilida PB4 como saída para o botão
    PORTB = 0b00010000;
    // ------------------------------------------
    
    // ---- Configura Botões de controle de modos e gravação -----
    DDRC  = 0b11111111;      // Todos os pinos do PORTC como saídas
    PORTC = 0b11111111;      // pull-up
    // -----------------------------------------------------------

    // Configura os Timers 3, 4 e 5 para gerar PWM no modo fast com frequência de 50 Hz (20 ms)
    setupPWMTimer(3, 3000); // Pinos OC3A e OC3B
    setupPWMTimer(4, 3000); // Pinos OC4A e OC4B
    setupPWMTimer(5, 3000); // Pinos OC5A e OC5B

    while (1) {
        if( !tst_bit(PINC, buttonManualPin) ) {
            set_bit(PORTB, PB7);
            currentMode = 1; // Manual Mode
            lastAddress = 0;
        }
        if ( !tst_bit(PINC, buttonExecuteModePin) ) {
            clr_bit(PORTB, PB7);
            currentMode = 2; // Execute Mode 
            lastAddress = 0; // Reinicializa o último endereço para 0          
        }

        switch(currentMode) {
            case 1:
                potValue1 = ADC_Read(0);
                potValue2 = ADC_Read(1);
                potValue3 = ADC_Read(2);
                potValue4 = ADC_Read(3);
                potValue5 = ADC_Read(4);
                potValue6 = ADC_Read(5);

                // Mapeamento dos valores do potenciometro 
                mappedValue1 = map(potValue1);    
                mappedValue2 = map(potValue2);    
                mappedValue3 = map(potValue3);    
                mappedValue4 = map(potValue4);    
                mappedValue5 = map(potValue5);    
                mappedValue6 = map(potValue6);    

                // Configura a posição dos motores
                set_servo_position(0, mappedValue1);        
                set_servo_position(1, mappedValue2);        
                set_servo_position(2, mappedValue3);        
                set_servo_position(3, mappedValue4);        
                set_servo_position(4, mappedValue5);        
                set_servo_position(5, mappedValue6); 

                if( !tst_bit(PINC, buttonRecordPositionPin0) ) {
                    while( !tst_bit(PINC, buttonRecordPositionPin0) );
                    _delay_ms(10);

                    cpl_bit(PORTB, PB7);
                    data_array_s1[0] = SERVO_1;
                    data_array_s1[1] = mappedValue1;
                    eeprom_write_block(data_array_s1, (void *)lastAddress, sizeof(data_array_s1));
                    eeprom_busy_wait(); // espera até que todas as operações de gravação anteriores na EEPROM sejam concluídas antes de continuar a execução.                                          
                    
                    // limita em até 5 gravações
                    // 0, 4, 8, 12, 16, 20;
                    if(lastAddress >= 16)
                        lastAddress = 0;

                    // Calcula o próximo endereço disponível
                    lastAddress += sizeof(data_array_s1); 
                }

                if( !tst_bit(PINC, buttonRecordPositionPin1) ) {
                    while( !tst_bit(PINC, buttonRecordPositionPin1) );
                    _delay_ms(10);

                    cpl_bit(PORTB, PB7);
                    data_array_s2[0] = SERVO_2;
                    data_array_s2[1] = mappedValue2;
                    eeprom_write_block(data_array_s2, (void *)lastAddress, sizeof(data_array_s2));
                    eeprom_busy_wait(); // espera até que todas as operações de gravação anteriores na EEPROM sejam concluídas antes de continuar a execução.                                          
                    
                    // limita em até 5 gravações
                    // 0, 4, 8, 12, 16, 20;
                    if(lastAddress >= 16)
                        lastAddress = 0;

                    // Calcula o próximo endereço disponível
                    lastAddress += sizeof(data_array_s2); 
                }
                
                _delay_ms(50);

                break;
            case 2:
                eeprom_read_block(data_array_s, (const void *)lastAddress, sizeof(data_array_s));
                set_servo_position(data_array_s[0], data_array_s[1]);    

                if( lastPosition1 > data_array_s[1] ) {
                    for(uint16_t i = lastPosition1; i > data_array_s[1]; i -= 50) {
                        set_servo_position(data_array_s[0], i);
                        _delay_ms(50);
                    }
                } else {
                    for(uint16_t i = lastPosition1; i < data_array_s[1]; i += 50) {
                        set_servo_position(data_array_s[0], i);
                        _delay_ms(50);
                    }
                }
                
                lastPosition1 = data_array_s[1];
                
                if(lastAddress >= 16) 
                    lastAddress = 0;

                lastAddress += sizeof(data_array_s);
                break;
        }        

    }
}
