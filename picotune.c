#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"


//// Configuração do ADC ////
// Canal do ADC e pino
#define ADC_CHAN 2
#define ADC_PIN 28
// Número de amostras por FFT
#define NUM_SAMPLES 1024
// Taxa de amostragem (Hz)
#define Fs 10000.0
// Taxa de clock do ADC (imutável!)
#define ADCCLK 480000.0

// Canais DMA para amostragem do ADC
int sample_chan;
int control_chan;

// Aqui é onde o canal DMA colocará as amostras do ADC
uint8_t sample_array[NUM_SAMPLES];

// Ponteiro para o endereço do início do buffer de amostras
uint8_t* sample_address_pointer = &sample_array[0];

int main()
{
    // Inicializa o stdio
    stdio_init_all();

    ////
    // ==== CONFIGURAÇÃO DO ADC ====
    ////
    // Inicializa o hardware do ADC
    // (reseta, habilita o clock, espera até que o hardware esteja pronto)
    adc_init();

    // Inicializa o GPIO para uso analógico: alta impedância, sem pull-ups/pull-downs, desabilita o buffer de entrada digital.
    adc_gpio_init(ADC_PIN);

    // Seleciona a entrada do multiplexador analógico (0...3 são GPIO 26, 27, 28, 29; 4 é o sensor de temperatura)
    adc_select_input(ADC_CHAN);

    // Configura o FIFO
    adc_fifo_setup(
    true,    // Escreve cada conversão concluída no FIFO de amostras
    true,    // Habilita a solicitação de dados DMA (DREQ)
    1,       // DREQ (e IRQ) ativado quando pelo menos 1 amostra estiver presente
    false,   // Não veremos o bit ERR devido às leituras de 8 bits; desabilitado.
    true     // Desloca cada amostra para 8 bits ao empurrar para o FIFO
    );

    // Divisor de 0 -> velocidade máxima. A captura em execução contínua com o divisor
    // é equivalente a pressionar o botão ADC_CS_START_ONCE uma vez a cada `div + 1`
    // ciclos (div não necessariamente um inteiro). Cada conversão leva 96
    // ciclos, então, em geral, você quer um divisor de 0 (segurar o botão
    // continuamente) ou > 95 (capturar amostras com menos frequência que intervalos de 96 ciclos).
    // Tudo isso é cronometrado pelo clock do ADC de 48 MHz. Configurado para capturar
    // uma amostra a 10kHz (48Mhz/10kHz - 1)
    adc_set_clkdiv(ADCCLK/Fs);


    ////
    // ==== CONFIGURAÇÃO DO DMA DO ADC ====
    ////

    sample_chan = dma_claim_unused_channel(true);
    control_chan = dma_claim_unused_channel(true);

    // Configurações dos canais
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);


    // CANAL DE AMOSTRAS DO ADC
    // Lendo de um endereço constante, escrevendo para endereços de bytes incrementais
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Ritma as transferências com base na disponibilidade de amostras do ADC
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configura o canal
    dma_channel_configure(sample_chan,
    &c2,    // configuração do canal
    sample_array,   // destino
    &adc_hw->fifo,  // origem
    NUM_SAMPLES,    // contagem de transferências
    false    // não inicia imediatamente
    );

    // CANAL DE CONTROLE
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);    // transferências de 32 bits
    channel_config_set_read_increment(&c3, false);    // sem incremento de leitura
    channel_config_set_write_increment(&c3, false);    // sem incremento de escrita
    channel_config_set_chain_to(&c3, sample_chan);    // encadeia para o canal de amostras

    dma_channel_configure(
    control_chan,    // Canal a ser configurado
    &c3,    // A configuração que acabamos de criar
    &dma_hw->ch[sample_chan].write_addr,  // Endereço de escrita (endereço de leitura do canal 0)
    &sample_address_pointer,    // Endereço de leitura (PONTEIRO PARA UM ENDEREÇO)
    1,    // Número de transferências, neste caso cada uma é de 4 bytes
    false    // Não inicia imediatamente.
    );

    printf("Iniciando captura\n");
    // Inicia o canal do ADC
    dma_start_channel_mask((1u << sample_chan));
    // Inicia o ADC
    adc_run(true);


    while (true) {
    
    // Aguarda o DMA terminar
    dma_channel_wait_for_finish_blocking(sample_chan);
    // Reseta os ponteiros do DMA
    // dma_channel_set_read_addr(control_chan, &sample_address_pointer, false);
    // Reinicia o DMA
    // dma_channel_start(control_chan);

    printf("----\n");
    // Imprime as primeiras 10 amostras
    for (int i = 0; i < NUM_SAMPLES; i++) {
    printf("%d\n", sample_array[i]);
    }
    printf("----\n");

    // Reinicia o canal de amostras, agora que temos nossa cópia das amostras
    dma_channel_start(control_chan);

    // Dorme por 1 segundo
    sleep_ms(1000);
    
    }

}