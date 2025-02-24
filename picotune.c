#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
//// Bibliotecas do display OLED ////
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include <string.h>

#include <math.h>


typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)


//// Configuração do ADC ////
// Canal do ADC e pino
#define ADC_CHAN 2
#define ADC_PIN 28
// Número de amostras por FFT
#define NUM_SAMPLES 1024
// Taxa de amostragem (Hz)
#define Fs 10000.0
// Taxa de clock do ADC (imutável!)
#define ADCCLK 48000000.0

// Canais DMA para amostragem do ADC
int sample_chan;
int control_chan;

// Aqui é onde o canal DMA colocará as amostras do ADC
uint8_t sample_array[NUM_SAMPLES];

// Ponteiro para o endereço do início do buffer de amostras
uint8_t* sample_address_pointer = &sample_array[0];

//// Configuração do display LED ////
// O Display OLED está conectado ao barramento I2C da BitDogLab
// através dos seguintes pinos:
// SDA: GPIO14
// SCL: GPIO15

#define I2C_SDA 14
#define I2C_SCL 15

//// Configuração do FFT ////
// Número de pontos da FFT menos 1
#define NUM_SAMPLES_M_1 1023
// log2 do numero de amostras coletadas
#define LOG2_NUM_SAMPLES 10
// tamanho do short (16 bits) menos log2 do número de amostras
#define SHIFT_AMOUNT 6


// array com a parte real das amostras
fix15 fr[NUM_SAMPLES];
// array com a parte imaginária das amostras
fix15 fi[NUM_SAMPLES];

fix15 Sinewave[NUM_SAMPLES];
fix15 window[NUM_SAMPLES];
// 0.4 in fixed point (used for alpha max plus beta min)
fix15 zero_point_4 = float2fix15(0.4) ;

// Max and min macros
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

//// Configuração da Máquina de Estado Finita (FSM) ////
#define BUTTON_A 5
#define LED_GREEN 11
#define LED_RED 13

// Estado para gerenciar os botões
typedef enum {
    IDLE,
    DO,
    RE,
    MI,
    FA,
    SOL,
    LA,
    SI
} State;

void FFTfix(fix15 fr[], fix15 fi[]) {

    unsigned short m;   // one of the indices being swapped
    unsigned short mr ; // the other index being swapped (r for reversed)
    fix15 tr, ti ; // for temporary storage while swapping, and during iteration

    int i, j ; // indices being combined in Danielson-Lanczos part of the algorithm
    int L ;    // length of the FFT's being combined
    int k ;    // used for looking up trig values from sine table

    int istep ; // length of the FFT which results from combining two FFT's

    fix15 wr, wi ; // trigonometric values from lookup table
    fix15 qr, qi ; // temporary variables used during DL part of the algorithm

    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// BIT REVERSAL //////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Bit reversal code below based on that found here: 
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
    for (m=1; m<NUM_SAMPLES_M_1; m++) {
        // swap odd and even bits
        mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ... 
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= SHIFT_AMOUNT ;
        // don't swap that which has already been swapped
        if (mr<=m) continue ;
        // swap the bit-reveresed indices
        tr = fr[m] ;
        fr[m] = fr[mr] ;
        fr[mr] = tr ;
        ti = fi[m] ;
        fi[m] = fi[mr] ;
        fi[mr] = ti ;
    }

    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// Danielson-Lanczos //////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    L = 1 ;
    // Log2 of number of samples, minus 1
    k = LOG2_NUM_SAMPLES - 1 ;
    // While the length of the FFT's being combined is less than the number of gathered samples
    while (L < NUM_SAMPLES) {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L<<1 ;
        // For each element in the FFT's that are being combined . . .
        for (m=0; m<L; ++m) { 
            // Lookup the trig values for that element
            j = m << k ;                         // index of the sine table
            wr =  Sinewave[j + NUM_SAMPLES/4] ; // cos(2pi m/N)
            wi = -Sinewave[j] ;                 // sin(2pi m/N)
            wr >>= 1 ;                          // divide by two
            wi >>= 1 ;                          // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i=m; i<NUM_SAMPLES; i+=istep) {
                // j gets the index of the FFT element being combined with i
                j = i + L ;
                // compute the trig terms (bottom half of the above matrix)
                tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]) ;
                ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]) ;
                // divide ith index elements by two (top half of above matrix)
                qr = fr[i]>>1 ;
                qi = fi[i]>>1 ;
                // compute the new values at each index
                fr[j] = qr - tr ;
                fi[j] = qi - ti ;
                fr[i] = qr + tr ;
                fi[i] = qi + ti ;
            }    
        }
        --k ;
        L = istep ;
    }
}

int calculate_fundamental_freq(){
        static fix15 max_fr ;           // temporary variable for max freq calculation
        static int max_fr_dex ;         // index of max frequency

        // Aguarda o DMA terminar
        dma_channel_wait_for_finish_blocking(sample_chan);
        // Reseta os ponteiros do DMA
        // dma_channel_set_read_addr(control_chan, &sample_address_pointer, false);
        // Reinicia o DMA
        // dma_channel_start(control_chan);

        // Copy/window elements into a fixed-point array
        for (int i=0; i<NUM_SAMPLES; i++) {
            fr[i] = multfix15(int2fix15((int)sample_array[i]), window[i]) ;
            fi[i] = (fix15) 0 ;
        }

        // Zero max frequency and max frequency index
        max_fr = 0 ;
        max_fr_dex = 0 ;

        // Reinicia o canal de amostras, agora que temos nossa cópia das amostras
        dma_channel_start(control_chan);

        
        // Compute the FFT
        FFTfix(fr, fi) ;

        // Find the magnitudes (alpha max plus beta min)
        for (int i = 0; i < (NUM_SAMPLES>>1); i++) {  
            // get the approx magnitude
            fr[i] = abs(fr[i]); 
            fi[i] = abs(fi[i]);
            // reuse fr to hold magnitude
            fr[i] = max(fr[i], fi[i]) + 
                    multfix15(min(fr[i], fi[i]), zero_point_4); 

            // Keep track of maximum
            if (fr[i] > max_fr && i>4) {
                max_fr = fr[i] ;
                max_fr_dex = i ;
            }
        }
        // Compute max frequency in Hz
        float max_freqency = max_fr_dex * (Fs/NUM_SAMPLES) ;

        return max_freqency;
        }

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

    // Inicia o canal do ADC
    dma_start_channel_mask((1u << sample_chan));
    // Inicia o ADC
    adc_run(true);

    // OLED
    // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    //  Processo de inicialização do OLED SSD1306
    ssd1306_init();

    // Preparar área de renderização para o display 
    // (ssd1306_width pixels por ssd1306_n_pages páginas)
    struct render_area frame_area = {
        start_column: 0,
        end_column: ssd1306_width - 1,
        start_page: 0,
        end_page: ssd1306_n_pages - 1
    };

    calculate_render_area_buffer_length(&frame_area);

    // zera o display inteiro
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);
    printf("Iniciando captura\n");

    char text_freq[50];
    char text_nota[50];

    
    char *text[] = {
        "  Iniciando   ",
        "  captura dos ",
        "    samples   "};

    int y = 0;
    for (uint i = 0; i < count_of(text); i++)
    {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);

    // FFT
    // Declare some static variables
    static int height ;             // for scaling display
    static float max_freqency ;     // holds max frequency
    static int i ;                  // incrementing loop variable

    static fix15 max_fr ;           // temporary variable for max freq calculation
    static int max_fr_dex ;         // index of max frequency

    // Populate the sine table and Hann window table
    int ii;
    for (ii = 0; ii < NUM_SAMPLES; ii++) {
        Sinewave[ii] = float2fix15(sin(6.283 * ((float) ii) / (float)NUM_SAMPLES));
        window[ii] = float2fix15(0.5 * (1.0 - cos(6.283 * ((float) ii) / ((float)NUM_SAMPLES))));
    }

    ////
    // ==== CONFIGURAÇÃO DA FSM ====
    ////
    static State s = IDLE;
    int buttonState = gpio_get(BUTTON_A);
    int edge;
    int buttonNow;
    gpio_set_function(BUTTON_A, GPIO_FUNC_SIO); // Set GPIO 5 Button A
    gpio_set_dir(BUTTON_A, GPIO_IN); // Set GPIO 5 to input
    gpio_pull_up(BUTTON_A);

    gpio_set_function(LED_GREEN, GPIO_FUNC_SIO); // Set GPIO 12 to SIO
    gpio_set_dir(LED_GREEN, GPIO_OUT); // Set GPIO 15 to output
    gpio_pull_up(LED_GREEN); // Enable pull-up on GPIO 12
    gpio_put(LED_GREEN, 0); // Set GPIO 12 to low

    gpio_set_function(LED_RED, GPIO_FUNC_SIO); // Set GPIO 12 to SIO
    gpio_set_dir(LED_RED, GPIO_OUT); // Set GPIO 15 to output
    gpio_pull_up(LED_RED); // Enable pull-up on GPIO 12
    gpio_put(LED_RED, 0); // Set GPIO 12 to low

    int freq_desejada = 0;

    while (true) {
        
        // Configuração do botão A de controle da FSM
        buttonNow = !gpio_get(BUTTON_A);
        edge = buttonState - buttonNow;
        buttonState = buttonNow;

        max_freqency = calculate_fundamental_freq();

        switch (s)
        {
            case IDLE:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                char *text[] = {
                    "   Aperte o   ",
                    " botao  A para",
                    " selecionar a  ",
                    "  nota musical "};

                int y = 0;
                for (uint i = 0; i < count_of(text); i++)
                {
                    ssd1306_draw_string(ssd, 5, y, text[i]);
                    y += 8;
                }
                render_on_display(ssd, &frame_area);

                if (edge == 1)
                {
                    s = DO;
                }
                break;
            case DO:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                freq_desejada = 263;
                
                strcpy(text_nota,"Afinando o DO");
                y = 0;
                ssd1306_draw_string(ssd, 5, y, text_nota);
                
                sprintf(text_freq, "Freq %d Hz",(int)max_freqency);
                y = 16;
                ssd1306_draw_string(ssd, 5, y, text_freq);
                render_on_display(ssd, &frame_area);

                if (max_freqency > freq_desejada - 5 && max_freqency < freq_desejada + 5)
                {
                    gpio_put(LED_GREEN, 1); 
                } else {
                    gpio_put(LED_GREEN, 0);
                    gpio_put(LED_RED, 1); 
                }

                if (edge == 1)
                {
                    s = RE;
                }
                break;
            case RE:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                freq_desejada = 300;
                
                strcpy(text_nota, "Afinando o RE");
                y = 0;
                ssd1306_draw_string(ssd, 5, y, text_nota);
                
                sprintf(text_freq, "Freq %d Hz",(int)max_freqency);
                y = 16;
                ssd1306_draw_string(ssd, 5, y, text_freq);
                render_on_display(ssd, &frame_area);

                if (max_freqency > freq_desejada - 5 && max_freqency < freq_desejada + 5)
                {
                    gpio_put(LED_GREEN, 1); 
                } else {
                    gpio_put(LED_GREEN, 0);
                    gpio_put(LED_RED, 1); 
                }

                if (edge == 1)
                {
                    s = MI;
                }
                break;
            case MI:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                freq_desejada = 330;
                
                strcpy(text_nota, "Afinando o MI");
                y = 0;
                ssd1306_draw_string(ssd, 5, y, text_nota);
                
                sprintf(text_freq, "Freq %d Hz",(int)max_freqency);
                y = 16;
                ssd1306_draw_string(ssd, 5, y, text_freq);
                render_on_display(ssd, &frame_area);

                if (max_freqency > freq_desejada - 5 && max_freqency < freq_desejada + 5)
                {
                    gpio_put(LED_GREEN, 1); 
                } else {
                    gpio_put(LED_GREEN, 0);
                    gpio_put(LED_RED, 1); 
                }

                if (edge == 1)
                {
                    s = FA;
                }
                break;
            case FA:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                freq_desejada = 352;
                
                strcpy(text_nota, "Afinando o FA");
                y = 0;
                ssd1306_draw_string(ssd, 5, y, text_nota);
                
                sprintf(text_freq, "Freq %d Hz",(int)max_freqency);
                y = 16;
                ssd1306_draw_string(ssd, 5, y, text_freq);
                render_on_display(ssd, &frame_area);

                if (max_freqency > freq_desejada - 5 && max_freqency < freq_desejada + 5)
                {
                    gpio_put(LED_GREEN, 1); 
                } else {
                    gpio_put(LED_GREEN, 0);
                    gpio_put(LED_RED, 1); 
                }

                if (edge == 1)
                {
                    s = SOL;
                }
                break;
            case SOL:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                freq_desejada = 396;
                
                strcpy(text_nota, "Afinando o SOL");
                y = 0;
                ssd1306_draw_string(ssd, 5, y, text_nota);
                
                sprintf(text_freq, "Freq %d Hz",(int)max_freqency);
                y = 16;
                ssd1306_draw_string(ssd, 5, y, text_freq);
                render_on_display(ssd, &frame_area);

                if (max_freqency > freq_desejada - 5 && max_freqency < freq_desejada + 5)
                {
                    gpio_put(LED_GREEN, 1); 
                } else {
                    gpio_put(LED_GREEN, 0);
                    gpio_put(LED_RED, 1); 
                }

                if (edge == 1)
                {
                    s = LA;
                }
                break;
            case LA:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                freq_desejada = 440;
                
                strcpy(text_nota, "Afinando o LA");
                y = 0;
                ssd1306_draw_string(ssd, 5, y, text_nota);
                
                sprintf(text_freq, "Freq %d Hz",(int)max_freqency);
                y = 16;
                ssd1306_draw_string(ssd, 5, y, text_freq);
                render_on_display(ssd, &frame_area);

                if (max_freqency > freq_desejada - 5 && max_freqency < freq_desejada + 5)
                {
                    gpio_put(LED_GREEN, 1); 
                } else {
                    gpio_put(LED_GREEN, 0);
                    gpio_put(LED_RED, 1); 
                }

                if (edge == 1)
                {
                    s = SI;
                }
                break;
            case SI:
                memset(ssd, 0, ssd1306_buffer_length);
                render_on_display(ssd, &frame_area);
                freq_desejada = 495;
                
                strcpy(text_nota, "Afinando o SI");
                y = 0;
                ssd1306_draw_string(ssd, 5, y, text_nota);
                
                sprintf(text_freq, "Freq %d Hz",(int)max_freqency);
                y = 16;
                ssd1306_draw_string(ssd, 5, y, text_freq);
                render_on_display(ssd, &frame_area);

                if (max_freqency > freq_desejada - 5 && max_freqency < freq_desejada + 5)
                {
                    gpio_put(LED_GREEN, 1); 
                } else {
                    gpio_put(LED_GREEN, 0);
                    gpio_put(LED_RED, 1); 
                }

                if (edge == 1)
                {
                    s = IDLE;
                }
                break;
            default:
                s = IDLE;
        }
        
        sleep_ms(100);
    
    }

}