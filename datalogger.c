#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pico/bootrom.h"
#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "lib/FatFs_SPI/ssd1306.h"
#include "hardware/i2c.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"
#include <math.h>
#include "pico/binary_info.h"

// =============================================
// DEFINIÇÕES DE CONSTANTES E PINOS
// =============================================
#define ADC_PIN 26               // GPIO 26 para ADC
#define I2C_PORT i2c0            // Porta I2C principal
#define I2C_SDA 0                // Pino SDA I2C
#define I2C_SCL 1                // Pino SCL I2C
#define I2C_DISPLAY i2c1         // Porta I2C para display
#define PIN_I2C_SDA_DISPLAY 14   // Pino SDA para display
#define PIN_I2C_SCL_DISPLAY 15   // Pino SCL para display
#define led_red 13               // LED Vermelho (GPIO 13)
#define led_blue 12              // LED Azul (GPIO 12)
#define led_green 11             // LED Verde (GPIO 11)
#define buttonA 5                // Botão A (GPIO 5)
#define buttonB 6                // Botão B (GPIO 6)
#define BUZZER_PIN 10            // Pino do buzzer (GPIO 10)
#define REST 0                   // Define repouso para o buzzer
#define WIDTH 128                // Largura do display OLED
#define HEIGHT 64                // Altura do display OLED

// =============================================
// VARIÁVEIS GLOBAIS
// =============================================
volatile bool toggle_sd_requested = false;  // Flag para solicitação de toggle SD
volatile bool logger_enabled = false;       // Flag para habilitar logger
bool montado = false;                       // Status de montagem do SD
bool borda = true;                          // Configuração de borda do display
bool recording = false;                     // Flag para gravação em andamento
ssd1306_t ssd;                              // Objeto do display OLED
static const uint32_t period = 1000;        // Período para operações periódicas
static absolute_time_t next_log_time;       // Tempo para próximo log
static char filename[20] = "imu_data.csv";  // Nome do arquivo de dados
static int addr = 0x68;                     // Endereço I2C do MPU6050

// =============================================
// PROTÓTIPOS DE FUNÇÕES
// =============================================

// Funções de hardware
void set_led_color(const char *estado);
void buzzer_play_note(int freq, int duration_ms);
void beep(int count);
void buzzer_init();
void led_init(int led);
void button_init(int button);
void display_init();

// Funções do MPU6050
static void mpu6050_reset();
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

// Funções do sistema de arquivos
bool is_sd_mounted();
static sd_card_t *sd_get_by_name(const char *const name);
static FATFS *sd_get_fs_by_name(const char *name);
void capture_mpu6050_data_and_save();
void read_file(const char *filename);

// Funções de comandos
static void run_setrtc();
static void run_format();
static void run_mount();
static void run_unmount();
static void run_getfree();
static void run_ls();
static void run_cat();
static void run_help();

// Funções auxiliares
void debounce(uint gpio, uint32_t events);
static void process_stdio(int cRxedChar);

// Estrutura para comandos
typedef void (*p_fn_t)();
typedef struct {
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"}
};

// =============================================
// FUNÇÃO PRINCIPAL
// =============================================
int main() {
    // Inicialização do hardware
    display_init();
    button_init(buttonB);
    button_init(buttonA);
    buzzer_init();
    led_init(led_blue);
    led_init(led_red);
    led_init(led_green);

    // Configuração inicial do display
    ssd1306_fill(&ssd, !borda);
    ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
    ssd1306_draw_string(&ssd, "Iniciando...", 10, 30);
    ssd1306_send_data(&ssd);

    // Configuração de interrupções para botões
    gpio_set_irq_enabled_with_callback(buttonA, GPIO_IRQ_EDGE_FALL, true, &debounce);
    gpio_set_irq_enabled(buttonB, GPIO_IRQ_EDGE_FALL, true);

    // Inicialização do I2C para comunicação com MPU6050
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Configuração inicial do sistema
    set_led_color("init");
    stdio_init_all();
    sleep_ms(5000);
    time_init();
    
    // Limpa os LEDs e a tela do terminal
    gpio_put(led_blue, 0);
    gpio_put(led_red, 0);
    gpio_put(led_green, 0);
    printf("FatFS SPI example\n");
    printf("\033[2J\033[H"); // Limpa tela
    printf("\n> ");
    stdio_flush();
    
    run_help(); // Exibe os comandos disponíveis

    // Loop principal
    while (true) {
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);

        if (toggle_sd_requested) {
            toggle_sd_requested = false;
            set_led_color("init");
            if (is_sd_mounted()) {
                ssd1306_fill(&ssd, !borda);
                ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
                ssd1306_draw_string(&ssd, "Desmontando", 10, 20);
                ssd1306_draw_string(&ssd, "SD...", 30, 30);
                ssd1306_send_data(&ssd);
                printf("\nDesmontando SD via botão B...\n");
                sleep_ms(1000);
                run_unmount();
                sleep_ms(1000);
            } else {
                ssd1306_fill(&ssd, !borda);
                ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
                ssd1306_draw_string(&ssd, "Montando", 10, 20);
                ssd1306_draw_string(&ssd, "SD...", 30, 30);
                ssd1306_send_data(&ssd);
                printf("\nMontando SD via botão B...\n");
                sleep_ms(1000);
                run_mount();
                sleep_ms(1000);
            }
        }
        else if (cRxedChar == 'c') {
            ssd1306_fill(&ssd, !borda);
            ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
            ssd1306_draw_string(&ssd, "Exibindo", 10, 20);
            ssd1306_draw_string(&ssd, "arquivos...", 30, 30);
            ssd1306_send_data(&ssd);
            buzzer_play_note(1200, 80);
            sleep_ms(1000);
            printf("\nListagem de arquivos no cartão SD.\n");
            run_ls();
            set_led_color("sd_rw");
            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (h = help):  ");
            sleep_ms(1000);
        }
        else if (cRxedChar == 'd') {
            ssd1306_fill(&ssd, !borda);
            ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
            ssd1306_draw_string(&ssd, "Exibindo", 10, 20);
            ssd1306_draw_string(&ssd, "arquivo...", 30, 30);
            ssd1306_send_data(&ssd);
            buzzer_play_note(1200, 80);
            sleep_ms(1000);
            read_file(filename);
            set_led_color("sd_rw");
            sleep_ms(1000);
            printf("Escolha o comando (h = help):  ");
        }
        else if (cRxedChar == 'e') {
            ssd1306_fill(&ssd, !borda);
            ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
            ssd1306_draw_string(&ssd, "Verificando", 10, 20);
            ssd1306_draw_string(&ssd, "espaco...", 30, 30);
            ssd1306_send_data(&ssd);
            buzzer_play_note(1200, 80);
            printf("\nObtendo espaço livre no SD.\n\n");
            sleep_ms(1000);
            run_getfree();
            set_led_color("sd_rw");
            sleep_ms(1000);
            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        else if (logger_enabled && !recording) {
            recording = true;
            capture_mpu6050_data_and_save();
            recording = false;
            sleep_ms(1000);
        }
        else if (cRxedChar == 'g') {
            ssd1306_fill(&ssd, !borda);
            ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
            ssd1306_draw_string(&ssd, "Formatando...", 10, 20);
            ssd1306_send_data(&ssd);
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            sleep_ms(1000);
            run_format();
            sleep_ms(1000);
            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        else if (cRxedChar == 'h') {
            run_help();
        }
        else {
            if (montado) {
                set_led_color("pronto");
                ssd1306_fill(&ssd, !borda);
                ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
                ssd1306_draw_string(&ssd, "Aguardando", 10, 20);
                ssd1306_draw_string(&ssd, "comando...", 30, 30);
                ssd1306_send_data(&ssd);
            } else {
                gpio_put(led_green, 0);
                gpio_put(led_red, 0);
                gpio_put(led_blue, 0);
                ssd1306_fill(&ssd, !borda);
                ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);
                ssd1306_draw_string(&ssd, "Aguardando", 10, 20);
                ssd1306_draw_string(&ssd, "Montagem...", 30, 30);
                ssd1306_send_data(&ssd);
            }
        }
        sleep_ms(500);
    }
    return 0;
}

// =============================================
// IMPLEMENTAÇÕES DAS FUNÇÕES
// =============================================

bool is_sd_mounted()
{
    sd_card_t *pSD = sd_get_by_num(0);
    return (pSD && pSD->mounted);
}
void set_led_color(const char *estado)
{
    // Desliga todos os LEDs antes
    gpio_put(led_red, 0);
    gpio_put(led_green, 0);
    gpio_put(led_blue, 0);

    if (strcmp(estado, "init") == 0)
    { // Amarelo (Vermelho + Verde)
        gpio_put(led_red, 1);
        gpio_put(led_green, 1);
    }
    else if (strcmp(estado, "pronto") == 0)
    { // Verde
        gpio_put(led_green, 1);
        gpio_put(led_blue, 0);
        gpio_put(led_red, 0);
    }
    else if (strcmp(estado, "gravando") == 0)
    { // Vermelho
        gpio_put(led_red, 1);
        gpio_put(led_blue, 0);
        gpio_put(led_green, 0);
    }
    else if (strcmp(estado, "sd_rw") == 0)
    { // Azul piscando
        gpio_put(led_red, 0);
        gpio_put(led_green, 0);
        gpio_put(led_blue, 1);
        sleep_ms(200);
        gpio_put(led_blue, 0);
        sleep_ms(200);
    }
    else if (strcmp(estado, "erro") == 0)
    { // Roxo piscando (Vermelho + Azul)
        for (int i = 0; i < 3; i++)
        {
            gpio_put(led_green, 0);
            gpio_put(led_red, 1);
            gpio_put(led_blue, 1);
            sleep_ms(200);
            gpio_put(led_red, 0);
            gpio_put(led_blue, 0);
            sleep_ms(200);
        }
    }
}
void buzzer_play_note(int freq, int duration_ms)
{
    if (freq == REST)
    {
        gpio_put(BUZZER_PIN, 0);
        sleep_ms(duration_ms);
        return;
    }

    // Calcula período e número de ciclos
    uint32_t period_us = 1000000 / freq;
    uint32_t cycles = (freq * duration_ms) / 1000;

    // Gera onda quadrada para o tom
    for (uint32_t i = 0; i < cycles; i++)
    {
        gpio_put(BUZZER_PIN, 1);
        sleep_us(period_us / 2);
        gpio_put(BUZZER_PIN, 0);
        sleep_us(period_us / 2);
    }
}

void beep(int count)
{
    const int freq = 1000;       // Frequência do beep em Hz
    const int duration_ms = 100; // Duração de cada beep

    for (int i = 0; i < count; i++)
    {
        buzzer_play_note(freq, duration_ms); // Emite o beep
        sleep_ms(150);                       // Pequena pausa entre beeps
    }
}
void buzzer_init()
{
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
}

static void mpu6050_reset()
{
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(10);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x43;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x41;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

static sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static void run_setrtc()
{
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr)
    {
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr)
    {
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr)
    {
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr)
    {
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr)
    {
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    rtc_set_datetime(&t);
}

static void run_format()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    /* Format the drive with default parameters */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
    {
        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "ERRO", 10, 20);        // Desenha uma string
        ssd1306_send_data(&ssd);
        buzzer_play_note(400, 500);
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
    }
    else
    {
        buzzer_play_note(1000, 150);
        buzzer_play_note(700, 150);
        buzzer_play_note(500, 200);
        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "SUCESSO", 10, 20);     // Desenha uma string
        ssd1306_send_data(&ssd);
    }
}
static void run_mount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "ERRO", 10, 20);        // Desenha uma string
        ssd1306_send_data(&ssd);
        set_led_color("erro");
        buzzer_play_note(400, 500);
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    ssd1306_fill(&ssd, !borda);                       // Limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
    ssd1306_draw_string(&ssd, "SD Montado", 10, 20);  // Desenha uma string
    ssd1306_send_data(&ssd);
    buzzer_play_note(800, 200);

    set_led_color("init");

    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    montado = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
}
static void run_unmount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr)
    {
        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "ERRO", 10, 20);        // Desenha uma string
        ssd1306_send_data(&ssd);
        set_led_color("erro");
        buzzer_play_note(400, 500);
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    set_led_color("init");
    ssd1306_fill(&ssd, !borda);                         // Limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);   // Desenha um retângulo
    ssd1306_draw_string(&ssd, "SD Desmontado", 10, 20); // Desenha uma string
    ssd1306_send_data(&ssd);

    beep(2);
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    montado = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    printf("SD ( %s ) desmontado\n", pSD->pcName);
}
static void run_getfree()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr)
    {
        set_led_color("erro");
        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "ERRO", 10, 20);        // Desenha uma string
        ssd1306_send_data(&ssd);
        buzzer_play_note(400, 500);
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    ssd1306_fill(&ssd, !borda);                       // Limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
    ssd1306_draw_string(&ssd, "SUCESSO", 10, 20);     // Desenha uma string
    ssd1306_send_data(&ssd);
    set_led_color("sd_rw");
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}
static void run_ls()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0])
    {
        p_dir = arg1;
    }
    else
    {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr)
        {
            ssd1306_fill(&ssd, !borda);                       // Limpa o display
            ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
            ssd1306_draw_string(&ssd, "ERRO", 10, 20);        // Desenha uma string
            ssd1306_send_data(&ssd);
            set_led_color("erro");
            buzzer_play_note(400, 500);
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            return;
        }

        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "SUCESSO", 10, 20);     // Desenha uma string
        ssd1306_send_data(&ssd);
        set_led_color("sd_rw");

        p_dir = cwdbuf;
    }

    printf("Directory Listing: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr)
    {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    while (fr == FR_OK && fno.fname[0])
    {
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
    gpio_put(led_red, 1);
    gpio_put(led_blue, 1);
    sleep_ms(200);
    gpio_put(led_red, 0);
    gpio_put(led_blue, 0);
    sleep_ms(200);
}
static void run_cat()
{
    char *arg1 = strtok(NULL, " ");
    if (!arg1)
    {
        printf("Missing argument\n");
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr)
    {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil))
    {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr)
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
}

void capture_mpu6050_data_and_save()
{
    FIL file;
    UINT bw;
    int count = 0;
    int16_t acc[3], gyro[3], temp;

    FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "ERRO", 10, 20);        // Desenha uma string
        ssd1306_send_data(&ssd);
        printf("Erro ao abrir o arquivo\n");
        set_led_color("erro");
        beep(3);
        return;
    }

    char header[] = "numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n";
    f_write(&file, header, strlen(header), &bw);

    set_led_color("gravando");
    beep(1);
    while (logger_enabled)
    {
        mpu6050_read_raw(acc, gyro, &temp);
        float ax = acc[0] / 16384.0f, ay = acc[1] / 16384.0f, az = acc[2] / 16384.0f;
        float gx = gyro[0] / 131.0f, gy = gyro[1] / 131.0f, gz = gyro[2] / 131.0f;

        char linha[100];
        snprintf(linha, sizeof(linha), "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                 ++count, ax, ay, az, gx, gy, gz);
        f_write(&file, linha, strlen(linha), &bw);

        // Display atualização
        ssd1306_fill(&ssd, 0);
        ssd1306_draw_string(&ssd, "Gravando...", 10, 20);
        char msg[32];
        sprintf(msg, "Amostras: %d", count);
        ssd1306_draw_string(&ssd, msg, 10, 35);
        ssd1306_send_data(&ssd);

        // LED azul piscando = acesso SD
        gpio_put(led_blue, 1);
        sleep_ms(100);
        gpio_put(led_blue, 0);

        sleep_ms(500);
    }

    f_close(&file);
    beep(2);
    set_led_color("pronto");
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename)
{
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)
    {
        ssd1306_fill(&ssd, !borda);                       // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
        ssd1306_draw_string(&ssd, "ERRO", 10, 20);        // Desenha uma string
        ssd1306_send_data(&ssd);
        set_led_color("erro");
        buzzer_play_note(400, 500);
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");

        return;
    }
    ssd1306_fill(&ssd, !borda);                       // Limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
    ssd1306_draw_string(&ssd, "SUCESSO", 10, 20);     // Desenha uma string
    ssd1306_send_data(&ssd);
    set_led_color("sd_rw");
    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);
    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0)
    {
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&file);
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}

// Trecho para modo BOOTSEL com botão B

void debounce(uint gpio, uint32_t events)
{
    static uint32_t last_time = 0;
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - last_time > 1000000)
    {
        last_time = current_time;
        if (gpio == buttonA)
        {
            logger_enabled = !logger_enabled; // TOGGLE (inicia ou para)
            // printf("Logger: %s\n", logger_enabled ? "ON" : "OFF");
        }
        else if (gpio == buttonB)
        {
            toggle_sd_requested = true;
        }
    }
}

static void run_help()
{
    printf("\n***Comandos disponíveis***\n\n");
    printf("Pressione o botao 'B' para montar e desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Press o botao 'A' para gravar os dados do sensor no SD em .csv e press novamente para parar\n");
    printf("Digite 'g' para formatar o cartão SD\n");
    printf("Digite 'h' para exibir os comandos disponíveis\n");
    printf("\nEscolha o comando:  ");
}

static void process_stdio(int cRxedChar)
{
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar); // echo
    stdio_flush();
    if (cRxedChar == '\r')
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            size_t i;
            for (i = 0; i < count_of(cmds); ++i)
            {
                if (0 == strcmp(cmds[i].command, cmdn))
                {
                    (*cmds[i].function)();
                    break;
                }
            }
            if (count_of(cmds) == i)
                printf("Command \"%s\" not found\n", cmdn);
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}

void led_init(int led)
{
    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);
    gpio_put(led, false);
}

void button_init(int button)
{
    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_pull_up(button);
}

void display_init()
{
    i2c_init(I2C_DISPLAY, 400 * 1000); // Inicializa I2C1
    gpio_set_function(PIN_I2C_SDA_DISPLAY, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL_DISPLAY, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA_DISPLAY);
    gpio_pull_up(PIN_I2C_SCL_DISPLAY);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, 0x3C, I2C_DISPLAY);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
}

// int main()
// {

//     display_init();

//     ssd1306_fill(&ssd, !borda);                       // Limpa o display
//     ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//     // // ssd1306_line(&ssd, 3, 15, 123, 15, cor);      // Desenha uma linha
//     // // ssd1306_line(&ssd, 3, 32, 123, 32, borda);
//     // // ssd1306_line(&ssd, 3, 45, 123, 45, borda);
//     ssd1306_draw_string(&ssd, "Iniciando...", 10, 30); // Desenha uma string
//     // // ssd1306_draw_string(&ssd, "Temp", 5, 20);       // Desenha uma string
//     // // ssd1306_draw_string(&ssd, "Umi", 6, 35);        // Desenha uma string
//     // // ssd1306_draw_string(&ssd, "Pres", 6, 50);       // Desenha uma string
//     // // ssd1306_line(&ssd, 40, 15, 40, 60, cor);        // Desenha uma linha vertical
//     ssd1306_send_data(&ssd);
//     // // Para ser utilizado o modo BOOTSEL com botão B
//     button_init(buttonB);
//     button_init(buttonA);
//     gpio_set_irq_enabled_with_callback(buttonA, GPIO_IRQ_EDGE_FALL, true, &debounce);
//     gpio_set_irq_enabled(buttonB, GPIO_IRQ_EDGE_FALL, true);

//     i2c_init(I2C_PORT, 400 * 1000); // 400kHz
//     gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
//     gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
//     gpio_pull_up(I2C_SDA);
//     gpio_pull_up(I2C_SCL);
//     buzzer_init();

//     led_init(led_blue);
//     led_init(led_red);
//     led_init(led_green);
//     set_led_color("init");

//     stdio_init_all();
//     sleep_ms(5000);
//     time_init();
//     gpio_put(led_blue, 0);
//     gpio_put(led_red, 0);
//     gpio_put(led_green, 0);
//     printf("FatFS SPI example\n");
//     printf("\033[2J\033[H"); // Limpa tela
//     printf("\n> ");
//     stdio_flush();
//     //    printf("A tela foi limpa...\n");
//     //    printf("Depois do Flush\n");
//     run_help();
//     while (true)
//     {
//         int cRxedChar = getchar_timeout_us(0);
//         if (PICO_ERROR_TIMEOUT != cRxedChar)
//             process_stdio(cRxedChar);

//         if (toggle_sd_requested)
//         {
//             toggle_sd_requested = false;
//             set_led_color("init");
//             if (is_sd_mounted())
//             {
//                 ssd1306_fill(&ssd, !borda);                       // Limpa o display
//                 ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//                 ssd1306_draw_string(&ssd, "Desmontando", 10, 20); // Desenha uma string
//                 ssd1306_draw_string(&ssd, "SD...", 30, 30);       // Desenha uma string
//                 ssd1306_send_data(&ssd);
//                 printf("\nDesmontando SD via botão B...\n");
//                 sleep_ms(1000);
//                 run_unmount();
//                 sleep_ms(1000);
//             }
//             else
//             {
//                 ssd1306_fill(&ssd, !borda);                       // Limpa o display
//                 ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//                 ssd1306_draw_string(&ssd, "Montando", 10, 20);    // Desenha uma string
//                 ssd1306_draw_string(&ssd, "SD...", 30, 30);       // Desenha uma string
//                 ssd1306_send_data(&ssd);
//                 printf("\nMontando SD via botão B...\n");
//                 sleep_ms(1000);
//                 run_mount();
//                 sleep_ms(1000);
//             }
//         }
//         else if (cRxedChar == 'c') // Lista diretórios e os arquivos se pressionar 'c'
//         {
//             ssd1306_fill(&ssd, !borda);                       // Limpa o display
//             ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//             ssd1306_draw_string(&ssd, "Exibindo", 10, 20);    // Desenha uma string
//             ssd1306_draw_string(&ssd, "arquivos...", 30, 30); // Desenha uma string
//             ssd1306_send_data(&ssd);
//             buzzer_play_note(1200, 80);
//             sleep_ms(1000);

//             printf("\nListagem de arquivos no cartão SD.\n");
//             run_ls();
//             set_led_color("sd_rw");

//             printf("\nListagem concluída.\n");

//             printf("\nEscolha o comando (h = help):  ");
//             sleep_ms(1000);
//         }
//         else if (cRxedChar == 'd') // Exibe o conteúdo do arquivo se pressionar 'd'
//         {
//             ssd1306_fill(&ssd, !borda);                       // Limpa o display
//             ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//             ssd1306_draw_string(&ssd, "Exibindo", 10, 20);    // Desenha uma string
//             ssd1306_draw_string(&ssd, "arquivo...", 30, 30);  // Desenha uma string
//             ssd1306_send_data(&ssd);
//             buzzer_play_note(1200, 80);
//             sleep_ms(1000);
//             read_file(filename);
//             set_led_color("sd_rw");
//             sleep_ms(1000);
//             printf("Escolha o comando (h = help):  ");
//         }
//         else if (cRxedChar == 'e') // Obtém o espaço livre no SD card se pressionar 'e'
//         {
//             ssd1306_fill(&ssd, !borda);                       // Limpa o display
//             ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//             ssd1306_draw_string(&ssd, "Verificando", 10, 20); // Desenha uma string
//             ssd1306_draw_string(&ssd, "espaco...", 30, 30);   // Desenha uma string
//             ssd1306_send_data(&ssd);
//             buzzer_play_note(1200, 80);
//             printf("\nObtendo espaço livre no SD.\n\n");
//             sleep_ms(1000);
//             run_getfree();
//             set_led_color("sd_rw");
//             sleep_ms(1000);
//             printf("\nEspaço livre obtido.\n");
//             printf("\nEscolha o comando (h = help):  ");
//         }
//         else if (logger_enabled && !recording) // Captura dados do ADC e salva no arquivo se pressionar 'f'
//         {
//             recording = true;
//             capture_mpu6050_data_and_save();
//             recording = false;
//             sleep_ms(1000);
//         }
//         else if (cRxedChar == 'g') // Formata o SD card se pressionar 'g'
//         {
//             ssd1306_fill(&ssd, !borda);                         // Limpa o display
//             ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda);   // Desenha um retângulo
//             ssd1306_draw_string(&ssd, "Formatando...", 10, 20); // Desenha uma string
//             ssd1306_send_data(&ssd);
//             printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
//             sleep_ms(1000);
//             run_format();
//             sleep_ms(1000);
//             printf("\nFormatação concluída.\n\n");
//             printf("\nEscolha o comando (h = help):  ");
//         }
//         else if (cRxedChar == 'h') // Exibe os comandos disponíveis se pressionar 'h'
//         {
//             run_help();
//         }
//         else
//         {
//             if (montado)
//             {
//                 set_led_color("pronto");
//                 ssd1306_fill(&ssd, !borda);                       // Limpa o display
//                 ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//                 ssd1306_draw_string(&ssd, "Aguardando", 10, 20);  // Desenha uma string
//                 ssd1306_draw_string(&ssd, "comando...", 30, 30);  // Desenha uma string
//                 ssd1306_send_data(&ssd);
//             }
//             else
//             {
//                 gpio_put(led_green, 0);
//                 gpio_put(led_red, 0);
//                 gpio_put(led_blue, 0);
//                 ssd1306_fill(&ssd, !borda);                       // Limpa o display
//                 ssd1306_rect(&ssd, 3, 3, 122, 60, borda, !borda); // Desenha um retângulo
//                 ssd1306_draw_string(&ssd, "Aguardando", 10, 20);  // Desenha uma string
//                 ssd1306_draw_string(&ssd, "Montagem...", 30, 30); // Desenha uma string
//                 ssd1306_send_data(&ssd);
//             }
//         }
//         sleep_ms(500);
//     }
//     return 0;
// }