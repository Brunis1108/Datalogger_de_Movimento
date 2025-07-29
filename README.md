# 📊 Datalogger com Raspberry Pi Pico, MPU6050, SD Card e Display OLED

Este projeto é um sistema embarcado desenvolvido para o **Raspberry Pi Pico**, capaz de capturar dados de movimento usando o sensor **MPU6050**, exibir mensagens em um **display OLED (SSD1306)**, salvar dados em um **cartão SD** em formato `.csv`, controlar status com **LED RGB** e emitir alertas com **buzzer piezoelétrico**. Todo o projeto foi estruturado com foco em **organização de código, reatividade via botões e comandos seriais interativos**.

---

## 🔧 Componentes Utilizados

| Componente        | Função                                |
|-------------------|----------------------------------------|
| Raspberry Pi Pico | Microcontrolador principal             |
| MPU6050           | Sensor de aceleração e giroscópio     |
| Cartão SD         | Armazenamento de dados `.csv`          |
| Display OLED I2C  | Exibição de mensagens/status           |
| LED RGB (3 pinos) | Indicação visual de estados do sistema |
| Buzzer            | Alerta sonoro para eventos             |
| Botões A e B      | Controle de gravação e montagem do SD  |

---

## 📁 Organização do Projeto

### Estrutura por Responsabilidade:

- `main()` → inicialização de periféricos e laço principal
- `capture_mpu6050_data_and_save()` → captura e grava os dados
- `run_mount()`, `run_unmount()` → comandos de montagem do SD
- `read_file()` → lê e exibe arquivo `.csv`
- `set_led_color()` → gerencia cor dos LEDs
- `buzzer_play_note()` / `beep()` → controla o buzzer
- `run_format()` → formata o cartão SD
- `run_ls()`, `run_cat()`, `run_getfree()` → comandos do terminal

---

## 🎮 Controles

### Botões físicos:

- **Botão A (GPIO 5)**: Inicia e para a gravação dos dados do sensor
- **Botão B (GPIO 6)**: Monta ou desmonta o cartão SD

### Comandos via terminal serial:

| Comando | Função                                     |
|--------|---------------------------------------------|
| `format` | Formata o cartão SD                       |
| `mount` | Monta o cartão SD                          |
| `unmount` | Desmonta o cartão SD                    |
| `getfree` | Mostra espaço livre do SD                |
| `ls`     | Lista arquivos no SD                      |
| `cat <arquivo>` | Mostra conteúdo do arquivo        |
| `h` ou `help` | Mostra todos os comandos disponíveis |

---

## 🟢 Indicações por LED RGB

| Cor         | Estado                          |
|-------------|---------------------------------|
| Amarelo     | Inicialização                   |
| Verde       | Pronto / Aguardando comandos    |
| Vermelho    | Gravando dados                  |
| Azul (piscando) | Acesso ao SD em andamento     |
| Roxo (piscando) | Erro                         |

---

## 🔊 Sinais Sonoros (Buzzer)

| Som                  | Evento                       |
|----------------------|------------------------------|
| 1 beep               | Início da gravação           |
| 2 beeps              | Fim da gravação              |
| Tom grave            | Erro                         |
| Escala descendente   | Sucesso na formatação        |

---

## 📝 Exemplo de Saída `.csv`

```csv
numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z
1,0.0012,-0.0048,1.0024,0.1200,-0.0870,0.0030
2,0.0008,-0.0044,1.0032,0.1198,-0.0873,0.0031
...
```
---

## 📈 Visualização dos Dados (Gráficos)

Os dados registrados no cartão SD podem ser visualizados graficamente por meio de um script Python chamado `plot_imu_data_completo.py`, incluído neste repositório.

Este script realiza automaticamente as seguintes etapas:

1. 📡 **Conecta ao Raspberry Pi Pico via porta serial** e envia o comando `'d'` para solicitar o conteúdo do arquivo `.csv`;
2. 💾 **Salva o conteúdo recebido** em um arquivo chamado `imu_data.csv` dentro da pasta `ArquivosDados/`;
3. 📊 **Gera dois gráficos separados** com base no número da amostra:
   - **Gráfico de aceleração**: aceleração nos eixos **X, Y e Z** (em g);
   - **Gráfico de giroscópio**: velocidade angular nos eixos **X, Y e Z** (em °/s).

Esses gráficos fornecem uma visualização clara e intuitiva dos dados de movimento capturados, permitindo análise de padrões e comportamento do sistema.

> ⚠️ **Atenção:** Antes de executar o script, verifique se:
> - O cartão SD está **montado**;
> - O arquivo `imu_data.csv` existe e está acessível no cartão SD;
> - A porta COM do dispositivo está corretamente configurada no script.


