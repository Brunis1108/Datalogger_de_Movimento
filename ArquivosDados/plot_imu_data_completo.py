import serial
import pandas as pd
import matplotlib.pyplot as plt
import time
#para funcionar digite a entrada da placa (ex.: COM3, COM4) na linha 72
#essa função vai executar o comando na placa via serial, então certifique-se de que o
#serial esteja disponível para receber comando

# === 1. Localiza automaticamente a porta do Pico ===

# === 2. Conecta à serial e envia comando 'd' ===
def ler_csv_via_serial(porta, baudrate=115200, timeout=10):
    print(f"Conectando ao Pico em {porta}...")
    with serial.Serial(porta, baudrate=baudrate, timeout=1) as ser:
        ser.reset_input_buffer()
        time.sleep(2)
        ser.write(b'd\n')  # Envia o comando para imprimir o CSV

        print("Lendo dados...")
        linhas = []
        inicio = time.time()

        while time.time() - inicio < timeout:
            if ser.in_waiting > 0:
                linha = ser.readline().decode(errors='ignore').strip()
                if linha.startswith("numero_amostra") or linha[:1].isdigit():
                    linhas.append(linha)
        return linhas

# === 3. Salva como imu_data.csv ===
def salvar_csv(linhas, nome_arquivo='ArquivosDados/imu_data.csv'):
    with open(nome_arquivo, 'w', encoding='utf-8') as f:
        for linha in linhas:
            f.write(linha + '\n')
    print(f"Arquivo salvo como {nome_arquivo}.")

# === 4. Plota os gráficos ===
def plotar_csv(nome_arquivo='ArquivoDados/imu_data.csv'):
    df = pd.read_csv(nome_arquivo)

    esperadas = ['numero_amostra','accel_x','accel_y','accel_z','giro_x','giro_y','giro_z']
    if not all(col in df.columns for col in esperadas):
        print("Erro: CSV com colunas inesperadas.")
        return

    print("Gerando gráficos...")

    plt.figure()
    plt.plot(df['numero_amostra'], df['accel_x'], label='Accel X')
    plt.plot(df['numero_amostra'], df['accel_y'], label='Accel Y')
    plt.plot(df['numero_amostra'], df['accel_z'], label='Accel Z')
    plt.xlabel('Tempo')
    plt.ylabel('Aceleração (g)')
    plt.title('Acelerômetro pelo tempo')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.figure()
    plt.plot(df['numero_amostra'], df['giro_x'], label='Gyro X')
    plt.plot(df['numero_amostra'], df['giro_y'], label='Gyro Y')
    plt.plot(df['numero_amostra'], df['giro_z'], label='Gyro Z')
    plt.xlabel('Tempo')
    plt.ylabel('Velocidade Angular (°/s)')
    plt.title('Giroscópio pelo Tempo')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.show()

# === Execução principal ===
if __name__ == "__main__":
    porta = "COM4"
    if not porta:
        print("Pico não encontrado. Conecte o dispositivo e tente novamente.")
        exit()

    dados = ler_csv_via_serial(porta)
    if not dados:
        print("Nenhum dado CSV foi capturado. Verifique se o SD está montado e o arquivo existe.")
        exit()

    salvar_csv(dados, 'ArquivosDados/imu_data.csv')
    plotar_csv('ArquivosDados/imu_data.csv')
