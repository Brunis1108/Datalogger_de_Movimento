import pandas as pd
import matplotlib.pyplot as plt

# Lê o arquivo CSV (duas colunas: tempo, amplitude)
df = pd.read_csv(r'ArquivosDados/imu_data.csv')

# Supondo que o CSV tem cabeçalho: tempo,valor
x = df['numero_amostra']
y = df['accel_x']

# Gera o gráfico
plt.plot(x, y, 'bo-')  # 'b' = azul, 'o' = marcador bolinha, '-' = linha contínua
plt.title("Gráfico dos dados coletados")
plt.xlabel("Tempo")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()
