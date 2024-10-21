import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# Parâmetros do sistema
m = 1542  # Massa do veículo [kg]
v0 = 25   # Velocidade inicial [m/s]
rho = 1.2041  # Densidade do ar [kg/m^3]
A = 2.5   # Área frontal [m^2]
c_d = 0.34  # Coeficiente de arrasto

# Função de transferência do sistema
numerador = [1]
denominador = [m * v0, 0.5 * rho * A * c_d * v0]
G = ctrl.TransferFunction(numerador, denominador)

# Parâmetros do controlador PID (ajustados)
K_P = 10000  # Ganho proporcional ajustado
K_I = 0  # Ganho integral ajustado
K_D = 0  # Ganho derivativo ajustado

# Controlador PID
C = ctrl.TransferFunction([K_D, K_P, K_I], [1, 0])

# Sistema em malha fechada
Sistema_Feedback = ctrl.feedback(C * G)

# Tempo de simulação
tempo = np.linspace(0, 200, 1000)

# Gerar perfil de setpoints
def gerar_setpoints(tempo, intervalos, valores):
    num_ticks = len(tempo)
    setpoints = np.zeros(num_ticks)
    tick_count = 0
    for i in range(len(intervalos)):
        ticks_para_intervalo = int(num_ticks * intervalos[i])
        setpoints[tick_count:tick_count + ticks_para_intervalo] = valores[i]
        tick_count += ticks_para_intervalo
    return setpoints

# Intervalos de tempo e valores dos setpoints
intervalos = [1/9, 1/6, 1/7, 1/3, 1/7]  # Proporções do tempo total para cada setpoint
valores = [v0, 37, 34, 25, 10]  # Velocidades desejadas

setpoint = gerar_setpoints(tempo, intervalos, valores)

# Simulação do sistema com perfil de setpoints variáveis
tempo_resposta, resposta = ctrl.forced_response(Sistema_Feedback, T=tempo, U=setpoint)

# Calcular a potência ao longo do tempo
potencia = 0.5 * rho * A * c_d * resposta**3  # P = F_drag * v(t) = 0.5 * rho * A * c_d * v(t)^3

# Plotagem dos resultados
plt.figure(figsize=(12, 8))

# Velocidade
plt.subplot(2, 1, 1)
plt.plot(tempo_resposta, resposta, label='Resposta do Sistema', color='b')
plt.plot(tempo, setpoint, 'r--', label='Velocidade Desejada (Setpoint)')
plt.title('Resposta do Sistema de Controle de Velocidade com Setpoints Variáveis')
plt.xlabel('Tempo [s]')
plt.ylabel('Velocidade [m/s]')
plt.legend(loc='best')
plt.grid(True)
plt.ylim(0, 100)

# Potência
plt.subplot(2, 1, 2)
plt.plot(tempo_resposta, potencia, label='Potência Necessária', color='g')
plt.title('Potência Necessária ao Longo do Tempo')
plt.xlabel('Tempo [s]')
plt.ylabel('Potência [W]')
plt.legend(loc='best')
plt.grid(True)

plt.tight_layout()
plt.show()
