import PyVehicle
import PyPID
import matplotlib.pyplot as plt
from math import ceil

# Configuração
V_init = 25 # velocidade inicial [m/s]
h_physics = 0.05 # passo de tempo da física [s]
h_ref = 0.01 # passo de tempo do integrador de referência [s]
physics_ticks_per_control_tick = 4 # chamar PID a cada x ticks de física
T_max = 200 # tempo final [s]

# Ganhos PID
K_P = 22000  # ganho proporcional
K_I = 2200   # ganho integral
K_D = 0      # ganho derivativo

num_ticks = int(T_max / h_physics) + 1
# Configurar pontos de referência para um degrau
set_points = [V_init] * (num_ticks // 2) + [37] * (num_ticks // 2)
# Adicionar valores restantes se necessário
if len(set_points) < num_ticks:
    set_points.extend([37] * (num_ticks - len(set_points)))

import PyVehicle
import PyPID

veh = PyVehicle.Vehicle(V_init, h_physics)
pid = PyPID.PID(K_P, K_I, K_D)

# RK4 propagator
def RK4(dv_dt, t, dt, v):
    k_1 = dv_dt(t, v)
    k_2 = dv_dt(t + dt * 0.5, v + dt * 0.5 * k_1)
    k_3 = dv_dt(t + dt * 0.5, v + dt * 0.5 * k_2)
    k_4 = dv_dt(t + dt, v + dt * k_3)
    return v + dt / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4)

# Propagar física e controle
h_control = h_physics * physics_ticks_per_control_tick
T = [None] * num_ticks
V = [None] * num_ticks
pwr = [None] * num_ticks
for i in range(num_ticks):
    V[i] = veh.stepPhysics()
    T[i] = veh.t
    if not i % physics_ticks_per_control_tick:
        pid.stepControl(veh, set_points[i], h_control)

    pwr[i] = veh.cur_pwr

# Plotar resposta ao degrau
plt.figure('Step Response - Velocities')
plt.title('Resposta ao Degrau do Controle de Cruzeiro\nVelocidades')
plt.xlabel('Tempo [s]')
plt.ylabel('Velocidade [m/s]')
plt.plot(T, set_points, color='r', linestyle='--', label='Referência (Degrau)')
plt.plot(T, V, color='b', linewidth=2, label='Velocidade do Controle de Cruzeiro')
plt.ylim(0, 10 * ceil(max(V + set_points) / 10) + 5)
plt.legend(loc='best')

plt.show()