# Simulación de controlador PID con feedback linearization para un motor DC

# Librerías 
import numpy as np
import matplotlib.pyplot as plt
plt.style.use('ggplot')

# Parámetros del motor 
L   = 0.02844     # Inductancia de armadura [H]
R   = 1.27        # Resistencia de armadura [ohms]
Kb  = 0.35        # Constante FEM [V/(rad/s)]
Kt  = 0.35        # Constante de par [N*m/A]
J   = 0.007       # Inercia del rotor [kg*m^2]
B   = 0.00173     # Coeficiente de fricción viscosa [N*m*s/rad]

# Límites de operación
vMax = 9.0  
vMin = -9.0 

# Setpoints
def omega_d(t):
    if 0 <= t < 5:
        return 4*t
    elif 5 <= t < 10:
        return 20
    elif 10 <= t < 15:
        return 20 + (t - 10)
    elif 15 <= t < 20:
        return 25.0
    elif 20 <= t < 25:
        return 25 - 5 * (t - 20)
    elif 25 <= t < 30:
        return 0
    elif 30 <= t < 35:
        return -5 * (t - 30)
    elif 35 <= t < 50:
        return -25.0
    elif 50 <= t < 55:
        return -25 + 5 * (t - 50)
    else:
        return 0.0

def domegaD(t):
    return 0.0

# Parámetros de simulación
t0 = 0.0      # Tiempo inicial [s]
tf = 60.0     # Tiempo final [s]
dt = 0.02     # Paso de integración
t = np.arange(t0, tf, dt)

# Ganancias del PID
Kp = 0.5
Ki = 5.0
Kd = 0.001

iActual = 0.0       # Corriente inicial [A]
omegaActual = 0.0   # Velocidad angular inicial [rad/s]

errorInteg = 0.0
errorPrev  = 0.0

# Inicializar vectores para guardar resultados
n = len(t)
refHist    = np.zeros(n)
omegaHist  = np.zeros(n)
errorHist  = np.zeros(n)
voltHist   = np.zeros(n)
iHist      = np.zeros(n)
torqueHist = np.zeros(n)

# Integración usando método de Euler y Ley de Kirchhof
for k in range(n):
    tk = t[k]
    
    # Referencia en el instante actual
    ref = omega_d(tk)
    refHist[k] = ref
    
    # Cálculo del error
    error = ref - omegaActual
    errorHist[k] = error
    
    # PID discreto
    derror = (error - errorPrev) / dt
    errorInteg += error * dt
    v_control = Kp * error + Ki * errorInteg + Kd * derror
    
    # Saturación del voltaje
    if v_control > vMax:
        v_control = vMax
    elif v_control < vMin:
        v_control = vMin
    voltHist[k] = v_control
    
    # Modelo del motor
    di_dt = (v_control - R * iActual - Kb * omegaActual) / L
    domegaDt = (Kt * iActual - B * omegaActual) / J
    
    # Integración de estados
    iActual += di_dt * dt
    omegaActual += domegaDt * dt
    
    # Calcular torque
    torque = Kt * iActual
    
    # Guardar resultados de la iteración
    iHist[k] = iActual
    omegaHist[k] = omegaActual
    torqueHist[k] = torque
    
    errorPrev = error

# Formato para las gráficas
def formatea_grafica(titulo, xlabel, ylabel):
    plt.title(titulo, fontsize=14)
    plt.xlabel(xlabel, fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.minorticks_on()
    plt.legend(fontsize=12)

# Graficar los vectores con respecto al tiempo 
fig1 = plt.figure(figsize=(10,6))
plt.plot(t, refHist, lw=2, label="Setpoint")
formatea_grafica("Setpoint vs Tiempo", "Tiempo [s]", "Velocidad [rad/s]")

fig2 = plt.figure(figsize=(10,6))
plt.plot(t, omegaHist, lw=2, label="Velocidad del motor")
formatea_grafica("Velocidad Angular vs Tiempo", "Tiempo [s]", "Velocidad [rad/s]")

fig3 = plt.figure(figsize=(10,6))
plt.plot(t, errorHist, lw=2, label="Error = omega_d - omega")
formatea_grafica("Error vs Tiempo", "Tiempo [s]", "Error [rad/s]")

fig4 = plt.figure(figsize=(10,6))
plt.plot(t, voltHist, lw=2, label="Voltaje de control")
formatea_grafica("Entrada de Control vs Tiempo", "Tiempo [s]", "Voltaje [V]")

fig5 = plt.figure(figsize=(10,6))
plt.plot(t, iHist, lw=2, label="Corriente de armadura")
formatea_grafica("Corriente vs Tiempo", "Tiempo [s]", "Corriente [A]")

fig6 = plt.figure(figsize=(10,6))
plt.plot(t, torqueHist, lw=2, label="Par electromagnético")
formatea_grafica("Par Electromagnético vs Tiempo", "Tiempo [s]", "Torque [N·m]")

plt.tight_layout()
plt.show()