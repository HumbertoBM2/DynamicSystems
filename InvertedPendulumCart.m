% Parámetros físicos
massPendulum = 0.1;
massCart = 1.5;
lengthPendulum = 0.4;
gravity = 9.81;
timeStep = 0.01;
totalTime = 15;
nIterations = totalTime / timeStep;

% Condiciones iniciales
cartPosition = 0;
cartVelocity = 0;
pendulumAngle = 0.1;
pendulumVelocity = 0;

% Parámetros de control PD
kpCart = 1.3;
kdCart = 5;
kpPendulum = 9;
kdPendulum = 4;

% Setpoints
cartPositionDesired = 1;
cartVelocityDesired = 0;
pendulumAngleDesired = 0;
pendulumVelocityDesired = 0;

% Variables de almacenamiento
cartPositionHist = zeros(1, nIterations);
cartVelocityHist = zeros(1, nIterations);
pendulumAngleHist = zeros(1, nIterations);
pendulumVelocityHist = zeros(1, nIterations);
controlHist = zeros(1, nIterations);

% Simulación
for k = 1:nIterations
    % Errores
    errorCart = cartPositionDesired - cartPosition;
    errorVelocityCart = cartVelocityDesired - cartVelocity;
    errorPendulum = pendulumAngleDesired - pendulumAngle;
    errorVelocityPendulum = pendulumVelocityDesired - pendulumVelocity;
    
    % Control PD
    controlInput = kpCart * errorCart + kdCart * errorVelocityCart + ...
                   kpPendulum * errorPendulum + kdPendulum * errorVelocityPendulum;
    
    % Dinámica del sistema
    cartAcceleration = (1/massCart) * (controlInput - massPendulum * gravity * pendulumAngle);
    pendulumAcceleration = (controlInput - (massPendulum + massCart) * gravity * pendulumAngle) / (massCart * lengthPendulum);
    
    % Integración numérica
    cartVelocity = cartVelocity + cartAcceleration * timeStep;
    cartPosition = cartPosition + cartVelocity * timeStep;
    
    pendulumVelocity = pendulumVelocity + pendulumAcceleration * timeStep;
    pendulumAngle = pendulumAngle + pendulumVelocity * timeStep;
    
    % Almacenar valores
    cartPositionHist(k) = cartPosition;
    cartVelocityHist(k) = cartVelocity;
    pendulumAngleHist(k) = pendulumAngle;
    pendulumVelocityHist(k) = pendulumVelocity;
    controlHist(k) = controlInput;
end

% Graficar resultados
timeVector = 0:timeStep:totalTime-timeStep;

% Posición y velocidad del carrito
figure;
subplot(2,1,1);
plot(timeVector, cartPositionHist,'k','LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Posición (m)');
title('Posición del carrito');
grid on;

subplot(2,1,2);
plot(timeVector, cartVelocityHist, 'k', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Velocidad (m/s)');
title('Velocidad del carrito');
grid on;

% Ángulo y velocidad angular del péndulo
figure;
subplot(2,1,1);
plot(timeVector, pendulumAngleHist, 'k', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)');
title('Ángulo del péndulo');
grid on;

subplot(2,1,2);
plot(timeVector, pendulumVelocityHist, 'k', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Velocidad (rad/s)');
title('Velocidad angular del péndulo');
grid on;

% Entrada de control
figure;
plot(timeVector, controlHist, 'k', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Control');
title('Entrada de control');
grid on;

