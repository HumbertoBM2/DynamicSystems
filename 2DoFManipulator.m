%% Simulación de manipulador de dos grados de libertad usando Euler - Lagrange

% Parámetros físicos del manipulador
link1Length = 0.450; 
link2Length = 0.450;
massCenter1 = 0.091;
massCenter2 = 0.048;
link1Mass = 23.902;
link2Mass = 3.880;
link1Inertia = 1.266;
link2Inertia = 0.093;
gravity = 9.81;

% Condiciones iniciales
initialPosition = [0; 0];
initialVelocity = [0; 0];
desiredPosition = [0.8; 0.5];

timeStep = 0.01;
totalTime = 10;
timeVector = 0:timeStep:totalTime;

% Ganancias PID
proportionalGain = [70, 50];    
derivativeGain = [18, 10];    
integralGain = [14, 6];    

% Variables de estado
position = zeros(2, length(timeVector));
velocity = zeros(2, length(timeVector));
acceleration = zeros(2, length(timeVector));
controlTorque = zeros(2, length(timeVector));

position(:,1) = initialPosition;
velocity(:,1) = initialVelocity;

% Errores acumulativos para el control integral
integralError = [0; 0];

% Simulación del sistema dinámico
for i = 1:length(timeVector)-1
    % Cálculo del error
    positionError = desiredPosition - position(:,i);
    velocityError = -velocity(:,i);
    integralError = integralError + positionError * timeStep;
    
    % Control PID
    controlTorque(:,i) = proportionalGain(:) .* positionError + derivativeGain(:) .* velocityError + integralGain(:) .* integralError;
    
    % Matriz de inercia
    inertiaMatrix = [link1Inertia + link2Inertia + link2Mass*link1Length^2 + 2*link2Mass*link1Length*massCenter2*cos(position(2,i)), link2Inertia + link2Mass*link1Length*massCenter2*cos(position(2,i));
                     link2Inertia + link2Mass*link1Length*massCenter2*cos(position(2,i)), link2Inertia];
    
    % Matriz de Coriolis y centrífugas
    coriolisMatrix = [-link2Mass*link1Length*massCenter2*sin(position(2,i))*velocity(2,i), -link2Mass*link1Length*massCenter2*sin(position(2,i))*(velocity(1,i) + velocity(2,i));
                      link2Mass*link1Length*massCenter2*sin(position(2,i))*velocity(1,i), 0];
    
    % Vector de gravedad
    gravityVector = [link1Mass*massCenter1*gravity*cos(position(1,i)) + link2Mass*gravity*(link1Length*cos(position(1,i)) + massCenter2*cos(position(1,i) + position(2,i)));
                     link2Mass*massCenter2*gravity*cos(position(1,i) + position(2,i))];
    
    % Cálculo de aceleraciones
    acceleration(:,i) = inertiaMatrix \ (controlTorque(:,i) - coriolisMatrix*velocity(:,i) - gravityVector);
    
    % Integración numérica para obtener velocidad y posición
    velocity(:,i+1) = velocity(:,i) + acceleration(:,i) * timeStep;
    position(:,i+1) = position(:,i) + velocity(:,i) * timeStep;
end

% Gráficas de resultados
figure;
subplot(3,1,1);
plot(timeVector, position(1,:), 'r', timeVector, position(2,:), 'b');
legend('q1', 'q2');
title('Evolución de la posición angular');
xlabel('Tiempo (s)'); ylabel('Ángulo (rad)'); grid on;

subplot(3,1,2);
plot(timeVector, velocity(1,:), 'r', timeVector, velocity(2,:), 'b');
legend('dq1', 'dq2');
title('Evolución de la velocidad angular');
xlabel('Tiempo (s)'); ylabel('Velocidad (rad/s)'); grid on;

subplot(3,1,3);
plot(timeVector, controlTorque(1,:), 'r', timeVector, controlTorque(2,:), 'b');
legend('\tau_1', '\tau_2');
title('Entrada de control aplicada');
xlabel('Tiempo (s)'); ylabel('Torque (Nm)'); grid on;