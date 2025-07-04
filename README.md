# Navegación Campo Potencial

Autores: 
- Eduardo Cuadros
- Nicolás Moreno

## Objetivo

El propósito de esta tarea fue resolver y simular una misión de navegación autónoma para un robot móvil con ruedas, utilizando el método de navegación por campo potencial artificial. Este método combina fuerzas atractivas (que llevan al robot hacia la meta) y repulsivas (que lo alejan de los obstáculos) para generar trayectorias seguras en entornos con obstáculos. Se trabajó específicamente con el robot DR12, tanto en MATLAB como en CoppeliaSim.

## Modelo Cinemático del Robot

Se inició creando el modelo matemático del robot DR12 que describe cómo se mueve en función de las velocidades de sus ruedas. Este modelo se implementó en MATLAB usando la función _differentialDriveKinematics_, que requiere como parámetros el radio de las ruedas y la distancia entre ellas. Las dimensiones del robot se obtuvieron del modelo disponible en CoppeliaSim.

El modelo permite simular cómo reacciona el robot ante distintos comandos de velocidad lineal y angular, lo cual es fundamental para luego aplicar el control de navegación.

### Dimensiones físicas
- Largo (X): 0.23 m
- Ancho (Y): 0.158 m
- Alto (Z): 0.1406 m
- Radio de rueda: 0.086 m
- Espesor de rueda: 0.026 m
- Distancia entre ruedas: 0.154 m

## Cálculo de radio R del círculo que circunscribe al robot

El radio que se quiere calcular se encuentra expresado en la siguiente figura:

![image](https://github.com/user-attachments/assets/2d928c75-5f68-4a37-9b36-ada15530e7a9)

Dicho radio se encuentra con la siguiente ecuación:

$$ R = \sqrt{ \left( \frac{0.158}{2} \right)^2 + \left( \frac{0.23}{2} \right)^2 } = 0.1395\ \text{metros} $$

## Creación del mapa

Una vez encontrado el radio, este valor modifica el factor de escala del mapa de la siguiente forma:

$$ k = 10 * R $$

Con este factor de escala se encuentran las dimensiones de los obstáculos circulares, obteniendo el siguiente mapa.

![mapa_obs](https://github.com/user-attachments/assets/a0c418d2-0027-40a3-9251-6618cd9fc19f)


## Descripción del Campo Potencial

$$ (1)\ U_x = U_{att}(x) + U_{rep}(x) $$

El campo potencial atractivo se define como una función a trozos. Esta función está divida por una distancia _d_, la cual es el cambio entre _distancia corta_ y _distancia larga_. Esto se hace con el objetivo de que la fuerza atractiva no crezca indefinidamente cuando el robot está lejos.


$$(2)\ U_{att}(x)= \left\lbrace \begin{array}{lcc} \frac{1}{2}K_{att} ||x - x_{goal}||{^2} & si & ||x - x_{goal}|| \leq d\\
dK_{att}||x - x_{goal}||{^2} - \frac{1}{2}K_{att}d{^2} & si & ||x - x_{goal}||> d  \end{array} \right. 
$$


La fuerza atractiva se define como el gradiente del campo potencial, como se muestra a continuación:

$$ (3)\ \nabla U_{att}(x)= \left\lbrace \begin{array}{lcc} K_{att} ||x - x_{goal}|| & si & ||x - x_{goal}|| \leq d\\ \\ 
\frac{dK_{att}(x - x_{goal})}{ ||x - x_{goal}||}  & si & ||x - x_{goal}||> d  \end{array} \right.   $$

De igual manera se definen el campo potencial repulsivo y su correspondiente fuerza repulsiva de la siguiente forma.

$$ (4)\ U_{rep}(x)= \left\lbrace \begin{array}{lcc} \frac{1}{2}K_{rep} \left( \frac{1}{\rho(x)} - \frac{1}{\rho_0} \right)^2   & si &\rho(x)\leq \rho_0\\ \\
0 & si & \rho(x) > \rho_0  \end{array} \right. $$

$$ (5)\ \nabla U_{rep}(x)= \left\lbrace \begin{array}{lcc} -K_{rep} \left( \frac{1}{\rho(x)} - \frac{1}{\rho_0} \right) \frac{\nabla \rho(x)}{\rho(x)^2}   & si &\rho(x)\leq \rho_0\\ \\
0 & si & \rho(x) > \rho_0  \end{array} \right. $$

Mientras que el objetivo del campo de atracción es que afecte globalmente, lo que se quiere con el campo repulsivo es que afecte localmente, es decir, en puntos cercanos a la superficie de estos. Esta distancia la definimos como $\rho_0$, y por consiguiente, $\rho(x)$ es la distancia entre el bot y punto más cercano de los obstáculos.

Con el campo potencial definido, el algoritmo consta de un bucle general donde se calcula para cada punto el gradiente del campo potencial. Luego se actualiza la posición del robot en cada iteración con la siguiente ecuación.

$$ x_{siguiente} = x_{actual} - step * F_{total}$$


El bucle finaliza cuando se cumplen una cantidad determinada de iteraciones o cuando la posición del robot es menor a un umbral.

```MATLAB
for iter = 1:max_iter
    F_att = attractive_force(path(end,:), x_goal,d, Katt);
    F_rep = repulsive_force(path(end,:), obstacle_points, rho_0, Krep);
    F_total = F_att + F_rep;
    x = x - step_size * F_total;
    path = [path; x];
    if norm(x - x_goal) <= threshold
        break;
    end
end
```

Donde las funciones de fuerza atractiva y repulsiva se definen de la siguiente forma:

```matlab
function F_att = attractive_force(x, x_goal, d, Katt)
    if norm(x - x_goal) <= d
        F_att = Katt * norm(x - x_goal);
    elseif norm(x - x_goal) > d
            F_att  = d * Katt * (x - x_goal)/ norm(x - x_goal) ; 
    end
end
```

```matlab
function F_rep = repulsive_force(x, obstacle_points, rho_0, Krep)

F_rep = [0, 0];
for i = 1:length(obstacle_points)
    deltas = obstacle_points{i} - x;
    dists = vecnorm(deltas, 2, 2);
    [rho, idx] = min(dists);
    if rho <= rho_0 && rho > 0
        b = obstacle_points{i}(idx,:);
        grad_rho = (x - b) / norm(x - b);
        F_rep = F_rep + Krep *(1/rho - 1/rho_0) * (1/rho)^2 * grad_rho;
    end
end
F_rep = - F_rep;
end
```

## Implementación del algoritmo de planificación.
 

Después de ajustar las ganancias de la fuerza atractiva y repulsiva se implementa el algoritmo para 4 orientaciones: 30°, 45° ,60°, 90°. Los puntos de inicio y fin para los 3 casos es:

- Punto de inicio: (-1.1858, -1.2555)
- Punto objetivo: (1.3253, 1.2555)

La trayectoria encontrada para cada orientación se ve en la siguiente figura. Los colores rojo, verde, azul oscuro y azul claro; hacen referencia a las orientaciones 30°, 45°, 60° y 90° respectivamente.

![trayeectorias](https://github.com/user-attachments/assets/74c8fca6-adc5-4c94-b85b-9121e43cbd52)

Como se puede ver, en un inicio las trayectorias son distintas debido a las diferentes orientaciones iniciales. Sin embargo, las trayectorias convergen en y se vuelven iguales después de unos pasos. Esto debido a que al ser la misma posición inicial, el robot solo necesita modificar su orientación. Una vez se modifica, la trayectoria para todos es la misma.
Los parámetros de atracción y repulsión encontrados son los siguientes:

- $K_{att}: 3.00 $
- $K_{rep}: 1.00 $

Los valores de _d_ y $\rho_0$ son:
- _d_ = 0.05 m
- $\rho_0$ = 0.2 m

El algoritmo completo se puede revisar en [este script](src/tarea3.mlx).

## Implementación de algoritmo de predicción.

Se decidió implementar un segundo algoritmo de planeación, el cual es basado en el mostrado anteriormente con modificaciones para predicción. A diferencia del APF estándar, este algoritmo introduce una predicción que ayuda a escapar de mínimos locales. También hace que el robot no tenga curvas cerradas, haciendo recorridos más cortos.

Este algoritmo funciona de la siguiente manera:
- Predice varios pasos hacia adelante usando el algoritmo estándar de campos potenciales.
- Mide la distancia perpendicular de cada punto predicho a la línea entre el robot y el objetivo.
- Selecciona como nuevo objetivo temporal el punto con mayor desviación.
- Calcula fuerzas atractiva y repulsiva hacia ese objetivo temporal.
- Actualiza la posición del robot con esas fuerzas.
- Repite hasta alcanzar el objetivo.

Consulta el [script de la tarea 3](src/tarea3_v2.mlx) para ver la implementación detallada.

## Mapa del gradiente del campo potencial

Se encontró el mapa general del gradiente del campo potencial para cada punto del mapa, el cual se ve en la siguiente figura.

![grad_campo](https://github.com/user-attachments/assets/4a07d481-3e25-4a73-baa5-d175a8dfa907)


## Animación en matlab de trayectoria 

- [Animación del primer algoritmo](videos/animacion_PurePursuit.mp4)
- [Animación del segundo algoritmo](videos/animacion2_PurePursuit.mp4).


### Simulación CoppeliaSim
Obtenidas las variables “path”, y “robotGoal” con el código anterior, se emplea el siguiente bloque de código para establecer la conexión con CoppeliaSim mediante la API remota, configurando los parámetros cinemáticos del robot diferencial (radio de rueda y distancia entre ejes) e inicializando un controlador "pure pursuit" para seguir la trayectoria predefinida. 

Verificando la posición y orientación actual del robot en la simulación, se procede a calcular las velocidades de los motores necesarias para cumplir con la trayectoria usando el controlador, se envía estos comandos al simulador, y se repite el proceso hasta que el robot alcanza el objetivo dentro de un radio de tolerancia, momento en el cual detiene los motores y finaliza la conexión.

```matlab
% Establecer la conexión
sim=remApi('remoteApi'); % usar el archivo prototipo (remoteApiProto.m)
sim.simxFinish(-1); % si se requiere, cerrar todas las conexiones abiertas.
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); % asigna el handle de identificación de cliente clientID
if (clientID>-1)
    disp('Conexión exitosa')
end

%definicion del control con las constantes del robot
DR12 = differentialDriveKinematics("WheelRadius", 0.086/2,"TrackWidth", 0.128, "VehicleInputs", "VehicleSpeedHeadingRate");

controller = controllerPurePursuit;
release(controller);
controller.Waypoints = path(:,1:2); % <-- trayectoria desde el campo potencial
controller.DesiredLinearVelocity = 0.13; % m/s
controller.MaxAngularVelocity = 3.0;
controller.LookaheadDistance = 0.04;

goalRadius = 0.1;

% Obtiene el handle de los objetos en la escena.
[~, robotHandle] = sim.simxGetObjectHandle(clientID, 'dr12', sim.simx_opmode_blocking);
[~, leftMotor] = sim.simxGetObjectHandle(clientID, 'dr12_leftJoint_', sim.simx_opmode_blocking);
[~, rightMotor] = sim.simxGetObjectHandle(clientID, 'dr12_rightJoint_', sim.simx_opmode_blocking);

sim.simxSynchronous(clientID, true);
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);

vizRate = 0.05; % tiempo de espera entre ciclos

% % Consulta la posición y orientacion inicial del robot
[~, P]= sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming);
[~, Or] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_streaming);

distanceToGoal = norm(P(1:2) - robotGoal(:));

pause(0.6);

while( distanceToGoal > goalRadius )
    
    % Pose actual
    [~, P] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_buffer);
    [~, Or] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_buffer);
    
    % Formato a [x, y, theta]

    robotCurrentPose = [P(1), P(2), Or(3)]';
    
    %salidas del controlador
    [v, omega] = controller(robotCurrentPose);

    % Convertir a velocidad de ruedas
    v_left = (v - omega * DR12.TrackWidth/2) / DR12.WheelRadius;
    v_right = (v + omega * DR12.TrackWidth/2) / DR12.WheelRadius;
    
    % Envio de comandos a CoppeliaSim
    sim.simxSetJointTargetVelocity(clientID, leftMotor, v_left, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rightMotor, v_right, sim.simx_opmode_oneshot);
    
    % actualiza la distancia al objetivo
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    

    waitfor(vizRate);
end

% Detiene al robot cuando alcance el objetivo
sim.simxSetJointTargetVelocity(clientID, leftMotor, 0, sim.simx_opmode_oneshot);
sim.simxSetJointTargetVelocity(clientID, rightMotor, 0, sim.simx_opmode_oneshot);

% Termina el programa y cierra la conexión de MATLAB.
disp('Programa terminado')
sim.delete();

```

El resultado en CoppeliaSim puede ser observado a continuación.

[![Watch the video](https://img.youtube.com/vi/sab9WU4T-UA/0.jpg)](https://www.youtube.com/watch?v=sab9WU4T-UA)


## Conclusiones
- Se logró simular con éxito ouna estrategia de navegación autónoma basada en campos potenciales para el robot DR12. El robot fue capaz de desplazarse desde un punto de inicio a una meta, evitando obstáculos de manera efectiva.
- La implementación en Coppelia del algoritmo de predicción tuvo varios problemas con el control de los motores y las direcciones que tomaba el robot.
- Los parámetros escogidos para el campo potencial fueron adecuadas. Sin embargo, hace falta un método de convergencia para los parámetros para encontrar los mejores que se acomodan al contexto.
