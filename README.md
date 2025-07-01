# Navegación Campo Potencial

Autores: 
- Eduardo Cuadros
- Nicolás Moreno

El propósito de esta tarea fue resolver y simular una misión de navegación autónoma para un robot móvil con ruedas, utilizando el método de navegación por campo potencial artificial. Este método combina fuerzas atractivas (que llevan al robot hacia la meta) y repulsivas (que lo alejan de los obstáculos) para generar trayectorias seguras en entornos con obstáculos. Se trabajó específicamente con el robot DR12, tanto en MATLAB como en CoppeliaSim.

2. Modelo Cinemático del Robot

Se inició creando el modelo matemático del robot DR12 que describe cómo se mueve en función de las velocidades de sus ruedas. Este modelo se implementó en MATLAB usando la función _differentialDriveKinematics_, que requiere como parámetros el radio de las ruedas y la distancia entre ellas. Las dimensiones del robot se obtuvieron del modelo disponible en CoppeliaSim.

El modelo permite simular cómo reacciona el robot ante distintos comandos de velocidad lineal y angular, lo cual es fundamental para luego aplicar el control de navegación.

2.1 Dimensiones físicas
- Largo (X): 0.23 m
- Ancho (Y): 0.158 m
- Alto (Z): 0.1406 m
- Radio de rueda: 0.086 m
- Espesor de rueda: 0.026 m
- Distancia entre ruedas: 0.154 m


3. Descripción del Campo Potencial

$$ (1)\ U_x = U_{att}(x) + U_{rep}(x)        $$

El campo potencial atractivo se define como una función a trozos. Esta función está divida por una distancia _d_, la cual es el cambio entre _distancia corta_ y _distancia larga_. Esto se hace con el objetivo de que la fuerza atractiva no crezca indefinidamente cuando el robot está lejos.


$$ (2)\ U_{att}(x)= \left\{ \begin{array}{lcc} \frac{1}{2}K_{att} ||x - x_{goal}||{^2} & si & ||x - x_{goal}|| \leq d\\ \\ dK_{att}||x - x_{goal}||{^2} - \frac{1}{2}K_{att}d{^2} & si & ||x - x_{goal}||> d  \end{array} \right. $$

La fuerza atractiva se define como el gradiente del campo potencial, como se muestra a continuación:

$$ (3)\ \nabla U_{att}(x)= \left\{ \begin{array}{lcc} K_{att} ||x - x_{goal}|| & si & ||x - x_{goal}|| \leq d\\ \\ \frac{dK_{att}(x - x_{goal})}{ ||x - x_{goal}||}  & si & ||x - x_{goal}||> d  \end{array} \right. $$

De igual manera se definen el campo potencial repulsivo y su correspondiente fuerza repulsiva de la siguiente forma.

$$ (4)\ U_{rep}(x)= \left\{ \begin{array}{lcc} \frac{1}{2}K_{rep} \left( \frac{1}{\rho(x)} - \frac{1}{\rho_0} \right)^2   & si &\rho(x)\leq \rho_0\\ \\0 & si & \rho(x) > \rho_0  \end{array} \right. $$

$$ (5)\ \nabla U_{rep}(x)= \left\{ \begin{array}{lcc} -K_{rep} \left( \frac{1}{\rho(x)} - \frac{1}{\rho_0} \right) \frac{\nabla \rho(x)}{\rho(x)^2}   & si &\rho(x)\leq \rho_0\\ \\0 & si & \rho(x) > \rho_0  \end{array} \right. $$

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

### Implementación del algoritmo de planificación.
 

Después de ajustar las ganancias de la fuerza atractiva y repulsiva se implementa el algoritmo para 4 orientaciones: 30°, 45° ,60°, 90°. Los puntos de inicio y fin para los 3 casos es:

- Punto de inicio: (-1.1858, -1.2555)
- Punto objetivo: (1.3253, 1.2555)

La trayectoria encontrada para cada orientación se ve en la siguiente figura:

INSERTAR MAPA DE TRAYECTORIAS

Los parámetros de atracción y repulsión encontrados son los siguientes:

- $K_{att}: 3.00 $
- $K_{rep}: 1.00 $

### Mapa del gradiente del campo potencial

Se encontró el mapa general del gradiente del campo potencial para cada punto del mapa, el cual se ve en la siguiente figura.

