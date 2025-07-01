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

3. Generación del Mapa de Obstáculos

4. Navegación por Campo Potencial
