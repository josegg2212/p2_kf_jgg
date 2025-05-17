# p2_kf_jgg
## Ejecución del proyecto
Clonar repositorio en `src` del workspace. 

Hacer `colcon_build` y `source install/setup.zsh`.

Ejecutar simulador:
 ```bash
    ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
 ```
Ejecutar modelo de estimación 3D:
 ```bash
    ros2 run p2_kf_adr kf_estimation
 ```

Ejecutar modelo de estimación 6D:
 ```bash
    ros2 run p2_kf_adr kf_estimation_vel
 ```
## Descripción del proyecto

El paquete contiene la implementación de un filtro de Kalman para estimación de la trayectoria de un robot móvil, empleando dos modelos de estado:

1. **Modelo 3D**: estado \[x, y, θ]. Implementado en `kf_estimation.py`.
2. **Modelo 6D** (Pure KF con velocidades): estado \[x, y, θ, vx, vy, ω] (velocidades lineal y angular añadidas). Implementado en `kf_estimation_vel.py`.

El núcleo del filtrado está en `kalman_filter.py`, donde se definen las clases:

* `KalmanFilter` (3D): Prediction y update con matrices A, B, C, R, Q.
* `KalmanFilter_2` (6D): Extensión a 6 dimensiones, con dinámica interna de velocidades.

Complementan la lógica módulos de utilidad:

* **Motion models** (`motion_models.py`): retorna matrices del modelo de movimiento A y B.
* **Observation models** (`observation_models.py`): define matrices de medición C.
* **Sensor utils** (`sensor_utils.py`): conversiones de `Odometry` ROS a pose 2D, normalización y generación de medidas ruidosas.
* **Visualizer** (`visualization.py`): despliega en tiempo real la estimación vs trayectoria real.

## Estructura de la implementación

1. **Inicialización**

   * Se define un estado inicial (`mu`) y covarianza (`sigma`) con incertidumbre moderada.
   * Se configuran las desviaciones estándar de ruido de proceso y observación.
   * Se crean instancias de las clases de filtro y visualizador.

2. **Callback de Odometría**

   * Se extrae la pose real (`odom_to_pose2D`) y se normaliza respecto al estado inicial.
   * Se computa el `dt` entre muestras.
   * Se construye la entrada de control `u` (velocidad lineal y angular).
   * Se genera la medición ruidosa `z` (pos.x, pos.y y ángulo para 3D; + velocidades para 6D).

3. **Paso de predicción**

   * Se invocan `predict(u, dt)`, internamente:

     * Se obtiene A(dt) y B(μ,dt).
     * Se calcula `mup = A·μ + B·u`.
     * Se actualiza covarianza `sigmap = A·σ·Aᵀ + R`.

4. **Paso de actualización**

   * Se invoca `update(z)`, internamente:

     * Cálculo de la ganancia de Kalman K.
     * Estado corregido `μ = mup + K·(z − C·mup)`.
     * Covarianza nueva `σ = (I − K·C)·sigmap`.

5. **Publicación y visualización**

   * Se publica el estado estimado en un topic ROS (`PoseWithCovarianceStamped`).
   * Se actualiza la visualización superponiendo la estimación y la trayectoria real.

## Experimentos y resultados

Se han realizado seis experimentos, tres con cada modelo (3D y 6D), variando la configuración de ruido. También hay que tener en cuenta que el modelo 3D es bastante bueno debido a que se han realizado aproximaciones. 

1. **Ruido bajo (baseline)**

   * Ruido de proceso y de observación pequeños, es decir, nos fiamos bastante del modelo y de la medición.
   * Ambas estimaciones (3D y 6D) siguen de cerca la trayectoria real, con ligeras oscilaciones. Esto es así debido a que aunque los modelos no sean los mejores, la medición corrige los errores, ya que esta viene de la odometría y es muy fiable. El problema vendrá cuando solo nos fiemos mucho en el modelo.
   * La dispersión de la estimación crece muy poco con el tiempo.

2. **Ruido alto en la medida (obs)**

   * Incremento de la desviación estándar de la observación (Q grande).
   * El filtro confía menos en las medidas y más en el modelo de movimiento. Por tanto, el modelo 6D tendrá problemas en la estimación, ya que el modelo no es muy bueno. Sin embargo, como el modelo 3D es bastante bueno ya que es una aproximación, seguirá muy bien la referencia.
   * Se observa que la estimación suaviza lecturas ruidosas, debido a que no se introduce practicamente nada el ruido de los sensores de medición, pero con cierto retraso al seguir cambios bruscos.

3. **Ruido alto en el proceso (proc)**

   * Incremento de la desviación estándar del modelo de proceso (R grande).
   * El filtro reconoce mayor incertidumbre en la predicción y gana más peso a las mediciones. Se observa mucho más ruido debido a la medición en los dos modelos, pero el modelo 6D va a seguir mejor la referencia ya que la medición es mejor que los resultados del modelo.
   * Resultado: estimación más fluctuante, siguiendo más de cerca la medición ruidosa.

## Análisis de resultados

* **Ruido bajo**: condiciones ideales donde modelo y sensores son fiables; la combinación predict–update converge rápidamente y mantiene baja incertidumbre.

* **Ruido alto en la observación**: elevar Q reduce la confianza en sensores. El filtro se comporta como un suavizador (low‑pass), priorizando la dinámica. Por eso atenúa picos, pero sufre lag ante cambios rápidos.

* **Ruido alto en el proceso**: elevar R indica incertidumbre en la predicción; el filtro pesa mucho la medición, resultando en una estimación más oscilante. Si la medición es ruidosa, la estimación hereda parte de ese ruido.







