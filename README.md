<img src="https://github.com/pedrotagliani/self-balancing-robot-with-marker-detection/blob/main/Images/FI%20UNLZ%20Logo.jpg" alt="FI UNLZ"/>


<h1 align="center">Robot balanceador con detección de marcadores ArUco</h1>

<p>
En este proyecto, se diseñó y construyó un robot balanceador con dos funcionalidades principales. En primer lugar, se implementó un sistema de control remoto mediante una aplicación móvil, que permite mover el robot libremente en cualquier dirección. En segundo lugar, se desarrolló un sistema de equilibrio dinámico que utiliza una cámara para detectar marcadores ArUco y avanzar automáticamente cuando identifica uno en la imagen capturada.

El sistema de control de balance se basó en un controlador PID, ajustado para mantener al robot erguido en todo momento. Para medir el ángulo de inclinación, se utilizó un sensor MPU6050, que combina acelerómetro y giroscopio. Los datos obtenidos por este sensor fueron procesados en un Arduino Nano, que calculaba las señales de control necesarias para los motores. Además, se diseñaron y fabricaron placas PCB personalizadas para integrar los componentes eléctricos del sistema de manera eficiente.

El robot utilizó dos motores paso a paso NEMA 17, seleccionados por su precisión y torque, junto con controladores Pololu A4988 que gestionaron la dirección y velocidad de los motores. Para la comunicación remota, se incorporó un módulo Bluetooth HC-05, que permitió manejar el robot desde una aplicación móvil con una interfaz tipo joystick, facilitada por la biblioteca BalancingWii. Este sistema proporcionó un control intuitivo y responsivo del robot mediante una conexión estable y confiable.

El equilibrio dinámico se logró gracias al controlador PID, que ajustaba de manera constante la velocidad de los motores en función de la inclinación detectada por el sensor. Este algoritmo permitió que el robot se adaptara rápidamente a cambios en el terreno o a variaciones en su carga, manteniéndose estable en diversas condiciones.

En un segundo código, se integraron nuevas funcionalidades para dotar al robot de mayor autonomía mediante el uso de una cámara ESP32-CAM. Este sistema permitió que el robot identificara marcadores ArUco en su entorno y actuara en consecuencia. La comunicación entre el ESP32-CAM y un script en Python, ejecutado en una computadora, se estableció a través de WiFi, utilizando una URL específica para acceder a las imágenes capturadas.

El script en Python procesa estas imágenes en tiempo real, tras realizar la calibración de la cámara mediante tableros ChArUco. Posteriormente, se emplean funciones avanzadas de OpenCV para detectar marcadores ArUco en las imágenes procesadas.

Finalmente, este script se conecta con el robot mediante Bluetooth, permitiendo que el robot recibiera alertas cada vez que se detectaba un marcador en las imágenes. Con esta información, el robot avanzaba de manera autónoma hacia el marcador identificado, logrando un desplazamiento eficiente y preciso hacia el objetivo.
</p>

<img src="https://github.com/pedrotagliani/self-balancing-robot-with-marker-detection/blob/main/Images/Self-balancing%20robot.png" alt="Robot balanceador"/>

---

## Integrantes
- Daiana Belén Viscarra Hernández.
- Pedro Tagliani.
