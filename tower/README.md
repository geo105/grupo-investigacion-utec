# Tower: banco torre-hélice de 2 GDL

Un brazo con dos motores *brushless* pivota sobre un carro que sube y baja por dos guías verticales. La suma de los
empujes controla la altura *z* (sensor ultrasónico) y su diferencia, la inclinación *θ* (unidad inercial con filtro
complementario).

Comparamos seis leyes de control en el mismo hardware. Cada una se ajustó para que su controlador de ángulo tenga la
misma ganancia a 6 rad/s (0.95 Hz), de modo que las diferencias reflejen la estructura de la ley y no su sintonía.

**Paper:** *Experimental Comparison of Control Laws on a 2-DOF Tower-Propeller Testbed*, enviado a la 14th
International Conference on Systems and Control (ICSC 2026), en revisión.

## Firmware

| Ley | Carpeta | Fase de Cθ a 6 rad/s |
|---|---|---|
| PID (referencia) | [`firmware/ctrl_1_pid`](firmware/ctrl_1_pid) | +60.8° |
| LQI | [`firmware/ctrl_2_lqi`](firmware/ctrl_2_lqi) | +79.2° |
| LQG | [`firmware/ctrl_3_lqg`](firmware/ctrl_3_lqg) | +78.8° |
| MPC | [`firmware/ctrl_4_mpc`](firmware/ctrl_4_mpc) | +73.3° |
| IMC | [`firmware/ctrl_5_imc`](firmware/ctrl_5_imc) | +48.8° |
| H∞ | [`firmware/ctrl_6_hinf`](firmware/ctrl_6_hinf) | +41.8° |

Los seis sketches son idénticos salvo el bloque de la ley de control. Las ganancias están en `controladores_gen.h`
(generado por el script de diseño; no editar a mano). Todo junto: [`firmware_tower.zip`](firmware_tower.zip).

## Cómo cargarlo

1. Abrir `ctrl_X_ley/ctrl_X_ley.ino` en el Arduino IDE (la carpeta lleva su `controladores_gen.h`).
2. Placa **Arduino Uno**; subir.
3. Monitor Serie a **115200 baudios**.
4. Antes del primer vuelo: `i` (identificar motores), `o` (cero del ángulo con el brazo nivelado) y `h` (estado).
5. `C` arranca y sube por la rampa, `L` aterriza, `S` apaga los motores.

Otros comandos: `z10` consigna de altura en cm, `a0.5` escala del lazo de ángulo (0 a 2), `+`/`-` trim entre
motores.

| Conexión | Pin |
|---|---|
| Motor izquierdo / derecho (ESC) | D10 / D9 |
| Botón de emergencia | D2 |
| Ultrasonido HC-SR04 TRIG / ECHO | D7 / D6 |
| IMU (I²C, 0x68) | A4 / A5 |
| Tensión de las fuentes izquierda / derecha | A0 / A1 |

> **Seguridad:** prueba siempre con el banco fijado, sin manos cerca de las hélices y con el botón de emergencia a
> mano. IMC y H∞ conviene volarlos primero con `a0.5`.
