// Generado por diseno_controladores.py. No editar a mano.
// Tower: banco torre-helice de 2 GDL - Grupo de Investigacion UTEC.
// Ts = 5 ms (200 Hz). Las seis leyes tienen la misma |C| a 6 rad/s (0.95 Hz).
#ifndef CONTROLADORES_GEN_H
#define CONTROLADORES_GEN_H
#include <avr/pgmspace.h>

// ---- 0) PID (referencia de la comparacion) ---------------------------
#define PID_TH_KP                  0.7f
#define PID_TH_KD                 0.22f
#define PID_TH_KI                  0.4f
#define PID_Z_KP                   2.0f
#define PID_Z_KD                   0.0f
#define PID_Z_KI                   2.6f

// ---- 1) LQI : LQR + integrador, estados medidos -----------------------
#define LQI_TH_KP           0.26835637f
#define LQI_TH_KD           0.23827994f
#define LQI_TH_KI           0.11652637f
#define LQI_Z_KP             2.9993049f
#define LQI_Z_KD             0.1906949f
#define LQI_Z_KI             1.5189156f

// ---- 2) LQG : LQI + observador de Kalman ------------------------------
// Rn = varianza medida en vuelo de las senales que recibe el observador
// (theta_filt: 0.25 deg^2; z_filt: 0.025 cm^2). Las K se ajustan por
// biseccion sobre el compensador completo, observador incluido, para que la
// |C| a 6 rad/s sea la misma que la del PID.
#define LQG_TH_KP           0.26867937f
#define LQG_TH_KD           0.23856674f
#define LQG_TH_KI           0.11666663f
#define LQG_TH_FUS                0.85f   // peso del giroscopio en el estado de velocidad
// TH: Ad discreto
const float LQG_TH_AD[4] PROGMEM = {0.99999394f, 0.004987411f, -0.0024238818f, 0.99496662f};
const float LQG_TH_BD[2] PROGMEM = {1.3340066e-05f, 0.0053315424f};
// TH: ganancia Kalman x Ts
const float LQG_TH_LD[2] PROGMEM = {0.048630579f, 0.23149332f};
#define LQG_Z_KP             5.3934782f
#define LQG_Z_KD            0.34291573f
#define LQG_Z_KI              2.731379f
// Z: Ad discreto
const float LQG_Z_AD[4] PROGMEM = {1.0f, 0.0047357941f, 0.0f, 0.89621271f};
const float LQG_Z_BD[2] PROGMEM = {0.00027873542f, 0.10949487f};
// Z: ganancia Kalman x Ts
const float LQG_Z_LD[2] PROGMEM = {0.012716217f, 0.011170218f};

// ---- 3) MPC : horizonte finito, ley precalculada ----------------------
// La solucion sin restricciones activas es una ganancia de estado, obtenida
// por recursion de Riccati hacia atras (en 2 KB de RAM no cabe resolver un QP
// a 200 Hz). Las restricciones las aplica el firmware: gobernador de
// referencia, limitador de pendiente y saturacion con prioridad al angulo.
#define MPC_TH_KP           0.41224704f
#define MPC_TH_KD           0.23032113f
#define MPC_TH_KI          0.040967724f
#define MPC_Z_KP             3.2597181f
#define MPC_Z_KD             0.2424002f
#define MPC_Z_KI            0.60878494f

// ---- 4) IMC : control por modelo interno, un solo parametro -----------
// El biquad incluye el reescalado que compensa el termino de amortiguamiento,
// de modo que la |C| queda igualada con ctrl_esc = 1.
// theta: b0 b1 b2 a1 a2 (a0 = 1), reescalado
const float IMC_TH_BQ[5] PROGMEM = {1.0308707f, -2.0565464f, 1.0256882f, -1.9884125f, 0.98841249f};
// z: b0 b1 b2 a1 a2 (a0 = 1)
const float IMC_Z_BQ[5] PROGMEM = {1.7668273f, -3.3438694f, 1.5776901f, -1.9316813f, 0.93168132f};
#define IMC_TH_KD           0.14784734f   // amortiguamiento directo de gyro, us/(deg/s)
#define IMC_LAM_TH           1.4168486f   // s
#define IMC_LAM_Z           0.70686431f   // s

// ---- 5) H-inf : Glover-McFarlane, factores coprimos normalizados ------
// gamma_min se obtiene en forma cerrada; 1/gamma es el margen de
// incertidumbre coprima admisible. Lazo cerrado verificado numericamente.
#define HINF_TH_KD          0.33005335f   // amortiguamiento directo de gyro
// TH: Ad discreto
const float HINF_TH_AD[16] PROGMEM = {0.9944218f, -0.048539293f, -0.0050538077f, 0.0f, 0.00099849046f, 0.95961392f, 0.0037721843f, 0.0f, 0.0023347518f, -0.052994304f, 0.99723308f, 0.0f, 0.0055551397f, 0.0035631841f, 0.0049665665f, 1.0f};
const float HINF_TH_BD[4] PROGMEM = {0.044795218f, 0.034559157f, 0.048633497f, 0.0003154805f};
const float HINF_TH_C[4] PROGMEM = {0.57375977f, 0.40345833f, 0.51363778f, 1.0314167f};
#define HINF_TH_D                 -0.0f   // paso directo
// Z: Ad discreto
const float HINF_Z_AD[16] PROGMEM = {0.98908997f, -0.10193513f, -0.0013653434f, 0.0f, 0.00012278427f, 0.83438419f, 0.0045617011f, 0.0f, 0.051870155f, -1.4868783f, 0.98974271f, 0.0f, 0.010905363f, -0.00072565517f, 0.0011015293f, 1.0f};
const float HINF_Z_BD[4] PROGMEM = {0.10695487f, 0.065301503f, 1.5664f, 0.0015005084f};
const float HINF_Z_C[4] PROGMEM = {0.54680475f, 0.038408327f, 0.055587848f, 1.0f};
#define HINF_Z_D                  -0.0f   // paso directo
#define HINF_TH_N        4   // orden del ctrl de angulo
#define HINF_Z_N         4   // orden del ctrl de altura

// ---- TABLA: ley|C_th|fase_th|C_z|fase_z  (la lee generar_sketches.py) ---
// TABLA PID|1.4356|+60.8|3.2802|-52.4
// TABLA LQI|1.4356|+79.2|3.2802|-23.9
// TABLA LQG|1.4356|+78.8|3.2802|-62.1
// TABLA MPC|1.4356|+73.3|3.2802|-6.4
// TABLA IMC|1.4356|+48.8|3.2802|-36.7
// TABLA Hinf|1.4356|+41.8|3.2802|-49.8

#endif  // CONTROLADORES_GEN_H
