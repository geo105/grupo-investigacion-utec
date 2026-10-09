// =========================================================================
//  Tower: banco torre-helice de 2 GDL - Grupo de Investigacion UTEC
//  ctrl_2_lqi.ino - ley LQI (regulador LQ con accion integral)
//
//  Planta: un brazo con dos motores brushless pivota sobre un carro que se
//  desliza por dos guias verticales. El empuje comun de los motores controla
//  la altura z y el empuje diferencial, la inclinacion theta.
//
//  Hardware: Arduino Uno, IMU MPU de 6 ejes (I2C), sensor ultrasonico HC-SR04,
//  dos ESC de 30 A (1000-2000 us) y dos fuentes de 12 V, una por motor.
//
//  Estructura del firmware, comun a las seis leyes:
//    - Lazo de control a 200 Hz. Ultrasonido a 50 Hz con estimador alfa-beta
//      multitasa de (z, vz) y rechazo de valores atipicos por innovacion.
//    - theta por filtro complementario, con compuerta por modulo del
//      acelerometro y estimacion en linea de la deriva del giroscopio.
//    - Despegue en dos fases: nivelacion del brazo y busqueda del empuje de
//      hover en lazo abierto. El integrador de altura se inicializa con ese
//      empuje, lo que evita el windup contra el piso.
//    - Gobernador de referencia: la consigna de altura solo avanza si la
//      planta la sigue (retraso < MAX_LAG) y el angulo esta dentro de margen.
//    - Anti-windup por back-calculation en ambos lazos, con el mando
//      realmente aplicado.
//    - Mezcla con prioridad al angulo: si un motor satura se recorta el modo
//      comun (altura), nunca el diferencial (angulo).
//    - Limitador de pendiente del modo comun y techo de empuje adaptativo
//      segun la tension de las fuentes.
//    - Aterrizaje con rampa y posado nivelado. Abortos por limites de angulo
//      y de altura, perdida del sensor de altura y caida de la fuente.
//
//  Entre los seis sketches solo cambia el bloque de la ley de control, mas
//  abajo. Las ganancias se generan con el script de diseno y estan en
//  controladores_gen.h.
//
//  Monitor Serie a 115200 baudios. Comandos:
//    C  arrancar             L  aterrizar             S  apagar motores
//    z<cm>    consigna de altura          t<deg>  consigna de angulo
//    v<cm/s>  velocidad de subida         a<k>    escala del lazo de angulo (0-2)
//    w<deg>   ventana de angulo           e<us>   techo del modo comun
//    +/-      trim entre motores (5 us)   m<us>   trim absoluto
//    h  estado                            o       recalibrar el cero del angulo
//    i  identificar motores               d<us>   prueba en lazo abierto
//    n  invertir el sentido del lazo      p<us>   empuje comun de la prueba d
//    b<V>     calibrar la medida del riel de 5 V con un multimetro
//    k, g, r  ajuste de ganancias en caliente; depende de la ley (ver comandos())
//    Tras una parada de emergencia, 'r' rearma el sistema.
//
//  Telemetria: una linea CSV cada 100 ms; la cabecera se imprime con C.
// =========================================================================

#include <Servo.h>
#include <Wire.h>
#include "controladores_gen.h"

// --- PINES ---
const int PIN_MOTOR_IZQ = 10;
const int PIN_MOTOR_DER = 9;
const int PIN_BOTON     = 2;
const int PIN_TRIG      = 7;
const int PIN_ECHO      = 6;
uint8_t   MPU_ADDR      = 0x68;

// =========================================================================
//  AJUSTES
// =========================================================================
const int PWM_MIN = 1000, PWM_MAX = 2000;
// Limites del PWM de los ESC (us). PWM_MAX_SEG es el tope de seguridad; por
// debajo de el actua ademas el techo adaptativo pwm_techo, que se reduce si la
// tension de las fuentes cae (ver VIGILANCIA DE LA FUENTE).
int   PWM_MAX_SEG = 1850;     // tope de seguridad del PWM de cada motor
int   PWM_BASE    = 1250;     // base del modo comun: con Uz = 0 el brazo reposa en el piso

// --- SETPOINTS ---
float SP_z_target = 5.0;     // altura objetivo (cm)
float SP_th       = 0.0;      // angulo objetivo (deg)

// --- LAZO DE ANGULO (theta): ganancias PID ---
// Valores en controladores_gen.h. Solo los usa la ley PID, que los ajusta en
// caliente con 'g' (Kp_th) y 'k' (Kd_th).
float Kp_th = PID_TH_KP, Ki_th = PID_TH_KI, Kd_th = PID_TH_KD;
float D_TH_MAX  = 25.0;       // tope del aporte derivativo (us)
                              // El pivote tiene friccion seca: al despegarse de
                              // golpe, el giroscopio registra un pico que el
                              // termino derivativo convertiria en un tiron.
float U_TH_MAX  = 180.0;      // aporte diferencial maximo (us)
float ITH_MAX = 44.0f / LQI_TH_KI;   // limite del integrador en vuelo (44 us de mando)
float ITH_MAX_PISO = 80.0f / LQI_TH_KI;   // limite en tierra (80 us de mando):
                              // la integral despega el brazo de su tope
float ITH_BANDA = 15.0;       // banda de integracion condicional (debe cubrir el angulo
                              // de reposo del brazo para que actue en tierra)
float AW_TH     = 2.0;        // ganancia de back-calculation del anti-windup

// --- LAZO DE ALTURA (z): ganancias PID ---
// Kd_z = 0 a proposito: la planta ya tiene un polo rapido (en -21.9) y la
// derivada de altura, que pasa por el retardo del estimador, introduce un
// modo oscilatorio (zeta = 0.555 y Mp = 12.3 % con Kd_z = 4). Sin ella el polo
// dominante es real. El sobrepaso en el despegue se evita con el gobernador
// de referencia y la captura del hover. En la ley PID, Kp_z se ajusta en
// caliente con 'r'.
float Kp_z = PID_Z_KP, Ki_z = PID_Z_KI;
float Kd_z = 0.0;             // se mantiene en 0 (ver arriba)
float UZ_MAX      = 400.0;    // aporte de altura maximo (us), se recalcula en setup()
float UZ_CAIDA    = 60.0;     // margen de empuje por debajo del hover capturado
float AW_Z        = 1.30;     // back-calculation (1/Tt, con Ti = Kp/Ki = 0.77 s)
// Limitador de pendiente del modo comun: acota la rapidez con que cambia la
// corriente pedida a las fuentes. El diferencial apenas cambia la corriente
// total, porque sube un motor y baja el otro.
float SLEW_UZ_UP  = 90.0;             // us/s de subida del modo comun
float SLEW_UZ_DN  = 600.0;    // us/s de bajada (cortar empuje siempre es seguro)

// --- PERFIL DE SUBIDA ---
float RAMP_CMS    = 0.20;             // cm/s de subida de la consigna
float BAJA_CMS    = 0.35;             // cm/s de bajada de la consigna
float MAX_LAG     = 1.5;      // gobernador: maximo adelanto de la consigna sobre z (cm)
float ANG_OK_NIVEL= 4.0;      // |theta| medio para pasar de nivelar a buscar hover
float ANG_OK_SUBIR= 6.0;      // |theta| medio por encima del cual la rampa se detiene
int   CICLOS_OK   = 200;      // ciclos estables antes de buscar hover (~1 s)

// --- NIVELACION Y BUSQUEDA DE HOVER ---
// Valores en PWM absoluto, no relativos a PWM_BASE.
int   PWM_NIVEL     = 1250;   // empuje comun durante la nivelacion. Debe quedar por
                              // encima de la zona muerta del ESC o el lazo de angulo
                              // no tiene autoridad y la maquina de fases se atasca.
int   PWM_BUSQ_MAX  = 1650;   // si no despega en este PWM comun -> aborta
int   PWM_COMUN_MAX = 1800;   // techo del modo comun (de aqui sale UZ_MAX en setup)
                              // Ajustable en caliente con 'e'.
float TASA_NIVEL    = 10.0;           // us/s de la rampa de nivelacion. Si el brazo no
                              // nivela, el empuje sube: con mas empuje, cada us
                              // de diferencial produce mas par.
float TASA_BUSQ     = 15.0;           // us/s de la rampa de busqueda de hover
float DZ_DESPEGUE   = 1.2;    // cm por encima del piso para declarar despegue

// --- DESCUENTO DEL EMPUJE AL DESPEGAR ------------------------------------
// Cuando z supera el piso en DZ_DESPEGUE el brazo ya sube con velocidad vz, asi
// que el empuje de ese instante es mayor que el de sustentacion. Antes de
// inicializar el integrador se descuenta K_VZ_HOVER * vz. El descuento es
// pequeno y acotado a proposito: sobrestimar el hover produce un sobrepico que
// el lazo corrige, mientras que subestimarlo deja al lazo sin autoridad para
// sostener el brazo.
float K_VZ_HOVER    = 1.5;    // unidades de empuje por cada cm/s de subida
float EXCESO_MAX    = 20.0;   // tope del descuento, por si vz viene con ruido

// --- POSADO NIVELADO -----------------------------------------------------
// Al tocar el piso, el empuje comun se retira con rampa mientras el lazo de
// angulo sigue activo. Asi el brazo queda nivelado y la siguiente calibracion
// del cero es correcta.
float UZ_POSAR      = 120.0;  // unidades/s con que se retira el empuje al posar
float POSAR_ANG     = 2.5;    // |theta| que se considera "nivelado" para cortar
unsigned long POSAR_MS = 4000;  // tiempo maximo de posado; despues se apaga igual
unsigned long NIVEL_MAX_MS = 30000;   // tiempo maximo de la fase de nivelacion

// --- SENTIDO DEL LAZO DE ANGULO ---
// +1 o -1. Con SIGNO_TH = +1 la mezcla asume que acelerar el motor derecho
// lleva theta hacia positivo. Depende del cableado y del montaje: se verifica
// con la prueba en lazo abierto 'd30' (ver MODO DIAGNOSTICO) y se puede
// invertir en caliente con 'n'. Un signo equivocado convierte el lazo en
// realimentacion positiva.
// Con el montaje actual (izquierdo en D10, derecho en D9), acelerar el motor
// derecho lleva theta hacia negativo; por eso SIGNO_TH = -1.
int   SIGNO_TH     = -1;

// --- COMPENSACION DE ASIMETRIA ---
// TRIM_BASE compensa la diferencia de empuje entre motores y puede ser
// negativo. Se determina con la prueba 'd0': sin diferencial, el lado hacia el
// que cae el brazo indica que motor empuja de mas. El valor se obtuvo en banco
// con los ESC actuales y debe repetirse si se cambian ESC, motores o helices.
// Ajuste en caliente con '+'/'-' (5 us) o 'm<us>'.
int   TRIM_BASE    = -6;
// Dependencia opcional del trim con el empuje comun (desactivada con K_ASIM = 0).
float K_ASIM       = 0.0;
int   PWM_REF_ASIM = 1300;
int   TRIM_MAX     = 200;     // cota de magnitud del trim (us)

// =========================================================================
//  VIGILANCIA DE LA FUENTE  (12 V / 5 A)
//
//  El bandgap del AVR solo mide el riel de 5 V del Arduino: una caida de la
//  fuente de 12 V no se ve ahi porque el regulador la absorbe. Para medir las
//  fuentes de los motores se usa un divisor resistivo por fuente:
//
//      12V ---[ R1 = 10k ]---+---[ R2 = 4.7k ]--- GND
//                            |
//                           A0
//
//  Con esos valores, 12 V dan 3.84 V en el pin (maximo medible: 15.6 V). La
//  lectura se escala con el Vcc medido por el bandgap, de modo que no se
//  falsea si el propio riel de 5 V varia.
//
//  Hay una fuente por motor: A0 = motor izquierdo, A1 = motor derecho. Medirlas
//  por separado permite distinguir los casos:
//    - solo A0 cae          -> la fuente izquierda llega a su limite de corriente
//    - solo A1 cae          -> la fuente derecha
//    - ambas estables y cae el riel de 5 V -> falla la alimentacion del Arduino;
//      los pulsos de los ESC se corrompen y ambos motores pierden empuje a la vez
//
//  USAR_VBAT = 1 solo con los divisores cableados: sin ellos los pines quedan
//  flotando. Con USAR_VBAT = 0 la proteccion usa solo el riel de 5 V.
// =========================================================================
#define USAR_VBAT 0           // 1 con los dos divisores cableados
const int PIN_VBAT_L = A0;    // divisor de la fuente del motor IZQUIERDO
const int PIN_VBAT_R = A1;    // divisor de la fuente del motor DERECHO
const float DIV_R1 = 10000.0, DIV_R2 = 4700.0;
const float DIV_RATIO = (DIV_R1 + DIV_R2) / DIV_R2;      // 3.128

float VBAT_SAG   = 10.5;      // por debajo: la fuente esta en su limite de corriente
float VBAT_MIN   = 9.5;       // por debajo: caida franca -> aterrizar
// Umbral del riel de 5 V para detectar un colapso de la alimentacion del
// Arduino (un brownout cae hacia 3 V). Ajustado a la tension real del riel en
// este banco, cercana a 4.35 V.
float VCC5_MIN   = 4.15;
// Constante del bandgap: Vcc = VCC5_K / ADC. El nominal es 1.1 V * 1024 = 1125.3,
// pero la referencia interna varia +-10 % entre chips. Se calibra una vez con
// el comando 'b': se mide el pin 5V con un multimetro y se envia, p. ej., 'b5.02'.
float VCC5_K     = 1125.3;
int   VBAT_CICLOS = 6;        // ciclos seguidos bajo umbral antes de actuar (~30 ms)

// --- TECHO DE EMPUJE ADAPTATIVO ---
// Si la tension de la fuente cae, el techo del PWM comun baja hasta donde la
// fuente se sostiene y luego se recupera despacio. Asi se aprovecha una fuente
// limitada en corriente sin que entre en proteccion.
float PWM_TECHO_MIN = 1250.0; // minimo del techo (debe permitir sostener el brazo)
float TECHO_BAJA = 250.0;     // us/s de recorte cuando la fuente se hunde
float TECHO_SUBE =  40.0;     // us/s de recuperacion (lento a proposito)

// --- MODO DIAGNOSTICO ---
unsigned long DIAG_MS = 3000; // duracion de cada prueba de diagnostico

// --- SEGURIDAD ---
// Envolvente mecanica: los topes fisicos del brazo estan en ~-18 y ~+26
// grados. No se modifica por comando.
float TH_MEC_NEG = -17.0;
float TH_MEC_POS =  25.0;
// Ventana de trabajo alrededor de SP_th (no del cero), recortada siempre por
// la envolvente mecanica. Permite ensayar consignas de angulo distintas de
// cero. Ajustable en caliente con 'w'.
float TH_VENT_NEG = 16.0;
float TH_VENT_POS = 20.0;
float Z_TOPE_SEG   =  42.0;   // tope de altura; el fisico esta en ~45 cm
// |theta - SP_th| sostenido por encima de ANG_MALO -> aterrizaje preventivo.
float ANG_MALO     =  12.0;
unsigned long ANG_MALO_MS = 1500;
unsigned long Z_WD_MS     = 400;   // sin lectura valida de z -> aborta
unsigned long BUSQ_MAX_MS = 35000;    // tiempo maximo de la busqueda de hover

// --- FILTROS ---
const float ALPHA = 0.995;    // filtro complementario (tau ~1 s a 200 Hz)
const float BETA  = 0.7;      // pasabajos extra de theta
// Filtro del giroscopio: dos EMA en cascada con tau = dt*FG/(1-FG) = 11.7 ms
// por etapa. En vuelo la planta tiene un modo resonante cercano a 1 Hz; a esa
// frecuencia el filtro introduce ~9 grados de retraso en el termino derivativo,
// que es el que aporta el amortiguamiento.
const float FG    = 0.70;     // gyro: 2 etapas -> 2do orden
// --- ESTIMADOR ALFA-BETA DE ALTURA ---
// Predice (z, vz) a la tasa del lazo con un modelo de velocidad constante y
// corrige solo cuando llega una medida nueva del ultrasonido. Frente a una
// media movil con diferencia sobre ventana, reduce a la mitad el desfase de
// vz (-21.6 frente a -39.6 grados a 0.5 Hz, en simulacion).
const unsigned long ULTRA_MS = 20;   // 50 Hz. A mayor frecuencia el HC-SR04 recibe ecos
                                     // del pulso anterior, y pulseIn bloquea el lazo.
const float ALFA_AB = 0.25;          // ganancia de correccion de posicion
const float BETA_AB = ALFA_AB * ALFA_AB / (2.0 - ALFA_AB);  // Benedict-Bordner
const float MAX_INNOV_CM = 4.0;      // rechazo de outliers por innovacion (contra la
                                     // PREDICCION, no contra la ultima muestra)
const float GYRO_SCALE = 131.0;
const float ACC_LSB    = 4096.0;  // LSB/g con el rango en +-8g
const float ACC_TOL    = 0.20;    // tolerancia del modulo: |a| debe estar en 1.00 +- esto
                                  // para que la muestra cuente como "hacia donde esta abajo"
// Limite de la velocidad de correccion por acelerometro (deg/s). La compuerta
// de modulo no basta: una vibracion rectificada puede dar |a| = 1 g apuntando
// en otra direccion. Como el acelerometro solo debe corregir la deriva lenta
// del giroscopio, limitar su correccion a 2 deg/s acota el efecto de esas
// muestras.
const float CORR_MAX_DPS = 2.0;

// --- DERIVA DEL CERO DEL GIROSCOPIO ---------------------------------------
// El cero del giroscopio deriva: entre dos calibraciones de una misma sesion se
// midieron diferencias de ~1.6 deg/s. La correccion del filtro complementario
// no basta en vuelo, porque solo actua con las muestras que pasan la compuerta
// y su efecto queda en 0.1-0.4 deg/s.
// Por eso la deriva se estima en linea y se descuenta de gyro_offset. Es un
// lazo lento (constante de ~20 s), unas 120 veces mas lento que 6 rad/s, la
// frecuencia donde se igualaron las seis leyes, y no altera la comparacion: a
// 6 rad/s la ganancia del filtro pasa de 0.9997 a 1.0032 y la fase, de 0.83 a
// 0.84 grados. En simulacion (1.6 deg/s de deriva) el error de theta a los
// 45-60 s baja de 74.9 a 0.27 grados.
const float DERIVA_K     = 0.30;   // deg/s de correccion por grado de error
const float F_ACC_LP     = 0.30;   // media del acelerometro (solo muestras buenas)
const float DERIVA_TOPE  = 8.0;    // desviacion maxima respecto del valor calibrado (deg/s)
const float DERIVA_W_MAX = 10.0;   // no adaptar si el brazo esta girando (deg/s)
const float DERIVA_E_MAX = 20.0;   // ni si el desacuerdo es excesivo
// La adaptacion se congela si el acelerometro no es fiable: con la vibracion
// de las helices puede medir 1 g apuntando en cualquier direccion. El cero del
// giroscopio varia en minutos, asi que congelarlo durante un vuelo no pierde nada.
const float DERIVA_ACC_MIN = 80.0; // % de muestras buenas por debajo del cual NO se adapta
// Tambien se congela por vibracion, que anticipa la degradacion del
// acelerometro antes que acc_pct.
const float DERIVA_VIB_MAX = 2.0;  // umbral del indice de vibracion
const float DT_MIN = 0.005;   // 200 Hz max
const float DT_MAX = 0.05;    // clamp: protege integradores tras un bloqueo
const int SOFT_PASO = 3, SOFT_MS = 40;// arranque suave de los ESC: paso (us) y periodo (ms)

// =========================================================================
//  ESTADO
// =========================================================================
enum Fase { F_NIVELAR = 0, F_HOVER = 1, F_SUBIR = 2, F_CRUCERO = 3, F_ATERRIZAR = 4,
            F_POSAR = 5 };

// prototipos (evita depender del generador automatico del IDE)
void control();
void diagnostico();
void reposo();
void comandos();
void actualizar_theta(float dt);
void actualizar_z(float dt);
float leer_ultra();
float medir_piso();
void vcc_init();
void vcc_actualizar();
float vbat_peor();
void identificar_motores();
bool capturar_despegue();
void abortar(const __FlashStringHelper *motivo);
void arranque_suave(int destino, int tr);
void calibrar();
void mpu_init();
void apagar();
void esperar_reset();
void ISR_emg();

Servo motorIzq, motorDer;
volatile bool emergencia = false;
bool controlActivo = false;

float theta_offset = 0, gyro_offset = 0;
float gyro_off_cal = 0;        // offset de la calibracion; referencia del tope de deriva
float th_acc_lp = 0;           // media del angulo por acelerometro (vertical)
bool  acc_visto = false;
float theta_filt = 0, gyro_f1 = 0, gyro_dps = 0, ang_abs = 0;
// Estado del filtro complementario (se reinicia al calibrar).
float th_comp = 0;
// Medidas sin filtrar para diagnostico: permiten ver cual de los dos sensores
// se degrada con la vibracion. th_acc_dbg = angulo por acelerometro,
// gyro_raw_dbg = velocidad angular sin offset.
float th_acc_dbg = 0, gyro_raw_dbg = 0;
// Indice de vibracion: salto medio de th_acc entre muestras consecutivas.
//   > 20 deg  -> acelerometro inutilizable
//   5-20 deg  -> marginal
//   < 5 deg   -> sano
float vib = 0;
// Porcentaje de muestras del acelerometro que pasan la compuerta de modulo.
//   ~100 = sensor sano;  20-80 = vibracion alta pero corregible;
//   ~0   = theta corre SOLO con giroscopio -> deriva libre, no es fiable.
float acc_pct = 100.0;
float vcc5_v = 0.0, vcc5_min = 99.0;      // riel de 5 V del Arduino (bandgap, sin hardware)
float vbat_l = 0.0, vbat_r = 0.0;         // fuentes de 12 V de cada motor (divisores)
float vbat_l_min = 99.0, vbat_r_min = 99.0;
int   vcc_bajo = 0;                       // ciclos consecutivos bajo umbral
float pwm_techo = 1750.0;                 // techo adaptativo del PWM comun
float z_filt = 3.0, z_piso = 3.0, vz_filt = 0;
unsigned long t_ultra = 0;   // ultimo disparo del ultrasonido
float iz = 0, ith = 0;
float sp_z_actual = 3.0;
float uz_ff = 0, uz_hover = 0, Uz_ant = 0, uz_posar = 0;
bool  en_vuelo = false, subiendo = false;
Fase  fase = F_NIVELAR;
int   cont_ok = 0, n_raros = 0;
float ang_abs_ini = 0;                 // |theta| al arrancar: referencia para detectar divergencia
bool  modo_diag = false;               // modo diagnostico de sentido/trim
float diag_uth = 0;
unsigned long t_anterior = 0, t_ultimo_z = 0, t_ang_malo = 0, t_fase = 0, t_diag = 0;

// =========================================================================
//  LEY DE CONTROL: LQI
// =========================================================================
// Autoridad a la frecuencia de diseno (generada por diseno_controladores.py):
//   angulo  a 6.0 rad/s (0.95 Hz):  |C| = 1.4356   fase = +79.2 deg
//   altura  a 1.0 rad/s          :  |C| = 3.2802   fase = -23.9 deg
// Las seis leyes tienen la misma |C| a esas frecuencias; solo cambia la fase,
// que depende de la estructura de cada ley y no de su sintonia.
//
// LQR con accion integral sobre los estados medidos (theta_filt y gyro_dps en
// angulo; z_filt y vz_filt en altura). Q = diag(120, 12, 40) en angulo; R se
// ajusto por biseccion hasta igualar la |C| del PID. La solucion LQ concentra
// la autoridad en el termino de velocidad (Kd = 0.226, practicamente el del
// PID) y reduce mucho el proporcional.
//
// Fuera de este bloque, el codigo es identico en los seis sketches.

// ---- escala de la salida de angulo (comando 'a') ------------------------
// Escala solo el lazo de angulo; escalar el modo comun alteraria el hover.
// Permite probar una ley primero con autoridad reducida, p. ej. con a0.5.
float ctrl_esc = 1.0f;
bool  acc_ok_g = false;        // estado de la compuerta del acelerometro

// ---- lectura de numeros desde la consola -------------------------------
// Alternativa a Serial.parseFloat(): acepta coma o punto decimal y espera a
// que llegue el numero completo. Devuelve false si no hay digitos, y entonces
// el llamador no cambia nada. Bloquea como maximo LEER_MS; 30 ms son 6 ciclos
// del lazo, que el gobernador de referencia absorbe.
const uint8_t LEER_MS = 30;

bool leer_num(float &v) {
  char buf[14];
  uint8_t n = 0;
  unsigned long t0 = millis(), tc = t0;
  while (millis() - t0 < LEER_MS && n < sizeof(buf) - 1) {
    if (!Serial.available()) {
      if (n && millis() - tc > 4) break;          // numero completo
      continue;
    }
    char d = (char)Serial.read();
    if (d == ',') d = '.';                        // coma decimal -> punto
    if ((d >= '0' && d <= '9') || d == '.' || d == '-' || d == '+') {
      buf[n++] = d;
      tc = millis();
    } else if (n) {
      break;                                      // fin del numero
    }
  }
  buf[n] = '\0';
  if (!n) return false;
  v = atof(buf);
  return true;
}

// =========================================================================
void setup() {
  Serial.begin(115200); Serial.setTimeout(10);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), ISR_emg, FALLING);
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  Wire.begin(); Wire.setWireTimeout(3000, true);
  for (uint8_t a = 0x68; a <= 0x69; a++) { Wire.beginTransmission(a); if (Wire.endTransmission()==0){MPU_ADDR=a;break;} }
  mpu_init();
  motorIzq.attach(PIN_MOTOR_IZQ, PWM_MIN, PWM_MAX);
  motorDer.attach(PIN_MOTOR_DER, PWM_MIN, PWM_MAX);
  // el techo de altura se deriva del PWM comun maximo -> inmune a cambios de PWM_BASE
  UZ_MAX = (float)(PWM_COMUN_MAX - PWM_BASE);
  vcc_init();
  apagar(); delay(3000); calibrar();
  Serial.println(F("# BALANCIN - ley LQI. C=arrancar L=aterrizar S=stop z# t# v# a# w# e# +/- m# h"));
  Serial.println(F("# DIAGNOSTICO: i=identificar motores  d0=asimetria  d30=sentido  n=invertir signo"));
  Serial.println(F("LISTO"));
  t_anterior = micros();
}

// =========================================================================
void loop() {
  if (emergencia) { esperar_reset(); return; }
  if (Serial.available()) comandos();
  if (modo_diag) diagnostico();
  else if (controlActivo) control();
  else reposo();          // mantiene theta_filt actualizado con los motores apagados
}

// =========================================================================
//  LAZO DE CONTROL
// =========================================================================
void control() {
  unsigned long ahora = micros();
  float dt = (ahora - t_anterior) / 1e6f;
  if (dt < DT_MIN) return;
  t_anterior = ahora;
  if (dt > DT_MAX) dt = DT_MAX;

  // ---------- SENSORES ----------
  actualizar_theta(dt);
  vcc_actualizar();
  ang_abs = 0.99f * ang_abs + 0.01f * fabs(theta_filt);   // |theta| suavizado (~0.5 s)

  // ---------- CAIDA DE FUENTE ----------
  // Si la alimentacion colapsa se pierde empuje y ningun control lo compensa:
  // se aterriza (o se aborta si aun no hubo despegue).
  float vb = vbat_peor();
  bool caida = (vb > 0.0f && vb < VBAT_MIN) || (vcc5_v > 0.0f && vcc5_v < VCC5_MIN);
  if (caida) {
    if (++vcc_bajo >= VBAT_CICLOS) {
      if (en_vuelo && fase != F_ATERRIZAR) {
        fase = F_ATERRIZAR;
        Serial.print(F("# FUENTE CAIDA  vL=")); Serial.print(vbat_l, 2);
        Serial.print(F(" vR=")); Serial.print(vbat_r, 2);
        Serial.print(F(" v5=")); Serial.print(vcc5_v, 2);
        Serial.println(F(" -> aterrizando"));
      } else if (!en_vuelo) {
        abortar(F("fuente caida antes de despegar")); return;
      }
    }
  } else vcc_bajo = 0;

  // ---------- TECHO DE EMPUJE ADAPTATIVO ----------
  // Una fuente en su limite de corriente no avisa: baja la tension. En lugar
  // de exigir hasta que colapse, se reduce el techo del PWM comun hasta donde
  // se sostiene y se recupera despacio.
  if (vb > 0.0f && vb < VBAT_SAG) pwm_techo -= TECHO_BAJA * dt;
  else if (pwm_techo < (float)PWM_MAX_SEG) pwm_techo += TECHO_SUBE * dt;
  pwm_techo = constrain(pwm_techo, PWM_TECHO_MIN, (float)PWM_MAX_SEG);
  actualizar_z(dt);      // marca t_ultimo_z solo cuando corrige con medida valida
  if (millis() - t_ultimo_z > Z_WD_MS) { abortar(F("sensor de altura mudo")); return; }

  // ---------- LIMITES DUROS ----------
  // Ventana relativa a SP_th, recortada siempre por la envolvente mecanica.
  // En tierra el brazo reposa sobre su tope, asi que solo se vigila la
  // envolvente ampliada; la ventana se aplica en vuelo.
  float ab_neg = TH_MEC_NEG - 10.0f, ab_pos = TH_MEC_POS + 10.0f;
  if (en_vuelo) {
    ab_neg = SP_th - TH_VENT_NEG; if (ab_neg < TH_MEC_NEG) ab_neg = TH_MEC_NEG;
    ab_pos = SP_th + TH_VENT_POS; if (ab_pos > TH_MEC_POS) ab_pos = TH_MEC_POS;
  }
  if (theta_filt > ab_pos || theta_filt < ab_neg) { abortar(F("tope de angulo")); return; }
  if (z_filt > Z_TOPE_SEG)                                    { abortar(F("tope de altura")); return; }

  // ---------- SUPERVISION: angulo fuera de margen sostenido -> aterriza ----------
  // Solo en vuelo: en tierra el brazo reposa inclinado sobre su tope.
  if (en_vuelo && fabs(theta_filt - SP_th) > ANG_MALO) {
    if (t_ang_malo == 0) t_ang_malo = millis();
    else if (millis() - t_ang_malo > ANG_MALO_MS && fase != F_ATERRIZAR) {
      fase = F_ATERRIZAR; Serial.println(F("# angulo inestable -> aterrizaje preventivo"));
    }
  } else t_ang_malo = 0;

  // ---------- MAQUINA DE FASES ----------
  switch (fase) {

    case F_NIVELAR:
      // Empuje comun de nivelacion: lleva los motores por encima de la zona
      // muerta del ESC para que el lazo de angulo tenga autoridad. Si el brazo
      // no nivela, el empuje sube con TASA_NIVEL.
      if (uz_ff < (float)(PWM_NIVEL - PWM_BASE)) uz_ff = (float)(PWM_NIVEL - PWM_BASE);
      else if (ang_abs > ANG_OK_NIVEL && PWM_BASE + uz_ff < PWM_BUSQ_MAX) uz_ff += TASA_NIVEL * dt;
      sp_z_actual = z_piso;
      if (capturar_despegue()) break;     // por si el empuje de nivelacion ya levanta
      if (ang_abs < ANG_OK_NIVEL) {
        if (++cont_ok > CICLOS_OK) { fase = F_HOVER; cont_ok = 0; t_fase = millis();
                                     Serial.println(F("# nivelado -> buscando hover...")); }
      } else {
        cont_ok = 0;
        // Deteccion de signo invertido: si el angulo empeora mientras el lazo lo
        // corrige, la realimentacion es positiva. Se aborta a los 3 s.
        if (millis() - t_fase > 3000 && ang_abs > ang_abs_ini + 6.0f) {
          abortar(F("el angulo EMPEORA: signo invertido? prueba 'n' o revisa TRIM")); return;
        }
        if (millis() - t_fase > NIVEL_MAX_MS) { abortar(F("no nivela: revisa TRIM y sentido de Uth")); return; }
      }
      break;

    case F_HOVER:                         // rampa de empuje en lazo abierto hasta despegar
      uz_ff += TASA_BUSQ * dt;
      sp_z_actual = z_piso;
      if (capturar_despegue()) break;
      if (PWM_BASE + uz_ff > PWM_BUSQ_MAX || millis() - t_fase > BUSQ_MAX_MS) { abortar(F("no despega")); return; }
      break;

    case F_SUBIR: {                       // gobernador: la rampa solo avanza si la planta la sigue
      bool sigue  = (sp_z_actual - z_filt) < MAX_LAG;
      bool ang_ok = (ang_abs < ANG_OK_SUBIR);
      subiendo = (sigue && ang_ok);
      if (subiendo) sp_z_actual += RAMP_CMS * dt;
      // Nota: si durante el ascenso se pide una consigna menor que la actual,
      // sp_z_actual pasa a ella en un solo paso. Para bajar desde el ascenso,
      // reducir la consigna en pasos pequenos.
      if (sp_z_actual >= SP_z_target) { sp_z_actual = SP_z_target; fase = F_CRUCERO; subiendo = false;
                                        Serial.println(F("# crucero")); }
      break; }

    case F_CRUCERO:
      subiendo = false;
      if (SP_z_target > sp_z_actual + 0.2f) fase = F_SUBIR;            // subir mas con z<val>
      else if (SP_z_target < sp_z_actual - 0.2f) sp_z_actual -= BAJA_CMS * dt;  // bajar con rampa
      else sp_z_actual = SP_z_target;
      break;

    case F_ATERRIZAR:
      subiendo = false;
      sp_z_actual -= BAJA_CMS * dt;
      // Termina al detectar contacto o al acabar la rampa, lo que ocurra
      // primero. Sin la deteccion de contacto, el error de altura se volveria
      // positivo en el piso y el lazo despegaria de nuevo.
      if (z_filt <= z_piso + 0.6f || sp_z_actual <= z_piso + 0.3f) {
        // Se pasa a posar en lugar de apagar en seco (ver POSADO NIVELADO).
        fase = F_POSAR; t_fase = millis();
        uz_posar = Uz_ant;                  // el empuje que llevaba al tocar
        en_vuelo = false;                   // el lazo de altura ya no manda
        Serial.println(F("# tocando piso -> posando nivelado"));
      }
      break;

    case F_POSAR:
      // El empuje comun se retira despacio mientras el lazo de angulo sigue
      // activo (no depende de en_vuelo), para que el brazo se pose nivelado.
      uz_posar -= UZ_POSAR * dt;
      if (uz_posar < 0.0f) uz_posar = 0.0f;
      uz_ff = uz_posar;
      if ((uz_posar <= 0.0f && fabs(theta_filt - SP_th) < POSAR_ANG)
          || millis() - t_fase > POSAR_MS) {
        Serial.print(F("# aterrizado  theta=")); Serial.print(theta_filt, 1);
        Serial.print(F(" deg  "));
        Serial.println(fabs(theta_filt - SP_th) < POSAR_ANG
                       ? F("(nivelado)") : F("(TORCIDO: nivelalo a mano)"));
        apagar(); controlActivo = false; en_vuelo = false; return;
      }
      break;
  }

  // ---------- LAZO DE ALTURA (LQI) ----------
  float err_z  = sp_z_actual - z_filt;
  float err_th = SP_th - theta_filt;
  float Uz_ideal, Uz, Uth_ideal;

  if (en_vuelo) {
    float vz_ref = subiendo ? RAMP_CMS : (fase == F_ATERRIZAR ? -BAJA_CMS : 0.0f);
    (void)vz_ref;
    Uz_ideal = LQI_Z_KP * err_z - LQI_Z_KD * vz_filt + LQI_Z_KI * iz;
    // Limite inferior del mando: el hover capturado menos UZ_CAIDA.
    float uz_lo = uz_hover - UZ_CAIDA; if (uz_lo < 0.0f) uz_lo = 0.0f;
    Uz = constrain(Uz_ideal, uz_lo, UZ_MAX);
  } else {
    // En tierra la altura va en lazo abierto (nivelacion y busqueda de hover),
    // pero el lazo de angulo ya esta activo: es el que levanta el brazo de su
    // tope antes del despegue.
    Uz_ideal = uz_ff;
    Uz = uz_ff;
    iz = uz_ff / LQI_Z_KI;                  // integrador espejo -> sin salto
  }

  // limitador de pendiente del modo comun
  float paso_up = SLEW_UZ_UP * dt, paso_dn = SLEW_UZ_DN * dt;
  if (Uz > Uz_ant + paso_up) Uz = Uz_ant + paso_up;
  if (Uz < Uz_ant - paso_dn) Uz = Uz_ant - paso_dn;
  Uz_ant = Uz;

  // ---------- LAZO DE ANGULO (LQI) ----------
  // El tope D_TH_MAX del termino de velocidad se aplica igual en todas las
  // leyes: protege frente a los picos del giroscopio por friccion seca y
  // mantiene la misma proteccion en la comparacion.
  Uth_ideal = (LQI_TH_KP * err_th + LQI_TH_KI * ith
            + constrain(-LQI_TH_KD * gyro_dps, -D_TH_MAX, D_TH_MAX)) * ctrl_esc;
  float Uth = constrain(Uth_ideal, -U_TH_MAX, U_TH_MAX);

  // ---------- MEZCLA CON PRIORIDAD AL ANGULO ----------
  float comun = PWM_BASE + Uz;
  int trim = TRIM_BASE + (int)(K_ASIM * (comun - PWM_REF_ASIM));
  trim = constrain(trim, -TRIM_MAX, TRIM_MAX);   // el trim puede ser negativo

  float uth_mix = SIGNO_TH * Uth;         // sentido fisico del par diferencial
  float dL = -uth_mix - trim;             // desviacion del motor izquierdo
  float dR =  uth_mix + trim;             // desviacion del motor derecho

  // si el diferencial no cabe en el rango, escalarlo (red de seguridad)
  float span  = fabs(dR - dL);
  float rango = pwm_techo - (float)PWM_MIN;
  if (span > rango) { float k = rango / span; dL *= k; dR *= k; }

  // si un lado satura, se mueve el modo comun (altura) y se respeta el diferencial (angulo)
  float hi = comun + (dL > dR ? dL : dR);
  float lo = comun + (dL < dR ? dL : dR);
  float corr = 0.0f;
  if (hi > pwm_techo) corr = pwm_techo - hi;
  if (lo + corr < (float)PWM_MIN) corr = (float)PWM_MIN - lo;
  comun += corr;

  int pl = constrain((int)(comun + dL), PWM_MIN, (int)pwm_techo);
  int pr = constrain((int)(comun + dR), PWM_MIN, (int)pwm_techo);
  motorIzq.writeMicroseconds(pl);
  motorDer.writeMicroseconds(pr);

  // ---------- ANTI-WINDUP con el mando realmente aplicado ----------
  float Uz_real = comun - PWM_BASE;       // incluye el limitador de pendiente y el recorte de la mezcla
  if (en_vuelo) {
    iz += err_z * dt + AW_Z * (Uz_real - Uz_ideal) * dt;
    iz = constrain(iz, 0.0f, UZ_MAX / LQI_Z_KI);
  }
  // En las leyes que usan ith, el integrador de angulo tambien actua en tierra:
  // es el que levanta el brazo de su tope de reposo. En tierra usa un limite
  // mayor; al despegar se recorta a ITH_MAX (ver capturar_despegue).
  if (fabs(err_th) < ITH_BANDA) {
    float clamp_ith = en_vuelo ? ITH_MAX : ITH_MAX_PISO;
    ith += err_th * dt + AW_TH * (Uth - Uth_ideal) * dt;
    ith = constrain(ith, -clamp_ith, clamp_ith);
  }

  // ---------- TELEMETRIA (10 Hz) ----------
  // Una linea CSV por muestra; la cabecera se imprime con el comando C.
  static unsigned long tp = 0;
  if (millis() - tp > 100) {
    tp = millis();
    Serial.print(millis());                 Serial.print(',');
    Serial.print(z_filt, 2);                Serial.print(',');
    Serial.print(theta_filt, 2);            Serial.print(',');
    Serial.print(SP_z_target - z_filt, 2);  Serial.print(',');
    Serial.print(err_th, 2);                Serial.print(',');
    Serial.print(pl);                       Serial.print(',');
    Serial.print(pr);                       Serial.print(',');
    Serial.print(sp_z_actual, 2);           Serial.print(',');
    Serial.print((int)fase);                Serial.print(',');
    Serial.print(Uz_real, 1);               Serial.print(',');
    Serial.print(Uth, 1);                   Serial.print(',');
    Serial.print(vz_filt, 2);               Serial.print(',');
    Serial.print(iz, 2);                    Serial.print(',');
    Serial.print(ang_abs, 2);               Serial.print(',');   // gobierna las fases
    Serial.print(acc_pct, 0);               Serial.print(',');
    Serial.print(vcc5_v, 2);                Serial.print(',');
    Serial.print(vbat_l, 2);                Serial.print(',');
    Serial.print(vbat_r, 2);                Serial.print(',');
    Serial.print((int)pwm_techo);           Serial.print(',');   // techo adaptativo
    // gyro y vib permiten distinguir un movimiento real de theta de un error
    // del acelerometro: si theta cambia con el giroscopio en cero, no es real.
    Serial.print(gyro_dps, 2);              Serial.print(',');
    Serial.print(vib, 1);                   Serial.print(',');
    // Deriva estimada del giroscopio y media del acelerometro. En vuelo:
    //   deriva crece y th_acc quieto       -> sesgo del giroscopio
    //   deriva ~ 0 y th_acc sigue a theta  -> giro real del brazo
    Serial.print(gyro_offset - gyro_off_cal, 2);  Serial.print(',');
    Serial.print(th_acc_lp, 2);             Serial.print(',');
    Serial.print(1);                             Serial.print(',');   // ley
    Serial.println(ctrl_esc, 2);
  }
}

// =========================================================================
//  SENSORES
// =========================================================================
void actualizar_theta(float dt) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return;
  int16_t ax = (Wire.read() << 8) | Wire.read();   // ax se lee para calcular el modulo
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); Wire.read(); Wire.read();

  float th_acc = atan2(ay / ACC_LSB, az / ACC_LSB) * 180.0f / PI - theta_offset;
  float g = (gx / GYRO_SCALE) - gyro_offset;
  th_acc_dbg = th_acc; gyro_raw_dbg = g;
  static float th_acc_ant = 0;
  vib = 0.98f * vib + 0.02f * fabs(th_acc - th_acc_ant);
  th_acc_ant = th_acc;

  // ---- COMPUERTA POR MODULO DEL ACELEROMETRO ----
  // En reposo el acelerometro mide 1 g. Si el modulo se aparta de 1 g, la
  // muestra contiene aceleracion lineal o vibracion y no indica la vertical:
  // se descarta y en ese ciclo se integra solo el giroscopio. Requiere el rango
  // de +-8g; a +-2g la vibracion satura el sensor y la compuerta no la detecta.
  float fax = ax / ACC_LSB, fay = ay / ACC_LSB, faz = az / ACC_LSB;
  float amag = sqrt(fax * fax + fay * fay + faz * faz);
  bool acc_ok = fabs(amag - 1.0f) < ACC_TOL;
  acc_ok_g = acc_ok;
  acc_pct = 0.99f * acc_pct + (acc_ok ? 1.0f : 0.0f);   // % de muestras utilizables

  // gyro: dos EMAs en cascada = pasabajos de 2do orden (~5.6 Hz a 200 Hz de muestreo)
  gyro_f1  = FG * gyro_f1  + (1.0f - FG) * g;
  gyro_dps = FG * gyro_dps + (1.0f - FG) * gyro_f1;

  th_comp += g * dt;                                              // prediccion: siempre con el giroscopio
  if (acc_ok) {                                                   // correccion: solo con muestra valida
    float corr = (1.0f - ALPHA) * (th_acc - th_comp);             // correccion pedida por el acelerometro
    float tope = CORR_MAX_DPS * dt;                               // correccion permitida
    th_comp += constrain(corr, -tope, tope);
    // Media del acelerometro, alimentada solo con muestras que pasan la
    // compuerta (en vuelo, una fraccion pequena).
    if (acc_visto) th_acc_lp += (th_acc - th_acc_lp) * F_ACC_LP;
    else { th_acc_lp = th_acc; acc_visto = true; }
  }

  // Estimacion de la deriva del cero del giroscopio (ver DERIVA_K). Corre en
  // todos los ciclos, no solo con muestras validas: lo que el acelerometro
  // indica de forma sostenida no es ruido sino un cambio del cero del
  // giroscopio, y se descuenta de gyro_offset.
  if (acc_visto && acc_pct > DERIVA_ACC_MIN && vib < DERIVA_VIB_MAX
      && fabs(gyro_dps) < DERIVA_W_MAX) {
    float e = th_acc_lp - th_comp;
    if (fabs(e) < DERIVA_E_MAX) {
      gyro_offset -= DERIVA_K * e * dt;
      gyro_offset = constrain(gyro_offset, gyro_off_cal - DERIVA_TOPE,
                                           gyro_off_cal + DERIVA_TOPE);
    }
  }
  theta_filt = BETA * theta_filt + (1.0f - BETA) * th_comp;
}

// Estimador alfa-beta de (z, vz), multitasa:
//   PREDICCION  a la tasa del lazo (200 Hz), con modelo de velocidad constante
//   CORRECCION  solo cuando el ultrasonido entrega medida nueva (50 Hz)
// Actualiza t_ultimo_z solo en una correccion valida, que es lo que vigila el watchdog.
void actualizar_z(float dt) {
  z_filt += vz_filt * dt;                                 // --- PREDICCION ---

  unsigned long ms = millis();
  if (ms - t_ultra < ULTRA_MS) return;                    // todavia no corresponde medir
  float dtu = (ms - t_ultra) / 1000.0f;
  t_ultra = ms;

  float z = leer_ultra();
  if (z < 0.5f || z > 60.0f) return;
  float zc = z * cos(theta_filt * PI / 180.0f);           // proyeccion vertical

  float r = zc - z_filt;                                  // innovacion
  if (fabs(r) > MAX_INNOV_CM && n_raros < 5) { n_raros++; return; }
  n_raros = 0;                                            // tras 5 rechazos seguidos se acepta (movimiento real)

  z_filt  += ALFA_AB * r;                                 // --- CORRECCION ---
  vz_filt += (BETA_AB / dtu) * r;
  t_ultimo_z = ms;
}

float leer_ultra() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long d = pulseIn(PIN_ECHO, HIGH, 25000);
  if (d == 0) return -1.0f;
  return d * 0.01723f;
}

float medir_piso() {
  float s = 0; int n = 0;
  for (int i = 0; i < 12; i++) { float z = leer_ultra(); if (z > 0.5f && z < 60.0f) { s += z; n++; } delay(20); }
  return (n > 3) ? s / n : 3.0f;
}

// =========================================================================
//  COMANDOS / UTILIDADES
// =========================================================================
void comandos() {
  char c = (char)Serial.read();
  if (c == 'd' || c == 'D') {                 // prueba de sentido/asimetria en lazo abierto
    diag_uth = Serial.parseFloat();
    diag_uth = constrain(diag_uth, -120.0f, 120.0f);
    // Por la friccion del pivote, el brazo queda donde lo dejo la prueba
    // anterior. Si arranca cerca del tope, la prueba se corta de inmediato.
    float margen = 0.6f * ((theta_filt > 0) ? TH_MEC_POS : -TH_MEC_NEG);
    if (fabs(theta_filt) > margen) {
      Serial.print(F("# diag NO inicia: nivela el brazo A MANO primero. theta="));
      Serial.println(theta_filt, 1);
    } else {
      arranque_suave(PWM_NIVEL, 0);           // sin trim: mide la asimetria sin corregir
      modo_diag = true; controlActivo = false; en_vuelo = false;
      t_diag = millis(); t_anterior = micros();
      Serial.print(F("# DIAG uth=")); Serial.print(diag_uth, 0);
      Serial.print(F("  pl=")); Serial.print(PWM_NIVEL - (int)diag_uth);
      Serial.print(F("  pr=")); Serial.print(PWM_NIVEL + (int)diag_uth);
      Serial.print(F("  theta_ini=")); Serial.println(theta_filt, 2);
      Serial.println(F("D,t_ms,theta,uth,pwm_l,pwm_r,th_acc,gyro,vib,acc_pct"));
    }
  }
  else if (c == 'b' || c == 'B') {   // calibra el bandgap contra un multimetro
    float real = Serial.parseFloat();
    Serial.print(F("# b interpreto: ")); Serial.println(real, 3);
    if (real > 3.0f && real < 6.0f && vcc5_v > 0.5f) {
      VCC5_K *= real / vcc5_v;
      vcc5_v = real; vcc5_min = real;
      Serial.print(F("# bandgap calibrado: VCC5_K=")); Serial.print(VCC5_K, 1);
      Serial.print(F("  v5=")); Serial.println(vcc5_v, 2);
    } else {
      Serial.println(F("# uso: b<tension real medida>  p.ej. b5.02"));
    }
  }
  else if (c == 'p' || c == 'P') {   // empuje comun del modo diagnostico
    int v = (int)Serial.parseFloat();
    if (v >= 1200 && v <= PWM_BUSQ_MAX) {
      PWM_NIVEL = v;
      Serial.print(F("# PWM_NIVEL=")); Serial.println(PWM_NIVEL);
    } else {
      Serial.print(F("# uso: p<1200.."));
      Serial.print(PWM_BUSQ_MAX); Serial.println(F(">  p.ej. p1470"));
    }
  }
  else if (c == 'o' || c == 'O') {   // recalibra el cero con el brazo nivelado
    if (controlActivo || modo_diag) {
      Serial.println(F("# 'o' solo con el control parado. Manda S primero."));
    } else {
      apagar();
      calibrar();
      if (fabs(theta_offset) <= 8.0f) Serial.println(F("# cero OK"));
    }
  }
  else if (c == 'i' || c == 'I') {
    controlActivo = false; modo_diag = false; en_vuelo = false;
    identificar_motores();
  }
  else if (c == 'n' || c == 'N') {
    SIGNO_TH = -SIGNO_TH;
    Serial.print(F("# SIGNO_TH=")); Serial.println(SIGNO_TH);
  }
  else if (c == 'C' || c == 'c') {
    z_piso = medir_piso();
    arranque_suave(PWM_BASE, TRIM_BASE);
    iz = 0; ith = 0; uz_ff = 0; uz_hover = 0; Uz_ant = 0; vz_filt = 0;
    en_vuelo = false; subiendo = false; fase = F_NIVELAR; cont_ok = 0; n_raros = 0;
    z_filt = z_piso; sp_z_actual = z_piso; ang_abs = fabs(theta_filt); ang_abs_ini = ang_abs;
    t_ang_malo = 0; t_ultimo_z = millis(); t_anterior = micros(); t_fase = millis(); t_ultra = millis();
    vcc5_min = 99.0; vbat_l_min = 99.0; vbat_r_min = 99.0;
    vcc_bajo = 0; pwm_techo = (float)PWM_MAX_SEG;
    controlActivo = true;
    Serial.print(F("# Control ON  piso=")); Serial.print(z_piso, 2); Serial.println(F(" cm"));
    Serial.print(F("# escala_angulo=")); Serial.println(ctrl_esc, 3);
    if (ctrl_esc < 0.01f)
      Serial.println(F("# !! OJO: vas a volar SIN lazo de angulo."));
    Serial.println(F("t_ms,z_cm,theta_deg,err_z,err_th,pwm_l,pwm_r,sp_z,fase,uz,uth,vz,iz,ang_abs,acc_pct,v5,vL,vR,techo,gyro,vib,deriva,th_acc,ley,esc"));
  }
  else if (c == 'L' || c == 'l') {
    if (controlActivo && en_vuelo) { fase = F_ATERRIZAR; Serial.println(F("# aterrizando...")); }
    else { controlActivo = false; apagar(); Serial.println(F("# Control OFF")); }
  }
  else if (c == 'S' || c == 's') { controlActivo = false; modo_diag = false; en_vuelo = false; apagar(); Serial.println(F("# Control OFF")); }
  // La consigna de altura no se acota aqui: la proteccion es el aborto por
  // Z_TOPE_SEG. Evitar consignas cercanas al tope fisico (~45 cm).
  else if (c == 'z' || c == 'Z') {
    float x; if (leer_num(x)) SP_z_target = x;
    Serial.print(F("# SP_z=")); Serial.println(SP_z_target);
  }
  else if (c == 't' || c == 'T') {
    float x; if (leer_num(x)) SP_th = x;
    Serial.print(F("# SP_th=")); Serial.println(SP_th);
  }
  else if (c == 'v' || c == 'V') {
    float x; if (leer_num(x)) RAMP_CMS = x;
    Serial.print(F("# rampa=")); Serial.println(RAMP_CMS);
  }
  else if (c == 'k' || c == 'K') {   // no aplica a esta ley
    Serial.parseFloat();
    Serial.println(F("# 'k' es del PID. Aqui usa 'a' para "
                     "escalar el angulo."));
  }
  else if (c == 'a' || c == 'A') {   // escala del lazo de angulo
    float x;
    if (leer_num(x)) ctrl_esc = constrain(x, 0.0f, 2.0f);
    else Serial.println(F("# 'a' sin numero: escala SIN CAMBIAR. Ej: a0.67"));
    Serial.print(F("# escala_angulo=")); Serial.println(ctrl_esc, 3);
    if (ctrl_esc < 0.01f)
      Serial.println(F("# !! LAZO DE ANGULO DESACTIVADO. Escribe a1 para volver."));
  }
  else if (c == 'w' || c == 'W') {   // ventana de angulo alrededor de SP_th
    float x;
    if (leer_num(x)) { TH_VENT_NEG = TH_VENT_POS = constrain(x, 3.0f, 45.0f); }
    else Serial.println(F("# 'w' sin numero: ventana SIN CAMBIAR. Ej: w25"));
    Serial.print(F("# ventana_th=SP")); Serial.print(-TH_VENT_NEG, 1);
    Serial.print(F("/+")); Serial.print(TH_VENT_POS, 1);
    Serial.print(F(" topada en [")); Serial.print(TH_MEC_NEG, 0);
    Serial.print(F(",")); Serial.print(TH_MEC_POS, 0);
    Serial.println(F("] mecanico"));
  }
  else if (c == 'e' || c == 'E') {   // techo del modo comun (altura alcanzable)
    float x;
    if (leer_num(x)) {
      PWM_COMUN_MAX = (int)constrain(x, 1500.0f, (float)PWM_MAX_SEG);
      UZ_MAX = (float)(PWM_COMUN_MAX - PWM_BASE);
    } else Serial.println(F("# 'e' sin numero: techo SIN CAMBIAR. Ej: e1800"));
    Serial.print(F("# PWM_COMUN_MAX=")); Serial.print(PWM_COMUN_MAX);
    Serial.print(F(" UZ_MAX=")); Serial.print(UZ_MAX, 0);
    Serial.print(F(" (tope duro ")); Serial.print(PWM_MAX_SEG);
    Serial.println(F(")"));
  }
  else if (c == 'r' || c == 'R') {   // no aplica a esta ley
    Serial.parseFloat();
    Serial.println(F("# 'r' es del PID. Aqui usa 'a' para "
                     "escalar el angulo."));
  }
  else if (c == 'g' || c == 'G') {   // no aplica a esta ley
    Serial.parseFloat();
    Serial.println(F("# 'g' es del PID. Aqui usa 'a' para "
                     "escalar el angulo."));
  }
  else if (c == '+') { TRIM_BASE += 5; Serial.print(F("# TRIM=")); Serial.println(TRIM_BASE); }
  else if (c == '-') { TRIM_BASE -= 5; Serial.print(F("# TRIM=")); Serial.println(TRIM_BASE); }
  else if (c == 'm' || c == 'M') { TRIM_BASE = Serial.parseInt(); Serial.print(F("# TRIM=")); Serial.println(TRIM_BASE); }
  else if (c == 'h' || c == 'H') {
    Serial.print(F("# theta=")); Serial.print(theta_filt, 2);
    Serial.print(F(" deriva=")); Serial.print(gyro_offset - gyro_off_cal, 2);
    Serial.print(F(" th_acc=")); Serial.print(th_acc_dbg, 2);
    Serial.print(F(" gyro=")); Serial.print(gyro_raw_dbg, 2); Serial.print(F(" vib=")); Serial.print(vib, 1); Serial.print(F(" acc%=")); Serial.print(acc_pct, 0);
    Serial.print(F(" v5=")); Serial.print(vcc5_v, 2);
    Serial.print(F("/")); Serial.print(vcc5_min, 2);
    Serial.print(F(" vL=")); Serial.print(vbat_l, 2);
    Serial.print(F("/")); Serial.print(vbat_l_min, 2);
    Serial.print(F(" vR=")); Serial.print(vbat_r, 2);
    Serial.print(F("/")); Serial.print(vbat_r_min, 2);
    Serial.print(F(" trim=")); Serial.print(TRIM_BASE);
    Serial.print(F(" signo=")); Serial.print(SIGNO_TH);
    Serial.print(F(" fase=")); Serial.print((int)fase);
    Serial.print(F(" piso=")); Serial.print(z_piso, 2);
    Serial.print(F(" hover=")); Serial.print(PWM_BASE + uz_hover, 0);
    Serial.print(F(" iz=")); Serial.print(iz, 2);
    Serial.print(F(" sp_z=")); Serial.println(sp_z_actual, 2);
  }
  while (Serial.available()) Serial.read();
}

// =========================================================================
//  REPOSO: motores apagados. La IMU se sigue leyendo para que theta_filt este
//  actualizado al arrancar o al iniciar una prueba de diagnostico.
// =========================================================================
void reposo() {
  unsigned long ahora = micros();
  float dt = (ahora - t_anterior) / 1e6f;
  if (dt < DT_MIN) return;
  t_anterior = ahora;
  if (dt > DT_MAX) dt = DT_MAX;
  actualizar_theta(dt);
  vcc_actualizar();
  ang_abs = 0.99f * ang_abs + 0.01f * fabs(theta_filt);
}

// =========================================================================
//  MODO DIAGNOSTICO  ('d<us>')
//  Aplica el empuje comun PWM_NIVEL con un diferencial fijo, sin trim y sin
//  lazo cerrado, durante DIAG_MS:
//    'd0'   -> sin diferencial: el lado hacia el que cae el brazo indica la
//              asimetria entre motores (base para TRIM_BASE)
//    'd30'  -> diferencial de 30 us a favor del motor derecho: el sentido en
//              que se mueve theta verifica SIGNO_TH
//  Los motores se mandan directamente (pl = NIVEL - uth, pr = NIVEL + uth),
//  sin pasar por SIGNO_TH; veredicto_diag() indica el sentido esperado segun
//  el SIGNO_TH configurado.
// =========================================================================
void veredicto_diag() {
  if (fabs(diag_uth) < 1.0f) {
    Serial.println(F("# 'd0' solo mide la asimetria; para el SENTIDO usa 'd30'."));
    return;
  }
  bool esperado_pos = (diag_uth > 0) == (SIGNO_TH > 0);
  Serial.print(F("# con uth=")); Serial.print(diag_uth, 0);
  Serial.print(F(" y SIGNO_TH=")); Serial.print(SIGNO_TH);
  Serial.print(F(", theta DEBE ir a "));
  Serial.println(esperado_pos ? F("POSITIVO") : F("NEGATIVO"));
  Serial.println(F("# Si fue en ese sentido, el signo esta BIEN: NO pulses 'n'."));
}

void diagnostico() {
  unsigned long ahora = micros();
  float dt = (ahora - t_anterior) / 1e6f;
  if (dt < DT_MIN) return;
  t_anterior = ahora;
  if (dt > DT_MAX) dt = DT_MAX;

  actualizar_theta(dt);
  vcc_actualizar();
  actualizar_z(dt);

  if (theta_filt > TH_MEC_POS || theta_filt < TH_MEC_NEG) {
    apagar(); modo_diag = false;
    Serial.print(F("# diag CORTADO por tope, theta=")); Serial.println(theta_filt, 2);
    veredicto_diag();     // el corte por tope tambien es un resultado
    return;
  }
  if (millis() - t_diag > DIAG_MS) {
    apagar(); modo_diag = false;
    Serial.print(F("# diag FIN  uth=")); Serial.print(diag_uth, 0);
    Serial.print(F("  theta_final=")); Serial.println(theta_filt, 2);
    veredicto_diag();
    return;
  }

  int pl = constrain(PWM_NIVEL - (int)diag_uth, PWM_MIN, PWM_MAX_SEG);
  int pr = constrain(PWM_NIVEL + (int)diag_uth, PWM_MIN, PWM_MAX_SEG);
  motorIzq.writeMicroseconds(pl);
  motorDer.writeMicroseconds(pr);

  static unsigned long tp = 0;
  if (millis() - tp > 100) {
    tp = millis();
    Serial.print(F("D,")); Serial.print(millis());      Serial.print(',');
    Serial.print(theta_filt, 2);                        Serial.print(',');
    Serial.print(diag_uth, 0);                          Serial.print(',');
    Serial.print(pl);                                   Serial.print(',');
    Serial.print(pr);                                   Serial.print(',');
    // --- acelerometro y giroscopio por separado ---
    Serial.print(th_acc_dbg, 2);                        Serial.print(',');
    Serial.print(gyro_raw_dbg, 2);                      Serial.print(',');
    Serial.print(vib, 1); Serial.print(','); Serial.println(acc_pct, 0);
  }
}

// =========================================================================
//  MEDICION DE Vcc SIN HARDWARE EXTERNO
//  Se lee la referencia de banda prohibida de 1.1 V usando AVcc como referencia
//  del ADC: Vcc = 1.1 * 1024 / ADC. El multiplexor se deja fijo en setup para no
//  pagar el tiempo de asentamiento en cada lectura, y las conversiones se disparan
//  y recogen SIN bloquear, una por ciclo de control (~200 Hz).
// =========================================================================
void vcc_init() {
#if defined(__AVR__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // AVcc como ref, canal = 1.1 V
  ADCSRA |= _BV(ADEN);
  delay(5);                                  // el bandgap necesita asentarse: solo aqui
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  (void)ADC;                                 // se descarta la primera conversion
  ADCSRA |= _BV(ADSC);                       // deja una en curso
#endif
}

void vcc_actualizar() {
#if defined(__AVR__)
  if (bit_is_set(ADCSRA, ADSC)) return;      // aun convirtiendo, no bloquear
  uint16_t r = ADC;
  if (r > 100) {                             // r < 100 equivale a Vcc > 11 V: lectura invalida
    float v = VCC5_K / (float)r;
    vcc5_v = (vcc5_v <= 0.0f) ? v : 0.8f * vcc5_v + 0.2f * v;
    if (vcc5_v < vcc5_min) vcc5_min = vcc5_v;
  }
  ADCSRA |= _BV(ADSC);                       // dispara la siguiente
#endif

#if USAR_VBAT
  // Los divisores se escalan con el Vcc REAL medido arriba, no con un 5.00 supuesto:
  // asi la medida de 12 V no se falsea si el propio riel de 5 V se mueve.
  float ref = (vcc5_v > 3.0f) ? vcc5_v : 5.0f;
  float kl = ref / 1023.0f * DIV_RATIO;
  float l = analogRead(PIN_VBAT_L) * kl;
  float d = analogRead(PIN_VBAT_R) * kl;
  vbat_l = (vbat_l <= 0.0f) ? l : 0.8f * vbat_l + 0.2f * l;
  vbat_r = (vbat_r <= 0.0f) ? d : 0.8f * vbat_r + 0.2f * d;
  if (vbat_l < vbat_l_min) vbat_l_min = vbat_l;
  if (vbat_r < vbat_r_min) vbat_r_min = vbat_r;
#endif
}

// Tension mas baja de las dos fuentes de motor; 0 si no hay divisores cableados.
float vbat_peor() {
#if USAR_VBAT
  return (vbat_l < vbat_r) ? vbat_l : vbat_r;
#else
  return 0.0f;
#endif
}

// =========================================================================
//  IDENTIFICACION DE MOTORES  ('i')
//  Hace girar un motor a la vez para comprobar que PIN_MOTOR_IZQ y
//  PIN_MOTOR_DER coinciden con el cableado. Si estan intercambiados, el trim
//  refuerza el motor equivocado y el lazo de angulo pasa a realimentacion
//  positiva. A 1300 us el giro es visible pero queda muy por debajo del hover,
//  y con un solo motor la plataforma no puede levantarse.
// =========================================================================
void identificar_motores() {
  const int PWM_ID = 1300;
  const unsigned long MS_ID = 2000;

  apagar();
  Serial.println(F("# IDENT: mira QUE motor gira en cada paso (2 s cada uno)."));
  for (int paso = 0; paso < 2 && !emergencia; paso++) {
    Servo &m = (paso == 0) ? motorIzq : motorDer;
    Serial.print(F("# --> pin D"));
    Serial.print(paso == 0 ? PIN_MOTOR_IZQ : PIN_MOTOR_DER);
    Serial.print(F("   el firmware lo llama "));
    Serial.println(paso == 0 ? F("IZQUIERDO") : F("DERECHO"));

    for (int p = PWM_MIN; p <= PWM_ID && !emergencia; p += SOFT_PASO) {
      m.writeMicroseconds(p); delay(SOFT_MS);
    }
    unsigned long t0 = millis();
    while (millis() - t0 < MS_ID && !emergencia) delay(10);
    apagar();
    delay(1500);
  }
  apagar();
  if (emergencia) { Serial.println(F("# IDENT abortado por emergencia")); return; }
  Serial.println(F("# IDENT fin."));
  Serial.println(F("# Si el motor que giro PRIMERO no es el izquierdo fisico,"));
  Serial.println(F("# hay que intercambiar PIN_MOTOR_IZQ y PIN_MOTOR_DER."));
}

// Detecta el despegue, fija el hover medido y pasa a subir. El integrador de
// altura arranca en el empuje que levanto la plataforma: transferencia sin
// salto y sin windup acumulado.
bool capturar_despegue() {
  if (z_filt <= z_piso + DZ_DESPEGUE) return false;
  // Descuenta el exceso de empuje por la velocidad de subida (ver K_VZ_HOVER).
  float exceso = K_VZ_HOVER * vz_filt;
  if (exceso < 0.0f) exceso = 0.0f;
  if (exceso > EXCESO_MAX) exceso = EXCESO_MAX;
  uz_hover = uz_ff - exceso;
  iz = uz_hover / LQI_Z_KI;                // inicializa el integrador en el hover
  sp_z_actual = z_filt;                        // la rampa arranca donde esta
  ith = constrain(ith, -ITH_MAX, ITH_MAX);     // recorta la autoridad extra de tierra
  en_vuelo = true; fase = F_SUBIR;
  Serial.print(F("# DESPEGUE  hover=")); Serial.print(PWM_BASE + uz_hover, 0);
  Serial.print(F(" us  z=")); Serial.print(z_filt, 2);
  Serial.print(F("  vz=")); Serial.print(vz_filt, 1);
  Serial.print(F("  descontado=")); Serial.println(exceso, 0);
  return true;
}

void abortar(const __FlashStringHelper *motivo) {
  apagar(); controlActivo = false; en_vuelo = false;
  Serial.print(F("# ABORTADO: ")); Serial.print(motivo);
  Serial.print(F("  z=")); Serial.print(z_filt, 1);
  Serial.print(F(" th=")); Serial.println(theta_filt, 1);
}

void arranque_suave(int destino, int tr) {
  destino = constrain(destino, PWM_MIN, PWM_MAX_SEG);
  for (int p = PWM_MIN; p <= destino; p += SOFT_PASO) {
    motorIzq.writeMicroseconds(constrain(p - tr, PWM_MIN, PWM_MAX_SEG));
    motorDer.writeMicroseconds(constrain(p + tr, PWM_MIN, PWM_MAX_SEG));
    if (emergencia) { apagar(); return; }
    delay(SOFT_MS);
  }
  delay(300);
}

void calibrar() {
  Serial.println(F("# Calibrando (brazo HORIZONTAL, no mover)..."));
  float sth = 0, sg = 0; const int N = 200;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    if (Wire.available() >= 14) {
      Wire.read(); Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();
      int16_t gx = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read(); Wire.read(); Wire.read();
      sth += atan2(ay / ACC_LSB, az / ACC_LSB) * 180.0f / PI;
      sg  += gx / GYRO_SCALE;
    }
    delay(5);
  }
  theta_offset = sth / N; gyro_offset = sg / N;
  gyro_off_cal = gyro_offset;
  th_acc_lp = 0; acc_visto = false;
  theta_filt = 0; gyro_f1 = 0; gyro_dps = 0; ang_abs = 0; th_comp = 0;
  Serial.print(F("# th_off=")); Serial.print(theta_offset, 3);
  Serial.print(F(" gyro_off=")); Serial.println(gyro_offset, 3);
  // El brazo debe estar horizontal al calibrar: th_off define el cero del lazo
  // de angulo. Valores tipicos en este banco: -3 a +4.5 grados.
  if (fabs(theta_offset) > 8.0f) {
    Serial.println(F("# !! th_off FUERA DE RANGO: el brazo NO estaba horizontal."));
    Serial.println(F("# !! Nivelalo y resetea, o el cero del control queda torcido."));
  }
}

void mpu_init() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true); delay(100);
  // ACCEL_CONFIG = 0x10 -> +-8g. A +-2g la vibracion de los motores satura el
  // acelerometro y el recorte se rectifica en un sesgo del angulo. theta sale
  // de atan2(ay, az), que no depende de la escala.
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(true);
  // GYRO_CONFIG = 0x00 -> +-250 deg/s.
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
  // DLPF a 21 Hz. No se baja mas porque el mismo filtro afecta al giroscopio y
  // el termino derivativo necesita su medida con poco retardo.
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x04); Wire.endTransmission(true); delay(50);
}

void apagar() { motorIzq.writeMicroseconds(PWM_MIN); motorDer.writeMicroseconds(PWM_MIN); }

void esperar_reset() {
  apagar(); controlActivo = false; en_vuelo = false;
  Serial.println(F("# EMERGENCIA. 'r' para reset."));
  while (emergencia) { if (Serial.available() && Serial.read() == 'r') { emergencia = false; Serial.println(F("LISTO")); } delay(200); }
}

void ISR_emg() { emergencia = true; motorIzq.writeMicroseconds(PWM_MIN); motorDer.writeMicroseconds(PWM_MIN); }
