// =========================================================================
//  pid_mimo_v2.ino  -  Control PID MIMO del balancin 2 GDL  (version 2)
//
//  Objetivo de esta version: SUBIDA LENTA Y SEGURA manteniendo el angulo.
//  Ataca los tres problemas medidos en el informe UTEC:
//    (1) Sobrepaso de altura 188.3 %  -> windup del integrador contra el piso
//    (2) Angulo oscilatorio (RMS 2.67 deg) -> ruido de gyro + windup de ith
//    (3) "las fuentes se apagaban al exigir mas corriente" -> picos de empuje
//
//  CAMBIOS ESTRUCTURALES vs v1:
//    A. Fase de BUSQUEDA DE HOVER en lazo abierto: sube el empuje a 25 us/s
//       hasta detectar despegue. El integrador se inicializa EN el hover real
//       medido -> transferencia sin salto y windup imposible en el piso.
//    B. Rampa "con correa" (reference governor): el setpoint solo avanza si
//       la planta lo esta siguiendo (lag < MAX_LAG) Y el angulo esta sano.
//       El error de altura queda acotado -> el integrador no puede dispararse.
//    C. Estimador ALFA-BETA multitasa de (z, vz): predice a 200 Hz y corrige a
//       50 Hz. Mitad de desfase que la diferencia sobre ventana.
//    D. Anti-windup por back-calculation en AMBOS lazos, alimentado con el
//       PWM realmente aplicado (incluye el recorte de la mezcla).
//    E. Mezcla con PRIORIDAD AL ANGULO: si un motor satura se recorta el modo
//       comun (altura), nunca el diferencial (angulo).
//    F. Limitador de pendiente del modo comun -> protege la fuente.
//    G. Filtro de gyro de 2do orden (recomendacion de las conclusiones).
//    H. Ultrasonido a 50 Hz (no 200) con rechazo por innovacion + watchdog.
//       Saca el pulseIn bloqueante de 3 de cada 4 ciclos del lazo de angulo.
//    I. Aterrizaje controlado (L) en vez de cortar motores en el aire.
//
//  Serial: C=arrancar  L=aterrizar  S=stop  z<val>=SP altura  t<val>=SP angulo
//          v<val>=rampa  k<val>=Kd_th  +/-=trim  m<val>=trim  h=estado
//          i=identificar cual motor esta en cada pin (hazlo ANTES de volar)
//          b<val>=calibrar la lectura de Vcc contra un multimetro
//                 (era 'w', pero 'w' es la ventana de angulo y habia
//                  DOS ramas 'w': ganaba esta y la ventana era codigo
//                  muerto. La campana manda w16 y no hacia nada.)
//          o=recalibrar el cero del angulo con el brazo ya nivelado a mano
//          p<val>=empuje comun del modo 'd' (por defecto PWM_NIVEL=1400)
//
//  Telemetria CSV: las 7 primeras columnas son IDENTICAS a la v1, por lo que
//  CAPTURA_CONTROL_MIMO.m sigue funcionando. Las nuevas van al final.
//
// -------------------------------------------------------------------------
//  CONFIGURACION VALIDADA EN VUELO  (2026-08-22)
//  Vuelo completo: despegue -> 5 cm -> 10 cm -> 5 cm -> aterrizaje.
//    hover medido    = 1477 us
//    TRIM_BASE       = -80    (el motor debil es el IZQUIERDO, no el derecho)
//    SIGNO_TH        = +1     (confirmado con el giroscopio, no con theta)
//    PWM_NIVEL       = 1400
//
//  CADENA DE DIAGNOSTICO, por si hay que repetirla tras tocar el hardware:
//   1. El acelerometro entrega +-60 deg de ruido con los motores girando y un
//      SESGO DC de +9 a +25 deg. El filtro complementario lo seguia, y ese era
//      el "movimiento" de theta que se veia. Era artefacto, no la planta.
//   2. Subir el rango a +-8g NO lo arregla solo (el sesgo empeoro de +9.5 a
//      +25.6): hace falta la COMPUERTA POR MODULO. El rango es su requisito.
//   3. El giroscopio SI es fiable (+-2 deg/s con motores). Cualquier conclusion
//      sobre sentido de giro debe sacarse del gyro, nunca de theta.
//   4. El pivote tiene ~+-70 us de zona muerta por friccion seca. Diferenciales
//      menores no mueven nada: no confundir con "el control no responde".
//  Vigilar 'vib' y 'acc_pct' con el comando 'h'. Si acc_pct cae a ~0, theta
//  corre solo con giroscopio y deriva: amortiguar el MPU antes de seguir.
// -------------------------------------------------------------------------
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
// El ESC acepta 1000-2000 us. 1750 NO era el limite del ESC sino este tope, y
// es el que impedia pasar de ~13 cm: el PID pidio z15 y se planto con el modo
// comun clavado en PWM_COMUN_MAX. Subido a 1850 para poder ensayar z = 15.
// Lo que SI limita de verdad no es el ESC de 30 A: es el empuje de la helice,
// la corriente de la bateria y el BEC de 5 V. Por eso el techo ADAPTATIVO
// (pwm_techo, que baja solo si la bateria se hunde) sigue activo encima.
int   PWM_MAX_SEG = 1850;     // tope de seguridad real (protege fuente y planta)
int   PWM_BASE    = 1250;     // base BAJA: en neutro descansa en el piso

// --- SETPOINTS ---
float SP_z_target = 5.0;     // altura objetivo (cm)
float SP_th       = 0.0;      // angulo objetivo (deg)

// --- PID ANGULO (theta) --- Tabla III del informe, columna "Implementado"
// Kd_th: VOLVIO a 0.07692, el valor con el que la planta volo.
// Se probo 0.60 (el que segun asignacion de polos recupera zeta=0.606) y la planta
// entro en CICLO LIMITE creciente, siempre en fase 0 con el brazo apoyado.
// No es inestabilidad lineal: con el modelo identificado, incluso agregando retardo
// de ESC/motor, el margen de ganancia es de 29-50 dB para cualquier Kd probado.
// Es STICK-SLIP: el brazo se pega por friccion seca, se desprende de golpe, el gyro
// ve el tiron y -Kd*gyro responde con un comando grande en sentido contrario.
// Mas Kd = tirones mas violentos. El modelo no tiene termino de friccion, asi que
// ningun analisis lineal puede predecir el limite: hay que encontrarlo midiendo.
// Usa el comando 'k<val>' para barrerlo EN CALIENTE (k0.15, k0.25, ...) y sube de
// a poco vigilando que uth no empiece a alternar de signo cada 2-3 muestras.
// Kd_th 0.15 (antes 0.07692, el de la ubicacion de polos del paper).
// Medido en el vuelo del 2026-09-03: theta oscila a 1.1 Hz creciendo x4 en 4 s,
// con el gyro en cuadratura -> movimiento real, no ruido. De ahi sale el polo
// 0.35 +- 7j, o sea zeta = -0.05: falta muy poco amortiguamiento.
// Adelanto de fase del PD a 7 rad/s = atan(Kd*w/Kp):
//   0.07692 -> 27.3 deg      0.15 -> 45.2 deg   (+18 deg, ganancia +26 %)
// Con los 13 deg del filtro del gyro son ~30 deg de margen, de sobra para zeta<0.05.
// Se barre en caliente con 'k'; Kp_th con 'g'.
// SINTONIA SUAVE (2026-09-03). Kp 1.041 -> 0.70, Ki 0.5581 -> 0.40, Kd se QUEDA.
// Kd no baja a proposito: es el que amortigua la resonancia de vuelo de 1.1 Hz.
// Ademas, al bajar Kp el adelanto de fase del PD SUBE -sale de atan(Kd*w/Kp)-:
//   antes  Kp=1.041 Kd=0.15 -> 45 deg de adelanto, |C| = 1.48 a 7 rad/s
//   ahora  Kp=0.70  Kd=0.15 -> 56 deg de adelanto, |C| = 1.26
// O sea: menos mando y MAS margen de estabilidad a la vez. Se barre con 'g' y 'k'.
// Kd_th 0.22 (antes 0.15). Medido en el vuelo largo del 2026-09-03: 67 s de
// crucero a z=10 con la amplitud de theta creciendo de +-2 a +-9 grados.
// Periodo 1.0 s (confirmado en dos tramos: 538 ms y 493 ms de pico a valle) y
// 2.3 % de crecimiento por ciclo -> zeta = -0.004. Falta MUY poco.
// Fase del PID a 6.3 rad/s = atan((Kd*w - Ki/w)/Kp):
//   0.15 -> 51.5 deg      0.22 -> 62.1 deg   (+10.6 deg, ganancia x1.32)
// Diez veces mas margen del que hace falta, a proposito: la resonancia baja de
// frecuencia con la altura (1.35 Hz cerca del piso, 0.95 Hz a 10 cm) y a 40 cm
// todavia no sabemos donde cae.
float Kp_th = PID_TH_KP, Ki_th = PID_TH_KI, Kd_th = PID_TH_KD;
float D_TH_MAX  = 25.0;       // <<< tope del aporte derivativo (us). Acota el tiron del
                              // stick-slip: por alto que este Kd, un desprendimiento
                              // brusco no puede producir un comando desmedido.
float U_TH_MAX  = 180.0;      // aporte diferencial maximo (us)
float ITH_MAX   = 110.0;      // clamp del integrador EN VUELO (~60 us de aporte)
float ITH_MAX_PISO = 200.0;   // clamp EN TIERRA (~110 us): hace falta para despegar
                              // el brazo de su tope. Kp_th*12deg son solo 12 us: NO alcanza.
float ITH_BANDA = 15.0;       // banda de integracion condicional (debe cubrir el angulo
                              // de reposo del brazo, si no el integrador nunca arranca)
float AW_TH     = 2.0;        // ganancia de back-calculation del anti-windup

// --- PID ALTURA (z) --- ganancias "Calculado" de la Tabla III (asignacion de polos)
// Kd_z = 0 NO es un olvido: analisis_pid.py seccion 8 muestra que la derivada de
// altura EMPEORA el lazo. Sin ella el polo dominante es real; con Kd_z=4 aparece
// un modo oscilatorio de zeta=0.555 y Mp=12.3 %. La planta ya trae un polo rapido
// en -21.9 y la derivada, pasando por el retardo del estimador, resuena con el.
// El sobrepaso del 188 % nunca fue falta de amortiguamiento: era windup, y de eso
// se encargan la correa y la captura de hover.
// SINTONIA SUAVE (2026-09-03): -34 % en las dos. Menos correccion por cm de error
// = escalones de empuje comun mas chicos = menos picos de corriente en las fuentes.
// OJO con la telemetria: 'iz' se escala con 1/Ki_z, asi que ahora reposa cerca de
// 111 en vez de 73 para el MISMO empuje. No es que se haya disparado.
float Kp_z = PID_Z_KP, Ki_z = PID_Z_KI;
float Kd_z = 0.0;             // <<< DEJAR EN 0. Ver analisis_pid.py seccion 8.
float UZ_MAX      = 400.0;    // aporte de altura maximo (us), se recalcula en setup()
float UZ_CAIDA    = 60.0;     // margen de empuje por debajo del equilibrio ACTUAL
float AW_Z        = 1.30;     // back-calculation (1/Tt, con Ti = Kp/Ki = 0.77 s)
// 150 (antes 250). De todo lo que hay aqui, ESTA es la que limita de verdad cuan
// rapido puede pedir corriente: acota la pendiente del empuje COMUN, que es el que
// mueve los dos motores en el mismo sentido. El diferencial no cuenta para esto
// porque sube uno y baja el otro y la corriente neta casi no cambia.
float SLEW_UZ_UP  = 90.0;             // RAMPA LENTA: us/s de subida del modo comun. 150 -> 90: es el limite principal
float SLEW_UZ_DN  = 600.0;    // us/s de bajada (cortar empuje siempre es seguro)

// --- PERFIL DE SUBIDA ---
float RAMP_CMS    = 0.20;             // RAMPA LENTA: cm/s de subida. Mas lento = menos empuje por encima del hover
float BAJA_CMS    = 0.35;             // RAMPA LENTA: cm/s de bajada
float MAX_LAG     = 1.5;      // <<< LA CORREA: el SP nunca se aleja mas de esto de z
float ANG_OK_NIVEL= 4.0;      // |theta| medio para pasar de nivelar a buscar hover
float ANG_OK_SUBIR= 6.0;      // |theta| medio por encima del cual la rampa SE CONGELA
int   CICLOS_OK   = 200;      // ciclos estables antes de buscar hover (~1 s)

// --- NIVELACION Y BUSQUEDA DE HOVER ---
// OJO: estos tres van en PWM ABSOLUTO, no en offsets sobre PWM_BASE. Asi puedes
// cambiar PWM_BASE sin romper la logica de despegue.
// 1350 elegido en banco el 2026-09-01 con los ESC nuevos (antes 1400). Los ESC
// nuevos tienen otra curva de acelerador, asi que el valor viejo ya no aplica.
int   PWM_NIVEL     = 1250;   // <<< empuje comun durante la nivelacion. DEBE estar por
                              // encima de la zona muerta del ESC o el lazo de angulo
                              // no tiene autoridad y la maquina de fases se atasca.
int   PWM_BUSQ_MAX  = 1650;   // si no despega en este PWM comun -> aborta
int   PWM_COMUN_MAX = 1800;   // techo del modo comun (de aqui sale UZ_MAX en setup)
                              // 1700 -> 1800: con 1700 el techo util eran ~13 cm.
                              // Ajustable en caliente con 'e' para barrer sin
                              // recompilar. Sube DE POCO EN POCO y mirando v5.
float TASA_NIVEL    = 10.0;           // RAMPA LENTA: us/s de la rampa de nivelacion. 15 -> 10
                              // empuje, mas torque por us de diferencial (el empuje va
                              // con el cuadrado del PWM), asi la autoridad se autoajusta.
float TASA_BUSQ     = 15.0;           // RAMPA LENTA: us/s de la busqueda de hover. 25 -> 15
float DZ_DESPEGUE   = 1.2;    // cm por encima del piso para declarar despegue

// --- DESCUENTO DEL EMPUJE SOBRANTE AL DESPEGAR ---------------------------
// Cuando z ha subido DZ_DESPEGUE el brazo YA VA LANZADO, asi que el empuje de
// ese instante no es el de sustentacion: es mayor. Medido en el vuelo
// a_pid_z05_r1 del 15-sep: al detectar el despegue uz_ff = 336 con vz = 6.9
// cm/s, y el crucero se equilibro en uz = 312. Veinticuatro unidades de exceso
// que ademas se precargaban en el integrador (iz = uz_hover / Ki_z), y por eso
// el brazo se disparaba a 9.45 cm con la consigna todavia en 4.45 y el crucero
// arrancaba con 3.7 cm de error.
// Se descuenta lo que explica la velocidad que ya lleva: 24/6.9 = 3.5.
// OJO: el 3.5 salia de UN vuelo, y ese vuelo era tambien un salto por rozamiento,
// no un despegue limpio, asi que la vz de la deteccion no medi­a exceso de empuje
// sino la sacudida al soltarse. Con 3.5 restaba de mas: el 15-sep el brazo
// despego con uz = 306 y se cayo al suelo con ese mismo empuje puesto, porque el
// de sustentacion estaba por encima de 320.
// El error NO es simetrico: pasarse por arriba da sobrepico y el lazo lo corrige
// solo; quedarse corto deja el brazo sin autoridad para volver -Kp_z son 2.0
// unidades por cm, harian falta 10 cm de error para poner 20 unidades-. Asi que
// el descuento se queda pequeno y topado a proposito.
float K_VZ_HOVER    = 1.5;    // unidades de empuje por cada cm/s de subida
float EXCESO_MAX    = 20.0;   // tope del descuento, por si vz viene con ruido

// --- POSADO NIVELADO -----------------------------------------------------
// Al tocar se cortaba en seco y el brazo quedaba donde cayera. La prueba
// siguiente arrancaba con ese cero torcido: es lo que dejo th_off = -14.9 y
// bloqueo la tanda del 15-sep en bucle de "CALIBRACION MALA".
// Ahora el empuje comun baja despacio con el lazo de ANGULO todavia vivo.
float UZ_POSAR      = 120.0;  // unidades/s con que se retira el empuje al posar
float POSAR_ANG     = 2.5;    // |theta| que se considera "nivelado" para cortar
unsigned long POSAR_MS = 4000;  // y si no lo consigue, se corta igual
unsigned long NIVEL_MAX_MS = 30000;   // RAMPA LENTA: acompana a TASA_NIVEL mas lenta

// --- SENTIDO DEL LAZO DE ANGULO ---
// +1 o -1. Con SIGNO_TH=+1 la mezcla asume que subir el motor DERECHO empuja theta
// hacia POSITIVO. Si en el modo diagnostico (comando d) ves lo contrario, ponlo en -1
// (o pulsa 'n' en caliente). Un signo invertido convierte el lazo en realimentacion
// POSITIVA: el angulo se va al tope mientras el control lo "corrige".
// ACTUALIZADO 2026-09-01 tras cambiar los ESC y el mapeo de pines: SIGNO_TH = -1.
// Barrido medido con IZQ=10 / DER=9, tiempo en llegar a -10 deg desde horizontal:
//     d0 -> 2.7 s    d10 -> 0.9 s    d20 -> 0.3 s    d40 -> 0.2 s
// Es decir: mas uth POSITIVO acelera la caida a NEGATIVO. Monotono, 4 puntos.
// Con SIGNO_TH=+1 eso era realimentacion POSITIVA. Con -1 el lazo cierra bien.
//
// -- historico, con el hardware anterior (ESC viejo, pines IZQ=9/DER=10) --
// CONFIRMADO CON EL GIROSCOPIO (2026-08-22). Se usa el gyro y no theta porque el
// acelerometro estaba saturado por vibracion y theta era un artefacto.
//   d-90 (pl=1490, pr=1310, izquierdo +180) -> gyro -39,-66,-59,-40 deg/s
//   => izquierdo mas fuerte gira a NEGATIVO, luego derecho mas fuerte a POSITIVO.
// Es la convencion que la mezcla ya asumia, y el lazo cierra bien:
//   theta>0 -> err<0 -> Uth<0 -> izquierdo mas fuerte -> theta baja. Correcto.
int   SIGNO_TH     = -1;

// --- COMPENSACION DE ASIMETRIA ---
// TRIM_BASE PUEDE SER NEGATIVO (el clamp "if (trim<0) trim=0" de la v1 lo impedia).
// MEDIDO con el barrido de diagnostico, leyendo el GIROSCOPIO (no theta, que estaba
// contaminado por el acelerometro saturado):
//   d0, d-15, d-30, d-60 -> gyro ~0: el brazo no se despega de su friccion
//   d-80  -> baja al centro y se QUEDA oscilando +-3 deg  <<< punto de balance
//   d-90  -> gyro -69 deg/s, se va a -16 y corta por tope: ya se paso
// La transicion entre -80 (equilibra) y -90 (diverge) es abrupta por la friccion seca
// del pivote, que ademas explica por que de 0 a -60 no pasa nada.
// ---------------------------------------------------------------------
//  TRIM_BASE = 0 desde el cambio de ESC (2026-09-01).
//  Los -80 us anteriores NO compensaban las helices ni los motores: estaban
//  compensando un ESC defectuoso. Reemplazados los dos por unos nuevos, esa
//  compensacion sobra, y ademas hacia dano: en el arranque suave dejaba al
//  motor derecho en 1170 us, por debajo de su zona muerta -> no arrancaba.
//  TODA la caracterizacion de asimetria previa (el barrido d0/d35/d-80, el
//  "deriva a +11 deg", el punto de balance en -80) quedo INVALIDA: se midio
//  con el ESC malo. Hay que repetirla con 'd0' antes de confiar en un trim.
// ---------------------------------------------------------------------
// -6 medido en banco el 2026-09-01 con los ESC nuevos y PWM_NIVEL=1350:
//   d0   -> el brazo cae al tope (-16), dos corridas
//   d-10 -> se estabiliza en +8.5 deg (equilibrio real, no tope)
// Sensibilidad ~2.5 deg/us -> centrar los +8.5 pide ~3.5 us menos que -10.
// Compara con los -80 del hardware anterior: aquello era el ESC defectuoso.
int   TRIM_BASE    = -6;
// K_ASIM en 0 a proposito: primero se acierta el trim base con UNA variable, despues
// se agrega la dependencia con el empuje si hace falta. Y si se agrega, aqui va
// NEGATIVO tambien, porque la asimetria apunta al otro lado de lo que suponia la v1.
float K_ASIM       = 0.0;
int   PWM_REF_ASIM = 1300;
int   TRIM_MAX     = 200;     // cota de magnitud del trim (antes se recortaba a >=0,
                              // lo que hacia IMPOSIBLE compensar hacia el otro lado)

// =========================================================================
//  VIGILANCIA DE LA FUENTE  (12 V / 5 A)
//
//  El bandgap del AVR solo mide el riel de 5 V del Arduino: un hundimiento de
//  12 a 9 V NO se ve ahi porque el regulador lo absorbe. Para ver la fuente de
//  los motores hace falta un divisor resistivo de dos resistencias:
//
//      12V ---[ R1 = 10k ]---+---[ R2 = 4.7k ]--- GND
//                            |
//                           A0
//
//  Con esos valores, 12 V dan 3.84 V en A0 (maximo seguro 15.6 V). El bandgap
//  se usa igual, pero como REFERENCIA para escalar la lectura del divisor: asi
//  la medida no se falsea si el propio riel de 5 V se mueve.
//
//  Hay DOS fuentes, una por motor, asi que van DOS divisores: A0 = motor izquierdo,
//  A1 = motor derecho. Medirlos por separado es lo que identifica al culpable:
//    - solo A0 se hunde  -> la fuente izquierda topa su limite de corriente
//    - solo A1 se hunde  -> la derecha
//    - ambas sanas y el riel de 5 V cae -> el que se reinicia es el ARDUINO, y los
//      pulsos de Servo se corrompen: los DOS motores pierden empuje a la vez
//  Ese ultimo caso es el que encaja con lo observado: el brazo cayo a -12.5 deg,
//  que es exactamente su reposo SIN empuje medido al inicio de la depuracion.
//
//  PON USAR_VBAT EN 1 DESPUES DE CABLEAR LOS DIVISORES. En 0 los pines quedan
//  flotando y leerian ruido, asi que la proteccion por tension queda inhibida
//  (el bandgap del riel de 5 V si funciona siempre, no necesita nada).
// =========================================================================
#define USAR_VBAT 0           // <<< 1 cuando los dos divisores esten cableados
const int PIN_VBAT_L = A0;    // divisor de la fuente del motor IZQUIERDO
const int PIN_VBAT_R = A1;    // divisor de la fuente del motor DERECHO
const float DIV_R1 = 10000.0, DIV_R2 = 4700.0;
const float DIV_RATIO = (DIV_R1 + DIV_R2) / DIV_R2;      // 3.128

float VBAT_SAG   = 10.5;      // por debajo: la fuente esta en su limite de corriente
float VBAT_MIN   = 9.5;       // por debajo: caida franca -> aterrizar
// VCC5_MIN bajado de 4.60 a 4.15 el 2026-09-01.
// El riel esta en 4.35 V REALES (confirmado con multimetro, y el bandgap coincide).
// Esta por debajo del minimo de 4.5 V del ATmega a 16 MHz, pero la placa funciona:
// el MPU lee con acc%=100 y vib=0.1. Con el umbral en 4.60 no se podia ni armar.
// 4.15 sigue atrapando un colapso de verdad (un brownout cae hacia 3 V, no a 4.3).
// ESTO NO ARREGLA LA FUENTE: es para poder seguir midiendo mientras se corrige.
float VCC5_MIN   = 4.15;
// Constante del bandgap: Vcc = VCC5_K / ADC. El nominal es 1.1 V * 1024 = 1125.3,
// pero la referencia interna varia +-10 % entre chips, asi que el valor ABSOLUTO
// puede estar corrido hasta medio volt. Calibralo una vez con el comando 'w':
// mides el pin 5V con multimetro y mandas p.ej. 'w5.02'.
float VCC5_K     = 1125.3;
int   VBAT_CICLOS = 6;        // ciclos seguidos bajo umbral antes de actuar (~30 ms)

// --- TECHO DE EMPUJE ADAPTATIVO ---
// Si la fuente se hunde, en vez de colapsar se baja el techo de PWM comun hasta
// donde la fuente aguante, y se recupera despacio. Es la unica forma de extraer
// el maximo que da una fuente limitada en corriente sin que entre en proteccion.
float PWM_TECHO_MIN = 1250.0; // nunca por debajo de esto (hay que poder sostenerse)
float TECHO_BAJA = 250.0;     // us/s de recorte cuando la fuente se hunde
float TECHO_SUBE =  40.0;     // us/s de recuperacion (lento a proposito)

// --- MODO DIAGNOSTICO ---
unsigned long DIAG_MS = 3000; // duracion de cada prueba de diagnostico

// --- SEGURIDAD ---
// LA ENVOLVENTE MECANICA. Los topes fisicos del brazo estan en ~-18 y ~+26
// grados: mas alla hay estructura, no margen. Esto NO se abre por comando.
float TH_MEC_NEG = -17.0;
float TH_MEC_POS =  25.0;
// La VENTANA de trabajo va alrededor de SP_th, no del cero. Antes el tope era
// absoluto (-16/+20) y eso impedia ensayar consignas de angulo: con SP_th = 10
// el brazo TIENE que estar a 10 grados y el tope saltaba a mitad del ensayo.
// Con ventana relativa se puede barrer SP_th dentro de la envolvente mecanica
// sin perder la proteccion. Ajustable en caliente con 'w'.
// Los valores por defecto reproducen EXACTAMENTE el comportamiento anterior
// cuando SP_th = 0: -16 / +20.
float TH_VENT_NEG = 16.0;
float TH_VENT_POS = 20.0;
float Z_TOPE_SEG   =  42.0;   // el tope FISICO esta en ~45 cm: 3 cm de margen.
                              // Pedir z=45 es tocar el tope; el maximo util es ~42.
// |theta - SP_th| sostenido -> aterrizaje preventivo. Tambien relativo: si no,
// cualquier consigna de angulo mayor que ANG_MALO se autoaterrizaba sola.
float ANG_MALO     =  12.0;
unsigned long ANG_MALO_MS = 1500;
unsigned long Z_WD_MS     = 400;   // sin lectura valida de z -> aborta
unsigned long BUSQ_MAX_MS = 35000;    // RAMPA LENTA: a 15 us/s el hover tarda ~20 s: sin esto abortaria antes

// --- FILTROS ---
// 0.995 (tau ~1 s a 200 Hz). Se subio de 0.99 para que el ruido del acelerometro
// entre a la mitad. OJO: esto NO arregla un sesgo DC del acelerometro, solo lo
// retrasa. El sesgo se arregla amortiguando el MPU. Ver 'vib'.
const float ALPHA = 0.995;    // filtro complementario
const float BETA  = 0.7;      // pasabajos extra de theta
// 0.70 (antes 0.85). En vuelo la planta tiene una resonancia a 1.1 Hz que las
// pruebas en tierra nunca mostraron (el modelo del paper da 0.111 Hz: es otro modo,
// el del brazo apoyado). Con dos EMAs en cascada, tau = dt*FG/(1-FG):
//   FG=0.85 -> tau=28.3 ms -> 22.4 deg de retraso a 1.1 Hz
//   FG=0.70 -> tau=11.7 ms ->  9.3 deg
// Son 13 deg de margen de fase recuperados sin tocar ninguna ganancia, y justo en
// el canal derivativo, que es el que aporta el amortiguamiento. El gyro aguanta:
// en el vuelo del 2026-09-03 sale como una senoide limpia con vib entre 30 y 88.
const float FG    = 0.70;     // gyro: 2 etapas -> 2do orden
// --- ESTIMADOR ALFA-BETA DE ALTURA ---
// Sustituye al EMA + diferencia sobre ventana de 50 ms. Predice (z,vz) a la tasa del
// lazo con modelo de velocidad constante y corrige solo cuando llega medida nueva.
// Medido en simulacion a 50 Hz: 18 % menos error de vz y la MITAD de desfase
// (-21.6 deg contra -39.6 deg a 0.5 Hz). Ver analisis_pid.py seccion 6.
const unsigned long ULTRA_MS = 20;   // 50 Hz. A 5 ms el HC-SR04 devuelve ecos fantasma
                                     // del ping anterior, y ademas el pulseIn bloqueante
                                     // frenaba el lazo de angulo en TODOS los ciclos.
const float ALFA_AB = 0.25;          // ganancia de correccion de posicion
const float BETA_AB = ALFA_AB * ALFA_AB / (2.0 - ALFA_AB);  // Benedict-Bordner
const float MAX_INNOV_CM = 4.0;      // rechazo de outliers por innovacion (contra la
                                     // PREDICCION, no contra la ultima muestra)
const float GYRO_SCALE = 131.0;
const float ACC_LSB    = 4096.0;  // LSB/g con el rango en +-8g (era 16384 a +-2g)
const float ACC_TOL    = 0.20;    // tolerancia del modulo: |a| debe estar en 1.00 +- esto
                                  // para que la muestra cuente como "hacia donde esta abajo"
// Limite de VELOCIDAD de la correccion por acelerometro, en deg/s.
// La compuerta de modulo (ACC_TOL) mira si |a| vale 1 g, pero una vibracion
// rectificada puede cumplir eso y aun asi apuntar mal: medido el 2026-09-01,
// th_acc con sesgo de +40 deg pasando la compuerta con acc%=90.
// Con ALPHA=0.995 a 200 Hz ese sesgo arrastra theta a 0.005*40*200 = 40 deg/s,
// que es justo el salto de 87 deg/s que aborto el vuelo en t=40749 ms.
// El acelerometro solo tiene que corregir la DERIVA del gyro, asi que 2 deg/s
// le quita toda autoridad al ruido.
// Esto es una MITIGACION: el arreglo de verdad es amortiguar el MPU.
const float CORR_MAX_DPS = 2.0;

// --- DERIVA DEL CERO DEL GIROSCOPO ---------------------------------------
// 2 deg/s NO bastaban, y el motivo es que esa autoridad hay que multiplicarla
// por la fraccion de muestras que pasan la compuerta: en vuelo acc_pct cae al
// 5-18 % por la vibracion, o sea 0.1-0.36 deg/s reales.
// Y la deriva contra la que pelea es mayor que eso. Medido el 15-sep-2026, dos
// calibraciones de la MISMA sesion (sketch del Hinf):
//     gyro_off = 3.113   ...  y minutos despues  gyro_off = 1.480
// 1.63 deg/s de diferencia. El filtro no puede ganar, y se ve en crudo en el
// volcado de 'h' de ese mismo vuelo:  theta=-7.83  th_acc=4.44  acc%=86
// -doce grados de error con el acelerometro sano y mirando-. De ahi salen la
// deriva de -0.27 deg/s del IMC y el Hinf que enrollo los dos integradores y
// no despego.
//
// Arreglo: estimar la deriva EN MARCHA y devolversela a gyro_offset. Es el
// lazo lento de siempre del filtro complementario. Ojo con el paper: esto
// actua con constante de ~20 s, o sea 120 veces mas lento que los 6 rad/s
// donde estan igualadas las seis leyes, asi que NO cambia la comparacion.
// Barrido en simulacion (200 Hz, acc al 8 %, ruido 0.5 deg, 1.6 deg/s de deriva):
//   |error| de theta a los 45-60 s     hoy 74.9 deg  ->  con esto 0.27 deg
//   caza el 90 % de la deriva en 2.8 s
//   y en el punto de diseno NO cambia nada: a 6 rad/s la ganancia del filtro
//   pasa de 0.9997 a 1.0032 y la fase de 0.83 a 0.84 deg. A 0.1 Hz vale 0.92,
//   sin pico: ahi es donde quita la deriva y no hay que resonar con el
//   integrador del angulo.
const float DERIVA_K     = 0.30;   // deg/s de correccion por grado de error
const float F_ACC_LP     = 0.30;   // media del acelerometro (solo muestras buenas)
const float DERIVA_TOPE  = 8.0;    // cuanto se le deja alejarse del valor calibrado
const float DERIVA_W_MAX = 10.0;   // no adaptar si el brazo gira de verdad (deg/s)
const float DERIVA_E_MAX = 20.0;   // ni si el desacuerdo es absurdo (acelerometro loco)
// ...ni si el acelerometro no es de fiar. MEDIDO en el vuelo del 15-sep: parado
// acc_pct = 99-100 y th_acc casa con theta a 0.1 deg; en vuelo acc_pct cae a
// 7-15 % y la columna th_acc salta entre -72 y +72 deg. La compuerta de MODULO
// no basta: un acelerometro sacudido puede medir 1 g justo apuntando a
// cualquier lado, asi que esas pocas muestras "validas" son basura y el
// estimador las perseguia -deriva paseandose de +4.66 a -2.96 dps en 5 s, casi
// el tope entero-. El cero del gyro se mueve en MINUTOS, no dentro de un vuelo
// de 10 s: congelarlo mientras hay vibracion no pierde nada.
const float DERIVA_ACC_MIN = 80.0; // % de muestras buenas por debajo del cual NO se adapta
// Y ademas por vibracion, que avisa ANTES que acc_pct. Medido en el vuelo del
// 15-sep: con los motores al ralenti, acc_pct seguia en 99 y th_acc ya marcaba
// +12.2 deg con theta en +2.9 -- vib valia 4.6. Parado ('h', motores off) vib
// esta en 0.3-1.4 y th_acc casa con theta dentro de 0.7 deg. O sea acc_pct por
// si solo llega tarde: deja que la media se contamine antes de congelarse.
const float DERIVA_VIB_MAX = 2.0;  // por encima de esto el acelerometro ya miente
const float DT_MIN = 0.005;   // 200 Hz max
const float DT_MAX = 0.05;    // clamp: protege integradores tras un bloqueo
const int SOFT_PASO = 3, SOFT_MS = 40;// RAMPA LENTA: arranque suave de los ESC: 133 -> 75 us/s

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
float gyro_off_cal = 0;        // el de la calibracion, ancla del tope de deriva
float th_acc_lp = 0;           // media del acelerometro = hacia donde esta abajo
bool  acc_visto = false;
float theta_filt = 0, gyro_f1 = 0, gyro_dps = 0, ang_abs = 0;
// th_comp era 'static' dentro de actualizar_theta() y NO se reiniciaba al
// calibrar. Con la correccion sin limitar eso se disimulaba (convergia en ~1 s);
// con CORR_MAX_DPS un cero viejo tardaria varios segundos en irse. Global y a 0.
float th_comp = 0;
// Crudos para diagnostico: permiten ver CUAL de los dos sensores se corrompe
// cuando los motores vibran. th_acc_dbg = angulo por acelerometro (sin filtrar),
// gyro_raw_dbg = velocidad angular ya sin offset pero sin filtrar.
float th_acc_dbg = 0, gyro_raw_dbg = 0;
// Medidor de vibracion: salto medio de th_acc entre muestras consecutivas.
// Es el numero que hay que bajar montando el MPU sobre amortiguacion.
//   > 20 deg  -> el acelerometro es inutilizable (asi estaba: ~40-60)
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
//  LEY DE CONTROL: IMC
// =========================================================================
// AUTORIDAD (generada por diseno_controladores.py, no editar):
//   angulo  a 6.0 rad/s (0.95 Hz):  |C| = 1.4356   fase = +48.8 deg
//   altura  a 1.0 rad/s          :  |C| = 3.2802   fase = -36.7 deg
// Los seis estan ajustados por biseccion a la MISMA |C| que el PID que vuela.
// Lo unico distinto es la fase, y eso es estructura, no sintonia.
//
// Control por Modelo Interno. Un solo parametro de sintonia (lambda), que la
// biseccion fijo para igualar la autoridad del PID.
//
// CANDIDATO A OSCILAR, y por una razon concreta: el IMC invierte el modelo
// explicitamente, asi que es el mas expuesto a que el modelo este mal. Y lo
// esta: G_th describe el brazo APOYADO y no contiene la resonancia de vuelo de
// 0.95 Hz. Con tan poco adelanto de fase queda poco amortiguamiento en ese
// modo. VUELA ESTE CON 'a0.5' Y SUBE DESDE AHI.
//
// El canal de altura no lleva integrador propio: G_z ya es tipo 1 y el IMC da
// error nulo en regimen sin anadir otro.
//
// Todo lo demas es IDENTICO en los seis sketches: sensores, filtros, maquina
// de fases, abortos, gobernador de referencia y mezcla con prioridad al
// angulo. Generado por controladores/generar_sketches.py - si hay que tocar
// el andamiaje, se toca ahi y se regeneran los seis.

// ---- escala viva de la salida de ANGULO (comando 'a') -------------------
// Solo toca el angulo: escalar el modo comun romperia el equilibrio de hover.
// Sirve para volar primero al 50 % una ley de la que no te fias y subir desde
// ahi sin recompilar.
float ctrl_esc = 1.0f;
bool  acc_ok_g = false;        // la compuerta del acelerometro, visible aqui

// ---- lectura de numeros de la consola ----------------------------------
// NO usa Serial.parseFloat(). Ese se para en el primer caracter no numerico,
// asi que con teclado espanol 'a0,5' -la coma del teclado numerico- devolvia
// 0 y apagaba el lazo de angulo; y con setTimeout(10) tambien devuelve 0 si el
// monitor manda la linea en dos trozos. Aqui se junta la linea entera, se
// acepta la coma como separador decimal y se espera un poco mas.
// Devuelve false si no habia ningun digito: el llamador NO cambia nada.
// Bloquea como mucho LEER_MS. Se teclea en vuelo, y el lazo va a 200 Hz: 30 ms
// son 6 ciclos perdidos, que el gobernador de referencia absorbe. Mas seria
// notarse en el brazo.
const uint8_t LEER_MS = 30;

bool leer_num(float &v) {
  char buf[14];
  uint8_t n = 0;
  unsigned long t0 = millis(), tc = t0;
  while (millis() - t0 < LEER_MS && n < sizeof(buf) - 1) {
    if (!Serial.available()) {
      if (n && millis() - tc > 4) break;          // numero completo, no esperes
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

// ---- biquads del IMC ----------------------------------------------------
// Forma directa II transpuesta: menos estado y mejor condicionada en float de
// 32 bits que la forma directa I.
float imc_th[2] = {0, 0}, imc_z[2] = {0, 0};
// amortiguamiento directo de giroscopio, ajustable en caliente con 'k'
float imc_kd = IMC_TH_KD;

static inline float pf(const float *p, uint8_t i) { return pgm_read_float(&p[i]); }

float biquad(float x, const float *bq, float *st) {
  float y = pf(bq, 0) * x + st[0];
  st[0] = pf(bq, 1) * x - pf(bq, 3) * y + st[1];
  st[1] = pf(bq, 2) * x - pf(bq, 4) * y;
  return y;
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
  Serial.println(F("# BALANCIN - ley IMC. C=arrancar L=aterrizar S=stop z# t# v# a# w# e# +/- m# h"));
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
  else reposo();          // mantiene theta_filt VIVO con los motores apagados
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
  // Si la alimentacion colapsa, el empuje se va y el brazo cae a su reposo natural.
  // No hay control que compense eso: lo unico sensato es dejar de exigir corriente.
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
  // Una fuente en su limite de corriente no avisa: recorta la tension. En vez de
  // insistir hasta que colapse, se baja el techo de PWM comun hasta donde aguante
  // y se recupera despacio. Extrae el maximo que la fuente puede dar de forma estable.
  if (vb > 0.0f && vb < VBAT_SAG) pwm_techo -= TECHO_BAJA * dt;
  else if (pwm_techo < (float)PWM_MAX_SEG) pwm_techo += TECHO_SUBE * dt;
  pwm_techo = constrain(pwm_techo, PWM_TECHO_MIN, (float)PWM_MAX_SEG);
  actualizar_z(dt);      // marca t_ultimo_z solo cuando corrige con medida valida
  if (millis() - t_ultimo_z > Z_WD_MS) { abortar(F("sensor de altura mudo")); return; }

  // ---------- LIMITES DUROS ----------
  // Ventana relativa a SP_th, recortada SIEMPRE por la envolvente mecanica.
  // En el piso el brazo descansa apoyado en su tope: vigilarlo ahi no mide la
  // ley, solo aborta despegues buenos (le paso al Hinf en c22, z = 2.1 cm).
  float ab_neg = TH_MEC_NEG - 10.0f, ab_pos = TH_MEC_POS + 10.0f;
  if (en_vuelo) {
    ab_neg = SP_th - TH_VENT_NEG; if (ab_neg < TH_MEC_NEG) ab_neg = TH_MEC_NEG;
    ab_pos = SP_th + TH_VENT_POS; if (ab_pos > TH_MEC_POS) ab_pos = TH_MEC_POS;
  }
  if (theta_filt > ab_pos || theta_filt < ab_neg) { abortar(F("tope de angulo")); return; }
  if (z_filt > Z_TOPE_SEG)                                    { abortar(F("tope de altura")); return; }

  // ---------- SUPERVISION SUAVE: angulo malo sostenido -> aterriza ----------
  // Solo EN VUELO: en tierra el brazo descansa inclinado sobre su tope y eso es
  // normal; vigilarlo ahi disparaba un aterrizaje preventivo espurio.
  if (en_vuelo && fabs(theta_filt - SP_th) > ANG_MALO) {
    if (t_ang_malo == 0) t_ang_malo = millis();
    else if (millis() - t_ang_malo > ANG_MALO_MS && fase != F_ATERRIZAR) {
      fase = F_ATERRIZAR; Serial.println(F("# angulo inestable -> aterrizaje preventivo"));
    }
  } else t_ang_malo = 0;

  // ---------- MAQUINA DE FASES ----------
  switch (fase) {

    case F_NIVELAR:
      // Empuje comun de NIVELACION. Sin esto los motores quedan en PWM_BASE, por
      // debajo de la zona muerta del ESC, el lazo de angulo no tiene autoridad y
      // la fase 0 no termina nunca.
      if (uz_ff < (float)(PWM_NIVEL - PWM_BASE)) uz_ff = (float)(PWM_NIVEL - PWM_BASE);
      else if (ang_abs > ANG_OK_NIVEL && PWM_BASE + uz_ff < PWM_BUSQ_MAX) uz_ff += TASA_NIVEL * dt;
      sp_z_actual = z_piso;
      if (capturar_despegue()) break;     // por si el empuje de nivelacion ya levanta
      if (ang_abs < ANG_OK_NIVEL) {
        if (++cont_ok > CICLOS_OK) { fase = F_HOVER; cont_ok = 0; t_fase = millis();
                                     Serial.println(F("# nivelado -> buscando hover...")); }
      } else {
        cont_ok = 0;
        // DETECTOR DE SIGNO INVERTIDO: si el angulo EMPEORA mientras el lazo lo combate,
        // la realimentacion es positiva. Cortar en 3 s en vez de esperar al tope.
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

    case F_SUBIR: {                       // rampa CON CORREA: solo avanza si la planta sigue
      bool sigue  = (sp_z_actual - z_filt) < MAX_LAG;
      bool ang_ok = (ang_abs < ANG_OK_SUBIR);
      subiendo = (sigue && ang_ok);
      if (subiendo) sp_z_actual += RAMP_CMS * dt;
      // REVERTIDO al estado del vuelo de 30 cm (2026-08-26).
      // !! PELIGRO CONOCIDO: si mandas un 'z' MUY POR DEBAJO del setpoint actual
      // estando en ascenso, esta linea pega sp_z_actual al objetivo de golpe. Un
      // 'z5' a 30 cm mete un escalon de -25 cm y la plataforma cae en caida libre
      // (medido: 29.7 -> 1.0 cm en 0.8 s). Baja de a poco: z25, z20, z15...
      if (sp_z_actual >= SP_z_target) { sp_z_actual = SP_z_target; fase = F_CRUCERO; subiendo = false;
                                        Serial.println(F("# crucero")); }
      break; }

    case F_CRUCERO:
      subiendo = false;
      if (SP_z_target > sp_z_actual + 0.2f) fase = F_SUBIR;            // subir mas con z<val>
      else if (SP_z_target < sp_z_actual - 0.2f) sp_z_actual -= BAJA_CMS * dt;  // bajar CON RAMPA, no en escalon
      else sp_z_actual = SP_z_target;
      break;

    case F_ATERRIZAR:
      subiendo = false;
      sp_z_actual -= BAJA_CMS * dt;
      // corta al DETECTAR contacto o al terminar la rampa, lo que ocurra primero.
      // Sin la deteccion de contacto el error se volveria positivo al tocar el piso
      // y el lazo volveria a despegar (rebote).
      if (z_filt <= z_piso + 0.6f || sp_z_actual <= z_piso + 0.3f) {
        // No se corta aqui: se pasa a posar. Cortando en seco el brazo se
        // quedaba donde cayera y la prueba siguiente calibraba el cero torcido.
        fase = F_POSAR; t_fase = millis();
        uz_posar = Uz_ant;                  // el empuje que llevaba al tocar
        en_vuelo = false;                   // el lazo de altura ya no manda
        Serial.println(F("# tocando piso -> posando nivelado"));
      }
      break;

    case F_POSAR:
      // El empuje comun se retira despacio mientras el lazo de ANGULO sigue
      // vivo (ese lazo no depende de en_vuelo), asi que el brazo se posa
      // nivelado y la siguiente calibracion sale buena.
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

  // ---------- LAZO DE ALTURA (IMC) ----------
  float err_z  = sp_z_actual - z_filt;
  float err_th = SP_th - theta_filt;
  float Uz_ideal, Uz, Uth_ideal;

  if (en_vuelo) {
    float vz_ref = subiendo ? RAMP_CMS : (fase == F_ATERRIZAR ? -BAJA_CMS : 0.0f);
    (void)vz_ref;
    Uz_ideal = uz_hover + biquad(err_z, IMC_Z_BQ, imc_z);
    // REVERTIDO al estado del vuelo de 30 cm: el piso se ancla al hover del DESPEGUE.
    // (Se probo anclarlo al integrador con un limitador de descenso a 3 cm/s, pero
    //  ese umbral quedaba por debajo del ruido del estimador -en crucero vz llega a
    //  -5 cm/s sin que la plataforma baje- y el lazo perdia autoridad para descender.)
    float uz_lo = uz_hover - UZ_CAIDA; if (uz_lo < 0.0f) uz_lo = 0.0f;
    Uz = constrain(Uz_ideal, uz_lo, UZ_MAX);
  } else {
    // En el piso la ALTURA va en lazo abierto (rampa de nivelacion y busqueda
    // de hover), pero el lazo de ANGULO ya tiene que estar vivo: es lo unico
    // capaz de levantar el brazo de su tope antes de despegar.
    Uz_ideal = uz_ff;
    Uz = uz_ff;
    iz = uz_ff;                           // esta ley no usa iz
  }

  // limitador de pendiente del modo comun (picos de corriente)
  float paso_up = SLEW_UZ_UP * dt, paso_dn = SLEW_UZ_DN * dt;
  if (Uz > Uz_ant + paso_up) Uz = Uz_ant + paso_up;
  if (Uz < Uz_ant - paso_dn) Uz = Uz_ant - paso_dn;
  Uz_ant = Uz;

  // ---------- LAZO DE ANGULO (IMC) ----------
  // El tope D_TH_MAX del aporte derivativo se aplica en TODAS las leyes con
  // termino en giroscopio, no solo en el PID: ante un desprendimiento por
  // friccion seca el gyro salta a decenas de deg/s y sin el el comando se
  // vuelve un tiron. Ademas asi la comparacion lleva la misma proteccion.
  Uth_ideal = (biquad(err_th, IMC_TH_BQ, imc_th)
            + constrain(-imc_kd * gyro_dps, -D_TH_MAX, D_TH_MAX)) * ctrl_esc;
  float Uth = constrain(Uth_ideal, -U_TH_MAX, U_TH_MAX);

  // ---------- MEZCLA CON PRIORIDAD AL ANGULO ----------
  float comun = PWM_BASE + Uz;
  int trim = TRIM_BASE + (int)(K_ASIM * (comun - PWM_REF_ASIM));
  trim = constrain(trim, -TRIM_MAX, TRIM_MAX);   // puede ser NEGATIVO: el motor debil
                                                 // no tiene por que ser siempre el derecho

  float uth_mix = SIGNO_TH * Uth;         // sentido fisico del par diferencial
  float dL = -uth_mix - trim;             // desviacion del motor izquierdo
  float dR =  uth_mix + trim;             // desviacion del motor derecho

  // si el diferencial no cabe en el rango, escalarlo (red de seguridad)
  float span  = fabs(dR - dL);
  float rango = pwm_techo - (float)PWM_MIN;
  if (span > rango) { float k = rango / span; dL *= k; dR *= k; }

  // si un lado satura, se mueve el COMUN (altura) y se respeta el diferencial (angulo)
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

  // ---------- ANTI-WINDUP con el empuje REALMENTE aplicado ----------
  float Uz_real = comun - PWM_BASE;       // incluye slew + recorte por prioridad de angulo
  if (en_vuelo) {
    iz += err_z * dt + AW_Z * (Uz_real - Uz_ideal) * dt;
    iz = constrain(iz, 0.0f, UZ_MAX);   // esta ley no usa iz
  }
  // El integrador de angulo SI trabaja en tierra: es lo unico capaz de levantar el
  // brazo de su tope de reposo (la parte proporcional da solo ~1 us por grado).
  // En tierra usa un clamp mas alto; al despegar se recorta a ITH_MAX (ver F_HOVER).
  if (fabs(err_th) < ITH_BANDA) {
    float clamp_ith = en_vuelo ? ITH_MAX : ITH_MAX_PISO;
    ith += err_th * dt + AW_TH * (Uth - Uth_ideal) * dt;
    ith = constrain(ith, -clamp_ith, clamp_ith);
  }

  // ---------- TELEMETRIA (10 Hz) ----------
  // Columnas 1-7 IDENTICAS a la v1 -> CAPTURA_CONTROL_MIMO.m sigue funcionando.
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
    // --- nuevas ---
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
    // gyro y vib: sin estas dos no se puede saber si un salto de theta fue
    // movimiento real o el acelerometro mintiendo. Si theta se mueve y el
    // gyro esta en cero, es mentira.
    Serial.print(gyro_dps, 2);              Serial.print(',');
    Serial.print(vib, 1);                   Serial.print(',');
    // Las dos columnas que faltaban para cerrar el diagnostico de la deriva.
    // Con los motores parados ('h' del 15-sep) theta seguia al acelerometro con
    // 0.1 deg de error, o sea el estimador esta bien EN REPOSO. Lo que no se
    // puede saber sin esto es que pasa EN VUELO, donde acc_pct cae al 5-18%:
    //   deriva crece y th_acc se queda quieto -> el gyro se sesga al vibrar
    //   deriva ~ 0 y th_acc SIGUE a theta     -> el brazo gira de verdad
    Serial.print(gyro_offset - gyro_off_cal, 2);  Serial.print(',');
    Serial.print(th_acc_lp, 2);             Serial.print(',');
    Serial.print(4);                             Serial.print(',');   // ley
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
  int16_t ax = (Wire.read() << 8) | Wire.read();   // ax ya NO se descarta: hace falta
  int16_t ay = (Wire.read() << 8) | Wire.read();   // para el modulo del vector
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
  // En reposo el acelerometro debe medir exactamente 1 g. Si el modulo se aparta de
  // 1 g, la muestra contiene aceleracion lineal o vibracion y NO dice hacia donde
  // esta abajo: usarla corrompe theta. Se descarta y ese ciclo se integra solo gyro.
  // (Esto es lo que necesita el rango de +-8g: a +-2g el modulo tambien se satura
  //  y la compuerta no puede distinguir una muestra sana de una saturada.)
  float fax = ax / ACC_LSB, fay = ay / ACC_LSB, faz = az / ACC_LSB;
  float amag = sqrt(fax * fax + fay * fay + faz * faz);
  bool acc_ok = fabs(amag - 1.0f) < ACC_TOL;
  acc_ok_g = acc_ok;
  acc_pct = 0.99f * acc_pct + (acc_ok ? 1.0f : 0.0f);   // % de muestras utilizables

  // gyro: dos EMAs en cascada = pasabajos de 2do orden (~5.6 Hz a 200 Hz de muestreo)
  gyro_f1  = FG * gyro_f1  + (1.0f - FG) * g;
  gyro_dps = FG * gyro_dps + (1.0f - FG) * gyro_f1;

  th_comp += g * dt;                                              // PREDICCION: gyro siempre
  if (acc_ok) {                                                   // CORRECCION: solo si limpia
    float corr = (1.0f - ALPHA) * (th_acc - th_comp);             // lo que pide el acelerometro
    float tope = CORR_MAX_DPS * dt;                               // ... y lo que se le permite
    th_comp += constrain(corr, -tope, tope);
    // Media del acelerometro. Se alimenta SOLO con muestras que pasan la
    // compuerta, que en vuelo son el 5-18 %.
    if (acc_visto) th_acc_lp += (th_acc - th_acc_lp) * F_ACC_LP;
    else { th_acc_lp = th_acc; acc_visto = true; }
  }

  // DERIVA DEL CERO DEL GIROSCOPO. Corre en TODOS los ciclos, no solo cuando
  // hay muestra buena: esa es la diferencia entre cazar la deriva en 3 s o no
  // cazarla nunca, porque la autoridad de la correccion de arriba hay que
  // multiplicarla por acc_pct y se queda en 0.1-0.36 deg/s.
  // Lo que el acelerometro pide de forma SOSTENIDA no es ruido: es que el cero
  // del gyro se ha movido. Se lo devolvemos a gyro_offset en vez de pelearlo.
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
  if (ms - t_ultra < ULTRA_MS) return;                    // aun no toca medir
  float dtu = (ms - t_ultra) / 1000.0f;
  t_ultra = ms;

  float z = leer_ultra();
  if (z < 0.5f || z > 60.0f) return;
  float zc = z * cos(theta_filt * PI / 180.0f);           // proyeccion vertical

  float r = zc - z_filt;                                  // innovacion
  if (fabs(r) > MAX_INNOV_CM && n_raros < 5) { n_raros++; return; }
  n_raros = 0;                                            // tras 5 seguidos se acepta (movimiento real)

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
  if (c == 'd' || c == 'D') {                 // prueba de sentido/asimetria en lazo ABIERTO
    diag_uth = Serial.parseFloat();
    diag_uth = constrain(diag_uth, -120.0f, 120.0f);
    // El brazo se queda donde lo deja la prueba anterior (hay mucha friccion en el
    // pivote). Si arranca cerca del tope, la prueba se corta al instante y no mide nada.
    float margen = 0.6f * ((theta_filt > 0) ? TH_MEC_POS : -TH_MEC_NEG);
    if (fabs(theta_filt) > margen) {
      Serial.print(F("# diag NO inicia: nivela el brazo A MANO primero. theta="));
      Serial.println(theta_filt, 1);
    } else {
      arranque_suave(PWM_NIVEL, 0);           // sin trim: queremos ver la asimetria cruda
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
  else if (c == 'o' || c == 'O') {   // recalibrar el cero con el brazo ya nivelado
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
  imc_th[0] = imc_th[1] = imc_z[0] = imc_z[1] = 0;
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
  // REVERTIDO: sin acote. !! Un tecleo como 'z59' con el tope fisico en 45 manda
  // la plataforma contra el final de la guia. El aborto por Z_TOPE_SEG (42) es la
  // unica red que queda.
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
  else if (c == 'k' || c == 'K') {   // amortiguamiento de gyro en caliente
    float x;
    if (leer_num(x)) imc_kd = constrain(x, 0.0f, 1.0f);
    else Serial.println(F("# 'k' sin numero: valor SIN CAMBIAR. Ej: k0.15"));
    Serial.print(F("# kd_gyro=")); Serial.println(imc_kd, 4);
    Serial.println(F("# !! cambiar kd rompe la igualdad de autoridad del paper"));
  }
  else if (c == 'a' || c == 'A') {   // escala viva del canal de angulo
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
    Serial.print(F("# theta=")); Serial.print(theta_filt, 2);   // lectura VIVA
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
//  REPOSO: motores apagados pero la IMU se sigue leyendo.
//  Sin esto theta_filt se congelaba en el ultimo valor de la prueba anterior, y el
//  guardia de 'd' rechazaba arrancar para siempre por mas que nivelaras el brazo.
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
//  Aplica empuje comun PWM_NIVEL con un diferencial FIJO y SIN trim, sin ningun
//  lazo cerrado, durante DIAG_MS. Sirve para responder dos preguntas de una:
//    'd0'   -> sin diferencial: hacia donde cae el brazo = asimetria real
//              (si cae a NEGATIVO el motor derecho empuja de mas -> TRIM_BASE < 0)
//    'd30'  -> diferencial positivo (derecho mas fuerte): theta DEBE ir a POSITIVO.
//              Si va a negativo, SIGNO_TH esta invertido.
// =========================================================================
// El veredicto del diagnostico de sentido, dicho BIEN.
//
// El mensaje viejo ("con uth>0 theta debe ir a POSITIVO") era una trampa: lo
// escupia despues de 'd0' -donde uth=0 y no dice nada- y se callaba despues de
// 'd30', que es el unico que mide el sentido, porque ese sale por el corte de
// tope. Y sobre todo IGNORABA SIGNO_TH. diagnostico() manda los motores en
// crudo (pl = NIVEL - uth, pr = NIVEL + uth) sin pasar por SIGNO_TH, asi que
// con SIGNO_TH = -1 lo CORRECTO es que theta se vaya a NEGATIVO. Leido al pie
// de la letra, el mensaje pedia pulsar 'n' sobre un signo que ya estaba bien:
// eso convierte el lazo de angulo en realimentacion POSITIVA.
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
    veredicto_diag();     // el corte por tope ES el resultado de 'd30': no callarse
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
    // --- LOS DOS SENSORES POR SEPARADO ---
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
  if (r > 100) {                             // r<100 seria Vcc>11 V: lectura absurda
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
//  Gira UN motor a la vez para poder ver cual es cual y confirmar que
//  PIN_MOTOR_IZQ / PIN_MOTOR_DER coinciden con el cableado real.
//  Por que importa: si estan intercambiados, el TRIM refuerza el motor
//  equivocado y el lazo de angulo se vuelve realimentacion POSITIVA -> el
//  brazo se va al tope mientras el control cree que lo esta corrigiendo.
//  A 1300 us hay giro claramente visible pero muy por debajo del hover
//  (~1470 us), y como solo gira UN motor la plataforma no puede levantarse.
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

// Detecta el despegue, congela el hover medido y pasa a subir.
// El integrador de altura arranca EXACTAMENTE en el empuje que acaba de levantar
// la plataforma -> transferencia sin salto, sin nada acumulado.
bool capturar_despegue() {
  if (z_filt <= z_piso + DZ_DESPEGUE) return false;
  // El empuje de AHORA no es el de sustentacion: el brazo ya sube a vz. Ver la
  // nota de K_VZ_HOVER. Sin este descuento el sobrepico era del 89 %.
  float exceso = K_VZ_HOVER * vz_filt;
  if (exceso < 0.0f) exceso = 0.0f;
  if (exceso > EXCESO_MAX) exceso = EXCESO_MAX;
  uz_hover = uz_ff - exceso;
  iz = uz_hover;                       // esta ley trabaja como delta
                                       // alrededor de uz_hover
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
  // El brazo DEBE estar horizontal al calibrar: th_off es el cero del lazo de angulo.
  // Historico de esta planta: -3 a +4.5 deg. Fuera de ahi, SP_th=0 apunta a una
  // actitud inclinada y el margen hasta los topes se reduce sin que se note.
  if (fabs(theta_offset) > 8.0f) {
    Serial.println(F("# !! th_off FUERA DE RANGO: el brazo NO estaba horizontal."));
    Serial.println(F("# !! Nivelalo y resetea, o el cero del control queda torcido."));
  }
}

void mpu_init() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true); delay(100);
  // ACCEL_CONFIG = 0x10 -> +-8g (antes 0x00 = +-2g).
  // MEDIDO: con motores a 1400 el acelerometro entregaba +-60 deg de ruido y un sesgo
  // DC de +9.5 deg. Eso es RECORTE: la vibracion excede +-2g, se satura, y el recorte
  // se rectifica en un offset. A +-8g deja de saturar.
  // No hay que tocar nada mas: theta sale de atan2(ay,az), que es invariante a escala.
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(true);
  // GYRO_CONFIG +-250 deg/s: el gyro esta limpio (+-2 deg/s con motores girando), no se toca.
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
  // DLPF en 21 Hz: no se baja mas porque el mismo registro filtra el gyro, y el termino
  // Kd depende de que el gyro no tenga retardo. El recorte se ataca con el rango, no aqui.
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x04); Wire.endTransmission(true); delay(50);
}

void apagar() { motorIzq.writeMicroseconds(PWM_MIN); motorDer.writeMicroseconds(PWM_MIN); }

void esperar_reset() {
  apagar(); controlActivo = false; en_vuelo = false;
  Serial.println(F("# EMERGENCIA. 'r' para reset."));
  while (emergencia) { if (Serial.available() && Serial.read() == 'r') { emergencia = false; Serial.println(F("LISTO")); } delay(200); }
}

void ISR_emg() { emergencia = true; motorIzq.writeMicroseconds(PWM_MIN); motorDer.writeMicroseconds(PWM_MIN); }
