# Grupo de Investigación UTEC

Proyectos de investigación de estudiantes de Ingeniería Mecatrónica de la UTEC (Lima, Perú): hardware, código y
documentación de cada uno.

**Web del grupo:** se publica con GitHub Pages desde este mismo repositorio (`index.html` en la raíz).

## Proyectos

| Proyecto | Qué es | Estado |
|---|---|---|
| [Tower](tower/) | Banco torre-hélice de 2 GDL: seis leyes de control (PID, LQI, LQG, MPC, IMC y H∞) en Arduino, comparadas en el mismo hardware. | Paper en revisión (ICSC 2026) |
| Dron | Próximamente. | — |

## Estructura

```
├── index.html           Página principal del grupo
├── assets/             Estilos compartidos e ícono del grupo
└── tower/
    ├── index.html       Página del proyecto Tower
    ├── README.md        Cómo usar el firmware
    ├── firmware/        Un sketch de Arduino por ley de control
    ├── firmware_tower.zip
    └── img/
```

Para agregar un proyecto nuevo: crear su carpeta (por ejemplo `dron/`) con su `index.html`, y añadir su tarjeta en
la sección *Proyectos* del `index.html` principal.

## Integrantes

George H. Ramos Astuhuamán, Enrique A. Chávez Depaz, Johan N. Naupay Fabian, Diego A. Miranda Zamora,
Bruno S. Ramos Cadenillas y Sebastián Rejas Córdova. Asesor: Prof. Dante Inga Narváez.
