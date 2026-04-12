# MAZEBOT T17 — Robot Labyrinthe 5x5

**Equipe 17 | ICAM Strasbourg Europe | 2026**

Robot autonome capable de resoudre un labyrinthe 5x5 (cases 20x20 cm) en deux passes, pilote par une interface web embarquee.

## Fonctionnement

### Run 1 — Exploration (Tremaux)
Le robot explore le labyrinthe case par case a vitesse reduite. A chaque case, il scanne les murs avec 4 capteurs ToF VL53L0X, met a jour sa carte interne et choisit la direction suivante selon l'algorithme de Tremaux. L'exploration se termine quand le capteur de ligne GT1140 detecte la case d'arrivee (sol noir).

### Run 2 — Resolution (BFS + trajectoire cloche)
A partir de la carte memorisee, l'algorithme BFS calcule le chemin optimal. Le robot execute ce chemin a vitesse maximale avec des virages en trajectoire continue (profil en cloche, sans arret).

## Architecture hardware

| Composant | Role |
|-----------|------|
| ESP8266 NodeMCU 1.0 | Microcontroleur principal |
| DRV8833 | Driver moteurs DC |
| 2x Moteurs DC + encodeurs 420 ticks/tour | Propulsion + odometrie |
| 4x VL53L0X (ToF) | Detection murs (2 frontaux, 2 lateraux a 45 deg) |
| MPU6050 | Gyroscope (verification rotation) |
| MCP23017 | Expandeur GPIO (XSHUT, LEDs, GT1140) |
| GT1140 | Capteur de ligne (detection case arrivee) |
| 3x LEDs (R/J/V) | Indication d'etat |

## Architecture logicielle

```
src/
  main.cpp          <- Machine d'etats principale
  config.h          <- Toutes les constantes
  mcp_io.h/.cpp     <- Wrapper MCP23017
  sensors.h/.cpp    <- VL53L0X + filtre de Kalman
  imu.h/.cpp        <- MPU6050 gyroscope
  encoders.h/.cpp   <- ISR + comptage ticks
  motors.h/.cpp     <- PWM + direction
  pid.h/.cpp        <- Correcteur discret en Z
  maze.h/.cpp       <- Grille 5x5 + memoire murs
  tremaux.h/.cpp    <- Algorithme exploration
  bfs.h/.cpp        <- Algorithme chemin optimal
  navigation.h/.cpp <- Primitives de mouvement
  leds.h/.cpp       <- Patterns LED
  web_ui.h/.cpp     <- IHM web embarquee
  diagnostic.h/.cpp <- Tests hardware au boot
```

## IHM Web embarquee

Le robot cree un point d'acces WiFi. Se connecter a `Robot_Laby_Eq17` (mdp: `icam2026`) puis ouvrir `http://192.168.4.1`.

**Fonctionnalites :**
- Dashboard avec etat du robot et diagnostic hardware
- Capteurs en temps reel (ToF, gyroscope, encodeurs)
- Tuning PID en direct (sliders Kp/Ki/Kd) avec assistant de reglage guide
- Carte du labyrinthe en SVG avec mapping en temps reel
- Boutons Start Run 1 / Start Run 2 / Arret d'urgence

## Asservissement

### PID vitesse moteur (transformee en Z)
Correcteur discret echantillonne a 50Hz. Chaque roue a sa propre boucle PID.
```
H(z) = Kp + Ki*Ts/(1 - z^-1) + Kd*(1 - z^-1)/Ts
```

### PID correction laterale
Centre le robot dans le couloir en utilisant l'erreur entre les ToF lateraux a 45 deg.

### Filtre de Kalman 1D
Applique sur chaque capteur ToF pour lisser les perturbations dues aux poteaux du labyrinthe (~10-15mm de large).

## Phases de developpement

| Phase | Description | Statut |
|-------|-------------|--------|
| 1 | Diagnostic hardware (I2C, capteurs, moteurs) | En cours |
| 2 | IHM web + moteurs + PID moteur | A faire |
| 3 | Navigation case par case + Kalman + auto-calage | A faire |
| 4 | Tremaux + carte live IHM | A faire |
| 5 | BFS + trajectoire cloche Run 2 | A faire |
| 6 | Integration + tuning final | A faire |

## Build & Flash

```bash
# Prerequis : PlatformIO CLI ou VS Code + extension PlatformIO
# Brancher le NodeMCU en USB

# Compiler
pio run

# Flasher
pio run --target upload

# Moniteur serie
pio device monitor -b 115200
```

## Specification technique

Le document de specification complet est disponible dans [`docs/superpowers/specs/2026-04-12-mazebot-software-design.md`](docs/superpowers/specs/2026-04-12-mazebot-software-design.md).

## Licence

Projet academique ICAM Strasbourg Europe — Equipe 17.
