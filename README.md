# MAZEBOT T17 — Robot Labyrinthe 5×5

**Équipe 17 | ICAM Strasbourg Europe | 2026**

Robot autonome résolvant un labyrinthe **5×5** (cases 20×20 cm) en deux passes, piloté par une interface web embarquée via WiFi.

---

## Fonctionnement

### Run 1 — Exploration (Trémaux)
Le robot explore le labyrinthe case par case à vitesse réduite (~35% PWM). À chaque case, il scanne les murs avec 4 capteurs ToF VL53L0X, met à jour sa carte interne et choisit la direction suivante selon l'algorithme de Trémaux.

### Run 2 — Résolution (BFS)
À partir de la carte mémorisée, l'algorithme BFS calcule le chemin optimal. Le robot exécute ce chemin à vitesse maximale (~90% PWM).

---

## Architecture hardware

| Composant | Rôle |
|-----------|------|
| ESP8266 NodeMCU 1.0 | Microcontrôleur principal |
| DRV8833 | Driver moteurs DC |
| 2× Moteurs DC + encodeurs | Propulsion + odométrie |
| 4× VL53L0X (ToF) | Détection murs (2 frontaux, 2 latéraux à 45°) |
| MPU6050 | Gyroscope (vérification rotation) |
| MCP23017 | Expandeur GPIO (XSHUT VL53L0X, LEDs) |
| 3× LEDs (R/J/V) | Indication d'état |

---

## Architecture logicielle

```
src/
  main.cpp            <- Machine d'états principale
  config.h            <- Toutes les constantes (pinout, PID, paramètres)
  motors.h/.cpp       <- PWM + direction + freinage
  pid.h/.cpp          <- Correcteurs PID discrets en Z (encodeurs + ToF + pivot)
  navigation.h/.cpp   <- Primitives de mouvement (avance, pivot 90°, virage 45°)
  wall_follower.h/.cpp<- Suivi de mur latéral ToF
  maze.h/.cpp         <- Grille 5×5 + mémoire des murs (bitmask)
  tremaux.h/.cpp      <- Algorithme Trémaux (Run 1)
  imu.h/.cpp          <- MPU6050 gyroscope
  calibration.h/.cpp  <- Calibration capteurs + PID persistée en LittleFS
  web_ui.h/.cpp       <- IHM web embarquée (carte SVG, tuning PID, arrêt urgence)
```

---

## IHM Web embarquée

Le robot crée un point d'accès WiFi.  
Se connecter à **`Robot_Laby_Eq17`** (mdp : `icam2026`) puis ouvrir **`http://192.168.4.1`**.

- Carte du labyrinthe en SVG avec mapping en temps réel
- Capteurs en temps réel (ToF, gyroscope, encodeurs)
- Tuning PID en direct (Kp/Ki/Kd encodeurs, ToF, pivot)
- Calibration des seuils ToF (mur, ouverture, poteau)
- Boutons Start Run 1 / Start Run 2 / Arrêt d'urgence

---

## Asservissement

### PID vitesse moteur (transformée en Z)
Correcteur discret échantillonné à 50 Hz. Chaque roue a sa propre boucle PID.

### PID correction latérale (ToF)
Centre le robot dans le couloir via l'erreur entre les capteurs ToF latéraux à 45°.

### PID pivot
Contrôle la précision des rotations 90° à partir des encodeurs.

### Filtre de Kalman 1D
Appliqué sur chaque capteur ToF pour lisser les perturbations dues aux poteaux (~10–15 mm).

---

## État d'avancement (soutenance — avril 2026)

| Phase | Description | Statut |
|-------|-------------|--------|
| 1 | I2C + MCP23017 + VL53L0X (adressage XSHUT dynamique) | ✅ Fait |
| 2 | MPU6050 — lecture cap + détection rotation 90° | ✅ Fait |
| 3 | Encodeurs + moteurs + avance en ligne droite | ✅ Fait |
| 4 | PID latéral ToF + PID pivot 90° | ✅ Fait |
| 5 | Trémaux sur labyrinthe physique | 🔄 En cours |
| 6 | BFS + exécution chemin Run 2 | 🔜 À faire |
| 7 | IHM WiFi — carte SVG + calibration + arrêt urgence | ✅ Fait |
| 8 | Intégration complète + tuning | 🔜 À faire |

---

## Build & Flash

```bash
# Prérequis : PlatformIO CLI ou VS Code + extension PlatformIO

# Compiler
pio run

# Flasher
pio run --target upload

# Moniteur série
pio device monitor -b 115200
```

---

## Licence

Projet académique — ICAM Strasbourg Europe — Équipe 17.
