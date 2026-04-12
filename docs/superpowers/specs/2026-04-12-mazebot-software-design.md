# MAZEBOT T17 — Software Design Specification
> Equipe 17 | ICAM Strasbourg Europe | 2026-04-12

## 1. Vue d'ensemble

Robot autonome résolvant un labyrinthe 5x5 (cases 20x20 cm) en deux passes :
- **Run 1 (Exploration)** : algorithme de Tremaux, vitesse reduite (~35% PWM), cartographie des murs
- **Run 2 (Resolution)** : algorithme BFS sur carte memorisee, trajectoire en cloche sans arret, vitesse max (~90% PWM)

### Contraintes
- **Deadline competition** : ~28 avril 2026
- **Soutenance** : ~3 mai 2026
- **MCU** : ESP8266 NodeMCU 1.0 (ESP-12E), ~80KB RAM libre
- **Framework** : Arduino via PlatformIO, C++17
- **Zero `delay()`** dans la boucle principale, toujours `millis()`
- **Zero magic number**, tout dans `config.h`
- **Zero allocation dynamique** dans les boucles

### Approche choisie : "IHM tot"
L'IHM web est montee des la phase 2 pour servir d'outil de calibration en temps reel (tuning PID, seuils ToF, Kalman). Cela evite les cycles recompilation/upload et accelere le developpement.

---

## 2. Hardware (existant, cable, non teste logiciellement)

### Pinout ESP8266

| Pin GPIO | Pin Board | Fonction |
|----------|-----------|----------|
| 5        | D1        | I2C SCL |
| 4        | D2        | I2C SDA |
| 0        | D3        | Moteur Gauche IN1 (pull-up 10k) |
| 2        | D4        | Moteur Gauche IN2 (pull-up 10k) |
| 13       | D7        | Moteur Droit IN1 |
| 15       | D8        | Moteur Droit IN2 (pull-down 10k) |
| 14       | D5        | Encodeur Gauche Phase A (interruption) |
| 16       | D0        | Encodeur Gauche Phase B |
| 12       | D6        | Encodeur Droit Phase A (interruption) |
| A0       | A0        | Encodeur Droit Phase B (analogique, seuil 512) |

### Bus I2C

| Adresse | Composant |
|---------|-----------|
| 0x20    | MCP23017 (expandeur GPIO) |
| 0x68    | MPU6050 (gyroscope/accelerometre) |
| 0x30    | VL53L0X Front Left (assigne au boot) |
| 0x31    | VL53L0X Front Right (assigne au boot) |
| 0x32    | VL53L0X Side Left 45 deg (assigne au boot) |
| 0x33    | VL53L0X Side Right 45 deg (assigne au boot) |

### MCP23017 — Attribution des ports

| Port | Fonction |
|------|----------|
| GPA0 | XSHUT VL53L0X Front Left |
| GPA1 | XSHUT VL53L0X Front Right |
| GPA2 | XSHUT VL53L0X Side Left 45 deg |
| GPA3 | XSHUT VL53L0X Side Right 45 deg |
| GPA4 | GT1140 Capteur de ligne (detection case arrivee) |
| GPB4 | LED Rouge (erreur/urgence) |
| GPB5 | LED Jaune (Run 1 exploration) |
| GPB6 | LED Verte (Run 2 / succes / idle) |

### Moteurs & Encodeurs

- Moteurs DC avec reducteur 30:1
- Encodeurs : 420 ticks/tour en sortie de reducteur
- Roues : diametre 43mm, largeur 9mm
- Perimetre roue : ~135.1mm
- Resolution : ~3.11 ticks/mm
- Distance par case : ~622 ticks pour 200mm
- Alimentation : 6V typique (via Li-ion 7.4V + regulateur)
- Vitesse en charge : 300 tr/min
- Robot : 15 x 14 cm

### Capteurs ToF VL53L0X — Geometrie de montage

- **Front Left / Front Right** : regardent droit devant, recules de 3.5cm par rapport a l'avant du robot (permet auto-calage mur frontal)
- **Side Left / Side Right** : a 45 deg par rapport a l'axe du robot, a 1.5cm de l'avant

### Capteur de ligne GT1140

- Monte sur GPA4 du MCP23017
- Sortie digitale (seuil regle par potentiometre)
- Detecte la case d'arrivee (sol noir)

---

## 3. Architecture logicielle

### Machine d'etats

```
IDLE -> DIAGNOSTIC -> CALIBRATION -> RUN1_TREMAUX -> MAZE_COMPLETE -> RUN2_BFS -> FINISHED
                                         |                              |
                                     EMERGENCY  <--- (depuis n'importe quel etat via IHM)
```

Etats :
- **IDLE** : LED verte fixe, attente commande depuis IHM
- **DIAGNOSTIC** : validation hardware automatique au boot
- **CALIBRATION** : routines de calibration (PID, ToF, ligne)
- **RUN1_TREMAUX** : LED jaune clignotante, exploration case par case
- **MAZE_COMPLETE** : LED verte + jaune, carte complete, pret Run 2
- **RUN2_BFS** : LED verte clignotante, resolution trajectoire en cloche
- **FINISHED** : LED verte fixe, robot a l'arrivee
- **EMERGENCY** : LED rouge fixe, arret immediat

### Modules (fichiers source)

| Module | Role | Dependances |
|--------|------|-------------|
| `config.h` | Toutes les constantes, pinout, params PID, seuils | Aucune |
| `mcp_io.h/.cpp` | Init MCP23017, wrapper read/write pour XSHUT, LEDs, GT1140 | Wire |
| `sensors.h/.cpp` | Init VL53L0X (adressage XSHUT), lecture distances, filtre Kalman | mcp_io |
| `imu.h/.cpp` | MPU6050, lecture angle yaw, detection rotation | Wire |
| `encoders.h/.cpp` | ISR ticks, calcul vitesse (ticks/s), distance parcourue (mm) | config |
| `motors.h/.cpp` | PWM, direction, freinage, rampes acceleration | config |
| `pid.h/.cpp` | Correcteur discret (Z) : PID vitesse moteur + PID correction laterale ToF | encoders, sensors |
| `maze.h/.cpp` | Grille 5x5, struct Cell, memoire murs (bitmask), position/orientation | config |
| `tremaux.h/.cpp` | Algo Tremaux, marquage passages, decision intersection | maze, sensors |
| `bfs.h/.cpp` | BFS sur carte memorisee, reconstruction chemin optimal | maze |
| `navigation.h/.cpp` | Execution mouvements : avancer 1 case, tourner 90 deg, auto-calage, trajectoire cloche | motors, pid, encoders, imu, sensors |
| `leds.h/.cpp` | Patterns LED via MCP23017 (clignotement, etat) | mcp_io |
| `web_ui.h/.cpp` | Serveur async, page HTML/CSS/JS embarquee, API JSON, tuning PID live | tous |
| `diagnostic.h/.cpp` | Routines de test : scan I2C, test chaque capteur, test moteurs | tous |
| `main.cpp` | Setup, loop, machine d'etats, dispatch | tous |

---

## 4. Phase 1 — Diagnostic hardware

Au premier boot, le robot execute un diagnostic automatique validant chaque composant.

### Sequence de diagnostic

```
1. Scanner I2C -> lister toutes les adresses trouvees
   Attendu : 0x20 (MCP23017), 0x68 (MPU6050)

2. Test MCP23017 -> allumer chaque LED sequentiellement (R, J, V)
   Validation visuelle

3. Init VL53L0X via XSHUT (un par un)
   - Tous XSHUT LOW via MCP23017
   - Activer XSHUT_1 -> assigner 0x30 -> lire distance -> OK/FAIL
   - Activer XSHUT_2 -> assigner 0x31 -> lire distance -> OK/FAIL
   - Activer XSHUT_3 -> assigner 0x32 -> lire distance -> OK/FAIL
   - Activer XSHUT_4 -> assigner 0x33 -> lire distance -> OK/FAIL
   Chaque capteur retourne une distance plausible (20-2000mm)

4. Test MPU6050 -> lire gyroscope, verifier pas de valeurs figees
   Valeurs qui bougent quand on bouge le robot

5. Test GT1140 -> lire etat digital via MCP23017 GPA4
   Verifier changement d'etat quand on passe sur surface noire

6. Test moteurs -> chaque moteur tourne 0.5s avant, 0.5s arriere, PWM faible
   Les encodeurs comptent des ticks dans les deux sens

7. Resume -> Serial affiche tableau OK/FAIL pour chaque composant
```

### Sortie Serial type

```
=== DIAGNOSTIC Robot Eq17 ===
[I2C]  Scan: 0x20 OK | 0x68 OK
[MCP]  LEDs: R=OK J=OK V=OK
[TOF]  FL(0x30): 152mm OK
[TOF]  FR(0x31): 148mm OK
[TOF]  SL(0x32): 201mm OK
[TOF]  SR(0x33): 195mm OK
[IMU]  MPU6050: yaw=0.3 deg/s OK
[LINE] GT1140: LOW (pas de ligne) OK
[MOT]  Left:  FWD=124t BWD=121t OK
[MOT]  Right: FWD=118t BWD=120t OK
=== TOUS SYSTEMES OK ===
```

### Gestion des erreurs
- Composant critique FAIL (VL53L0X, encodeur, moteur) -> etat EMERGENCY, LED rouge, rien ne demarre
- Composant non-critique FAIL (une LED, MPU6050) -> warning Serial, on continue

---

## 5. Phase 2 — IHM Web + Moteurs + PID

### Architecture serveur

- ESPAsyncWebServer en mode Access Point
- SSID : `Robot_Laby_Eq17` / MDP : `icam2026`
- Adresse : `192.168.4.1`
- HTML/CSS/JS stocke en PROGMEM (memoire flash)

### API REST

```
GET  /              -> page HTML complete
GET  /state         -> JSON { state, position, orientation, speeds }
GET  /sensors       -> JSON { tof_fl, tof_fr, tof_sl, tof_sr, yaw, enc_l, enc_r, line }
GET  /maze          -> JSON { cells: [[walls, visits]...], robot: {row,col,dir} }
GET  /pid           -> JSON { kp, ki, kd, kp_lat, ki_lat, kd_lat }
POST /pid           -> body JSON -> met a jour les coefficients PID en RAM
POST /start1        -> lance Run 1
POST /start2        -> lance Run 2
POST /stop          -> arret urgence immediat
POST /calibrate/pid -> lance routine auto-calibration PID moteur
POST /calibrate/tof -> lance calibration seuils ToF
```

### Design IHM — Style Figma dark pro

Theme base sur le prototype existant, ameliore :
- **Palette** : fond #121212, cards #1e1e1e, accent #00ff88, erreur #ff3333
- **Header** : "MAZEBOT T17" + badge etat avec dot anime (vert=idle, jaune=run1, etc.)
- **Cards avec icones SVG inline** : pas de librairie externe
- **Layout responsive** : grid auto-fit, cards empilees en colonne sur mobile
- **Barre de controle fixe en bas** : START RUN1 / START RUN2 / STOP (gros bouton rouge toujours visible)
- **Animations** : transitions 0.2s, blink uniquement en EMERGENCY
- **Police** : 'Segoe UI' / 'Inter' fallback, monospace pour les valeurs numeriques

### 4 panneaux principaux

**1. Dashboard**
- Etat machine avec badge couleur
- Resultat diagnostic (OK/FAIL par composant)
- Boutons Start Run1 / Start Run2 / STOP urgence

**2. Capteurs live**
- Representation schematique du robot vu de dessus
- 4 distances ToF affichees autour avec fleches directionnelles
- Angle yaw MPU6050
- Etat GT1140 (ligne detectee ou non)
- Rafraichissement : 200ms via AJAX

**3. Tuning PID**
- Sliders avec valeur numerique pour Kp, Ki, Kd (moteur + lateral)
- Bouton "Appliquer" sans recompiler
- Graphique Canvas : consigne vs mesure en temps reel
- **Assistant de tuning guide (Ziegler-Nichols simplifie)** :
  - Etape 1 : Ki=0, Kd=0, augmenter Kp jusqu'a oscillation -> indicateur visuel
  - Etape 2 : garder Kp, augmenter Ki pour eliminer erreur statique
  - Etape 3 : ajouter Kd pour amortir oscillations
  - Etape 4 : validation automatique (ligne droite 1m + retour, score affiche)
- Sliders Q et R pour le filtre de Kalman ToF avec courbes brute vs filtree

**4. Carte labyrinthe**
- Grille SVG 5x5
- Etat initial : tous les murs exterieurs affiches en blanc solide, interieur vide
- Murs interieurs apparaissent en fade-in quand le robot les decouvre
- Cases visitees : gradient vert avec compteur de passages (1, 2, 3...)
- Cases non visitees : sombres
- Position robot : surbrillance bleue + fleche orientation
- Chemin BFS (Run 2) : ligne rouge animee de l'entree a la sortie
- Rafraichissement : 300ms via polling /maze

### PID Moteur — Correcteur discret en Z

**Deux boucles PID distinctes :**

**PID vitesse moteur (par roue)** :
```
Entree : consigne vitesse (ticks/s) vs vitesse mesuree (encodeur)
Sortie : PWM applique au moteur
Echantillonnage : 50Hz (20ms)
H(z) = Kp + Ki*Ts/(1 - z^-1) + Kd*(1 - z^-1)/Ts
```

**PID correction laterale (centrage couloir)** :
```
Entree : erreur = distance_gauche - distance_droite (ToF 45 deg)
Sortie : delta vitesse ajoute/soustrait aux consignes des deux roues
```

---

## 6. Phase 3 — Navigation case par case

### 3 primitives de mouvement (Run 1)

**1. `advanceOneCell()` — Avancer de 200mm**
- Reset compteur encodeurs
- Consigne PID moteur : vitesse cible selon PWM_RUN1
- PID lateral actif : correction centrage via ToF 45 deg filtres Kalman
- Condition d'arret : encodeurs atteignent ~622 ticks (200mm)
- Si mur frontal detecte < 50mm -> arret anticipe
- A l'arret : auto-calage si mur devant (FL~=FR et distance cible ~40mm)

**2. `turnLeft()` / `turnRight()` — Rotation 90 deg**
- Freiner, attendre stabilisation (50ms via millis)
- Rotation sur place : une roue avant, l'autre arriere, PWM faible
- **Condition d'arret primaire : encodeurs** (nombre de ticks calcule depuis entraxe roues)
- **Condition secondaire : gyroscope MPU6050** (verification + affichage IHM)
- Si ecart encodeurs vs gyro > 5 deg -> warning IHM
- Tolerance : +/-3 deg, micro-corrections si necessaire

**3. `turnAround()` — Demi-tour 180 deg**
- Meme logique que turnLeft() mais objectif 180 deg
- Utilise par Tremaux en cul-de-sac

### Primitives de mouvement (Run 2) — Trajectoire en cloche

**`advanceWithCurve()` — Virage sans arret**
- En approchant d'un virage, le robot ne s'arrete pas
- Decelere la roue interieure, accelere la roue exterieure
- Profil de vitesse en cloche (courbe en S)
- Debut anticipation : ~30mm avant fin de case
- PID moteur gere le profil de chaque roue independamment
- IMU + encodeurs verifient l'angle parcouru

### Cycle case par case (Run 1)

```
1. SCAN    -> lire 4 ToF filtres + orientation IMU -> deduire murs (N/E/S/W)
2. DECIDE  -> Tremaux choisit la direction
3. ORIENT  -> tourner si necessaire (stop-turn-go)
4. MOVE    -> advanceOneCell()
5. CHECK   -> lire GT1140 : si ligne noire detectee -> FINISHED
6. UPDATE  -> mettre a jour maze[row][col] + position robot
7. RETOUR A 1
```

### Filtre de Kalman 1D sur ToF

```
Par capteur VL53L0X :
- Etat : distance reelle estimee
- Prediction : distance precedente (modele statique)
- Mesure : lecture brute VL53L0X
- R (bruit mesure) : eleve (poteaux creent des pics)
- Q (bruit processus) : faible (mur ne bouge pas)
- Q et R ajustables depuis l'IHM avec visualisation brute vs filtree
```

Objectif : les poteaux (pics brefs) sont lisses, les vraies ouvertures passent.

### Seuils de detection de mur

- **Mur lateral present** : ToF 45 deg ~70-100mm
- **Pas de mur lateral** : ToF 45 deg > 150mm
- **Mur frontal** : ToF front < 120mm
- Seuils calibrables via IHM : bouton "Calibrer ToF" -> robot dans un couloir droit -> moyenne 50 echantillons -> seuils deduits automatiquement

### Auto-calage frontal

```
Quand mur devant :
- Erreur_angle = ToF_FL - ToF_FR
- Si |erreur_angle| > 2mm -> micro-rotation pour aligner
- Puis avancer/reculer pour atteindre distance cible (40mm)
- Corrige la derive accumulee a chaque case
```

---

## 7. Phase 4 — Tremaux + BFS + Carte live

### Structure de donnees maze.h

```cpp
struct Cell {
    uint8_t walls;      // bitmask : N=0x01, E=0x02, S=0x04, W=0x08
    uint8_t visited;    // nombre de fois visitee
    uint8_t marks[4];   // marquage Tremaux par direction (N/E/S/W) : 0, 1 ou 2
};

Cell maze[5][5];        // grille statique, zero allocation dynamique
uint8_t robotRow, robotCol;
uint8_t robotDir;       // 0=N, 1=E, 2=S, 3=W
```

### Algorithme de Tremaux (Run 1)

```
A chaque intersection (apres SCAN des murs) :

1. Marquer l'entree du passage d'ou on vient (+1)
2. Lister les directions possibles (pas de mur)
3. Priorite de choix :
   a) Passage non marque (marks=0) -> explorer du neuf
   b) Passage marque 1 fois -> on peut encore y passer
   c) Jamais un passage marque 2 fois -> dead-end confirme
4. Si toutes les directions sont marquees >=1 -> revenir par le passage marque 1 (backtrack)
5. Marquer la sortie du passage choisi (+1)
```

### Condition de fin Run 1

- GT1140 detecte la surface noire de la case d'arrivee
- OU le robot revient a (0,0) et toutes les cases accessibles sont visitees
- OU toutes les directions depuis la position actuelle sont marquees 2

### Depart et arrivee

- **Depart** : case (0,0), orientation configurable (defaut Nord)
- **Arrivee** : case configurable via IHM (defaut (4,4))
- L'arrivee est materialisee au sol par une surface noire, detectee par le GT1140
- Les murs exterieurs de la grille sont TOUS pleins (initialises au boot)

### Algorithme BFS (Run 2)

```
1. File FIFO statique (tableau 25 elements max, pas de malloc)
2. Depart : case (0,0)
3. Arrivee : case detectee/configuree
4. Explorer voisins sans mur entre eux
5. Stocker parent[] pour reconstruction chemin
6. Backtrack -> liste ordonnee de cases
7. Convertir en sequence de commandes avec anticipation des virages
```

### Execution Run 2

- Vitesse PWM_RUN2 (230 ~= 90%)
- Trajectoire en cloche pour les virages (pas d'arret)
- PID lateral + Kalman toujours actifs
- Deceleration anticipee avant les virages (~30mm)
- Le robot connait les murs -> peut anticiper et optimiser la trajectoire

---

## 8. Bibliotheques (platformio.ini)

```ini
lib_deps =
  pololu/VL53L0X @ ^1.3.1
  adafruit/Adafruit MCP23017 Arduino Library @ ^2.3.2
  electroniccats/MPU6050 @ ^1.3.0
  me-no-dev/ESPAsyncWebServer @ ^1.2.4
  me-no-dev/ESPAsyncTCP @ ^1.2.2
  bblanchon/ArduinoJson @ ^7.0.0

build_flags =
  -D PIO_FRAMEWORK_ARDUINO_LWIP2_LOW_MEMORY
```

---

## 9. Phases de developpement (ordre d'implementation)

| Phase | Objectif | Validation | Duree estimee |
|-------|----------|------------|---------------|
| **1** | Diagnostic hardware : I2C scan, MCP23017, VL53L0X, MPU6050, GT1140, moteurs, encodeurs | Sortie Serial OK/FAIL | Jour 1-2 |
| **2** | IHM web basique + moteurs + PID moteur (transf. Z) | Page web accessible, PID reglable | Jour 3-5 |
| **3** | Navigation : avancer 1 case, tourner 90 deg, auto-calage, Kalman ToF | Trajectoire droite 1m + virage precis | Jour 6-8 |
| **4** | Tremaux + carte live IHM | Exploration complete affichee sur IHM | Jour 9-12 |
| **5** | BFS + trajectoire cloche Run 2 | Chemin optimal execute sans arret | Jour 13-15 |
| **6** | Integration + tuning + polish IHM | Run 1 + Run 2 enchaines, IHM finale | Jour 16-17 |

---

## 10. Points critiques

1. **Boot ESP8266** : D3, D4 doivent etre HIGH et D8 LOW au demarrage (pull-ups/downs cables)
2. **XSHUT VL53L0X** : tous LOW au boot via MCP23017, activer un par un pour adressage
3. **A0 encodeur** : jamais d'attachInterrupt sur A0, lire analogRead(A0) > 512 dans ISR de ENC_R_A
4. **ESPAsyncWebServer** : handlers en contexte d'interruption, utiliser des flags, pas de fonctions bloquantes
5. **DRV8833** : nSLP doit etre HIGH pour activer le driver
6. **GT1140** : calibrer le potentiometre manuellement sur la surface noire avant le Run
7. **Memoire** : HTML en PROGMEM, pas de String dynamiques, tableaux statiques uniquement
8. **Poteaux labyrinthe** : filtre Kalman obligatoire sur ToF pour eviter fausses detections
