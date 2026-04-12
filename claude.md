# CLAUDE.md — Robot Labyrinthe 5×5 | ICAM Strasbourg Europe | Équipe 17
> Mis à jour : 2026-03-08 | Schéma REV 2.0

## 🎯 Objectif du projet

Robot autonome résolvant un labyrinthe **5×5** (cases de **20×20 cm**) en deux passes :
- **Run 1 — Exploration** : algorithme de **Trémaux** à vitesse réduite (~35% PWM), cartographie complète des murs
- **Run 2 — Résolution** : algorithme **BFS** sur la carte mémorisée, vitesse maximale (~90% PWM)

Le développeur est **débutant/intermédiaire en C++**. Priorité absolue :
- Code **modulaire**, **commenté pédagogiquement**, avec explications inline
- Chaque module testable **indépendamment** avant intégration
- **Zéro `delay()`** dans la boucle principale → toujours `millis()`
- **Zéro magic number** → tout dans `config.h`

---

## 🧠 Plateforme & Environnement

| Élément | Valeur |
|---|---|
| MCU | ESP8266 NodeMCU 1.0 (ESP-12E)|
| Framework | Arduino (via PlatformIO) |
| IDE | VS Code + PlatformIO |
| Langage | C++17 |
| Alimentation | Li-ion 7.4V → LM7805 → 5V → 3.3V (régulateur NodeMCU) |

> ⚠️ Mémoire RAM limitée (~80KB heap libre). Pas d'allocation dynamique dans les boucles.
> Préférer les tableaux statiques et les `struct` simples.

---

## 🔌 Pinout Complet (Master Pinout — DÉFINITIF)

```cpp
// ===================== config.h =====================

// --- I2C Bus (MCP23017 + MPU6050 + VL53L0X via MCP) ---
#define PIN_SCL         5   // D1 — Bus I2C SCL
#define PIN_SDA         4   // D2 — Bus I2C SDA

// --- Moteur GAUCHE (DRV8833 sorties A) ---
#define MOTOR_L_IN1    0   // D3 — AIN1 | Pull-up 10kΩ → boot safe HIGH
#define MOTOR_L_IN2    2   // D4 — AIN2 | Pull-up 10kΩ → boot safe HIGH
// NB: HIGH,HIGH = freinage immédiat au boot → sécurité native

// --- Moteur DROIT (DRV8833 sorties B) ---
#define MOTOR_R_IN1    13  // D7 — BIN1
#define MOTOR_R_IN2    15  // D8 — BIN2 | Pull-down 10kΩ → boot safe LOW

// --- Encodeurs Moteur GAUCHE ---
#define ENC_L_A        14  // D5 — Phase A | Interruption (comptage ticks)
#define ENC_L_B        16  // D0 — Phase B | Sens de rotation

// --- Encodeurs Moteur DROIT ---
#define ENC_R_A        12  // D6 — Phase A | Interruption (comptage ticks)
#define ENC_R_B        A0  // A0 — Phase B | HACK analogique (lire dans ISR de ENC_R_A)
// NB: analogRead(A0) > 512 = HIGH, sinon LOW (tension 3.3V sur ADC 1V max → diviser !)

// --- Adresses I2C ---
#define I2C_MCP23017   0x20  // Expandeur GPIO (A0=A1=A2=GND)
#define I2C_MPU6050    0x68  // Gyroscope (AD0=GND)
// VL53L0X : adresses assignées dynamiquement au boot via XSHUT (voir sensors.cpp)
#define TOF_ADDR_FRONT_L  0x30
#define TOF_ADDR_FRONT_R  0x31
#define TOF_ADDR_SIDE_L   0x32  // 45° gauche
#define TOF_ADDR_SIDE_R   0x33  // 45° droite

// --- Pins XSHUT des VL53L0X sur MCP23017 (ports GPA) ---
#define MCP_XSHUT_1    0  // GPA0 → VL53L0X Front Left
#define MCP_XSHUT_2    1  // GPA1 → VL53L0X Front Right
#define MCP_XSHUT_3    2  // GPA2 → VL53L0X Side Left 45°
#define MCP_XSHUT_4    3  // GPA3 → VL53L0X Side Right 45°

// --- LEDs sur MCP23017 (ports GPB) ---
#define MCP_LED_RED    4  // GPB4 → LED1 Rouge  (erreur / urgence)
#define MCP_LED_YELLOW 5  // GPB5 → LED2 Jaune  (Run 1 exploration)
#define MCP_LED_GREEN  6  // GPB6 → LED3 Verte  (Run 2 / succès / idle)

// --- Paramètres moteurs ---
#define PWM_FREQ_HZ    10000  // 10kHz — évite les sifflements audibles
#define PWM_RUN1       90    // ~35% de 255 — vitesse exploration
#define PWM_RUN2       230   // ~90% de 255 — vitesse résolution

// --- Labyrinthe ---
#define MAZE_SIZE      5    // Grille 5×5
#define CELL_SIZE_MM   200  // 200mm par case

// --- PID Asservissement (à calibrer) ---
#define PID_KP         1.2f
#define PID_KI         0.05f
#define PID_KD         0.8f
#define PID_SAMPLE_MS  20   // Période échantillonnage PID = 50Hz

// --- WiFi AP ---
#define WIFI_SSID      "Robot_Laby_Eq17"
#define WIFI_PASSWORD  "icam2026"
#define WIFI_PORT      80
```

---

## 📦 Bibliothèques choisies (platformio.ini)

```ini
[env:nodemcuv2]
platform = espressif8266
board = nodemcuv2
framework = arduino
monitor_speed = 115200
upload_speed = a definir

lib_deps =
  ; Capteurs ToF VL53L0X — bibliothèque officielle Pololu, légère et fiable
  pololu/VL53L0X @ ^1.3.1

  ; Expandeur MCP23017 — Adafruit, compatible Wire, simple d'usage
  adafruit/Adafruit MCP23017 Arduino Library @ ^2.3.2

  ; Gyroscope MPU6050 — I2Cdevlib, la référence pour cet IMU
  electroniccats/MPU6050 @ ^1.3.0

  ; Serveur web async — non-bloquant, indispensable pour ne pas figer la boucle
  me-no-dev/ESPAsyncWebServer @ ^1.2.4
  me-no-dev/ESPAsyncTCP @ ^1.2.2

  ; JSON léger pour l'IHM (envoi état robot en AJAX)
  bblanchon/ArduinoJson @ ^7.0.0

build_flags =
  -D PIO_FRAMEWORK_ARDUINO_LWIP2_LOW_MEMORY
```

> **Pourquoi ces choix ?**
> - `pololu/VL53L0X` : plus légère qu'Adafruit, parfaite pour ESP8266
> - `ESPAsyncWebServer` : non-bloquant, crucial pour ne pas perturber le PID
> - `ArduinoJson` : envoi de la carte du labyrinthe en JSON vers l'IHM web

---

## 🏗️ Architecture des fichiers

```
/
├── CLAUDE.md               ← CE FICHIER (mémoire du projet)
├── platformio.ini
└── src/
    ├── main.cpp            ← Setup, loop, machine d'états
    ├── config.h            ← TOUTES les constantes (pinout, params)
    ├── maze.h / .cpp       ← Grille 5×5, mémoire des murs, struct Cell
    ├── tremaux.h / .cpp    ← Algorithme Trémaux (Run 1)
    ├── bfs.h / .cpp        ← Algorithme BFS (Run 2)
    ├── motors.h / .cpp     ← PWM, direction, freinage
    ├── pid.h / .cpp        ← Correcteur PID discret (Z) sur encodeurs
    ├── encoders.h / .cpp   ← ISR, comptage ticks, calcul vitesse
    ├── sensors.h / .cpp    ← Init VL53L0X (adressage XSHUT), lecture ToF
    ├── imu.h / .cpp        ← MPU6050, calcul cap, détection rotation 90°
    ├── leds.h / .cpp       ← Contrôle LEDs via MCP23017
    └── web_ui.h / .cpp     ← Serveur async, carte JSON, arrêt urgence
```

---

## 🤖 Algorithmes

### Trémaux (Run 1)
```
- Marquer chaque passage traversé (+1 à chaque traversée)
- Ne jamais revenir dans un passage marqué 2 fois (dead-end confirmé)
- À chaque intersection : priorité aux passages non visités
- Mémoriser les murs dans maze[row][col].walls (bitmask : N=0b0001, E=0b0010, S=0b0100, W=0b1000)
- Vitesse = PWM_RUN1
```

### BFS (Run 2)
```
- File FIFO sur le graphe de la carte mémorisée
- Calculer le chemin optimal entrée→sortie
- Reconstruire le chemin par backtrack de parents[]
- Exécuter les mouvements séquentiellement
- Vitesse = PWM_RUN2
```

---

## ⚙️ Asservissement PID (correcteur discret en Z)

Signal d'erreur = `distance_mur_gauche - distance_mur_droite` (capteurs ToF latéraux)

```
correction = Kp*e[n] + Ki*somme(e) + Kd*(e[n]-e[n-1])
PWM_gauche = PWM_base - correction
PWM_droite = PWM_base + correction
```

- Échantillonnage toutes les `PID_SAMPLE_MS` ms via `millis()`
- Coefficients dans `config.h`, modifiables sans recompiler via IHM web (bonus)
- Phase B encodeur droit sur A0 : lire `analogRead(A0) > 512` dans l'ISR de ENC_R_A

---

## 🌐 IHM WiFi (Access Point)

- SSID : `Robot_Laby_Eq17` / MDP : `icam2026`
- Se connecter depuis PC/mobile → `http://192.168.4.1`
- Routes HTTP :
  - `GET /` → page HTML principale
  - `GET /state` → JSON état robot (position, run, vitesses)
  - `GET /maze` → JSON carte des murs découverts
  - `POST /stop` → **ARRÊT D'URGENCE** (coupe PWM immédiatement)
  - `POST /start1` → Lance Run 1
  - `POST /start2` → Lance Run 2 (si carte complète)
- Mise à jour carte : polling AJAX toutes les 500ms

---

## 🚦 Machine d'états principale

```cpp
enum RobotState {
  STATE_IDLE,           // LED verte fixe — attente commande
  STATE_RUN1_TREMAUX,   // LED jaune clignotante — exploration
  STATE_MAZE_COMPLETE,  // LED verte + jaune — carte ok, prêt Run 2
  STATE_RUN2_BFS,       // LED verte clignotante — résolution
  STATE_FINISHED,       // LED verte fixe 3 bips
  STATE_EMERGENCY       // LED rouge fixe — STOP
};
```

---

## 🔁 Phases de développement

| Phase | Objectif | Validation |
|---|---|---|
| **1** | Config I2C + MCP23017 + VL53L0X (adressage XSHUT) | 4 distances affichées en Serial |
| **2** | MPU6050 — lecture cap et détection rotation 90° | Rotation précise ±5° |
| **3** | Encodeurs + moteurs + avance en ligne droite | Trajectoire droite 1m sans dérive |
| **4** | PID sur erreur ToF latéraux | Correction visible en Serial |
| **5** | Trémaux sur labyrinthe physique | Carte complète affichée en Serial |
| **6** | BFS + exécution chemin | Robot suit le chemin optimal |
| **7** | IHM WiFi — carte + arrêt urgence | Page web fonctionnelle sur mobile |
| **8** | Intégration complète + tuning | Run 1 + Run 2 enchaînés |

---

## ⚠️ Points critiques à ne jamais oublier

1. **Boot ESP8266** : D3, D4 doivent être HIGH et D8 LOW au démarrage → les pull-ups/pull-downs sont déjà câblés, ne pas les court-circuiter dans le code
2. **XSHUT VL53L0X** : au boot, mettre tous les XSHUT à LOW (via MCP23017), puis les activer UN PAR UN pour assigner des adresses I2C différentes avant de tous les activer
3. **A0 encodeur** : jamais d'`attachInterrupt` sur A0 — lire uniquement dans l'ISR de D6 (ENC_R_A)
4. **ESPAsyncWebServer** : les handlers s'exécutent dans un contexte d'interruption — ne pas appeler de fonctions bloquantes dedans, utiliser des flags
5. **DRV8833** : `nSLP` doit être HIGH pour activer le driver — vérifier la connexion dans le schéma
6. **I2C Bus Bottleneck (6 devices on 1 bus)**
   - *Constraint:* The I2C bus is heavily loaded (MCP, MPU, 4x VL53L0X). 
   - *Rule:* You MUST initialize `Wire.setClock(400000);` (Fast Mode) in `setup()` to avoid PID loop starvation. Ensure no I2C read is blocking the main state machine.

7. **ESP8266 Single-Core vs Real-Time Constraints**
   - *Constraint:* The ESP8266 has only one core. AsyncWebServer requests from the Web UI will interrupt the PID loop (which needs strict 50Hz execution) and Encoder ISRs.
   - *Rule:* Web UI polling must be optimized. 
   - *Rule:* For `STATE_RUN2_BFS` (speed run), you must prioritize CPU for navigation. 

8. **Kalman Filter vs ToF Outliers**
   - *Constraint:* VL53L0X sensors occasionally return massive out-of-bounds values (e.g., 8190mm or 65535mm) when hitting a corner or the void.
   - *Rule:* NEVER feed raw outlier values directly into `kalmanUpdate()`. 
   - *Rule:* You MUST implement an outlier rejection logic (e.g., reject physical impossibilities like a delta > 500mm in 50ms) BEFORE passing the measurement to the Kalman filter.