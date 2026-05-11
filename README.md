# État du projet — Eurobot 2026 "Winter is Coming"
## Document V1 final · Vision / Stratégie / Communication / Maman

---

## 1. Vue d'ensemble

Le système est composé de **trois couches** :

1. **Jetson** (PC NVIDIA + caméra overhead) : voit la table, calcule la stratégie
2. **Maman** (Raspberry Pi sur le robot) : dispatche les ordres entre Jetson et bas niveau
3. **Cartes esclaves** (3 cartes : moteurs, actionneurs, LiDAR) : exécution physique

```
┌───────────────────────────────────────────────────────────┐
│                       JETSON (PC)                         │
│                                                           │
│   Caméra ─► vision_aruco ─► world_updater ─► WorldState   │
│                                              │            │
│                              JetsonStrategyRunner         │
│                                              │            │
│                                          Command          │
└───────────────────────────────────────────────┬───────────┘
                                                │ UDP/JSON 5005
                                                │ ◄──── 5006 ACK
                              ┌─────────────────▼──────────┐
                              │      MAMAN (Raspberry Pi)  │
                              │           maman.cpp        │
                              │       (dispatcher pur)     │
                              └─┬─────────┬──────────┬─────┘
                                │         │          │
                            UART          UART       UART
                            115200        115200     115200
                                │         │          │
                          ┌─────▼──┐ ┌────▼────┐ ┌───▼────┐
                          │ Carte  │ │ Carte   │ │ Carte  │
                          │moteurs │ │actionn. │ │ LiDAR  │
                          └────────┘ └─────────┘ └────────┘
```

Le simulateur (`main_simu.py`) est un **miroir indépendant** qui rejoue le même système sans hardware ni Jetson :

```
sim_core.py (WorldState sim, SimEngine physique)
      │
strategy_runner.py  →  Command
      │
sim_render.py (matplotlib)
```

---

## 2. Organisation du repo

```
ensmasteel_comms_v1/
│
├── README.md                   ← projet, qui fait quoi, lancement rapide
├── .gitignore
│
├── maman/                      ← TOURNE SUR LA RASPBERRY PI EN MATCH
│   ├── maman.cpp               ← dispatcher UDP↔UART (C++17)
│   └── README.md               ← build + run sur la Pi
│   └── PROTOCOL_UART.md        ← contrat avec collègues cartes esclaves
│
├── jetson/                     ← TOURNE SUR LA JETSON EN MATCH
│   ├── json_main.py            ← entrypoint match (vision→stratégie→UDP)
│   ├── json_strategy.py        ← JetsonStrategyRunner, machine à états
│   ├── vision_aruco.py         ← capture caméra, détection ArUco
│   ├── mapping.py              ← homographie pixel → table mm
│   ├── world_state.py          ← classes Robot/Caisse/Opponent/Zone
│   ├── world_init.py           ← chargement initial depuis zones.json
│   ├── world_updater.py        ← classification détections → WorldState
│   ├── zones.json              ← géométrie des zones (mm)
│   └── scenario.json           ← position départ + strategy_plan
│
│
└── simu/                       ← OUTILS DEV — NE TOURNE PAS EN MATCH
    ├── README.md               ← procédure de test sans hardware
    ├── main_simu.py            ← simulateur pur matplotlib
    ├── sim_core.py             ← WorldState sim, SimEngine, scoring
    ├── sim_render.py           ← rendu matplotlib temps réel
    ├── strategy_runner.py      ← StrategyRunner (miroir de json_strategy)
    ├── maman_fictive.py        ← maman simulée Python (legacy, encore utile)
    ├── Fake_motor_card.py      ← simule la carte moteurs en UART
    ├── Fake_actuator_card.py   ← simule la carte actionneurs
    ├── Fake_lidar_card.py      ← simule la carte LiDAR
    ├── test_maman.py           ← validateur de maman.cpp (sans Jetson)
    ├── genetic.py              ← optimisation stratégie (futur)
    ├── test_individual.py
    └── opponent_scenario*.json ← scénarios adversaire pour la simu
```

> **Principe** : sur le robot et sur la Jetson, on clone le repo et on n'utilise que `maman/` ou `jetson/`. Le dossier `simu/` n'est nécessaire que pour le développement et la validation.

---

## 3. Repère de coordonnées

```
        y+ (1000 mm)
        ↑
        │
-x ─────┼────── x+ (1500 mm)
        │
        ↓
       y- (-1000 mm)

Origine  : centre de la table
Unité    : millimètres
```

| Zone | x_mm | y_mm |
|------|------|------|
| Nid bleu (notre départ) | 975 → 1325 | 625 → 975 |
| Nid jaune (adversaire)  | -1325 → -975 | 625 → 975 |
| Garde-mangers 1–8 | voir `zones.json` | voir `zones.json` |
| Curseur départ | 1250 | -1000 |

---

## 4. IDs ArUco

Dictionnaire OpenCV : **DICT_4X4_50**

| Usage | ID | Statut |
|-------|----|----|
| Coin TL table | 21 | ✅ |
| Coin TR table | 23 | ✅ |
| Coin BR table | 22 | ✅ |
| Coin BL table | 20 | ✅ |
| Caisse bleue  | 36 | ✅ règlement |
| Caisse jaune  | 47 | ✅ règlement |
| Caisse vide   | 41 | ✅ règlement |
| **Notre robot**   | **❌ à définir** | À renseigner dans `world_updater.py → ROBOT_IDS` |
| **Adversaire**    | **❌ à définir** | À renseigner dans `world_updater.py → OPPONENT_IDS` |

---

## 5. Protocole UDP (Jetson ↔ Maman)

### Jetson → Maman, port 5005, à 10 Hz

```json
{
  "type":       "world_state",
  "t_ms":       1234567890,
  "frame_id":   42,
  "mapping_ok": true,
  "world": {
    "robot":    {"x_mm": 1150.0, "y_mm": 200.0, "theta_rad": -1.57},
    "opponent": {"x_mm": -800.0, "y_mm": 300.0, "theta_rad": 0.0},
    "caisses":  [{"id": 36, "x_mm": 1150.0, "y_mm": 200.0, "status": "on_ground"}],
    "zones":    ["our_nest", "pantry_1"],
    "matchtime": 12.4
  },
  "command": {
    "kind": "GOTO",
    "x_mm": 1150.0,
    "y_mm": -600.0
  }
}
```

| `command.kind` | Champs | Description |
|----------------|--------|-------------|
| `GOTO`         | `x_mm`, `y_mm`              | Naviguer vers ce point |
| `PICKUP`       | `crate_id`                  | Saisir une caisse (robot en position) |
| `DROP_ALL`     | `zone_name` (info seulement)| Lâcher toutes les caisses |
| `MOVE_CURSOR`  | `x_mm`, `y_mm`              | Pousser le curseur thermomètre |
| `STOP`         | —                           | Arrêt immédiat |
| `null`         | —                           | Heartbeat sans ordre |

### Maman → Jetson, port 5006, ACK pour chaque message reçu

```json
{
  "type":     "ack",
  "frame_id": 42,
  "t_ms":     1234567891,
  "robot_state": {
    "x_mm":               1148.0,
    "y_mm":               198.0,
    "theta_rad":          -1.57,
    "action":             "going",
    "carried":            [36, 41],
    "obstacle":           false,
    "obstacle_dist_mm":   0.0,
    "obstacle_angle_rad": 0.0
  }
}
```

> **Évolution V1** : `carried` (liste d'IDs) remplace `carried_count`. `action_done` n'est plus envoyé explicitement — la Jetson détecte le passage `going` → `idle` comme signal d'arrivée. Les 3 champs `obstacle*` sont ajoutés pour forwarder les données LiDAR.

---

## 6. Protocole UART (Maman ↔ Cartes esclaves) — NOUVEAU EN V1

### Paramètres communs

| Paramètre | Valeur |
|---|---|
| Baud | 115200 |
| Format | 8N1 |
| Flow control | aucun |
| Encodage | ASCII, lignes terminées par `\n` |

### 6.1 — Carte moteurs (`/dev/ttyUSB0`)

**Maman → Carte :**
- `GOTO <x_mm> <y_mm>\n`
- `STOP\n`
- `STATUS\n` (optionnel)

**Carte → Maman :**
- `POS <x> <y> <theta>\n` — push à 10 Hz minimum
- `DONE\n` — cible GOTO atteinte
- `ERR <message>\n`

### 6.2 — Carte actionneurs (`/dev/ttyUSB1`)

**Maman → Carte :**
- `PICK\n` — fermer les bras
- `DROP\n` — ouvrir les bras
- `STATUS\n`

**Carte → Maman :**
- `STATUS <state>\n` — state ∈ {idle, picking, dropping}
- `DONE\n`
- `ERR <message>\n`

### 6.3 — Carte LiDAR (`/dev/ttyUSB2`)

**Carte → Maman uniquement :**
- `OBST <dist_mm> <angle_rad>\n`
- `CLEAR\n`

> Le détail complet, les exemples et le squelette de code à donner aux collègues sont dans `docs/PROTOCOL_UART.md`.

---

## 7. Architecture interne de maman.cpp

Boucle principale **single-thread** avec `poll()` sur 4 file descriptors :

```cpp
while (true) {
    poll(fds, 4, 50);  // UDP socket + 3 ports série, timeout 50ms

    // 1. Lire toutes les lignes UART en attente
    while (motor_port.read_line(line))    motor.on_line(line);
    while (actuator_port.read_line(line)) actuator.on_line(line);
    while (lidar_port.read_line(line))    lidar.on_line(line);

    // 2. Si paquet UDP de la Jetson :
    //    - parser le JSON
    //    - dispatcher la commande sur la bonne carte UART
    //    - construire l'ACK avec l'état courant
    //    - renvoyer l'ACK
}
```

**Maman ne contient AUCUNE logique métier.** Pas de stratégie, pas de calcul de trajectoire, pas de décision. Elle traduit JSON → texte UART et inversement. Toute l'intelligence reste sur la Jetson, tout l'asservissement reste dans les cartes esclaves.

**Build :**
```bash
sudo apt install nlohmann-json3-dev
cd maman
g++ -std=c++17 -O2 maman.cpp -o maman
```

---

## 8. JetsonStrategyRunner — machine à états

```
            ┌─────────────────────────────────────────────┐
            │                                             │
    IDLE ──►  _next_step()                                │
            │     GOTO        → WAITING_GOTO              │
            │     PICKUP      → WAITING_PICKUP            │
            │     DROP_ALL    → WAITING_DROP              │
            │     MOVE_CURSOR → WAITING_CURSOR            │
            └─────────────────────────────────────────────┘
                    │ action_done OU arrivée vision OU timeout
                    ▼
                  IDLE → étape suivante
```

**Confirmation d'arrivée GOTO** (double source, première à confirmer l'emporte) :
1. Transition `robot_state.action: going → idle` dans l'ACK maman
2. Distance vision < `ARRIVAL_TOL` (60 mm) du target

---

## 9. Paramètres à ajuster avant test réel

| Paramètre | Fichier | Valeur actuelle | À vérifier |
|-----------|---------|-----------------|------------|
| `ROBOT_IDS` | `world_updater.py` | `{None}` | **ID ArUco à définir** |
| `OPPONENT_IDS` | `world_updater.py` | `{None}` | ID ArUco adversaire |
| `ARRIVAL_TOL` | `json_strategy.py` | 60 mm | Distance arrivée GOTO |
| `PICKUP_RANGE` | `json_strategy.py` | 150 mm | Portée bras |
| `PICKUP_TIMEOUT_S` | `json_strategy.py` | 3.0 s | Durée fermeture bras |
| `DROP_TIMEOUT_S` | `json_strategy.py` | 2.5 s | Durée ouverture bras |
| `GOTO_TIMEOUT_S` | `json_strategy.py` | 30.0 s | Timeout GOTO max |
| `world_corners_mm` | `mapping.py` | ±1500 / ±1000 | Dimensions table |
| `USE_CAMERA` | `vision_aruco.py` | `True` | `False` pour tests sans caméra |
| `MAMAN_IP` | `json_main.py` | `127.0.0.1` | IP de la Pi sur le réseau |
| Ports UART (maman) | argv[1..3] | `/dev/ttyUSB0/1/2` | À fixer avec règles udev |

---

## 10. Validation V1 — état au 9 mai 2026

### ✅ Validé

- [x] Pipeline simulation pure (`main_simu.py`) — stratégie complète tournée et scorée
- [x] `maman_fictive.py` — boucle Jetson↔maman simulée en UDP
- [x] **`maman.cpp` compile et tourne sur Linux**
- [x] **`maman.cpp` validée contre 3 fakes Python** via socat — voir test ci-dessous
- [x] Protocole UART documenté dans `docs/PROTOCOL_UART.md`
- [x] Test d'intégration `test_maman.py` validé avec succès :
  - heartbeat → ACK correct
  - GOTO (1000, -500) → mouvement progressif observable dans les ACK
  - PICKUP → `carried` mis à jour
  - DROP_ALL → `action: dropping` puis retour à `idle`
  - STOP → accepté

## 11. Comment lancer le système

### Test simulation pure (sans rien)

```bash
cd simu
python3 main_simu.py
```

### Test maman seule (validation hors hardware)

```bash
# Terminal 1-3 : créer 3 paires de PTY virtuels
socat -d -d pty,raw,echo=0,link=/tmp/tty_motor_master    pty,raw,echo=0,link=/tmp/tty_motor_slave &
socat -d -d pty,raw,echo=0,link=/tmp/tty_actuator_master pty,raw,echo=0,link=/tmp/tty_actuator_slave &
socat -d -d pty,raw,echo=0,link=/tmp/tty_lidar_master    pty,raw,echo=0,link=/tmp/tty_lidar_slave &

# Terminal 4-6 : lancer les fakes
cd simu
python3 Fake_motor_card.py    /tmp/tty_motor_slave
python3 Fake_actuator_card.py /tmp/tty_actuator_slave
python3 Fake_lidar_card.py    /tmp/tty_lidar_slave

# Terminal 7 : maman
cd maman
./maman /tmp/tty_motor_master /tmp/tty_actuator_master /tmp/tty_lidar_master

# Terminal 8 : testeur (simule la Jetson)
cd simu
python3 test_maman.py
```

### Test pipeline Jetson + maman (avec fakes)

Mêmes terminaux 1-7, puis :
```bash
# Terminal 8 : Jetson sans caméra (USE_CAMERA=False)
cd jetson
python3 json_main.py
```

### Production (jour J)

**Sur la Raspberry Pi (maman) :**
```bash
cd maman
./maman /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
```

**Sur la Jetson :**
```bash
cd jetson
python3 json_main.py
```

Deux machines, deux commandes. Aucun fichier de simu n'est nécessaire.

---
