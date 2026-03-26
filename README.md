# État du projet — Eurobot 2026 "Winter is Coming"
## Document de consolidation · Vision / Stratégie / Communication

---

## 1. Vue d'ensemble

Le système est une **caméra overhead Jetson** qui voit toute la table, localise les objets via ArUco, et pilote un robot ("maman") via UDP. Le simulateur (`main_simu.py`) sert à valider la stratégie de jeu avant déploiement réel.

```
Caméra overhead
      │  ArUco DICT_4X4_50
      ▼
vision_aruco.py  ──────────────────────────────────────────────────┐
      │  objects[], table_markers{}                                  │
      ▼                                                              │
mapping.py (TableMapper)                                            │
      │  homographie pixel → mm (4 coins ArUco fixes)               │
      ▼                                                              │
world_updater.py  →  WorldState  (world_state.py)                   │
                          │                                          │
                   JetsonStrategyRunner  (jetson_strategy_v2.py)    │
                          │  Command                                 │
                          ▼                                          │
                   json_main_v3.py  ──── UDP 10Hz ──►  Maman        │
                          ◄──────── ACK + robot_state ──────────────┘
```

Le simulateur (`main_simu.py`) est un **miroir indépendant** du même système :

```
sim_core.py (WorldState sim, SimEngine physique)
      │
strategy_runner.py  →  Command
      │
sim_render.py (matplotlib)
```

---

## 2. Fichiers du projet — rôle de chacun

### Pipeline production (Jetson)

| Fichier | Rôle |
|---------|------|
| `vision_aruco.py` | Capture caméra, détection ArUco, retourne `objects` + `table_markers` |
| `mapping.py` | `TableMapper` : homographie 4 coins → pixel→mm, `pixel_to_world()` |
| `world_state.py` | Classes `Robot`, `Caisse`, `Opponent`, `Zone`, `WorldState` (conteneurs d'état) |
| `world_init.py` | Charge `zones.json` et initialise le `WorldState` |
| `world_updater.py` | Classe les détections en robot/adversaire/caisse, remplit le `WorldState` |
| `json_main.py` | **Boucle principale** — vision→world→stratégie→UDP avec `command` |
| `jetson_strategy.py` | `JetsonStrategyRunner` — exécute `strategy_plan` de `scenario.json`, machine à états, attente `action_done` maman |
| `maman_fictive.py` | **Simulateur maman** — exécute vraiment GOTO/PICKUP/DROP, renvoie `robot_state` |

### Simulateur (validation stratégie)

| Fichier | Rôle |
|---------|------|
| `main_simu.py` | Boucle principale simulateur (20 Hz) |
| `sim_core.py` | `WorldState` sim, `SimEngine` physique (holonome 500mm/s), scoring Eurobot |
| `sim_render.py` | Rendu matplotlib : table, zones, robot, caisses, score en temps réel |
| `strategy_runner.py` | `StrategyRunner` sim — même logique que `jetson_strategy.py` |

### Configuration

| Fichier | Contenu |
|---------|---------|
| `zones.json` | 8 garde-mangers, nid, table (dimensions mm, origine centre table) |
| `scenario.json` | Position départ robot, caisses, `strategy_plan` séquentiel |

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
| Nid jaune (adversaire) | ≈ -1325 → -975 | 625 → 975 |
| Garde-mangers 1–8 | voir zones.json | voir zones.json |
| Curseur départ | 1250 | −1000 (bord table) |

---

## 4. IDs ArUco — état actuel

Dictionnaire utilisé : **DICT_4X4_50** (OpenCV)

| Usage | ID(s) | Statut |
|-------|-------|--------|
| Coin TL table | 21 | ✅ défini dans `vision_aruco.py` |
| Coin TR table | 23 | ✅ défini |
| Coin BR table | 22 | ✅ défini |
| Coin BL table | 20 | ✅ défini |
| Caisse bleue | 36 (règlement officiel) | ✅ règlement p.8 |
| Caisse jaune | 47 (règlement officiel) | ✅ règlement p.8 |
| Caisse vide | 41 (règlement officiel) | ✅ règlement p.8 |
| **Notre robot** | **❌ À DÉFINIR** | À renseigner dans `world_updater.py` → `ROBOT_IDS` |
| **Adversaire** | **❌ À DÉFINIR** | À renseigner dans `world_updater.py` → `OPPONENT_IDS` |

> **Action requise** : choisir un ID ArUco pour notre robot (ex: 10) et mettre à jour :
> ```python
> # world_updater.py
> ROBOT_IDS    = {10}   # ← remplacer None par l'ID choisi
> OPPONENT_IDS = {11}   # ← si on veut tracker l'adversaire
> ```

---

## 5. Protocole UDP Jetson ↔ Maman

### Jetson → Maman (port 5005, 10 Hz)

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
    "zones":    ["our_nest", "pantry_1", ...],
    "matchtime": 12.4
  },
  "command": {
    "kind": "GOTO",
    "x_mm": 1150.0,
    "y_mm": -600.0
  }
}
```

**Types de `command.kind`** :

| Kind | Champs supplémentaires | Description |
|------|----------------------|-------------|
| `GOTO` | `x_mm`, `y_mm` | Naviguer vers ce point |
| `PICKUP` | `crate_id` | Fermer les bras (robot déjà en position) |
| `DROP_ALL` | `zone_name` | Ouvrir les bras (robot déjà en zone) |
| `MOVE_CURSOR` | `x_mm`, `y_mm`, `from_x_mm`, `from_y_mm` | Tirer le curseur thermomètre |
| `STOP` | — | Arrêt immédiat |
| `null` | — | Rien à faire ce cycle |

### Maman → Jetson (port 5006, ACK)

```json
{
  "type":     "ack",
  "frame_id": 42,
  "t_ms":     1234567891,
  "robot_state": {
    "x_mm":       1148.0,
    "y_mm":       198.0,
    "theta_rad":  -1.57,
    "action":     "going",
    "action_done": false,
    "carried_count": 0
  }
}
```

> **`action_done: true`** est le signal clé que la Jetson attend pour passer à l'étape suivante du plan. Sans ce signal, le `JetsonStrategyRunner` a un timeout de secours (GOTO: 30s, PICKUP: 3s, DROP: 2.5s).

---

## 6. `JetsonStrategyRunner` — machine à états

```
            ┌─────────────────────────────────────────────┐
            │                                             │
    IDLE ──►  _next_step()                                │
            │     GOTO      → WAITING_GOTO                │
            │     PICKUP    → WAITING_PICKUP              │
            │     DROP_ALL  → WAITING_DROP                │
            │     CURSOR    → WAITING_CURSOR              │
            └─────────────────────────────────────────────┘
                    │ action_done OU arrivée vision OU timeout
                    ▼
                  IDLE → étape suivante
```

**Confirmation d'arrivée GOTO** (double source, première à confirmer l'emporte) :
1. `robot_state.action_done == true` (odométrie maman)
2. Distance vision < 60 mm de la cible (ArUco robot visible)

---

## 7. Paramètres à ajuster avant test réel

| Paramètre | Fichier | Valeur actuelle | À vérifier |
|-----------|---------|-----------------|------------|
| `ROBOT_IDS` | `world_updater.py` | `{None}` | **ID ArUco à définir** |
| `OPPONENT_IDS` | `world_updater.py` | `{None}` | ID ArUco adversaire |
| `ARRIVAL_TOL` | `jetson_strategy_v2.py` | 60 mm | Distance arrivée GOTO |
| `PICKUP_RANGE` | `jetson_strategy_v2.py` | 150 mm | Portée bras |
| `PICKUP_TIMEOUT_S` | `jetson_strategy_v2.py` | 3.0 s | Durée fermeture bras |
| `DROP_TIMEOUT_S` | `jetson_strategy_v2.py` | 2.5 s | Durée ouverture bras |
| `GOTO_TIMEOUT_S` | `jetson_strategy_v2.py` | 30.0 s | Timeout GOTO max |
| `world_corners_mm` | `mapping.py` | ±1500 / ±1000 | Dimensions table (à confirmer si décalage) |
| `USE_CAMERA` | `vision_aruco.py` | `True` | Passer à `False` pour tests sans caméra |
| `MAMAN_IP` | `json_main_v3.py` | `127.0.0.1` | IP réelle de maman sur le réseau |

---

## 8. Questions ouvertes — décisions à prendre

### Priorité haute (bloque le test réel)

- [ ] **ID ArUco robot** : choisir un ID (≠ 20/21/22/23/36/47/41) et coller le marqueur sur le robot
- [ ] **Protocole maman** : est-ce que maman renvoie bien `action_done` dans son ACK ? Partager `protocole_jetson_maman.md` avec les collègues maman
- [ ] **IP maman** : adresse IP de la carte de maman sur le réseau local

### Priorité moyenne

- [ ] **PAMI** : canal de communication séparé (UDP direct Jetson→PAMI) ou relayé par maman ?
- [ ] **Odométrie maman** : dans quel repère maman exprime-t-elle sa position ? (table mm, ou repère local ?) → pour le recalage
- [ ] **Nombre de PAMI** : 1 (grenier seulement) ou plusieurs ? → impacte la complexité du canal commandes

### Priorité basse (phase 2 et 3)

- [ ] **Stratégies alternatives** : tester d'autres ordres dans `strategy_plan` de `scenario.json`
- [ ] **Évitement adversaire** (phase 2 Safety/Avoid) : détecter et contourner `world.opponent`
- [ ] **Optimisation NN** (phase 3) : réseau de neurones sur les paramètres de stratégie

---

## 9. Comment lancer le système

### Test simulation seule (sans robot ni caméra)

```bash
# Dans un terminal
python main_simu.py
# → Lance le simulateur matplotlib avec stratégie complète
```

### Test pipeline Jetson + maman fictive (sans robot réel)

```bash
# Terminal 1 — simulateur maman
python maman_fictive_v2.py

# Terminal 2 — Jetson (mettre USE_CAMERA=False dans vision_aruco.py)
python json_main_v3.py
```

### Production (avec caméra et robot réel)

```bash
# S'assurer que USE_CAMERA=True dans vision_aruco.py
# S'assurer que ROBOT_IDS est renseigné dans world_updater.py
# S'assurer que MAMAN_IP pointe vers la carte de maman

# Terminal 1 — démarrer maman (côté collègues)
# Terminal 2 — Jetson
python json_main_v3.py
```

---

## 10. Roadmap des phases

```
Phase 1 — Stratégie déterministe (EN COURS)
├── ✅ Pipeline vision → WorldState
├── ✅ Simulateur complet (sim_core + render + strategy_runner)
├── ✅ JetsonStrategyRunner (plan fixe scenario.json)
├── ✅ Protocole UDP Jetson ↔ Maman défini
├── ✅ maman_fictive_v2 (test en boucle fermée)
├── ⏳ ID ArUco robot à définir
├── ⏳ Test sur robot réel
└── ⏳ Optimisation du strategy_plan (score maximal)

Phase 2 — Safety / Avoid
├── ○ Détection adversaire (OPPONENT_IDS à renseigner)
├── ○ Modification dynamique du plan si adversaire en chemin
└── ○ Comportement défensif (vol de garde-mangers)

Phase 3 — Optimisation NN
├── ○ Définir les paramètres d'entrée (positions caisses, temps restant, score)
├── ○ Définir la fonction de récompense (score Eurobot)
├── ○ Entraînement sur le simulateur (sim_core comme environnement)
└── ○ Déploiement côté JetsonStrategyRunner
```
