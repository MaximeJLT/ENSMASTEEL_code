# Interface Jetson ⇄ Maman — Protocole UDP

**Pour :** Équipe Maman (moteurs, actionneurs, LiDAR)
**Par :** Équipe Jetson (vision, stratégie, cerveau)
**Date :** avril 2026

---

## 1. Vue d'ensemble

La Jetson et la carte Maman communiquent par **UDP** à **10 Hz** (un échange toutes les 100 ms).

```
     Jetson                                 Maman
  (cerveau/vision)                    (moteurs/actionneurs)
        │                                     │
        │  1. world_state + command           │
        ├────────────────────────────────────►│
        │      port 5005                      │
        │                                     │
        │  2. ACK + robot_state               │
        │◄────────────────────────────────────┤
        │      port 5006                      │
        │                                     │
      répété toutes les 100 ms
```

**Principe :**
- La Jetson envoie à chaque cycle **l'état du monde** (positions vues par la caméra) **+ la commande courante**.
- La Maman exécute physiquement la commande et renvoie un **ACK avec l'état réel du robot** (odométrie, bras, etc.).

**Important :** tant que la Jetson n'a pas reçu la confirmation qu'une action est finie, elle **renvoie la même commande à chaque cycle**. C'est normal, c'est pour robustesse réseau. La Maman doit juste ignorer les commandes redondantes.

---

## 2. Configuration réseau

| Rôle | IP | Port écoute |
|---|---|---|
| Jetson | 127.0.0.1 (à adapter en réel) | 5006 |
| Maman | 127.0.0.1 (à adapter en réel) | 5005 |

En local pour tests : tout sur `127.0.0.1`.
En conditions réelles : Maman et Jetson sur IPs distinctes reliées par Ethernet.

---

## 3. Message Jetson → Maman

Format : **JSON encodé en UTF-8**, envoyé en UDP sur le port **5005**.

```json
{
  "type": "world_state",
  "t_ms": 1745500000123,
  "frame_id": 42,
  "mapping_ok": true,
  "world": {
    "robot":    {"x_mm": 1150, "y_mm": 800, "theta_rad": 0.0},
    "opponent": {"x_mm": -500, "y_mm": 300, "theta_rad": 0.0},
    "caisses": [
      {"id": 101, "x_mm": 1150, "y_mm": 200, "status": "on_ground"},
      {"id": 102, "x_mm": 1200, "y_mm": 200, "status": "on_ground"}
    ],
    "zones":     ["table", "our_nest", "pantry_1", "..."],
    "matchtime": 12.5
  },
  "command": {
    "kind":      "GOTO",
    "x_mm":      1150,
    "y_mm":      200
  }
}
```

### Champs

| Champ | Type | Description |
|---|---|---|
| `type` | string | Toujours `"world_state"` |
| `frame_id` | int | Compteur incrémenté à chaque cycle, **à renvoyer dans l'ACK** |
| `t_ms` | int | Timestamp Jetson en millisecondes |
| `mapping_ok` | bool | `true` si la vision a bien calibré la table (les 4 ArUco coins sont vus) |
| `world` | objet | Vue de la Jetson sur le monde. Peut être null pour certains champs si pas vu. |
| `command` | objet ou null | Commande à exécuter. **Peut être null** si la Jetson n'a rien à ordonner (ex: plan fini, robot en attente). |

---

## 4. Types de commandes (`command.kind`)

### 4.1 `GOTO` — se déplacer à une coordonnée
```json
{"kind": "GOTO", "x_mm": 1150, "y_mm": 200}
```
**Action attendue :** déplacer le robot vers (x_mm, y_mm) en tenant compte de l'obstacle éventuel (LiDAR). Pas besoin d'orientation imposée.

**Fin d'action :** quand le robot est à < 30 mm de la cible.

---

### 4.2 `PICKUP` — ramasser une caisse
```json
{"kind": "PICKUP", "crate_id": 101}
```
**Action attendue :** déployer le bras, refermer la pince sur la caisse la plus proche (dans le cône de saisie du bras), la charger dans le robot, refermer.

**Note :** `crate_id` est indicatif. La Jetson identifie une caisse dans sa vue ; c'est le bras de la Maman qui prend réellement ce qu'il trouve à portée. Pas besoin de vérifier que l'ID correspond.

**Fin d'action :** quand la séquence bras+pince est terminée (peu importe si une caisse a été effectivement saisie).

---

### 4.3 `DROP_ALL` — déposer toutes les caisses
```json
{"kind": "DROP_ALL", "zone_name": "pantry_1"}
```
**Action attendue :** vider toutes les caisses transportées dans la zone nommée (déposer sur place à la position actuelle du robot).

**Note :** `zone_name` est informatif pour le log. Le robot est déjà positionné dans la zone par un GOTO précédent.

**Fin d'action :** quand toutes les caisses ont été relâchées.

---

### 4.4 `MOVE_CURSOR` — tirer le curseur thermomètre
```json
{"kind": "MOVE_CURSOR", "from_x_mm": 1250, "from_y_mm": -950, "x_mm": 800, "y_mm": -950}
```
**Action attendue :** le robot est déjà accroché au curseur à (from_x, from_y). Il doit se déplacer jusqu'à (x_mm, y_mm) en traînant le curseur avec lui, puis se détacher.

**Fin d'action :** quand le robot est à la position finale et détaché du curseur.

---

### 4.5 `STOP` — arrêt d'urgence (optionnel, non utilisé actuellement)
```json
{"kind": "STOP"}
```
**Action attendue :** arrêter tout mouvement immédiatement.

---

## 5. Message Maman → Jetson (ACK)

Format : **JSON encodé en UTF-8**, envoyé en UDP sur le port **5006** (celui de la Jetson).

```json
{
  "type":     "ack",
  "frame_id": 42,
  "t_ms":     1745500000145,
  "robot_state": {
    "x_mm":         1150.0,
    "y_mm":         450.0,
    "theta_rad":    -1.57,
    "action":       "going",
    "action_done":  false,
    "carried":      [],
    "carried_count": 0
  }
}
```

### Champs obligatoires

| Champ | Type | Description |
|---|---|---|
| `type` | string | Toujours `"ack"` |
| `frame_id` | int | **Doit correspondre au frame_id du message Jetson reçu** (anti-désync) |
| `t_ms` | int | Timestamp Maman |
| `robot_state` | objet | Voir ci-dessous |

### `robot_state` — État du robot renvoyé par la Maman

| Champ | Type | Description | Critique ? |
|---|---|---|---|
| `x_mm` | float | Position X courante (odométrie Maman) | Utile mais optionnel (la Jetson a aussi la vision) |
| `y_mm` | float | Position Y courante | Idem |
| `theta_rad` | float | Orientation courante | Idem |
| `action` | string | État actuel : `"idle"` \| `"going"` \| `"picking"` \| `"dropping"` \| `"dragging"` | Informatif |
| **`action_done`** | **bool** | **`true` quand l'action courante vient de se terminer** | **⚠️ CRITIQUE** |
| `carried` | liste[int] | IDs des caisses portées (si tracké) | Optionnel |
| `carried_count` | int | Nombre de caisses portées | Utile |

---

## 6. ⚠️ Le champ `action_done` — le plus important

**C'est le signal que la Jetson attend pour passer à l'étape suivante de la stratégie.**

### Comportement attendu

```
Temps  Commande reçue        État Maman              action_done
─────  ──────────────────    ──────────────────      ───────────
t=0s   PICKUP crate 101      idle → picking          false
t=0.5s PICKUP crate 101      picking (bras descend)  false
t=1.0s PICKUP crate 101      picking (pince ferme)   false
t=1.2s PICKUP crate 101      picking → idle          TRUE   ← 1 cycle
t=1.3s (new) GOTO 400,-700   idle → going            false
t=1.4s GOTO 400,-700         going                   false
...
t=2.7s GOTO 400,-700         going → idle (arrivé)   TRUE   ← 1 cycle
```

**Règle :** `action_done` doit passer à `true` **pendant exactement 1 cycle** (soit 1 ACK) au moment où l'action vient de se terminer, puis repasser à `false`. C'est un "flag pulse", pas un état continu.

### Comment l'implémenter (suggestion)

```python
# Côté Maman, pseudo-code
if robot.action_state == "picking" and bras_a_termine():
    robot.action_state = "idle"
    robot.action_done_flag = True   # sera renvoyé dans le prochain ACK

# Quand on construit l'ACK :
ack["robot_state"]["action_done"] = robot.action_done_flag
robot.action_done_flag = False   # reset après envoi (pulse 1 cycle)
```

### Pour quelles actions faut-il signaler `action_done` ?

| Action | `action_done` nécessaire ? |
|---|---|
| `GOTO` | Utile mais pas critique (la Jetson vérifie aussi par vision) |
| `PICKUP` | **Oui, critique** — la Jetson n'a aucun autre moyen de savoir |
| `DROP_ALL` | **Oui, critique** — idem |
| `MOVE_CURSOR` | **Oui, critique** — idem |

---

## 7. Cas limites et erreurs

- **Paquet UDP perdu** : la Jetson réémet la commande au cycle suivant (dans 100 ms). Pas besoin de retransmission côté Maman.
- **Commande null** : si `command == null`, la Maman n'a rien à faire. Doit quand même renvoyer un ACK avec l'état courant.
- **Commande inconnue** : la Maman doit logger et renvoyer un ACK normal (ne pas crasher).
- **ACK avec `frame_id` inconnu** : la Jetson l'ignore. La Maman doit toujours renvoyer le `frame_id` reçu dans le message.

---

## 8. Checklist de branchement

Avant les tests en conditions réelles (semaine du 1er mai) :

- [ ] Les IPs réelles de la Jetson et de la Maman sont configurées (remplacent `127.0.0.1`)
- [ ] La Maman écoute bien sur le port `5005`
- [ ] La Maman envoie ses ACKs sur le port `5006`
- [ ] Le champ `frame_id` est bien renvoyé à l'identique dans l'ACK
- [ ] Le champ `action_done` est bien géré avec le comportement "pulse 1 cycle"
- [ ] Les 4 types de commandes (`GOTO`, `PICKUP`, `DROP_ALL`, `MOVE_CURSOR`) sont exécutés correctement
- [ ] La Maman ne crash pas sur des commandes redondantes (la Jetson réémet 10×/s)
- [ ] Le LiDAR gère l'évitement d'obstacles rapprochés côté Maman (la Jetson gère l'évitement longue portée via vision)

---

## 9. Outil de test : `maman_fictive.py`

La Jetson a développé un simulateur de Maman (`maman_fictive.py`) qui implémente ce protocole. Vous pouvez vous en inspirer pour vérifier que votre vraie Maman respecte le contrat : lancer la Jetson contre le simulateur permet de voir le comportement attendu.

Pour tester votre Maman sans la Jetson réelle, vous pouvez simuler les messages entrants à la main (voir le format section 3) et vérifier que vos ACKs respectent le format section 5.

---
