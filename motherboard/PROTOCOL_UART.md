# Protocole UART entre maman et les cartes esclaves

> **Pour : équipe cartes moteurs / actionneurs / LiDAR**
> **De : maman (Raspberry Pi, dispatcher central)**
> **Date : J-3 avant la coupe**

Ce document spécifie **exactement** ce que maman attend de chaque carte esclave. Maman est déjà codée et testable contre des cartes simulées en Python. **Si vos cartes respectent ce protocole, l'intégration est plug-and-play.**

---

## Paramètres communs

| Paramètre | Valeur |
|---|---|
| Baud rate | **115200** |
| Format | **8N1** (8 bits, pas de parité, 1 stop) |
| Flow control | aucun |
| Encodage | ASCII, lignes terminées par `\n` |
| Repère table | mm, origine au centre, théta en rad |

> **Pourquoi ASCII ?** Debugging trivial avec `screen`/`minicom`/`cat`, et la perte de perf est négligeable à 10 Hz.

---

## 1. Carte moteurs (`/dev/ttyUSB0` côté maman)

### Maman → Carte (commandes)

| Commande | Effet |
|---|---|
| `GOTO <x> <y>\n` | Aller à la position (x, y) en mm dans le repère table. **Maman peut renvoyer le même GOTO plusieurs fois** : ignorez si déjà en cours. |
| `STOP\n` | Arrêt immédiat des moteurs. |
| `STATUS\n` | (Optionnel) Demander un push immédiat de la position. |

**Exemples :**
```
GOTO 1150.0 -600.0
STOP
```

### Carte → Maman (états et événements)

| Message | Quand | Effet côté maman |
|---|---|---|
| `POS <x> <y> <theta>\n` | **Push périodique à 10 Hz minimum** | Maman met à jour `robot_state.x_mm/y_mm/theta_rad` envoyé à la Jetson |
| `DONE\n` | Cible GOTO atteinte | Maman passe l'état à `idle` |
| `ERR <message>\n` | Erreur (blocage, etc.) | Maman log l'erreur |

**Exemples :**
```
POS 1148.3 -598.1 1.5707
DONE
ERR motor_left_stalled
```

> ⚠️ **Important :** la position doit être dans le repère **table** (origine au centre, x ∈ [-1500, +1500] mm, y ∈ [-1000, +1000] mm). Si votre odométrie est en repère robot/local, faites la transformation côté carte moteurs ou demandez-moi.

---

## 2. Carte actionneurs (`/dev/ttyUSB1` côté maman)

### Maman → Carte

| Commande | Effet |
|---|---|
| `PICK\n` | Fermer les bras, saisir la caisse devant le robot |
| `DROP\n` | Ouvrir les bras, déposer toutes les caisses |
| `STATUS\n` | Demander l'état courant |

### Carte → Maman

| Message | Quand |
|---|---|
| `STATUS <state>\n` | Sur demande, ou push si change. `state` ∈ `idle`, `picking`, `dropping` |
| `DONE\n` | Action terminée (bras refermés/ouverts, position finale atteinte) |
| `ERR <message>\n` | Erreur (servo bloqué, etc.) |

**Exemples :**
```
STATUS picking
DONE
ERR servo_left_no_response
```

> Les durées ne sont **pas** gérées par maman. Vous envoyez `DONE` quand c'est vraiment fini de votre côté. Maman attendra.

---

## 3. Carte LiDAR (`/dev/ttyUSB2` côté maman)

Cette carte est en **push uniquement** : maman ne demande rien, la carte envoie quand quelque chose change.

### Carte → Maman

| Message | Quand |
|---|---|
| `OBST <dist_mm> <angle_rad>\n` | Obstacle détecté à moins de [seuil] mm. `angle_rad` est l'angle relatif au robot (0 = devant, +pi/2 = gauche). |
| `CLEAR\n` | Plus aucun obstacle proche |

**Exemples :**
```
OBST 320.5 0.18
OBST 280.0 0.05
CLEAR
```

> **Comportement maman :** maman ne fait **rien** elle-même avec ces données. Elle les forwarde à la Jetson dans l'ACK (`obstacle`, `obstacle_dist_mm`, `obstacle_angle_rad`). C'est la stratégie Jetson qui décide d'éviter/freiner.
>
> **Si vous voulez un freinage d'urgence local** (au cas où la Jetson plante), demandez-moi : j'ajoute 5 lignes dans maman pour STOP automatique sous un certain seuil.

---

## Vérification rapide depuis votre côté

Quand vous avez fini votre carte, vous pouvez la tester **sans maman** avec un simple terminal :

```bash
# Sur la Pi ou un PC connecté à la carte
screen /dev/ttyUSB0 115200

# Tapez :
GOTO 1000 0
# Vous devez voir :
# POS 50.2 0.1 0.0
# POS 100.8 0.3 0.0
# ...
# POS 999.5 0.0 0.0
# DONE
```

Si ça marche : c'est bon, je branche maman dessus et on est en intégration.

---

## Récapitulatif des fonctions à exposer côté carte

### Carte moteurs
- [ ] Parser `GOTO x y` → asservissement vers la cible
- [ ] Parser `STOP` → arrêt
- [ ] Push `POS x y theta` toutes les 100 ms
- [ ] Push `DONE` à l'arrivée

### Carte actionneurs
- [ ] Parser `PICK` → fermer les bras
- [ ] Parser `DROP` → ouvrir les bras
- [ ] Push `DONE` quand fini

### Carte LiDAR
- [ ] Push `OBST dist angle` quand obstacle détecté
- [ ] Push `CLEAR` quand zone dégagée

C'est tout. Pas besoin de connaître JSON, UDP, ou la stratégie haut niveau. Vous restez sur du texte ASCII basique.

---

## Si vous avez déjà codé autre chose

Si vos fonctions s'appellent par exemple `move_to_position(x, y)` au lieu de parser `GOTO x y`, c'est trivial à adapter : ajoutez une seule fonction `parse_uart_line()` qui appelle vos fonctions existantes selon la commande reçue. **Vous ne touchez pas à votre code métier**, vous ajoutez juste une couche I/O série.

Squelette type pour la carte moteurs (Arduino-style, à adapter à votre toolchain) :

```cpp
void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.startsWith("GOTO")) {
            float x, y;
            sscanf(line.c_str(), "GOTO %f %f", &x, &y);
            move_to_position(x, y);   // ← votre fonction existante
        } else if (line == "STOP") {
            stop_motors();             // ← votre fonction existante
        }
    }

    // Push position 10 Hz
    static unsigned long last_push = 0;
    if (millis() - last_push >= 100) {
        last_push = millis();
        Serial.printf("POS %.1f %.1f %.4f\n",
                      get_x(), get_y(), get_theta());
    }

    // Quand l'asservissement signale arrivée :
    if (just_arrived()) {
        Serial.println("DONE");
    }
}
```

