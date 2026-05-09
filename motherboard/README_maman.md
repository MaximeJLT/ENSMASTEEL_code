# Maman — Carte centrale (Raspberry Pi)

Dispatcher entre la Jetson (stratégie haut niveau) et les 3 cartes esclaves (moteurs, actionneurs, LiDAR) pour Eurobot 2026.

```
Jetson  ──UDP/JSON 5005──►  maman  ──UART──►  carte_moteurs
        ◄────────── 5006──             ──UART──►  carte_actionneurs
                                       ──UART──►  carte_LiDAR
```

## Fichiers

| Fichier | Rôle |
|---|---|
| `maman.cpp` | Code C++17 du dispatcher, single-file |
| `PROTOCOL_UART.md` | Spec à envoyer aux collègues sur les cartes esclaves |
| `fake_motor_card.py` | Simulateur de carte moteurs (Python) |
| `fake_actuator_card.py` | Simulateur de carte actionneurs |
| `fake_lidar_card.py` | Simulateur de carte LiDAR |

---

## 1. Build sur la Raspberry Pi

```bash
sudo apt update
sudo apt install -y g++ nlohmann-json3-dev socat python3-serial
g++ -std=c++17 -O2 -Wall maman.cpp -o maman
```

Si `nlohmann-json3-dev` n'est pas dispo via apt :

```bash
mkdir -p ~/include/nlohmann
wget -O ~/include/nlohmann/json.hpp \
  https://github.com/nlohmann/json/releases/latest/download/json.hpp
g++ -std=c++17 -O2 -I ~/include maman.cpp -o maman
```

---

## 2. Test ce soir, **sans aucune vraie carte**

L'idée : on crée 3 paires de PTY virtuels avec `socat`, maman ouvre un côté, les fakes ouvrent l'autre. Tout tourne sur la Pi (ou ton PC, peu importe).

### Terminal 1 — créer les ports virtuels

```bash
# Carte moteurs
socat -d -d \
  pty,raw,echo=0,link=/tmp/tty_motor_master \
  pty,raw,echo=0,link=/tmp/tty_motor_slave &

# Carte actionneurs
socat -d -d \
  pty,raw,echo=0,link=/tmp/tty_actuator_master \
  pty,raw,echo=0,link=/tmp/tty_actuator_slave &

# Carte LiDAR
socat -d -d \
  pty,raw,echo=0,link=/tmp/tty_lidar_master \
  pty,raw,echo=0,link=/tmp/tty_lidar_slave &
```

> Astuce : copie ça dans un petit `start_socat.sh` pour pas le retaper à chaque fois.

### Terminal 2, 3, 4 — lancer les fakes

```bash
python3 fake_motor_card.py    /tmp/tty_motor_slave
python3 fake_actuator_card.py /tmp/tty_actuator_slave
python3 fake_lidar_card.py    /tmp/tty_lidar_slave
```

### Terminal 5 — lancer maman

```bash
./maman /tmp/tty_motor_master /tmp/tty_actuator_master /tmp/tty_lidar_master
```

Tu devrais voir :
```
[serial] /tmp/tty_motor_master opened (fd=3)
[serial] /tmp/tty_actuator_master opened (fd=4)
[serial] /tmp/tty_lidar_master opened (fd=5)
[maman] UDP listen 5005 → ACK 5006
```

### Terminal 6 — lancer la Jetson (en simu sur la même machine)

```bash
# Dans le repertoire du projet
python3 json_main.py
# ou main_simu.py + json_strategy.py selon ce que tu utilises
```

Tu devrais voir maman log toutes les ~1 s, par ex :
```
[maman] f=42 pos=(1148,798) act=going carried=0/8 lidar=clear
[maman] f=52 pos=(1052,720) act=going carried=0/8 lidar=OBST
[maman] f=62 pos=(1000,690) act=going carried=0/8 lidar=clear
```

Et côté fakes, tu vois les commandes arriver et les `DONE` partir.

**Si tout ce flux tourne en boucle pendant 30 s sans crash : maman est validée.**

---

## 3. Test partiel (juste le côté Jetson↔maman)

Pour tester sans même les fakes, tu peux lancer maman avec les 3 mêmes PTYs (sans rien au bout). UDP fonctionne, ACK part — juste le `robot_state` ne se met pas à jour côté position. Suffisant pour valider la couche réseau.

---

## 4. Déploiement sur le robot

Une fois les vraies cartes branchées, identifie-les :

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Puis lance avec les bons chemins :

```bash
./maman /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
```

> ⚠️ Les numéros USB peuvent changer au reboot. Pour fixer, crée des règles udev avec les serial numbers des adaptateurs FTDI (je peux te filer le snippet udev quand le hard sera là).

---

## 5. Ordre des tests d'intégration (ordre de priorité)

1. ✅ **maman seule** : se compile, tourne, ne crash pas
2. ✅ **maman + Jetson** : ACK reçus côté Jetson, frame_id incrémente
3. ✅ **maman + 3 fakes + Jetson** : boucle complète sans erreur
4. 🔲 **maman + carte_moteurs réelle** (les autres en fakes) : un GOTO physique fonctionne
5. 🔲 **maman + cartes_moteurs + actionneurs réelles** : un PICKUP physique fonctionne
6. 🔲 **système complet** : match simulé en intégration

Étapes 1-3 = ce soir.
Étape 4-5 = quand les collègues ont leurs cartes prêtes.
Étape 6 = la veille de la coupe.

---

## 6. Si ça plante

| Symptôme | Cause probable |
|---|---|
| `[serial] open(...) failed: Permission denied` | Ajoute ton user au groupe `dialout` : `sudo usermod -aG dialout $USER`, puis relogue |
| `bind: Address already in use` | Une autre instance de maman tourne déjà : `pkill maman` |
| Jetson reçoit pas d'ACK | Vérifie les firewalls et que la Jetson envoie bien sur l'IP de la Pi (pas 127.0.0.1) |
| `POS` jamais reçu | Les PTYs `_master` et `_slave` sont peut-être inversés. Maman ouvre `_master`, le fake ouvre `_slave`. |
| `JSON parse error` côté maman | La Jetson envoie du non-JSON ou tronqué. Affiche `n` et `buf` pour debug. |

---
