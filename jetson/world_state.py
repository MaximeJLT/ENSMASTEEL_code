class Robot:            #Tout ça sont des conteneurs d'état
    def __init__(self):
        self.x_mm = 0
        self.y_mm = 0
        self.theta_rad = 0
        self.whatisdoing = "idle" #inactif quoi, il changera plus tard
        self.whatiscarrying = [] #liste des id des caisses qu'il transporte, de 1 à 9

class Caisse: #plus tard on mettra a jour les status des caisses (on_ground/carried/delivered)
    def __init__(self):
        self.id = 0
        self.x_mm = 0
        self.y_mm = 0
        self.status = "unknown" #peut etre "on_ground", "carried", "delivered"

class Opponent:                             #On detecte pour l'instant qu'un seul adversaire
    def __init__(self):
        self.x_mm = 0
        self.y_mm = 0
        self.theta_rad = 0
     
class Zone: #contient les zones importantes (zone de depot, zone de départ, etc)
    def __init__(self):
        self.name = ""
        self.x_mm = 0
        self.y_mm = 0
        self.width_mm = 0
        self.height_mm = 0
 
class WorldState: # Conteneur d'état du monde, dynamique a part pour la zone, définie au début
    def __init__(self):
        self.robot = {}  # dict robot_id -> Robot
        self.caisses = {} # dict caisse_id -> Caisse
        self.opponent = {} # dict opponent_id -> Opponent
        self.zones = {} # dict zone_name -> Zone
        self.matchtime = 0  # en secondes

