// =============================================================================
//  maman.cpp  —  Carte centrale du robot (Raspberry Pi, Linux)
// =============================================================================
//
//  Architecture :
//
//    Jetson  ──UDP/JSON  5005──►  maman  ──UART──►  carte_moteurs
//            ◄────────── 5006──             ──UART──►  carte_actionneurs
//                                           ──UART──►  carte_LiDAR
//
//  Maman est un *dispatcher* : pas de logique métier, juste un routeur entre
//  la Jetson (haut niveau, stratégie) et les 3 cartes esclaves (bas niveau).
//
//  Build :
//      sudo apt install nlohmann-json3-dev
//      g++ -std=c++17 -O2 maman.cpp -o maman -pthread
//
//  Run :
//      ./maman /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
//      # ou avec socat pour tester :
//      ./maman /tmp/tty_motor /tmp/tty_actuator /tmp/tty_lidar
//
//  Protocole UART (texte, ligne terminée par \n) : voir PROTOCOL_UART.md
// =============================================================================

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <cstdio> //Pour faire parser les lignes UART

#include <unistd.h> //Pour les réseaux UDP
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <nlohmann/json.hpp> //pour lire du json en cpp

using json = nlohmann::json;

// ============================================================
//  Utilitaire
// ============================================================

static int64_t now_ms_wall() {  //renvoie le temps actuel depuis l'epoch Unix (1er janvier 1970)
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()).count();
}

// ============================================================
//  SerialPort  —  POSIX UART non-bloquant, orienté ligne
// ============================================================

class SerialPort {
public:
    SerialPort(const std::string& path, speed_t baud) : path_(path) { //ouvre le port
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); //securitées lors de l'ouverture du port
        if (fd_ < 0) {
            std::cerr << "[serial] open(" << path << ") failed: " //permet de savoir si on a un echec d'ecriture dans le fd
                      << strerror(errno) << "\n";
            return;
        }

        struct termios tty{}; //configuration du port
        if (tcgetattr(fd_, &tty) != 0) { //gère la vitesse d'envoie dans le port serie
            std::cerr << "[serial] tcgetattr failed on " << path << "\n";
            ::close(fd_); fd_ = -1; return;
        }
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);
        // Mode raw 8N1, pas de flow control
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;  tty.c_cflag |= CS8; //config de base des UART pour les bits de controle, pour les eteindres ou les allumer 
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG); //desactive le mode canonique pour que Linux sache que c'est une carte qui communique et non un humain derriere son clavier
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); //ECHO desactiver toujours sur un port serie pour cartes embarquees
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP //evite un probleme de parsing
                         | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VMIN]  = 0; //pour retourner tout de suite, octets ou pas octets -> lecture non bloquante
        tty.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { //applique la config sans attendre
            std::cerr << "[serial] tcsetattr failed on " << path << "\n";
            ::close(fd_); fd_ = -1; return;
        }
        std::cout << "[serial] " << path << " opened (fd=" << fd_ << ")\n";
    }

    ~SerialPort() { if (fd_ >= 0) ::close(fd_); } //permet de sortir du fd quand l'objet n'est plus la (fin  du programme)

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    int  fd()   const { return fd_; }
    bool ok()   const { return fd_ >= 0; }
    const std::string& path() const { return path_; }

    void send_line(const std::string& s) { //appel de zrite qui balance les octets sur le port serie
        if (fd_ < 0) return;
        std::string out = s + "\n";
        ssize_t r = ::write(fd_, out.data(), out.size());
        (void)r;  // best-effort
    }

    // Récupère une ligne complète si dispo. Retourne false sinon.
    bool read_line(std::string& out) { //boucle pour lire tous les octets accumulés dans rx un buffer interne, buffer pour stocker les messages
        if (fd_ < 0) return false;
        char buf[256];
        for (;;) {
            ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n <= 0) break;
            rx_.append(buf, n);
        }
        size_t pos = rx_.find('\n'); //cherche les lignes dans le buffer
        if (pos == std::string::npos) return false;
        out = rx_.substr(0, pos);
        rx_.erase(0, pos + 1);
        if (!out.empty() && out.back() == '\r') out.pop_back();
        return true;
    }

private:
    int         fd_ = -1;
    std::string path_;
    std::string rx_;
};

// ============================================================
//  MotorCard  —  pilote la carte moteurs
// ============================================================
//
//  Maman → carte :  GOTO <x_mm> <y_mm>
//                   STOP
//                   STATUS
//
//  Carte → maman :  POS <x> <y> <theta_rad>     (push ~10 Hz)
//                   DONE                         (cible GOTO atteinte)
//                   ERR <msg>
//
struct MotorCard {
    SerialPort* port; //pointeur vers le serialport a utiliser
    double x_mm = 1150.0, y_mm = 800.0, theta_rad = 0.0;
    bool   moving = false; //position
    bool   just_arrived = false;

    // Évite de re-spammer le même GOTO
    double last_target_x = 1e18, last_target_y = 1e18;

    void send_goto(double x, double y) {
        if (x == last_target_x && y == last_target_y && moving) return;
        char buf[64];
        // On envoie le theta courant pour que le robot maintienne son orientation
        // pendant la translation (avantage du holonome : pas de rotation parasite)
        snprintf(buf, sizeof(buf), "GOTO %.1f %.1f %.4f", x, y, theta_rad);
        port->send_line(buf);
        moving = true;
        last_target_x = x;
        last_target_y = y;
    }

    void send_stop() {
        port->send_line("STOP");
        moving = false;
        last_target_x = last_target_y = 1e18;
    }

    void on_line(const std::string& line) { //parser des messages venant de la carte moteurs
        if (line.rfind("POS ", 0) == 0) {
            double x, y, th;
            if (sscanf(line.c_str() + 4, "%lf %lf %lf", &x, &y, &th) == 3) {
                x_mm = x; y_mm = y; theta_rad = th;
            }
        } else if (line == "DONE") {
            moving = false;
            just_arrived = true;
            last_target_x = last_target_y = 1e18;
        } else if (line.rfind("ERR", 0) == 0) {
            std::cerr << "[motor] " << line << "\n";
        }
    }
};

// ============================================================
//  ActuatorCard  —  pilote la carte actionneurs (servos, bras)
// ============================================================
//
//  Maman → carte :  PICK
//                   DROP
//                   STATUS
//
//  Carte → maman :  STATUS <state>      (state ∈ idle | picking | dropping)
//                   DONE
//                   ERR <msg>
//
struct ActuatorCard { //meme principe que pour la carte des moteurs
    SerialPort* port;
    std::string action = "idle";
    bool just_finished = false;

    void send_pick() {
        port->send_line("PICK");
        action = "picking";
    }
    void send_drop() {
        port->send_line("DROP");
        action = "dropping";
    }

    void on_line(const std::string& line) {
        if (line.rfind("STATUS ", 0) == 0) {
            action = line.substr(7);
        } else if (line == "DONE") {
            just_finished = true;
            action = "idle";
        } else if (line.rfind("ERR", 0) == 0) {
            std::cerr << "[actuator] " << line << "\n";
        }
    }
};

// ============================================================
//  LidarCard  —  reçoit les obstacles détectés
// ============================================================
//
//  Carte → maman :  OBST <dist_mm> <angle_rad>   (push à chaque détection)
//                   CLEAR                          (push quand plus d'obstacle)
//
struct LidarCard { //idem meme principe pour la carte LiDAR
    SerialPort* port;
    bool   obstacle = false;
    double dist_mm = 0.0;
    double angle_rad = 0.0;
    int64_t last_update_ms = 0;

    void on_line(const std::string& line) {
        if (line.rfind("OBST ", 0) == 0) {
            double d, a;
            if (sscanf(line.c_str() + 5, "%lf %lf", &d, &a) == 2) {
                obstacle = true;
                dist_mm = d;
                angle_rad = a;
                last_update_ms = now_ms_wall();
            }
        } else if (line == "CLEAR") {
            obstacle = false;
            last_update_ms = now_ms_wall();
        }
    }
};

// ============================================================
//  Main
// ============================================================

int main(int argc, char** argv) { //on branche tout ici, return 1 si le programme a plante, return 0 si succes
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <motor_tty> <actuator_tty> <lidar_tty>\n"
                  << "Exemple: " << argv[0]
                  << " /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2\n";
        return 1;
    }

    SerialPort motor_port(argv[1],     B115200); //ouvre les 3 ports serie pour les 3 cartes, logique
    SerialPort actuator_port(argv[2],  B115200);
    SerialPort lidar_port(argv[3],     B115200);
    if (!motor_port.ok() || !actuator_port.ok() || !lidar_port.ok()) {
        std::cerr << "[maman] impossible d'ouvrir les ports série, abandon\n";
        return 1;
    }

    MotorCard    motor    {&motor_port};
    ActuatorCard actuator {&actuator_port};
    LidarCard    lidar    {&lidar_port};

    // ---- Socket UDP ----
    int udp = socket(AF_INET, SOCK_DGRAM, 0); //creer le socket UDP pour communiquer avec le jetson
    if (udp < 0) { perror("socket"); return 1; }

    int yes = 1; //permet de reprendre les ports SI maman redemarre 
    setsockopt(udp, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{}; //linux ecoutera sur le port 5005 (UDP)
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(5005);
    if (bind(udp, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }
    int flags = fcntl(udp, F_GETFL, 0); //bascule le socket en non bloquant
    fcntl(udp, F_SETFL, flags | O_NONBLOCK);

    std::cout << "[maman] UDP listen 5005 → ACK 5006\n";
    std::cout << "[maman] motor=" << motor_port.path()
              << " actuator=" << actuator_port.path()
              << " lidar=" << lidar_port.path() << "\n";

    // ---- État haut niveau (ce que la stratégie Jetson voit) ----
    std::vector<int> carried;
    const int MAX_CARRIED = 8;

    int log_count = 0;

    // ---- Boucle principale (single-threaded, poll() sur 4 fds) ----
    while (true) { // on appelle a chaque tour poll() qui dit a Linux de surveiller les 4 fds pour notifier si il y en a un qui (r)envoie quelque chose
        struct pollfd fds[4];
        fds[0] = { udp,                 POLLIN, 0 };
        fds[1] = { motor_port.fd(),     POLLIN, 0 };
        fds[2] = { actuator_port.fd(),  POLLIN, 0 };
        fds[3] = { lidar_port.fd(),     POLLIN, 0 };
        poll(fds, 4, 50);

        // 1) Lire les lignes UART
        std::string line; //on vide ce qui est dispo sur chaque port serie et on file chaque ligne au parser de la carte consernee
        while (motor_port.read_line(line))    motor.on_line(line);
        while (actuator_port.read_line(line)) actuator.on_line(line);
        while (lidar_port.read_line(line))    lidar.on_line(line);

        // 2) Traiter un paquet Jetson (s'il y en a un)
        if (fds[0].revents & POLLIN) { //on lit le packet si poll nous notice de quelque chose sur l'UDP (structure from a remplire avec l'adresse de la jetson)
            char buf[65536];
            sockaddr_in from{};
            socklen_t fromlen = sizeof(from);
            ssize_t n = recvfrom(udp, buf, sizeof(buf), 0,
                                 (sockaddr*)&from, &fromlen);
            if (n > 0) {
                json msg;
                try {
                    msg = json::parse(std::string(buf, n)); //on parse le JSON reçu
                } catch (std::exception& e) {
                    std::cerr << "[maman] JSON parse error: " << e.what() << "\n";
                    continue;
                }

                int frame_id = msg.value("frame_id", -1); //envoie l'ordre du jetson en UART a la bonne carte, tout le role de maman ^^

                // ---- Dispatch de la commande ----
                if (msg.contains("command") && !msg["command"].is_null()) {
                    auto& cmd = msg["command"];
                    std::string kind = cmd.value("kind", "");

                    if (kind == "GOTO") {
                        double x = cmd.value("x_mm", 0.0);
                        double y = cmd.value("y_mm", 0.0);
                        motor.send_goto(x, y);

                    } else if (kind == "PICKUP") {
                        if (actuator.action == "idle") {
                            int cid = cmd.value("crate_id", -1);
                            actuator.send_pick();
                            if ((int)carried.size() < MAX_CARRIED)
                                carried.push_back(cid);
                        }

                    } else if (kind == "DROP_ALL") {
                        if (actuator.action == "idle") {
                            actuator.send_drop();
                            carried.clear();
                        }

                    } else if (kind == "MOVE_CURSOR") {
                        // Pour Eurobot 2026 cursor : on traite comme un GOTO
                        // pur sur l'axe X. La carte moteurs gère l'asservissement.
                        double x = cmd.value("x_mm", 0.0);
                        double y = cmd.value("y_mm", motor.y_mm);
                        motor.send_goto(x, y);

                    } else if (kind == "STOP") {
                        motor.send_stop();
                    }
                }

                // ---- Construire l'action courante (vue Jetson) ----
                std::string action = "idle"; //calcul l'etat visible du robot, s'il bouge : going, sinon idle s'il fait une action en cours
                if      (motor.moving)              action = "going";
                else if (actuator.action != "idle") action = actuator.action;

                // ---- ACK ----
                json ack = { //json de reponse dans le format attendu par la jetson
                    {"type",     "ack"},
                    {"frame_id", frame_id},
                    {"t_ms",     now_ms_wall()},
                    {"robot_state", {
                        {"x_mm",       motor.x_mm},
                        {"y_mm",       motor.y_mm},
                        {"theta_rad",  motor.theta_rad},
                        {"action",     action},
                        {"carried",    carried},
                        // Champs LiDAR (extension par rapport à maman_fictive)
                        {"obstacle",            lidar.obstacle},
                        {"obstacle_dist_mm",    lidar.dist_mm},
                        {"obstacle_angle_rad",  lidar.angle_rad},
                    }}
                };
                std::string ack_str = ack.dump(); //transforme l'objet JSON en chaine de caracteres

                sockaddr_in dest = from; //change le port (5006) pour le JETSON
                dest.sin_port = htons(5006);  // Jetson écoute sur 5006
                sendto(udp, ack_str.data(), ack_str.size(), 0,
                       (sockaddr*)&dest, sizeof(dest));

                // Log toutes les ~1 s
                if (++log_count % 10 == 0) {
                    std::cout << "[maman] f=" << frame_id
                              << " pos=(" << (int)motor.x_mm
                              << "," << (int)motor.y_mm << ")"
                              << " act=" << action
                              << " carried=" << carried.size() << "/" << MAX_CARRIED
                              << " lidar=" << (lidar.obstacle ? "OBST" : "clear")
                              << "\n";
                }
            }
        }
    }

    return 0;
}
