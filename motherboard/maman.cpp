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
//  Maman = dispatcher entre la Jetson (qui pense en mm) et les 3 cartes
//  esclaves. ATTENTION : la carte moteurs (Antoine) travaille en MÈTRES
//  car OTOS est configuré en mètres. Maman convertit donc mm <-> m
//  uniquement pour la carte moteurs.
//
//  Commandes acceptées (de la Jetson) :
//      GOTO         x_mm y_mm                   → translation, theta maintenu
//      PICKUP       crate_id                    → ramassage
//      DROP_ALL                                 → dépose les caisses portées
//      MOVE_CURSOR  x_mm y_mm                   → comme GOTO sur axe X
//      STOP                                     → arrêt immédiat
//      CALIBRATION                              → calibration odo (avant match)
//      COULEUR      x_mm y_mm theta_rad         → repositionne l'odométrie
//
//  Build :
//      sudo apt install nlohmann-json3-dev
//      g++ -std=c++17 -O2 maman.cpp -o maman -pthread
//
//  Run :
//      ./maman /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
// =============================================================================

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <cstdio>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ============================================================
//  Utilitaire
// ============================================================

static int64_t now_ms_wall() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()).count();
}

// ============================================================
//  SerialPort  —  POSIX UART non-bloquant, orienté ligne
// ============================================================

class SerialPort {
public:
    SerialPort(const std::string& path, speed_t baud) : path_(path) {
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "[serial] open(" << path << ") failed: "
                      << strerror(errno) << "\n";
            return;
        }

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "[serial] tcgetattr failed on " << path << "\n";
            ::close(fd_); fd_ = -1; return;
        }
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;  tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "[serial] tcsetattr failed on " << path << "\n";
            ::close(fd_); fd_ = -1; return;
        }
        std::cout << "[serial] " << path << " opened (fd=" << fd_ << ")\n";
    }

    ~SerialPort() { if (fd_ >= 0) ::close(fd_); }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    int  fd()   const { return fd_; }
    bool ok()   const { return fd_ >= 0; }
    const std::string& path() const { return path_; }

    void send_line(const std::string& s) {
        if (fd_ < 0) return;
        std::string out = s + "\n";
        ssize_t r = ::write(fd_, out.data(), out.size());
        (void)r;
    }

    bool read_line(std::string& out) {
        if (fd_ < 0) return false;
        char buf[256];
        for (;;) {
            ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n <= 0) break;
            rx_.append(buf, n);
        }
        size_t pos = rx_.find('\n');
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
//  ATTENTION : la carte moteurs (Antoine) parle en MÈTRES + RADIANS
//  (OTOS configuré ainsi). Maman convertit donc mm <-> m.
//
//  Maman → carte (UART, texte) :
//      GOTO        <x_m> <y_m> <theta_rad>
//      STOP
//      CALIBRATION
//      COULEUR     <x_m> <y_m> <theta_rad>
//
//  Carte → maman (UART, texte) :
//      POS         <x_m> <y_m> <theta_rad>     (push ~10 Hz)
//      DONE                                    (cible atteinte / calib finie)
//      ERR         <msg>
//
struct MotorCard {
    SerialPort* port;
    // État stocké en MILLIMÈTRES (convention maman/Jetson)
    double x_mm = 1150.0, y_mm = 800.0, theta_rad = 0.0;
    bool   moving = false;
    bool   just_arrived = false;
    bool   calibrating = false;

    // Évite de re-spammer le même GOTO
    double last_target_x = 1e18, last_target_y = 1e18;

    void send_goto(double x_mm_in, double y_mm_in) {
        if (x_mm_in == last_target_x && y_mm_in == last_target_y && moving) return;
        // mm -> m pour la carte moteurs
        double x_m = x_mm_in / 1000.0;
        double y_m = y_mm_in / 1000.0;
        char buf[64];
        // theta courant pour maintenir l'orientation (holonome → translation pure)
        snprintf(buf, sizeof(buf), "GOTO %.4f %.4f %.4f", x_m, y_m, theta_rad);
        port->send_line(buf);
        moving = true;
        last_target_x = x_mm_in;
        last_target_y = y_mm_in;
    }

    void send_stop() {
        port->send_line("STOP");
        moving = false;
        last_target_x = last_target_y = 1e18;
    }

    void send_calibration() {
        port->send_line("CALIBRATION");
        calibrating = true;
        std::cout << "[maman] -> CALIBRATION envoyée\n";
    }

    void send_couleur(double x_mm_in, double y_mm_in, double theta_rad_in) {
        // mm -> m pour la carte moteurs
        double x_m = x_mm_in / 1000.0;
        double y_m = y_mm_in / 1000.0;
        char buf[64];
        snprintf(buf, sizeof(buf), "COULEUR %.4f %.4f %.4f",
                 x_m, y_m, theta_rad_in);
        port->send_line(buf);
        // On force aussi notre vue locale
        x_mm = x_mm_in;
        y_mm = y_mm_in;
        theta_rad = theta_rad_in;
        std::cout << "[maman] -> COULEUR envoyée : ("
                  << x_mm_in << "mm, " << y_mm_in << "mm, "
                  << theta_rad_in << "rad)\n";
    }

    void on_line(const std::string& line) {
        if (line.rfind("POS ", 0) == 0) {
            double x_m, y_m, th;
            if (sscanf(line.c_str() + 4, "%lf %lf %lf", &x_m, &y_m, &th) == 3) {
                // m -> mm reçu de la carte moteurs
                x_mm = x_m * 1000.0;
                y_mm = y_m * 1000.0;
                theta_rad = th;
            }
        } else if (line == "DONE") {
            if (calibrating) {
                calibrating = false;
                std::cout << "[maman] CALIBRATION terminée\n";
            } else {
                moving = false;
                just_arrived = true;
                last_target_x = last_target_y = 1e18;
            }
        } else if (line.rfind("ERR", 0) == 0) {
            std::cerr << "[motor] " << line << "\n";
        }
    }
};

// ============================================================
//  ActuatorCard
// ============================================================
struct ActuatorCard {
    SerialPort* port;
    std::string action = "idle";
    bool just_finished = false;
    bool   obstacle = false;
    double dist_mm = 0.0;
    double angle_rad = 0.0;
    int64_t last_update_ms = 0;

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
        } else if (line.rfind("OBST ", 0) == 0) {
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

/*
// ============================================================
//  LidarCard
// ============================================================
struct LidarCard {
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
*/

// ============================================================
//  Main
// ============================================================

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <motor_tty> <actuator_tty> <lidar_tty>\n";
        return 1;
    }

    SerialPort motor_port(argv[1],     B115200);
    SerialPort actuator_port(argv[2],  B115200);
    //SerialPort lidar_port(argv[3],     B115200);
    //if (!motor_port.ok() || !actuator_port.ok() || !lidar_port.ok()) {
    if (!motor_port.ok() || !actuator_port.ok()) {
        std::cerr << "[maman] impossible d'ouvrir les ports série, abandon\n";
        return 1;
    }

    MotorCard    motor    {&motor_port};
    ActuatorCard actuator {&actuator_port};
    //LidarCard    lidar    {&lidar_port};

    int udp = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp < 0) { perror("socket"); return 1; }

    int yes = 1;
    setsockopt(udp, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(5005);
    if (bind(udp, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }
    int flags = fcntl(udp, F_GETFL, 0);
    fcntl(udp, F_SETFL, flags | O_NONBLOCK);

    std::cout << "[maman] UDP listen 5005 -> ACK 5006\n";
    std::cout << "[maman] motor=" << motor_port.path()
              << " actuator=" << actuator_port.path() << "\n";
              //<< " lidar=" << lidar_port.path() << "\n";

    std::vector<int> carried;
    const int MAX_CARRIED = 8;

    int log_count = 0;

    while (true) {
        struct pollfd fds[4];
        fds[0] = { udp,                 POLLIN, 0 };
        fds[1] = { motor_port.fd(),     POLLIN, 0 };
        fds[2] = { actuator_port.fd(),  POLLIN, 0 };
        //fds[3] = { lidar_port.fd(),     POLLIN, 0 };
        poll(fds, 4, 50);

        // 1) Lire les lignes UART
        std::string line;
        while (motor_port.read_line(line))    motor.on_line(line);
        while (actuator_port.read_line(line)) actuator.on_line(line);
        //while (lidar_port.read_line(line))    lidar.on_line(line);

        // 2) Traiter un paquet Jetson
        if (fds[0].revents & POLLIN) {
            char buf[65536];
            sockaddr_in from{};
            socklen_t fromlen = sizeof(from);
            ssize_t n = recvfrom(udp, buf, sizeof(buf), 0,
                                 (sockaddr*)&from, &fromlen);
            if (n > 0) {
                json msg;
                try {
                    msg = json::parse(std::string(buf, n));
                } catch (std::exception& e) {
                    std::cerr << "[maman] JSON parse error: " << e.what() << "\n";
                    continue;
                }

                int frame_id = msg.value("frame_id", -1);

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
                        double x = cmd.value("x_mm", 0.0);
                        double y = cmd.value("y_mm", motor.y_mm);
                        motor.send_goto(x, y);

                    } else if (kind == "STOP") {
                        motor.send_stop();

                    } else if (kind == "CALIBRATION") {
                        // === NOUVEAU ===
                        // Déclenche la séquence de calibration de l'odométrie
                        // sur la carte moteurs (10 tours angulaires + 10 aller-retour
                        // pour calibrer les gains du capteur OTOS).
                        //
                        // À envoyer AVANT le match (le robot tourne en rond ~30s).
                        // NE JAMAIS envoyer pendant un match.
                        motor.send_calibration();

                    } else if (kind == "COULEUR") {
                        // === NOUVEAU ===
                        // Repositionne l'odométrie selon notre position de départ.
                        // À envoyer UNE fois au début du match avec la position
                        // correspondant à notre couleur d'équipe.
                        //
                        // Exemple : si on est côté droit, COULEUR 1150 800 0
                        //          si on est côté gauche (couleur inverse), COULEUR -1150 800 3.1416
                        double x  = cmd.value("x_mm", 0.0);
                        double y  = cmd.value("y_mm", 0.0);
                        double th = cmd.value("theta_rad", 0.0);
                        motor.send_couleur(x, y, th);
                    }
                }

                // ---- Action courante (vue Jetson) ----
                std::string action = "idle";
                if      (motor.calibrating)         action = "calibrating";
                else if (motor.moving)              action = "going";
                else if (actuator.action != "idle") action = actuator.action;

                // ---- ACK ----
                json ack = {
                    {"type",     "ack"},
                    {"frame_id", frame_id},
                    {"t_ms",     now_ms_wall()},
                    {"robot_state", {
                        {"x_mm",       motor.x_mm},
                        {"y_mm",       motor.y_mm},
                        {"theta_rad",  motor.theta_rad},
                        {"action",     action},
                        {"carried",    carried},
                        {"obstacle",            actuator.obstacle},
                        {"obstacle_dist_mm",    actuator.dist_mm},
                        {"obstacle_angle_rad",  actuator.angle_rad},
                    }}
                };
                std::string ack_str = ack.dump();

                sockaddr_in dest = from;
                dest.sin_port = htons(5006);
                sendto(udp, ack_str.data(), ack_str.size(), 0,
                       (sockaddr*)&dest, sizeof(dest));

                if (++log_count % 10 == 0) {
                    std::cout << "[maman] f=" << frame_id
                              << " pos=(" << (int)motor.x_mm
                              << "," << (int)motor.y_mm << ")"
                              << " act=" << action
                              << " carried=" << carried.size() << "/" << MAX_CARRIED
                              << " lidar=" << (actuator.obstacle ? "OBST" : "clear")
                              << "\n";
                }
            }
        }
    }

    return 0;
}
