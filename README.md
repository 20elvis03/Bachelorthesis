# Flughafenterminal Multi-Roboter Autonomes Reinigungssystem

Eine ROS 2 Kilted / Gazebo Ionic Simulation von drei autonomen Reinigungsrobotern in einer detaillierten Flughafenterminal Umgebung. Die Roboter navigieren mittels eines Bug2-basierten Hindernisvermeidungsalgorithmus, koordinieren sich untereinander zur Kollisionsvermeidung und kehren bei niedrigem Akkustand selbstständig zu ihren Ladestationen zurück.

## Voraussetzungen

- ROS 2 Kilted
- Gazebo Ionic
- (bei Nutzung von WSL 2.0+ Ubuntu 24.04)
- `ros_gz_sim`, `ros_gz_bridge`
- `tf_transformations` Python-Paket

## Installation

### 1. WSL & Ubuntu einrichten (nur unter Windows 11)

In PowerShell (als Administrator):

```powershell
wsl --install
# Rechner neu starten
```

```powershell
wsl --install -d Ubuntu-24.04
```

Nach der Installation werden ein Benutzername und ein Passwort abgefragt. Anschließend ein Ubuntu-Fenster öffnen.

### 2. System aktualisieren & Grundpakete installieren

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
```

### 3. ROS 2 Paketquelle hinzufügen

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
```

### 4. ROS 2 Kilted & Projektpakete installieren

```bash
sudo apt install -y \
  ros-kilted-desktop \
  ros-kilted-ros-gz \
  ros-kilted-ros-gz-sim ros-kilted-ros-gz-bridge \
  ros-kilted-robot-state-publisher \
  ros-kilted-joint-state-publisher-gui \
  ros-kilted-rviz2 ros-kilted-xacro \
  ros-kilted-tf-transformations \
  python3-transforms3d \
  ros-dev-tools python3-colcon-common-extensions
```

### 5. Workspace anlegen & Repository klonen

```bash
mkdir -p ~/ros2_kilted/src
git clone -b main            https://github.com/20elvis03/Bachelorthesis.git ~/ros2_kilted/src/my_robot_description
git clone -b my_robot_gazebo https://github.com/20elvis03/Bachelorthesis.git ~/ros2_kilted/src/my_robot_gazebo
```

### 6. Pakete bauen

```bash
cd ~/ros2_kilted
source /opt/ros/kilted/setup.bash
colcon build --packages-select my_robot_description my_robot_gazebo
```

### 7. Umgebung konfigurieren

Die folgenden Zeilen an `~/.bashrc` anhängen (`$HOME` sorgt dafür, dass es unabhängig vom Benutzernamen funktioniert):

```bash
cat >> ~/.bashrc <<'EOF'
```

```bash
# --- ROS 2 Kilted + Projekt ---
source /opt/ros/kilted/setup.bash
export ROS_DOMAIN_ID=007
source ~/ros2_kilted/install/local_setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_kilted/install/my_robot_description/share:/opt/ros/kilted/share
EOF
```

```bash
source ~/.bashrc
```

## Projektübersicht

Dieses Projekt simuliert eine autonome Flotte von Reinigungsrobotern in einem realistischen Flughafenterminal. Jeder Roboter fährt eigenständig den Terminalboden in einem Rasenmäher-Muster ab, weicht Hindernissen und anderen Robotern mittels eines Bug2-basierten Hindernisvermeidungsalgorithmus aus und verwaltet seinen eigenen Akku-Lebenszyklus, fährt also bei niedrigem Akkustand zum Ladepad (fiktiv wird dort auch das Wasser getauscht), lädt auf und nimmt die Arbeit wieder auf.

Die Simulation beinhaltet ein vollständig modelliertes Flughafenterminal mit Boarding-Gates, Sicherheitskontrolle, Toiletten, Sitzbänken, Fluginformationsanzeigen/displays und einer eigenen Roboter-Garage mit individuellen Ladepads.

## Architektur

```
multi_robot_gazebo_launch.py
├── Gazebo Ionic Welt (airport_terminal_world.sdf)
├── Shared Bridge (bridge_multi_shared.yaml 
│   └── /clock, /world/pose_info, Überwachungskameras
├── Roboter 1 (robot_1/)
│   ├── robot_state_publisher (my_robot_description.urdf)
│   ├── Per-Robot Bridge (bridge_per_robot.yaml)
│   └── autonomousbug.py (AutoDrive Node)
├── Roboter 2 (robot_2/)
│   └── ... (gleiche Struktur)
└── Roboter 3 (robot_3/)
    └── ... (gleiche Struktur)
```
Um die Simulation performanter zu machen verfügen bridge_multi_shared.yaml und bridge_per_robot.yaml beide über eine weitere Kopie jeweils ohne die Kamera (bridge_multi_shared_without_camera.yaml, bridge_per_robot_without_camera.yaml). Dadurch läuft die Simulation mit RTF (Real Time Factor) ~99% statt den ~10% mit Kameras.

## Dateien

### Kern

| Datei | Beschreibung |
|-------|-------------|
| `autonomousbug.py` | Haupt-Autonomieknoten. Implementiert Navigation, Hindernisvermeidung, Multi-Roboter-Koordination, Akkuverwaltung und Ladelogik. |
| `multi_robot_gazebo_launch.py` | ROS 2 Launch-Datei, die die Gazebo-Welt startet, drei Roboter mit namespaced Topics spawnt, Bridge-Konfigurationen und Auto-Drive-Nodes einrichtet. |
| `manually.py` | Tastatursteuerung zur manuellen Kontrolle eines der drei Roboter (WASD-Steuerung). |

### Roboterbeschreibung

| Datei | Beschreibung |
|-------|-------------|
| `my_robot_description.urdf` | Roboter-URDF mit zweiteiligem Chassis (Ober-/Unterkörper), Ackermann-Lenkung, GPU-LiDAR (360°) und 6 RGBD-Kameras (vorne, vorne-Boden, links, rechts, hinten, hinten-Boden). |

### Welt

| Datei | Beschreibung |
|-------|-------------|
| `airport_terminal_world.sdf` | Vollständiges Flughafenterminal (~365 Modelle) mit Wänden, Böden, Boarding-Gates A1–A4, Sicherheitskontrolle, Toiletten, Sitzbänken, Fluganzeigen/displays, Roboter-Garage mit Ladepads, Überwachungskameras und Beleuchtung. |

### Bridge-Konfigurationen

| Datei | Beschreibung |
|-------|-------------|
| `bridge_multi_shared.yaml` | Gemeinsame Gazebo↔ROS-Bridge für globale Topics: Clock, Weltpositionen, Überwachungskameras. |
| `bridge_multi_shared_ohne_kamera.yaml` | Wie oben, aber ohne Kamera-Topics (ressourcenschonender). |
| `bridge_per_robot.yaml` | Per-Roboter Bridge-Template für cmd_vel, Lenkung, Odometrie, LiDAR und alle Kameras. `{ns}` wird beim Start ersetzt. |
| `bridge_per_robot_ohne_kamera.yaml` | Per-Roboter Bridge ohne Kamera-Topics (ressourcenschonender). |

## autonomousbug.py — Zustandsautomat & Funktionen

### Zustände

| Zustand | Beschreibung |
|---------|-------------|
| `DRIVE` | Normalbetrieb: Spurhaltung (lane_gx) mit Lenkkorrektur, Rasenmäher-Muster. |
| `BRAKE` | Kurzer Stopp vor der Einleitung einer Kehrtwende an Grenzen. |
| `TURN_CHECK` | Bestimmung der Wenderichtung basierend auf Sweep-Richtung und Seitenfreiraum. |
| `TURN` | Ausführung einer 180°-Kehrtwende mit Vorwärtsbogen + Lenkung. |
| `REVERSE_TURN` | Rückwärtsmanöver bei Feststecken während einer Wende. |
| `BUG2_WALL` | Bug2-Wandverfolgung zur Hindernisvermeidung (Drehen → Geradeaus → Verfolgen). |
| `BUG2_RETURN` | Rückkehr zur M-Linie nach Bug2-Wandverfolgung. |
| `GO_HOME` | Mehrstufige Navigation zum Ladepad (nav_to_lane → drive_to_pad → reverse_uturn → exit_garage). |
| `CHARGING` | Stationär auf dem Ladepad, Akku wird geladen. |
| `YIELD` | Angehalten, wartet bis ein höher priorisierter Roboter vorbeigefahren ist. |
| `DONE` | Akku leer, Roboter dauerhaft gestoppt. |
| `EMERGENCY` | Geschwindigkeitslimit überschritten oder zu nah an einem Objekt, Notabschaltung. Reset über `/emergency_reset`. |

### GO_HOME-Phasen

| Phase | Beschreibung |
|-------|-------------|
| `nav_to_lane` | Richtung Süden bis Y≈-22.5 (Garageneinfahrt) fahren, dann X auf Ladepad ausrichten. |
| `uturn_entry` | 180°-Vorwärtskehrtwende bei falscher Blickrichtung. |
| `drive_to_pad` | Von jeder Position/Ausrichtung aus auf das Ladepad fahren. Letzte Meter gerade nach Süden. |
| `reverse_uturn` | Nach dem Laden: Rückwärts-Kehrtwende direkt vom Pad, um nach Norden zu schauen. |
| `exit_garage` | Richtung Norden aus der Garage fahren bis Y=-22.5, dabei X-Korrektur Richtung gespeicherter Bahn. |

### Wichtige Funktionen

| Funktion | Beschreibung |
|----------|-------------|
| `_pub(lin, ang, steer)` | Geschwindigkeits- und Lenkbefehle publizieren. Enthält Notfall-Geschwindigkeitsprüfung. |
| `_adiff(a, b)` | Winkeldifferenz normalisiert auf [-π, π]. |
| `_oob()` | Prüft ob Roboter außerhalb der Grenzen ist (Hauptbereich oder Garagenzone). |
| `_near_oob_boundary(margin)` | Prüft ob Roboter nahe einer Begrenzungskante ist. |
| `_in_charge_zone()` | Prüft ob innerhalb des Ladepad-Radius (deaktiviert Hindernissensoren). |
| `_robot_in_front()` | Erkennt einen anderen Roboter im Frontkegel auf Kollisionskurs. |
| `_should_yield(other)` | Prioritätsprüfung: niedrigerer Name = höhere Priorität (robot_1 > robot_2 > robot_3). |
| `_scan_cb(msg)` | Verarbeitet LiDAR-PointCloud2 zu 6 Richtungsdistanzen (vorne, links, rechts, hinten, vorne-links, vorne-rechts). |
| `_pose_cb(msg)` | Verfolgt die globale Roboterposition über Welt-Pose-Transforms (Spawn-Matching + Kontinuität). |
| `_publish_coordination()` | Eigene Position auf `/robot_coordination` senden für Multi-Roboter-Koordination. |
| `_start_bug2()` | Bug2-Hindernisvermeidung starten mit intelligenter Seitenwahl. |
| `_loop(dt)` | Haupt-Steuerungsschleife (20Hz): Akkuverwaltung, Zustandsautomat, Feststeck-Erkennung. |

### Wichtige Parameter

| Parameter | Standard | Beschreibung |
|-----------|----------|-------------|
| `DRIVE_SPEED` | 0.4 m/s | Normale Vorwärtsgeschwindigkeit |
| `OBSTACLE_FRONT` | 2.9 m | Vordere Hinderniserkennung (ab LiDAR, sitzt recht mittig vom Roboter) |
| `OBSTACLE_STOP` | 1.2 m | Notstopp-Distanz |
| `BAT_LOW_PCT` | 70% | Akkuschwelle für Heimfahrt |
| `CHARGE_RATE_PCT` | 1.0 %/s | Ladegeschwindigkeit |
| `MAX_STEER` | 0.49 rad | Maximaler Lenkwinkel |
| `YIELD_TIMEOUT` | 15 s | Maximale Wartezeit vor Bug2-Ausweichen |

### Wiederherstellungsmechanismen

Das System enthält mehrere Ebenen der Wiederherstellung für Robustheit. Die **Feststeck-Erkennung** prüft, ob sich der Roboter innerhalb von 7 Sekunden weniger als 0.15m bewegt hat, und löst ein Rückwärtsmanöver aus. Die **Globale Feststeck-Erkennung** überwacht die Bewegung über 50 Sekunden in allen Zuständen (außer CHARGING) und eskaliert nach 3 aufeinanderfolgenden Auslösungen im selben Bereich zu einem erzwungenen Bug2-Ausweichen oder GO_HOME-Neustart. Der **Bug2-Abbruch** erkennt, wenn ein Hindernis während der Wandverfolgung verschwindet (z.B. ein anderer Roboter ist weggefahren), und kehrt sofort zum vorherigen Navigationszustand zurück.

## Roboter-Spezifikationen

Der Roboter nutzt Ackermann-Lenkung (Vorderradantrieb + Lenkung) und verfügt über ein zweiteiliges Chassis: Oberkörper (1.4 × 0.8m) und Unterkörper (0.8 × 0.6m). Er ist ausgestattet mit einem 360°-GPU-LiDAR an Position (-0.308, 0, 1.22) relativ zur Basis, der Hinderniserkennung in sechs Richtungskegeln ermöglicht. Sechs RGBD-Kameras decken Vorne, Vorne-Boden, Links, Rechts, Hinten und Hinten-Boden ab. Die LiDAR-Körperversätze für die Interpretation der Scandaten betragen ca. 1.0m nach vorne, 0.7m zu den Seiten und 0.7m nach hinten.

## Weltlayout

Das Terminal erstreckt sich über ca. 50 × 40 Meter. Der Hauptbereich liegt bei X: -23 bis +23 und Y: -23 bis +15, die Roboter-Garage darunter bei X: 10.5–24.5, Y: -34.5 bis -23.0. Vier Boarding-Gates (A1–A4) befinden sich an der Nordwand bei X-Positionen -15, -5, +5 und +15, jeweils mit Schaltern, Monitoren, Beschilderung und gelben Wartelinien auf dem Boden. Die Ostwand enthält drei Toilettenanlagen (Damen, Herren, Barrierefrei) mit einem eigenen Raum dahinter. Eine Sicherheitskontrolle mit Absperrungen befindet sich am Terminaleingang, und eine gelbe Grenzlinie bei Y=16 markiert die Gate-Bereichsgrenze. Die Garage enthält drei individuelle Ladepads an den Positionen (21.0, -28.2), (15.0, -28.2) und (9.0, -28.2).

## Starten
### Vollständige Simulation mit 3 autonomen Robotern
In das ROS 2 Workspace wechseln
```bash
#Die ganzen Packages bauen
colcon build --packages-select my_robot_description my_robot_gazebo
```
```bash
#Sourcen
source ~/ros2_kilted/install/setup.bash
```
```bash
#Simulation starten
ros2 launch my_robot_description multi_robot_gazebo_launch.py
```
### Manuelle Steuerung eines bestimmten Roboters
```bash
ros2 run my_robot_description manually.py -- robot_1
```
### Notfall-Reset (falls ein Roboter die Geschwindigkeitsbegrenzung auslöst oder zu nah an ein Hindernis gerät)
```bash
ros2 topic pub --once /robot_1/emergency_reset std_msgs/Float64 "data: 1.0"
```

Lidar Scan in RVIZ 2 anschauen durch den das eingeben von **rviz** in einem neuen Terminal

Das Bild von bestimmten Kameras anzeigen durch das eingeben von **rqt** in einem neuen Terminal 

## Multi-Roboter-Koordination

Die Roboter koordinieren sich über ein gemeinsames `/robot_coordination`-Topic, auf dem jeder seine Position sendet. Das Prioritätssystem ist namensbasiert: robot_1 hat die höchste Priorität, robot_3 die niedrigste. Wenn zwei Roboter frontal aufeinander zufahren, geht der niedrigpriorisierte in den YIELD-Zustand und stoppt, bis der andere vorbeigefahren ist oder ein 15-Sekunden-Timeout Bug2-Ausweichen auslöst. Während der GO_HOME-Navigation weichen Roboter ebenfalls höherpriorisierten aus, wechseln aber zu Bug2 wenn sie von statischen Hindernissen blockiert werden.
