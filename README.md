# Cyber IS (ROS 2 Humble)

Zestaw pakietów ROS 2 do uruchomienia robota Cyber IS na Ubuntu 22.04 (ROS 2 Humble). Dokument zawiera pełne instrukcje instalacji, budowy i uruchamiania na docelowym robocie, wraz z wariantem systemd.

- W skład wchodzą pakiety: `cyber_is_bringup`, `cyber_is_description`, `cyber_is_led_controller`, `cyber_is_manual_controller`, `cyber_is_navigation`, `cyber_is_supervisor`, `cyber_is_filters`, `uart_bridge`.
- Budowa i uruchomienie: `colcon build`, `ros2 launch ...`.

## Wymagania i zależności

- System: Ubuntu 22.04 (Jammy), zalecany RPi 4 lub x86_64.
- ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html
- DDS (RMW):
  - Zalecane: Cyclone DDS. Zainstaluj: `sudo apt install ros-humble-rmw-cyclonedds-cpp` i ustaw: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
  - Alternatywnie: Fast DDS (domyślnie dostępny): `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` lub usuń zmienną środowiskową.

### Pakiety apt (minimum)
```bash
sudo apt update && sudo apt install -y \
  ros-humble-rclcpp ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-diagnostic-msgs \
  ros-humble-launch ros-humble-launch-ros \
  ros-humble-robot-state-publisher ros-humble-urdf ros-humble-xacro ros-humble-rviz2 \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-ros \
  ros-humble-nav2-bringup ros-humble-nav2-costmap-2d ros-humble-nav2-planner ros-humble-nav2-controller \
  ros-humble-nav2-bt-navigator ros-humble-nav2-behaviors ros-humble-nav2-lifecycle-manager ros-humble-nav2-amcl ros-humble-nav2-map-server \
  ros-humble-joy \
  ros-humble-rmw-cyclonedds-cpp
```

Użytkownik powinien należeć do grupy `dialout` (dostęp do `/dev/ttyACM0`):
```bash
sudo usermod -a -G dialout $USER
# wyloguj i zaloguj ponownie
```

## Struktura workspace i budowa

Przykładowy układ katalogów na robocie:
```bash
mkdir -p ~/cyber_is_ws/src
cd ~/cyber_is_ws/src
git clone https://github.com/KubaArrow/cyber_is.git
cd ..

# (opcjonalnie) czyste środowisko
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash

colcon build
source install/setup.bash
```

W razie problemów z cache CMake: `colcon build --cmake-clean-cache` lub wyczyść `build/ install/ log/`.

## Szybki start (bringup all-in-one)

Bringup uruchamia kluczowe węzły (UART bridge, supervisor, LED controller, Nav2 i opcjonalnie opis robota + RViz2).

Przykład pełnego uruchomienia:
```bash
source /opt/ros/humble/setup.bash
source ~/cyber_is_ws/install/setup.bash

ros2 launch cyber_is_bringup is_bringup.launch.py \
  start_description:=false use_gui:=false \
  start_navigation:=true autostart:=true map:=/path/to/map.yaml \
  start_uart_bridge:=true \
  start_led_controller:=true \
  start_supervisor:=true
```

Parametry domyślne dla węzłów w bringup:
- UART bridge: `cyber_is_bringup/config/uart_bridge.yaml` (port, częstotliwość, nazwy topiców).
- LED controller: `cyber_is_bringup/config/leds_controller.yaml`.
- Supervisor: `cyber_is_bringup/config/supervisor.yaml`.
- Nav2: `cyber_is_navigation/config/nav2_params.yaml` (wymaga podania pliku mapy).

Uruchomienie samego opisu robota + RViz2:
```bash
ros2 launch cyber_is_description display.launch.py use_gui:=false
```

## Pakiety i uruchamianie

- cyber_is_bringup
  - Główny launch: `cyber_is_bringup/launch/is_bringup.launch.py` z przełącznikami startowania poszczególnych modułów.
- uart_bridge
  - Węzeł C++ łączy ROS 2 z mikrokontrolerem przez SLIP na porcie szeregowym.
  - Domyślne topiki: `/cmd_vel` (sub), `/leds` (sub), `/low_level_odom`, `/imu`, `/magnet`, `/line_detector`, `/battery`, `/status_topic` (pub).
  - Ważne: ustaw poprawny port w parametrach (`/dev/ttyACM0`) i grupę `dialout` dla użytkownika.
  - Przykład standalone: `ros2 launch uart_bridge start_bridge.launch.py` (parametry również w bringup).
- cyber_is_supervisor
  - Węzeł zarządzający trybami MANUAL/AUTO. Subskrybuje `/robot_mode` (String), publikuje `/robot_state`, `/leds_mode`, `/supervisor/heartbeat`.
  - Standalone: `ros2 launch cyber_is_supervisor supervisor.launch.py`.
  - Zmiana trybu: `ros2 topic pub /robot_mode std_msgs/String '{data: "AUTO"}' -1`.
- cyber_is_led_controller
  - Węzeł Python sterujący diodami LED. Subskrybuje `/leds_mode`, publikuje kolory na `/leds`.
  - Standalone: `ros2 launch cyber_is_led_controller led_controller.launch.py`.
- cyber_is_manual_controller
  - Teleop z joysticka: `sensor_msgs/Joy` -> `geometry_msgs/Twist`.
  - Uruchom sterownik joysticka: `ros2 run joy joy_node`, potem: `ros2 launch cyber_is_manual_controller start_manual_mode.launch.py`.
- cyber_is_navigation
  - Nav2 w kompozycji (map_server, amcl, planner, controller, behaviors, bt_navigator + lifecycle manager).
  - Standalone: `ros2 launch cyber_is_navigation start_navigation.launch.py map:=/path/to/map.yaml`.
- cyber_is_filters
  - Filtry C++ dla linii i magnetometru. Standalone: `ros2 launch cyber_is_filters start_filters.launch.py`.
- cyber_is_description
  - URDF/Xacro + RViz2. `ros2 launch cyber_is_description display.launch.py`.

## Systemd (uruchamianie na starcie)

Przykładowa jednostka systemd do uruchomienia bringup w ROS 2 (dostosuj ścieżki i użytkownika):
```ini
[Unit]
Description=Cyber IS Bringup (ROS 2 Humble)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=victoria
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/victoria/cyber_is_ws/install/setup.bash && ros2 launch cyber_is_bringup is_bringup.launch.py start_description:=false use_gui:=false start_navigation:=true autostart:=true map:=/home/victoria/cyber_is_ws/src/cyber_is/maps/map.yaml start_uart_bridge:=true start_led_controller:=true start_supervisor:=true'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Aktywacja:
```bash
sudo nano /etc/systemd/system/cyber-is-bringup.service  # wklej i dostosuj wzór z sekcji powyżej
sudo systemctl daemon-reload
sudo systemctl enable cyber-is-bringup.service
sudo systemctl start cyber-is-bringup.service
sudo systemctl status cyber-is-bringup.service
```

Uwaga: plik `cyber_is_bringup/services/robot_bringup.service` w repo był przeznaczony dla ROS 1 – użyj wzoru powyżej dla ROS 2.

## Konfiguracja sieci (opcjonalnie statyczne IP)

Jeśli chcesz nadać robotowi stały adres IP przez Wi‑Fi, przykładowa konfiguracja netplan (dostosuj SSID/hasło i adresy):
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
```yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      access-points:
        "YOUR_SSID":
          password: "YOUR_PASSWORD"
      dhcp4: no
      addresses: [192.168.1.11/24]
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
```
```bash
sudo netplan apply
```

## Typowe problemy i rozwiązania

- Błąd CMake: „Could not find ROS middleware implementation 'rmw_cyclonedds_cpp'”
  - Zainstaluj: `sudo apt install ros-humble-rmw-cyclonedds-cpp` i ustaw `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, lub przełącz na Fast DDS: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- Ostrzeżenia AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH do nieistniejącego `install/...`
  - Otwórz nową powłokę i wykonaj tylko: `source /opt/ros/humble/setup.bash` przed budową. Wyczyść `build/ install/ log/` jeśli potrzeba.
- Brak uprawnień do `/dev/ttyACM0`
  - Dodaj użytkownika do `dialout`: `sudo usermod -a -G dialout $USER` i zaloguj się ponownie.
- RViz2/joint_state_publisher_gui nie startuje
  - Zainstaluj: `sudo apt install ros-humble-joint-state-publisher-gui`. Uruchom `use_gui:=false` jeśli niepotrzebne.
- Nav2 nie ładuje mapy
  - Podaj pełną ścieżkę do pliku `.yaml` mapy w argumencie `map:=...`. Sprawdź prawa dostępu.

## Szybkie sprawdzenie działania

- LED controller: `ros2 topic pub /leds_mode std_msgs/String '{data: "SIDE_GREEN_BREATH"}' -1`
- Supervisor: `ros2 topic pub /robot_mode std_msgs/String '{data: "MANUAL"}' -1`
- Joystick: `ros2 run joy joy_node` oraz `ros2 launch cyber_is_manual_controller start_manual_mode.launch.py`
- UART bridge: `ros2 topic echo /low_level_odom`, `ros2 topic echo /imu`

## Notatki dev

- Styl: ROS 2 Humble, `colcon`, Python (ament_python) i C++ (ament_cmake).
- Topiki kluczowe: `/cmd_vel`, `/low_level_odom`, `/imu`, `/magnet`, `/line_detector`, `/leds`, `/leds_mode`, `/robot_state`, `/robot_mode`.
- Pliki parametrów znajdują się w `cyber_is_bringup/config` oraz `cyber_is_navigation/config`.
