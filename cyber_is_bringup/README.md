# Cyber IS Bringup (ROS 2 Humble)

`cyber_is_bringup` uruchamia komplet węzłów robota (UART bridge, supervisor, kontroler LED, Nav2 + opcjonalnie SLAM Toolbox, opis robota i RViz2). Poniższe kroki zakładają Ubuntu 22.04 / ROS 2 Humble.

## Budowa workspacu
```bash
cd ~/cyber_ws
colcon build
source install/setup.bash
```

## Uruchomienie (wszystko w jednym)
```bash
ros2 launch cyber_is_bringup is_bringup.launch.py \
  start_description:=false use_gui:=false \
  start_navigation:=true autostart:=true \
  map:=/path/to/map.yaml \
  start_uart_bridge:=true \
  start_led_controller:=true \
  start_supervisor:=true
```
- Podaj własną mapę (`map:=...`) lub skorzystaj z domyślnej `/share/cyber_is_navigation/maps/map.yaml` instalowanej z pakietu `cyber_is_navigation`.
- Aby wystartować SLAM Toolbox + Nav2 bez mapy, ustaw `use_slam:=true`.

## Skrypt systemd / start automatyczny
`bash/setup_robot.sh` przygotowuje środowisko (CycloneDDS, ROS_DOMAIN_ID) i uruchamia `is_bringup.launch.py`. Obsługuje:
- `USE_GUI=true|false` – wymuszenie GUI (domyślnie wykrywa DISPLAY)
- `USE_SLAM=true` – startuje wariant ze SLAM Toolbox
- `MAP_FILE=/pełna/ścieżka/map.yaml` – własna mapa zamiast domyślnej

Przykładowa jednostka systemd (`services/robot_bringup.service`):
```ini
[Unit]
Description=Cyber IS ROS 2 Bringup
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=is
WorkingDirectory=/home/is/cyber_ws
Environment=LANG=C.UTF-8
Environment=LC_ALL=C.UTF-8
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ExecStart=/home/is/cyber_ws/src/cyber_is/cyber_is_bringup/bash/setup_robot.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
Aktywacja:
```bash
sudo cp services/robot_bringup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot_bringup.service
sudo systemctl start robot_bringup.service
```

## Pakiety zależne
Bringup zakłada, że dostępne i skonfigurowane są:
- `uart_bridge` – komunikacja z mikrokontrolerem (domyślnie `/dev/ttyACM0`)
- `cyber_is_supervisor`, `cyber_is_led_controller`
- `cyber_is_navigation` – Nav2 + SLAM Toolbox (patrz README pakietu)
- Lidar `ldlidar_sl_ros2` oraz `rosbridge_server` (opcjonalnie)

## Dostosowanie
- Parametry poszczególnych węzłów znajdują się w katalogu `config/`.
- Jeśli potrzebujesz innych tematów LIDAR-u lub portu szeregowego, przygotuj własne pliki YAML i przekaż przez odpowiednie argumenty launchera (`uart_bridge_params_file`, `slam_params_file`, itd.).
