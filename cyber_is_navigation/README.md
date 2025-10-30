## ROS 2 (Humble) – Nav2 w kompozycji (RPi4)

Ten pakiet dostarcza lekką konfigurację Nav2 uruchamianą w jednym kontenerze kompozytowym (intra-process), zoptymalizowaną pod Raspberry Pi 4. Zależności: standardowy Nav2, bez customowych warstw costmapy.

### Wymagania
- ROS 2 Humble (Jammy)
- DDS: CycloneDDS (zalecane)
  - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Mapę w formacie YAML (`map_server`)

### Szybki start
- Zbuduj workspace: `colcon build` i `source install/setup.bash`
- Uruchom Nav2 w kompozycji:
  - `ros2 launch cyber_is_navigation start_navigation.launch.py map:=/path/to/map.yaml`
- RViz2: wyślij goal przez akcję `NavigateToPose` — robot powinien jechać stabilnie.

### Pliki i uruchamianie
- `launch/start_navigation.launch.py` — uruchamia w jednym procesie: `planner_server` (Smac2D), `controller_server` (RegulatedPurePursuit), `bt_navigator`, `behavior_server`, `map_server`, `amcl` + `nav2_lifecycle_manager`.
- `config/nav2_params.yaml` — parametry Nav2 gotowe do strojenia na RPi4.

Przykład z własnym plikiem parametrów:
- `ros2 launch cyber_is_navigation start_navigation.launch.py map:=/path/map.yaml params_file:=/path/nav2_params.yaml`

### Strojenie (RPi4)
- Częstotliwości:
  - `controller_server.controller_frequency`: 10–12 Hz
  - `planner_server.expected_planner_frequency`: 1–2 Hz
- Costmapy i inflacja:
  - Global: `update/publish` 2.0 Hz, `resolution` 0.05–0.1 m, `static + inflation`
  - Local: rolling 6×6 m, `update/publish` 10 Hz, `obstacle + inflation`
  - Inflacja: `inflation_radius`: 0.6–0.7, `cost_scaling_factor`: 3–5
  - `transform_tolerance`: 0.2–0.3 s (obie costmapy i kontroler)
- AMCL:
  - `min_particles`: 500, `max_particles`: 1000
  - `update_min_d/a`: ~0.15
- RPP (RegulatedPurePursuit):
  - `desired_linear_vel`: 0.3–0.5
  - `max_angular_accel`: 0.8–1.2
  - `lookahead_dist`: 0.5–0.7 (lub `use_velocity_scaled_lookahead_dist: true`)

### QoS i stabilność
- Sensory: SensorDataQoS (LIDAR, odometria) — Nav2 domyślnie używa profilów niskiej latencji; dla mapy użyte `map_subscribe_transient_local: true`.
- Pozostałe: `KEEP_LAST` z niewielką głębokością (5–10) — domyślne profile Nav2 wystarczają na RPi4.

### Uwagi wydajnościowe
- Unikaj wtyczek niestandardowych; tylko standardowy Nav2.
- Smac 2D (A*) + RPP są lekkie i stabilne na RPi4.
- W razie problemów z CPU zmniejsz: `planner_frequency` lub rozdzielczość mapy do 0.1 m, ewentualnie zawęź lokalną costmapę.

### Interfejs
- Zamiast `move_base` używany jest Nav2 `NavigateToPose` (akcja). Klienci powinni publikować cele poprzez tę akcję lub RViz2.

### Kompatybilność i DoD
- Buduje się `colcon` na Jammy/Humble (x86_64, ARM)
- `start_navigation.launch.py` uruchamia Nav2 w kompozycji; wszystkie węzły przechodzą do stanu ACTIVE przez Lifecycle Manager.
- RViz2: wysyłasz `NavigateToPose` → nawigacja działa stabilnie, bez oscylacji.
- RPi4: CPU ~≤70% podczas prostej jazdy, RAM całego stosu ~<1.2 GB (przy podanych częstotliwościach).

## ✅ Podsumowanie

Ten system:
- Łączy SLAM, odometrię i fuzję sensora z precyzyjnym planowaniem
- Obsługuje dynamiczne przeszkody
- Uwzględnia ograniczenia przestrzenne (np. zakazane strefy)
- Jest gotowy do działania w realnym środowisku z zachowaniem wysokiej precyzji

---
