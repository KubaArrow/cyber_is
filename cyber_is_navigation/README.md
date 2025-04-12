## 🧭 README – System Nawigacji z Move Base, GlobalPlanner i TebLocalPlanner

### 📦 Wymagane pakiety (ROS Noetic)

Zainstaluj wszystkie potrzebne paczki do działania systemu:

```bash
sudo apt update
sudo apt install ros-noetic-move-base \
                 ros-noetic-teb-local-planner \
                 ros-noetic-global-planner \
                 ros-noetic-robot-localization \
                 ros-noetic-map-server \
                 ros-noetic-costmap-2d
```

Jeśli korzystasz z `catkin_ws`, nie zapomnij zbudować:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🗂️ Struktura projektu

```
your_package/
├── launch/
│   └── move_base.launch                 # Główne uruchomienie move_base
├── config/
│   ├── costmap_common.yaml             # Wspólna konfiguracja dla globalnej i lokalnej mapy
│   ├── global_costmap.yaml             # Costmapa globalna (mapa statyczna + przeszkody + ograniczenia)
│   ├── local_costmap.yaml              # Costmapa lokalna (dynamiczna, wokół robota)
│   ├── global_planner.yaml             # Parametry globalnego planera (A*)
│   └── teb_local_planner.yaml          # Parametry TEB lokalnego planera
```

---

## 🚀 Uruchamianie

Upewnij się, że masz odpalone:
- `roscore`
- `robot_localization` (EKF)
- `hector_slam`
- `map_server` jeśli używasz warstwy ograniczeń

Następnie uruchom:

```bash
roslaunch your_package move_base.launch
```

---

## 🧠 Opis plików konfiguracyjnych

### 📁 `costmap_common.yaml`

Zawiera wspólne parametry dla obu map (globalnej i lokalnej):

| Parametr | Opis |
|---------|------|
| `obstacle_range` | Zasięg detekcji przeszkód (w metrach) |
| `raytrace_range` | Zasięg promienia do czyszczenia przeszkód |
| `robot_radius` | Promień robota |
| `inflation_radius` | Promień rozszerzania przeszkód |
| `plugins` | Lista aktywnych warstw w costmapie |

### 📁 `global_costmap.yaml`

Mapa wykorzystywana do **planowania ścieżki globalnej**:

| Parametr | Opis |
|----------|------|
| `global_frame` | Rama odniesienia – zwykle `map` |
| `static_map` | Czy używać statycznej mapy (np. z SLAMa) |
| `rolling_window` | `false` – mapa nie podąża za robotem |
| `plugins` | Warstwy: `static`, `obstacle`, `inflation`, `restriction` |

### 📁 `local_costmap.yaml`

Służy do lokalnego omijania przeszkód blisko robota:

| Parametr | Opis |
|----------|------|
| `global_frame` | Zwykle `odom` |
| `rolling_window` | `true` – mapa porusza się z robotem |
| `width`, `height` | Rozmiar okna lokalnej mapy |
| `resolution` | Rozdzielczość (im mniejsza, tym dokładniej) |

---

### 📁 `global_planner.yaml`

Ustawienia dla `global_planner/GlobalPlanner`:

| Parametr | Opis |
|----------|------|
| `use_dijkstra` | `false` = A\* |
| `use_quadratic` | Lepsze wyznaczanie kosztów ruchu |
| `use_grid_path` | Czy uprościć trasę do siatki |
| `orientation_mode` | Kierunek końcowy robota |
| `orientation_window_size` | Wielkość okna do oceny orientacji |

---

### 📁 `teb_local_planner.yaml`

Konfiguracja dla lokalnego planera TEB:

#### 🔸 Ograniczenia ruchu:
| Parametr | Opis |
|----------|------|
| `max_vel_x` | Max prędkość do przodu |
| `max_vel_x_backwards` | Max do tyłu |
| `max_vel_theta` | Max prędkość kątowa (tu: `1/8 rad/s`) |
| `acc_lim_x`, `acc_lim_theta` | Maksymalne przyspieszenia |

#### 🔸 Trajektoria:
| Parametr | Opis |
|----------|------|
| `teb_autosize` | Dynamicznie dobiera długość trajektorii |
| `dt_ref` | Czas między punktami trajektorii |
| `max_global_plan_lookahead_dist` | Jak daleko patrzy na globalną ścieżkę |

#### 🔸 Przeszkody:
| Parametr | Opis |
|----------|------|
| `min_obstacle_dist` | Minimalny dystans od przeszkód |
| `inflation_dist` | Dystans buforowy wokół przeszkód |
| `include_costmap_obstacles` | Czy brać przeszkody z costmapy |

#### 🔸 Inne:
| Parametr | Opis |
|----------|------|
| `xy_goal_tolerance` | Dokładność dojechania do celu XY |
| `yaw_goal_tolerance` | Dokładność obrotu w celu |
| `enable_multithreading` | Wydajniejsze obliczenia |

---

### 📁 Warstwa ograniczeń (`restriction_layer`)

Warstwa ta ładowana jest z osobnej mapy lub może być częścią SLAM-a. Użyj `map_server`:

```bash
rosrun map_server map_server restricted_map.yaml _frame_id:=map
```

W plikach costmap dodajesz ją jako:

```yaml
- {name: restriction_layer, type: "costmap_2d::StaticLayer"}
```

---

## ✅ Podsumowanie

Ten system:
- Łączy SLAM, odometrię i fuzję sensora z precyzyjnym planowaniem
- Obsługuje dynamiczne przeszkody
- Uwzględnia ograniczenia przestrzenne (np. zakazane strefy)
- Jest gotowy do działania w realnym środowisku z zachowaniem wysokiej precyzji

---
