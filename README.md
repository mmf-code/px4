# PX4 + Gazebo + ROS 2 Humble Offboard Demo (TR)

Bu depo, Baykar staj/mülakatı için hızlı bir PX4 SITL + Gazebo + ROS 2 (Humble) offboard kontrol demosu içerir. Üç terminal ile simülasyonu açıp MAVROS2 üzerinden offboard komutları gönderirsiniz.

## Genel Bakış
- PX4 SITL: Gazebo (sim) içinde X500 quad.
- MAVROS2: PX4’e MAVLink üzerinden bağlanır.
- Offboard Node (ROS 2, Python): Kalkış, daire çizme ve iniş.

## Gereksinimler
- Ubuntu 22.04 (önerilir), ROS 2 Humble
- Gazebo (Sim) veya Gazebo Classic; PX4 SITL bunlardan biriyle çalışır
- Git, colcon, Python 3.10+

ROS 2 Humble ve MAVROS2 kurulumu:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
  ros-humble-mavros ros-humble-mavros-extras
# (İlk kurulum ise)
sudo rosdep init || true
rosdep update
```

Gazebo: PX4, Gazebo Classic 11 veya Gazebo (Sim) ile çalışır. Humble için Gazebo (Fortress/Garden) veya Classic kullanılabilir. Yoksa PX4 betiği gerekli paketleri işaret edecektir.

## 0) ROS 2 Humble kurulu mu? (hızlı test)
Kurulumdan önce durumunu test etmek için:

```bash
bash scripts/check_ros2_humble.sh
```

Tüm kontroller [OK] ise devam edin. Eksik bileşenler için betik önerdiği `apt` komutlarını gösterir.

## 1) PX4 SITL’ı getirme ve çalıştırma
Bu dizinde scriptler altına bir yardımcı ekledim.

```bash
# PX4 kodunu klonla ve SITL bağımlılıklarını kur + derle
bash scripts/get_px4.sh

# SITL + Gazebo’yu başlat (X500 quad)
bash scripts/run_px4_sitl.sh

# Eğer Gazebo (Sim) GUI/bridge hata verirse (Wayland/EGL, gz_bridge timeout),
# Classic'e zorla ya da headless çalıştır:
# - Classic GUI:     bash scripts/run_px4_sitl.sh classic
# - Classic headless:HEADLESS=1 bash scripts/run_px4_sitl.sh classic
# Wayland ortamında GUI için: export QT_QPA_PLATFORM=xcb veya `sudo apt install -y qtwayland5`
```

Başarılıysa bir Gazebo penceresi ve PX4 konsolu açılır. PX4 ilk instance UDP 14540 portunu dinler (MAVROS buna bağlanacak).

## 2) ROS 2 workspace’i derleme
ROS 2 offboard paketimiz `ros2_ws/src/px4_offboard_demo` içinde.

```bash
# ROS 2 Humble ortamını yükle
source /opt/ros/humble/setup.bash

# Bağımlılıkları çöz (mavros_msgs vb.)
rosdep install --rosdistro humble --from-paths ros2_ws/src -y --ignore-src

# Derle ve ortamı yükle
colcon build --base-paths ros2_ws
# Not: İlk derlemede install klasörü proje köküne oluştuysa 'source install/setup.bash' yapın.
source ros2_ws/install/setup.bash || source install/setup.bash
```

## 3) MAVROS2 ve Offboard düğümünü çalıştırma
İki ayrı terminal açın (veya tek bir launch dosyası kullanın):

Seçenek A — Ayrı başlatma:

```bash
# Terminal A: MAVROS2 (PX4 profiliyle)
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14557 \
  pluginlists_yaml:=/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml \
  config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml
```

```bash
# Terminal B: Offboard node
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch px4_offboard_demo offboard_demo.launch.py
```

Seçenek B — Tek launch:

```bash
# MAVROS + Offboard tek seferde (paket, sisteminizdeki MAVROS sürümüne göre
# otomatik olarak mavros_container.launch.py ya da px4.launch kullanır)
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch px4_offboard_demo offboard_with_mavros.launch.py
```

## 4) PID tabanlı kapalı çevrim (Baykar odaklı)
Basit açık çevrim hız komutları yerine, yerel ENU pozisyonuna göre PID ile kapalı çevrim hız setpoint üreten bir düğüm eklendi. Geofence ve hız limitleri içerir.

```bash
# PID hover (sabit XY, hedef irtifa)
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=ros2_ws/src/px4_offboard_demo/config/pid_hover.yaml

# PID circle (XY daire, hedef irtifa)
ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=ros2_ws/src/px4_offboard_demo/config/pid_circle.yaml

# Waypoint görevi (otopilot benzeri mission)
ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=ros2_ws/src/px4_offboard_demo/config/waypoints_demo.yaml

# Zigzag + geri izleme + uyarlama (tekrar eden görev)
ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=ros2_ws/src/px4_offboard_demo/config/zigzag.yaml
```

Parametreler (örnek): PID kazançları, hız limitleri, geofence kutusu ve daire yarıçap/periyodu. Çalışma sırasında dinamik parametre güncellenebilir:

```bash
ros2 param set /px4_offboard_demo pid_xy.kp 1.0
ros2 param set /px4_offboard_demo target_alt 3.0
```

Metrix: Düğümler log’da periyodik olarak ortalama XY takip hatasını (m) yazdırır.
Zigzag modunda her epizot sonunda ortalama ve maksimum hata; hız ve kp güncellemeleri CSV’ye yazılır: `log/zigzag_metrics.csv`.

## 6) Zigzag + Uyarlama (tur bazlı ölçüm)
Zigzag (lawnmower) üretilir, ileri ve geri izlenir (epizot). Her epizot sonunda hata ölçülür ve küçük adımlarla hız/kp ayarlanır. Tur sayısını ve bitiş davranışını kontrol edebilirsiniz.

Örnek (10 tur, bittiğinde iniş):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_offboard_demo pid_mission.launch.py \
  params_file:=ros2_ws/src/px4_offboard_demo/config/zigzag.yaml \
  zigzag.loops:=10 zigzag.end_action:=land
```

Çıktılar (tarih/saat klasörleri):
- `log/zigzag/YYYY-MM-DD/HHMMSS/zigzag_metrics.csv` — epizot özeti (timestamp, episode, stage, avg_err, max_err, v_xy, kp_xy, points)
- `log/zigzag/YYYY-MM-DD/HHMMSS/avg_err_5s.csv` — 5 sn ortalama hata zaman serisi
- `log/zigzag/latest` — son oturuma kısayol

Hızlı kontrol:
- `ros2 topic echo /offboard_demo/avg_err_5s`
- `ros2 topic echo /offboard_demo/episode_summary`
- `tail -f log/zigzag/latest/zigzag_metrics.csv`

Notlar:
- `zigzag.end_action`: `hover` (varsayılan) | `land` | `hold`
- Dinamik ayar örneği: `ros2 param set /px4_offboard_demo adapt.avg_err_target 1.0`

## 5) RViz görselleştirme (opsiyonel)
RViz2 kurulu değilse:

```bash
sudo apt install -y ros-humble-rviz2
```

Görselleştirme:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_offboard_demo viz.launch.py \
  params_file:=ros2_ws/src/px4_offboard_demo/config/pid_circle.yaml
```

RViz hazır display’ler:
- /offboard_demo/current_path (yeşil)
- /offboard_demo/target_path (kırmızı)
- /offboard_demo/marker (hedef çizgisi)

## Beklenen Davranış
- Offboard node, bağlantıyı bekler, ~2 s setpoint gönderir, `OFFBOARD` moda alır, armar.
- ~2.5 m’ye yükselir, sonra yatayda daire çizerek ~30 sn uçar.
- Sonra `AUTO.LAND` ile iner.

PX4 konsolunda moda geçişleri ve MAVROS’ta bağlantı logları görülür.


## Dizin Yapısı
- scripts/get_px4.sh: PX4 klonlama/derleme + bağımlılıklar
- scripts/run_px4_sitl.sh: PX4 SITL + Gazebo başlatma
- ros2_ws/src/px4_offboard_demo: ROS 2 offboard paketi (Python)

Başarılar — ihtiyaç olursa tek komutla video kaydı/animasyon için ek script de ekleyebilirim.
