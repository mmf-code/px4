PX4 + Gazebo Classic + ROS 2 Humble Offboard — Mülakat Notları (TR)

Bu doküman, PX4 SITL + Gazebo Classic + ROS 2 Humble + MAVROS2 ile hazırladığımız basit Offboard demosunu mülakatta rahatça anlatman için hazırlandı. Aşağıdaki akışla canlı demo yapabilir, nedenlerini ve olası sorunları açıklayabilirsin.

**Demo Mimarisi (Özet)**
- PX4 SITL: Gazebo Classic içinde iris modelini uçurur.
- MAVROS2: PX4’e MAVLink/UDP üzerinden köprü kurar.
- Offboard (ROS 2, Python): 20 Hz hız setpoint’i gönderir; OFFBOARD moda geçer, armar, kalkış–daire–iniş.

**Hızlı Çalıştırma (3 Terminal)**
- Terminal 1 — PX4 + Gazebo Classic
  - bash scripts/run_px4_sitl.sh classic
  - GUI açılmazsa: HEADLESS=1 bash scripts/run_px4_sitl.sh classic
- Terminal 2 — MAVROS (PX4 profili)
  - source /opt/ros/humble/setup.bash
  - ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 \
    pluginlists_yaml:=/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml \
    config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml
- Terminal 3 — Offboard
  - source /opt/ros/humble/setup.bash
  - source install/setup.bash
  - ros2 launch px4_offboard_demo offboard_demo.launch.py

Not: Tek launch alternatifi (MAVROS + Offboard birlikte):
- source /opt/ros/humble/setup.bash && source install/setup.bash
- ros2 launch px4_offboard_demo offboard_with_mavros.launch.py

**Canlı Gösterim Komutları (Hızlı Kanıtlar)**
- Mod/arm durumu: ros2 topic echo /mavros/state -n 1
- Pozisyon örneği: ros2 topic echo /mavros/local_position/pose -n 1
- Setpoint frekansı (20 Hz): ros2 topic hz /mavros/setpoint_velocity/cmd_vel
- OFFBOARD + ARM servisleri:
  - ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
  - ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
- MAVROS topikleri: ros2 topic list | rg mavros
- Kayıt almak (kanıt dosyası):
  - ros2 bag record /mavros/state /mavros/local_position/pose /mavros/setpoint_velocity/cmd_vel

**Beklenen Davranış ve Kanıtlar**
- Offboard node ~2 s setpoint akışı gönderir; OFFBOARD moda geçer ve arm eder.
- ~2.5 m kalkış; XY düzleminde daire (~30 s), ardından AUTO.LAND ile iniş.
- PX4 konsolunda: “Armed by external command”, “Takeoff detected”.
- MAVROS’ta: “CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot”.

**Teknik Noktalar (Kısa Anlatım)**
- OFFBOARD gereksinimi: PX4, OFFBOARD’a geçmeden önce >2 Hz setpoint akışı bekler; biz 20 Hz yayınlıyoruz.
- Referans çerçeveleri: MAVROS local pose ENU (z yukarı), PX4 NED; kod ENU kabul edilip +z hızla kalkış yapar.
- Portlar: PX4 SITL ilk instance MAVLink UDP 14540; MAVROS fcu_url:=udp://:14540@127.0.0.1:14557 ile bağlanır.
- QoS uyumu: /mavros/local_position/pose BEST_EFFORT yayın; aboneliği BEST_EFFORT QoS ile açtık.
- Zaman senkronu: MAVROS “TM: Timesync mode: MAVLINK”; yüksek RTT uyarıları sim/CPU yükünden, uçuşu engellemez.
- Güvenlik/failsafe: OFFBOARD setpoint kaybında davranış COM_OF_LOSS_T, COM_OBL_ACT ile ayarlanır (Hold/Land).

**Sık Sorunlar ve Çözümleri**
- GeographicLib dataset hatası (egm96-5.pgm):
  - sudo apt install -y geographiclib-tools
  - sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
- Gazebo (Sim) Wayland/EGL veya gz_bridge timeout:
  - Classic’e geç: bash scripts/run_px4_sitl.sh classic
  - Gerekirse headless: HEADLESS=1 ...
  - Wayland GUI için: export QT_QPA_PLATFORM=xcb veya sudo apt install -y qtwayland5
- rosdep ament_python şikâyeti:
  - Build betiği --skip-keys "ament_python ament_cmake" ile devam eder.
- QoS uyumsuzluğu uyarısı:
  - offboard_control.py içinde abonelik BEST_EFFORT QoS ile oluşturuldu.

**Kod/Dosya Referansları**
- Offboard node: ros2_ws/src/px4_offboard_demo/px4_offboard_demo/offboard_control.py:1
- Launch (yalnız offboard): ros2_ws/src/px4_offboard_demo/launch/offboard_demo.launch.py:1
- Launch (MAVROS + offboard): ros2_ws/src/px4_offboard_demo/launch/offboard_with_mavros.launch.py:1
- PX4 klon/derle: scripts/get_px4.sh:1
- SITL başlat (gz veya classic): scripts/run_px4_sitl.sh:1
- ROS 2 build + kontroller: scripts/build_ros2_ws.sh:1, scripts/check_ros2_humble.sh:1

**Neler Yapılabilir? (Genişletme Fikirleri)**
- uXRCE-DDS + px4_msgs ile MAVROS’suz yol (ROS 2 native bridge).
- rviz2 ile ENU poz/trajectory görselleştirme.
- Trajektori jeneratörü: daire yerine sekiz (lemniscate) veya waypoint takibi.
- Yaw/yükseklik profil kontrolü; PID/limit parametrelerini deneme.
- ros2 bag kayıtlarını otomatik alan script; tmux ile 3 terminali otomatik başlatma.

**SSS (Kısa Cevaplar)**
- Neden 20 Hz? OFFBOARD kabul için >2 Hz gerekir; jitter’a dayanıklı bir marj sağlar.
- Neden velocity setpoint? Basit/robust; pozisyon/trajectory’ye göre daha az tuning gerektirir.
- ENU/NED farkı? MAVROS local pose ENU; PX4 NED. Dikey eksen işareti kodda dikkate alındı.
- Setpoint kesilirse? PX4 parametrelerine göre Hold/Land gibi failsafe’e geçer.

**Özet Cümle (Kapanış)**
"PX4 SITL’i Gazebo Classic ile çalıştırıp MAVROS2 üzerinden UDP 14540’a bağlanıyorum. ROS 2 offboard node’um 20 Hz hız setpoint akışıyla OFFBOARD’a geçip armlıyor, ~2.5 m kalkış yapıp XY düzleminde daire çiziyor ve AUTO.LAND ile indiriyor. QoS (BEST_EFFORT), ENU/NED farkı, GeographicLib dataset ve Gazebo GUI sorunlarını çözebilecek fallback’ler hazır."

**Hızlı Başlangıç Anlatısı (30–60 sn)**
- Amaç: PX4’ü simülasyonda uçurup ROS 2 ile dışarıdan (offboard) kontrol etmek.
- Yapı: Gazebo Classic içinde PX4 SITL; MAVROS köprüsü; Python ROS 2 düğümü hız setpoint’i gönderiyor.
- Akış: 20 Hz setpoint → OFFBOARD moda geçme → arm → 2.5 m kalkış → XY düzleminde daire → AUTO.LAND.
- Kanıt: PX4 konsolunda “Armed by external command”, MAVROS’ta “Got HEARTBEAT”.

**Terimler Sözlüğü (Akılda Kalıcı)**
- PX4: Açık kaynak uçuş kontrol yazılımı. Donanımda çalışır; burada simülasyon (SITL).
- SITL (Software In The Loop): Uçuş kodu PC’de çalışır, sensör/aktüatör simüle edilir. Hızlı, güvenli.
- HIL (Hardware In The Loop): Gerçek donanım + simüle çevre. SIL: Yazılımın saf simülasyonu.
- Gazebo Classic vs (gz/Sim): Fizik ve sensör simülatörü. Classic kararlı, gz (Fortress/Garden) yeni nesil.
- MAVLink: Uçuş kontrolcüsü ile yer/companiona mesajlaşma protokolü. Hafif, binary çerçeveler.
- MAVROS: ROS 2 <-> MAVLink köprüsü. Konu/servis arayüzü sağlar (state, setpoint, komutlar).
- Offboard: Dış bir bilgisayar dümeni tutar. “OFFBOARD = Dış Beyin”. Sık setpoint gerekir (>2 Hz).
- Onboard: FCU iç kontrolü (PX4’ün kendi kontrollörleri). “ONBOARD = İç Beyin”.
- ENU vs NED: ENU (X doğu, Y kuzey, Z yukarı), NED (X kuzey, Y doğu, Z aşağı). Ezber: “ENu Up, NeD Down”.
- QoS (Quality of Service): ROS 2 mesaj güvenilirliği. Best Effort ~ UDP, Reliable ~ TCP gibi düşünebilirsin.
- Timesync: PX4-MAVROS saat senkronu. Yüksek RTT uyarı verir ama simde genelde kritik değil.
- GeographicLib: Coğrafi modeller (jeoid, manyetik). MAVROS bazı hesaplar için ister (egm96-5.pgm).
- uORB: PX4’ün dahili publish/subscribe sistemi. MAVLink ile dış dünyaya çıkar.
- uXRCE-DDS / px4_msgs: PX4’ün ROS 2’ye yerel (DDS) köprüsü. MAVROS’a alternatif yol.

**Demo Nasıl Çalışıyor? (Adım Adım)**
1) PX4 SITL başlar, iris modelini yükler, MAVLink UDP 14540’ı dinler.
2) MAVROS `fcu_url:=udp://:14540@127.0.0.1:14557` ile PX4’e bağlanır, konuları açar.
3) Offboard node 20 Hz hız setpoint’i yayınlar, 2 sn sonra OFFBOARD mode + arming ister.
4) Kalkışta +z (ENU) hızla ~2.5 m’ye çıkar, sonra XY düzleminde daire hareketi komutu gönderir.
5) Süre dolunca `AUTO.LAND` moda geçer ve iner.

**Kontrol Teorisi (PID ve PX4’teki Kademeler)**
- PID özet: P (hata oranı), I (bias giderir), D (trend/denge). Ezber: “P hızlı, I kalıcı, D sakinleştirir”.
- Anti-windup: Doyumda I integratör taşmasını sınırla (integrator clamp/back-calculation).
- Türev süzgeci: Ölçüm gürültüsüne karşı düşük geçiren filtre (derivative-on-measurement).
- Örnekleme zamanı: Sabit dt ile kararlı davranış. Gürültü yüksekse D’yi kıs, dt’yi artırma.
- PX4 kademeli kontrol: İçte rate (açısal hız), sonra attitude (açı), sonra velocity/position döngüsü.
  - Offboard velocity verirsen: PX4’ün içteki rate/attitude döngüleri çalışır, dış hız referansını takip eder.
  - Bu nedenle offboard tarafında PID tuning yerine referans üretimi basit kalır (robust): daire/merdiven vb.
- Tuning pratikleri: Önce P, sonra D ile aşımı azalt, en son I ile steady-state hatayı temizle.

**Güvenlik ve Failsafe**
- OFFBOARD gereksinimi: Setpoint akışı >2 Hz. Kesilirse OFFBOARD’tan çıkar.
- Parametreler: `COM_OF_LOSS_T` (kaç saniye), `COM_OBL_ACT` (Hold/Land vb.).
- Arming kontrolleri: Sensör ve ekf kontrolleri; simde otomatik sağlanır.

**Sık Sorulanlar (Hızlı Cevaplar)**
- Offboard vs Mission? Offboard canlı komut akışı ister; Mission önceden planlı, otopilot yürütür.
- Neden velocity setpoint? Basit ve sağlam; pozisyon setpoint’te tuning/derivatif daha hassas olabilir.
- MAVROS yerine px4_msgs? uXRCE-DDS ile ROS 2 native; fakat kurulum/uyumluluk ve dokümantasyon maliyeti var.
- ENU/NED farkını nasıl ele aldın? MAVROS local_pose ENU; +z yukarı kabul edilerek hız setpoint’i verildi.
- Timesync uyarıları? Simde normal; CPU yükünü azaltmak, GUI’yi kapatmak iyileştirir.

**Akılda Kalıcı İpuçları**
- OFFBOARD = “Outside Feeds Frequently” → Sık setpoint şart.
- ENu Up / NeD Down → Z işaretini şaşırma.
- QoS: Best Effort = “yakala gelirse”, Reliable = “garanti” (gecikme pahasına).

**Genişletme Fikirleri (Neler Yapılabilir?)**
- Trajektori: Sekiz (lemniscate), spiral, waypoint takip (nav_msgs/Path).
- Görselleştirme: rviz2’de /mavros/local_position/pose ve setpoint vektörleri.
- Güvenlik: Sanal çit (geofence) + hız/ivme saturasyonları.
- Planlama: A* / RRT ile basit yol planlayıcı ve hız profili üretimi.
- Algılama: YOLO/DeepSORT ile engel algıla, setpoint’i kaçınma vektörü ile düzelt.
- Kayıt/Analiz: ros2 bag ve PX4 ULog ile performans kıyaslama (izlenen hız vs istenen hız).

**Kapalı Çevrim PID Kontrol (Somut Katma Değer)**
- Ne ekledim: `offboard_pid` düğümü kapalı çevrim pozisyon hatasından hız setpoint üretir.
- Kazançlar/Limitler: YAML ile ayarlanır (config/pid_hover.yaml, pid_circle.yaml). Dinamik param güncellenebilir.
- Güvenlik: Geofence (x/y/z min-max), hız limitleri ve anti-windup integral sınırı.
- Metrix: Konsola periyodik ortalama XY hata (m) basılır; ros2 bag ile kayda uygundur.
- Çalıştırma: `ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=.../pid_circle.yaml`

**Mission/Waypoint (Otopilot Benzeri)**
- YAML waypoint modu: `config/waypoints_demo.yaml` ile sıralı hedefler (x,y,alt) izlenir (home’a göre ENU).
- Kabul yarıçapı ve bekleme: `wp_accept_radius`, `wp_hold_time`; döngüleme: `wp_loop`.
- Metrikler aynı; RViz ile hedef/iz yolları görülebilir.
- Başlatma: `ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=ros2_ws/src/px4_offboard_demo/config/waypoints_demo.yaml`

**Operasyon Kolaylığı**
- Tek komutta başlatma: `bash scripts/tmux_up.sh` (tmux ile 3 panel: SITL, MAVROS, PID)

**Savunma Sanayi Odaklı Yaklaşımlar (Baykar Perspektifi)**
- Emniyet: OFFBOARD kaybı failsafe (COM_OF_LOSS_T/COM_OBL_ACT); geofence; hız/ivme limitleri; mod geçiş koşulları.
- Yetkinlik: Kademeli kontrol mimarisi (inner rate/attitude PX4, outer velocity/position offboard). Tuning ve test.
- Operasyon: Kayıt/geri izleme (ULog + ros2 bag), performans metrikleri (RMS hata, settle time, overshoot).
- Entegrasyon: Mission/waypoint desteği, RTK/IMU entegrasyonu, keşif/engel kaçınma modülleri.
- Güvenilirlik: Network gecikmelerine tolerans (20 Hz akış), watchdog, yeniden bağlanma, parametre doğrulama.

**Yapay Zeka ile Neler Denenebilir?**
- Takviye Öğrenme (RL): Eylem = hız setpoint, ödül = hedefe yakınlık − ivme/hız cezaları.
- Taklit Öğrenme: İyi bir klasik denetleyicinin çıktılarından veri toplayıp taklit modeli eğitme.
- Görsel Navigasyon: Monoküler/derinlik kamerası ile öğrenilmiş iniş/engel kaçınma.
- Güvenli RL: Geofence ve saturasyon ile action-guard; simde domain randomization.

**Demo Kanıtları (Gösterirken Kullan)**
- State/arm: `ros2 topic echo /mavros/state -n 1`
- Pose örneği: `ros2 topic echo /mavros/local_position/pose -n 1`
- Setpoint hızı: `ros2 topic hz /mavros/setpoint_velocity/cmd_vel`
- Servisler: OFFBOARD/ARM çağrıları (mavros_msgs/srv/SetMode, CommandBool)
- Kayıt: `ros2 bag record` ile üç temel konu.

**Hata/Çözüm Hikâyeleri (Deneyim)**
- GeographicLib dataset yok → kurulum scripti ile düzeltildi.
- Gazebo (gz) Wayland/EGL & gz_bridge timeout → Classic/Headless’a düştüm.
- rosdep ‘ament_python’ anahtarı → `--skip-keys` ile build akışı bozulmadan devam.
- QoS uyumsuzluğu → BEST_EFFORT abonelikle düzeldi.

**Kısa “Pitch” (1–2 dk)**
“SITL üzerinde PX4’ü Gazebo Classic ile uçurup MAVROS’la 14540/UDP üzerinden ROS 2 ağına bağlıyorum. Python offboard düğümü 20 Hz hız setpoint’i akıtarak OFFBOARD moda geçiş ve arming yapıyor; 2.5 m kalkış sonrası XY düzleminde daire hareketi ve AUTO.LAND ile iniş. ENU/NED farkını ve QoS’i doğru yapılandırdım. Kurulum sorunlarını (GeographicLib, gz GUI, rosdep anahtarları) çözdüm. Bir sonraki adımda waypoint takip/rviz görselleştirme veya RL tabanlı referans üretimi ekleyebilirim.”

**Örnek Soru–Cevap (Pratik)**
- Offboard moda nasıl güvenilir geçilir?
  - 2–3 saniye boyunca >2 Hz setpoint akışı, sonra SetMode=OFFBOARD ve arming. Akış kesilirse PX4 offboard’tan çıkar.
- Neden velocity setpoint kullandın, pozisyon yerine?
  - PX4’ün içteki rate/attitude/velocity kademeleri stabil; dışarıda hız referansı üretmek daha az tuning ister ve robust’tur.
- ENU ve NED farkını nasıl ele aldın?
  - ENU: X-doğu, Y-kuzey, Z-yukarı; NED: X-kuzey, Y-doğu, Z-aşağı. Dönüşüm: x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu, yaw_enu≈-yaw_ned.
- MAVROS’ta QoS “incompatible QoS” uyarısı neden geldi?
  - MAVROS bazı sensör konularını Best Effort yayınlar. Aboneyi Best Effort QoS ile oluşturunca uyum sağlandı.
- Timesync RTT yüksek uyarıları neden olur?
  - Sim/GUI yükünden veya VM ortamından. Headless çalışmak, CPU yükünü düşürmek uyarıları azaltır; uçuşu engellemez.
- GeographicLib neden gerekli?
  - Jeoid ve manyetik modeller için. MAVROS çalışırken egm96-5.pgm gibi veri setlerine ihtiyaç duyar; script ile kurulur.
- Gazebo (gz) neden yerine Classic?
  - Wayland/EGL/gz_bridge sorunlarına karşı Classic daha stabil. Gerekirse gz için QT_QPA_PLATFORM=xcb denenir.
- Velocity gains nerede ayarlanır?
  - PX4’de velocity/position katmanı MPC_* parametreleri, attitude/rate katmanı MC_* parametreleridir (kademeli kontrol).
- Failsafe durumda ne olur?
  - `COM_OF_LOSS_T` süresi dolunca `COM_OBL_ACT`’e göre Hold/Land yapılır; demo bunu güvenli kılar.

**Demo Öncesi Kontrol Listesi**
- ROS 2 Humble source: `source /opt/ros/humble/setup.bash`
- Workspace source: `source install/setup.bash`
- MAVROS bağlı mı: `ros2 topic echo /mavros/state -n 1`
- PX4 heartbeat geliyor mu: MAVROS log’unda “Got HEARTBEAT, connected”
- Setpoint akışı: `ros2 topic hz /mavros/setpoint_velocity/cmd_vel` ~20 Hz
- Gerekirse kayıt: `ros2 bag record …`

**Hızlı Arıza Giderme**
- MAVROS GeographicLib hatası: install script ile dataset kur (egm96-5.pgm).
- “gz_bridge timeout” veya Wayland uyarıları: Classic/Headless çalıştır.
- rosdep ‘ament_python’ uyarısı: build betiği `--skip-keys` ile devam eder.
- QoS uyumsuzluğu: Best Effort abonelik kullan.

**Kısa Teori: PID Tuning İpucu**
- P: hızlı tepki, fazla olursa overshoot/sallanma.
- D: tepkiyi sönümler, gürültüye duyarlı → ölçümde low-pass.
- I: kalıcı hatayı giderir, doyumda anti-windup şart.
- Sıra: P → D ile osilasyon azalt → I ile bias temizle. Kademeli döngülerde iç döngüleri önce stabilize et.

**Yapay Zeka Fikirleri (Özet Cevaplar)**
- RL ile referans üretimi: Eylem hız setpoint, ödül hedef yakınlığı − ivme/hız cezaları; güvenlik için geofence/saturasyon.
- Taklit öğrenme: Klasik denetleyici çıktılarını toplayıp öğrenci ağa öğretmek, sonra sim2real için domain randomization.
- Görsel navigasyon: Kamera akışından iniş/engel kaçınma; simde etiketli veri üretimi ve ROS 2 pipeline.
**Zigzag + Geri İzleme + Uyarlama (İteratif Gelişim)**
- Zigzag poligon yolu (lawnmower): uzunluk, genişlik, adım (dx) ile üretilir; önce ileri, sonra tam ters yönde geri izlenir.
- Epizot: ileri + geri. Her epizot sonunda ortalama/maks XY hata hesaplanır ve `log/zigzag_metrics.csv`’ye kaydedilir.
- Uyarlama: Hedef hata `adapt.avg_err_target`’tan büyükse `limit.v_xy` azaltılır ve `pid_xy.kp` az miktar artırılır; küçükse hız artırılır.
- Amaç: hız/izleme hatası arasında denge; gösterimde “kendi kendini iyileştiren” bir yaklaşım.
- Başlatma: `ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=ros2_ws/src/px4_offboard_demo/config/zigzag.yaml`

**Deney Planı (Mülakat için kısa anlatı)**
- Amaç: Kapalı çevrim offboard ile üretilen referansın tur bazlı performansını görmek ve küçük uyarlamalarla hatayı iyileştirmek.
- Kurulum: PX4 SITL (Gazebo Classic) + MAVROS + ROS 2 PID düğümü. Hız limitleri, slew, geofence aktif.
- Prosedür: 10 epizot zigzag; her epizotta ortalama/maks hata ölç; hedefe göre `v_xy` ve `kp_xy`yi küçük adımlarla düzelt.
- Kayıt: CSV’ye (tarih/saat klasörleri) metrik yaz; gerekirse ros2 bag ile ek konular kaydet.
- Gözlem: `avg_err` trendi ve parametre değişimleri; stabiliteyi bozmadan iyileştirme.

**Kayıt & Analiz**
- Klasörleme: `log/zigzag/YYYY-MM-DD/HHMMSS/` ve `log/zigzag/latest` kısayolu.
- Dosyalar:
  - `zigzag_metrics.csv`: timestamp, episode, stage, avg_err, max_err, v_xy, kp_xy, points
  - `avg_err_5s.csv`: timestamp, avg_err_5s (zaman serisi)
- Hızlı bakış: `tail -f log/zigzag/latest/zigzag_metrics.csv`
- Excel/LibreOffice: `avg_err` düşüşü, `v_xy/kp` değişimleri ve korelasyon grafikleri.

**Sunum İpuçları**
- Mimarinin netliği: PX4 iç denetleyici + offboard rehberlik; güvenlik (geofence, hız limitleri), QoS/ENU-NED gibi pratikler.
- Ölçüm kültürü: Metriği kaydedip epizot bazlı iyileştirme; rasgele “tuning” yerine kontrollü küçük adımlar.
- Sonuç odaklı: Hedef hata, kaç turda hangi parametrelerde denge bulundu, stabilite bozulmadan iyileşme.
