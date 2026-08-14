<p align="center">
  <img width=400 src="doc/bievr_final.svg">
</p>

# BIEVR-SLAM

LiDAR-инерциальная одометрия, локализация по заранее построенной карте и конвертация
готовых облаков точек в формат этой карты. В основе представление карты, в котором
каждый воксель хранит ориентированное изображение высот (bump image): регистрация скана идёт напрямую по этим изображениям, поэтому слабые вариации геометрии в туннелях и других
малоинформативных сценах остаются заметными.

Ядро (`bievr_lio`) не зависит от ROS. Поверх него собирается ROS 2 интерфейс
(`bievr_lio_ros2`) и отдельная утилита конвертации карт. Проверено на Jazzy и Humble.

Алгоритм и исходная реализация: **BIEVR-LIO**, ETH Zurich ASL —
[статья (arXiv:2604.14421)](https://arxiv.org/abs/2604.14421),
[страница проекта](https://patripfr.github.io/bievr-lio/),
[видео](https://youtu.be/TsDJOdthhNk),
[исходный репозиторий](https://github.com/ethz-asl/BIEVR-LIO).

<p align="center">
  <img width='100%' src="doc/tunnel_detail.png">
</p>

## Что входит в пакет

| Сценарий | Исполняемый файл | Результат |
| --- | --- | --- |
| **Маппинг** | `process_bag`, `process_topics` (пакет `bievr_lio_ros2`) | траектория в формате TUM, `map.bumpmap`, `map.pcd`, при необходимости - накопленное облако сырых сканов |
| **Локализация** | те же две ноды, режим переключается конфигом (`map.load_path` + `map.update: false`) | траектория в системе координат загруженной карты |
| **Конвертация карты** | `bumpmap_from_pcd` (пакет `bievr_lio`) | `map.bumpmap` + `map.pcd` из произвольного накопленного облака (PCD/PLY) |



## Установка

### Docker

Быстрый вариант, если не нужно собирать зависимости на хосте:

```bash
cd docker/
./run_docker_ros2.sh -b
```

Флаг `-b` собирает образ, при последующих запусках его можно опустить. Каталог `~/data`
хоста монтируется в `/home/bievr/data` внутри контейнера. 
Образ собирается из текущей рабочей копии репозитория, поэтому правки в дереве попадают в образ
при пересборке (флаг `-b`).

### Сборка на хосте

Требуется [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) и
`python3-colcon-common-extensions`.

Зависимости: [Eigen](https://eigen.tuxfamily.org),
[Ceres](http://ceres-solver.org) 2.2.0, TBB, glog. Утилите `bumpmap_from_pcd`
дополнительно нужны PCL (`common`, `io`) и yaml-cpp; если PCL в системе нет, сборку
утилиты можно отключить: `--cmake-args -DBIEVR_BUILD_TOOLS=OFF`.

Ceres ставится скриптом (собирает 2.2.0 из исходников):

```bash
./BIEVR-SLAM/docker/scripts/install_ceres.sh
```

Поддержка Livox `CustomMsg` компилируется только если `livox_ros_driver2` найден в
workspace на момент сборки; без него всё собирается и работает с обычным
`sensor_msgs/msg/PointCloud2`. Если Livox нужен, клонируйте и соберите
[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) вместе с
[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) **до** сборки BIEVR.

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to bievr_lio_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Собираются три пакета: `bievr_lio` (ядро + `bumpmap_from_pcd`), `bievr_ros_common`
(header-only конвертации и публикация) и `bievr_lio_ros2` (ноды и launch-файлы).


### Готовые конфиги

| Конфиг | Данные |
| --- | --- |
| `nora` | Наш бэг |
| `gamma`, `geode`, `geode_alpha` | [GEODE](https://thisparticle.github.io/geode), устройства γ (Livox Avia) и α |
| `enwide` | [ENWIDE](https://projects.asl.ethz.ch/datasets/enwide/) |
| `ncd` | [Newer College Dataset](https://drive.google.com/drive/u/0/folders/1uR476FzjN3PfAiCknVKtuZi3_QfVvSdA) |
| `mars` | [MARS-LVIG](https://mars.hku.hk/dataset.html) |
| `grandtour` | [GrandTour](https://grand-tour.leggedrobotics.com/) |

## Запуск

Две ноды, обе принимают одни и те же конфиги:

- **`process_topics`** - подписывается на топики LiDAR и IMU и обрабатывает сообщения по
  мере поступления. Для live сенсоров.
- **`process_bag`** - читает bag напрямую и прогоняет его настолько быстро, насколько
  позволяет железо (без DDS и потерь сообщений). Предпочтительный вариант для
  офлайн-обработки.

### Маппинг

```bash
# топики
ros2 launch bievr_lio_ros2 process_topics.launch.py sensor_config:=<sensor_config>

# бэг
ros2 launch bievr_lio_ros2 process_bag.launch.py \
  sensor_config:=<sensor_config> rosbag:=/path/to/bag_dir
```
Добавьте `rviz:=true` для визуализации.

### Локализация в готовой карте

Карта загружается с диска, её обновление выключается:

```yaml
map:
  load_path: "/path/to/mine.bumpmap"
  update: False                          # ни один скан не интегрируется
  initial_pose: [0, 0, 0, 0, 0, 0, 1]    # опционально; [x, y, z, qx, qy, qz, qw]

debug:
  publish_map_stride: 10                 # показать карту в RViz
```

| Ключ | Назначение |
| --- | --- |
| `map.load_path` | Путь к `.bumpmap`.  |
| `map.update` | `False` - карта заморожена: новые точки не интегрируются |
| `map.initial_pose` | Стартовая поза `T_W_I` **в системе координат загруженной карты**, порядок TUM (`w` последним). Без неё старт из начала координат с ориентацией по силе тяжести - сработает только если бэг локализации и маппинга совпадают |
| `debug.publish_map_stride` | Публикация загруженной карты в `points/map` (latched, каждая N-я точка) |


### Конвертация облака в карту

`bumpmap_from_pcd` строит `.bumpmap` из уже готового облака точек (например. карты из другого SLAM-алгоритма). 

```bash
./install/bievr_lio/bin/bumpmap_from_pcd \
  --input /path/to/cloud.pcd \
  --output-dir /path/to/out \
  --config config/params.yaml \
  --sensor-config config/sensor_configs/<name>.yaml
```

| Аргумент | Назначение |
| --- | --- |
| `--input PATH` | Входное облако, PCD (ascii / binary / binary_compressed) или PLY. Можно указать несколько раз для мерджа нескольких облаков |
| `--output-dir DIR` | Каталог результата: `map.bumpmap`, `map.pcd` и `.run_config/sensor.yaml` |
| `--config PATH` | `params.yaml`; `map.voxel_size_m` и `map.pixel_size_m` отсюда задают геометрию выходной карты |
| `--sensor-config PATH` | Строго говоря не нужен, но полезно для оверрайдов |
| `--override K=V` | Точечное переопределение по составному ключу, например `map.voxel_size_m=0.25`. Повторяемый, значение разбирается как YAML. Добавлено, чтобы прогонять подборы параметрво в скриптах |
| `--chunk-size N` | Число точек на один вызов интеграции (по умолчанию 2000000) |
| `--stride N` | Брать каждую N-ю точку после конкатенации — для быстрых прикидок |

Выходной каталог по структуре совпадает с результатом обычного прогона маппинга,
поэтому все остальные инструменты работают с ним без изменений.


## Конфигурация

Конфигурация разнесена по двум YAML-файлам, оба читаются напрямую через yaml-cpp (а не
через параметры ROS):

- **`config/params.yaml`** - параметры алгоритма (разрешение карты, семплирование,
  оптимизация, инерциальное окно). Не зависят от набора данных, значения по умолчанию
  проверены на разных сенсорах и платформах.
- **`config/sensor_configs/<name>.yaml`** - параметры конкретного набора сенсоров: имена
  топиков, калибровка LiDAR→IMU, фильтрация по дальности + оверрайды для `params.yaml`. 

<details>
  <summary>Описание параметров</summary>

  ### Настройки сенсоров

| Ключ | Назначение |
| --- | --- |
| `topics.pointcloud` / `topics.imu` | Топики облака и IMU |
| `calibration.translation` / `.rotation` | `T_IMU_LIDAR` (LiDAR → IMU): смещение и матрица поворота 3×3 построчно |
| `lidar.min_range_m` / `lidar.max_range_m` | Рабочий диапазон дальности |

### Параметры алгоритма

| Ключ | По умолчанию | Назначение |
| --- | --- | --- |
| `map.pixel_size_m` | 0.05 | Сторона пикселя bump-изображения [м] |
| `map.voxel_size_m` | 0.5 | Сторона вокселя [м] |
| `map.normal_tolerance_deg` | 3 | Порог изменения нормали, после которого содержимое вокселя пересчитывается |
| `map.smooth` / `map.weighted` | — | Сглаживание изображения вокселя; взвешенное обновление пикселей |
| `map.max_size` | 5000000 | Максимальное число вокселей в карте (LRU-вытеснение) |
| `map.frame` | `odom` | Родительская система координат публикуемых поз и облаков |
| `preprocess.downsample_resolution_m` | 0.15 | Разрешение прореживания входного скана [м] |
| `preprocess.informed_sampling` | false | Отбор точек по «информативности» вокселей вместо равномерного прореживания |
| `preprocess.informed_sample_count` | 300 | Сколько вокселей остаётся в полном разрешении при `informed_sampling` |
| `optimization.huber_delta` | 100 | Порог функции Хубера в регистрации |
| `optimization.img_residual` / `.img_jacobian` | true | Использовать bump-изображение в невязке и в якобиане |
| `imu.window_s` | 10 | Длина инерциального окна оптимизации [с] |
| `imu.t_init` | 0.2 | Время оценки смещений и вектора силы тяжести на старте [с] |
| `imu.normalized` | -1 | Единицы акселерометра: `<0` — автоопределение, `0` — м/с², `>0` — g (множитель) |
| `imu.frame` | — | Дочерняя система координат публикуемой одометрии |
| `max_num_threads` | 0 | 0 — по числу ядер |

`config/params.yaml` часть из параметров переопределяет. 

`pixel_size_m` — основной параметр по разрешению/памяти. Оптимальное значение зависит от
плотности исходного облака: на плотной карте, построенной самим BIEVR, выигрывает
мелкий пиксель (0.025–0.05), на разреженной сконвертированной — крупный (0.1). Значение
`preprocess.downsample_resolution_m` следует менять вместе с ним. `voxel_size_m: 0.5`
устойчиво и менять его обычно не требуется.

### Отладочные параметры

| Ключ | Назначение |
| --- | --- |
| `debug.trajectory_path` | Директория для сохранения траектории в формате TUM (`t x y z qx qy qz qw`) |
| `debug.map_save_path` | Директория для сохранения `<path>.pcd` и `<path>.bumpmap` |
| `debug.accumulated_map_save_path` | Директория для сохранения `<path>.pcd`: объединение сырых сканов лидара вдоль траектории. Существенно больше по памяти |
| `debug.accumulated_map_leaf_m` | Размер вокселя для него (по умолчанию 0.05); `<= 0` — сохранять все точки |
| `debug.diagnostics_path` | Директория для сохранения диагностики солвера в CSV, по строке на каждый скан |
| `debug.publish_map_stride` | Если стоит, публикует разово для визуализации загруженную карту в `points/map` (каждую N-ю точку для экономии ресурсов); `0` — выключено |
| `debug.publish_all_clouds` | Публиковать промежуточные облака (`points/fine`, `points/coarse`, `points/effective`, `points/undistorted`) |
| `debug.timing` / `debug.log` | Тайминги и подробный лог |
| `debug.dashboard` / `debug.dashboard_ascii_path` | Живой статус в консоли (позиция, смещения, тайминги) |

Пустая строка в любом из путей означает «не сохранять».


</details>




## ROS-интерфейс

Все топики публикуются в пространстве имён `bievr_lio`.

| Топик | Тип | Условие |
| --- | --- | --- |
| `/bievr_lio/odom` | `nav_msgs/msg/Odometry` | всегда; `map.frame` → `imu.frame` |
| `/bievr_lio/points/registered` | `sensor_msgs/msg/PointCloud2` | всегда |
| `/bievr_lio/bias/acc`, `/bievr_lio/bias/gyro` | `geometry_msgs/msg/Vector3Stamped` | всегда |
| `/bievr_lio/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | всегда; те же величины, что и в CSV |
| `/bievr_lio/points/map` | `sensor_msgs/msg/PointCloud2` | `debug.publish_map_stride > 0`, один раз, latched |
| `/bievr_lio/points/fine`, `points/coarse`, `points/effective`, `points/undistorted` | `sensor_msgs/msg/PointCloud2` | `debug.publish_all_clouds` |

Дополнительно публикуется TF `map.frame` → `imu.frame`.

## Диагностика

При заданном `debug.diagnostics_path` пишется CSV по строке на скан. Те же величины
уходят в `/bievr_lio/diagnostics`. CSV остаётся основным источником при офлайн-прогонах, так как они идутт быстрее, чем любой подписчик успевает читать.


<details>
  <summary>Описание строчек в csv</summary>

| Колонка | Смысл |
| --- | --- |
| `t` | Временная метка скана |
| `effective_points`, `downsampled_points`, `ratio` | Точек, вошедших в решение; точек после прореживания; их отношение |
| `residual` | Средний модуль point-to-plane residual. **`-1`, если ни одна точка не вошла в решение**, — потеря захвата|
| `inlier_points`, `no_correspondence` | Инлайеры и точки, для которых не нашлось соответствия |
| `huber_cost`, `iterations`, `converged`, `lm_lambda` | Состояние солвера |
| `lambda_min_6`, `kappa_6`, `lambda_min_3`, `kappa_3` | Минимальное собственное число и число определённости информационной матрицы (полная 6х6 и её T блок 3х3) |
| `speed`, `pos_*`, `roll`, `pitch`, `yaw` | Состояние оценки |

</details>


### Утилиты

| Скрипт | Назначение |
| --- | --- |
| `scripts/load_bumpmap.py MAP` | Методы для парсинга bievr-карт: `load_bumpmap()` для небольших карт, `iter_bumpmap()` для потокового чтения больших |
| `scripts/compare_bumpmap.py A B` | Сравнение двух карт по ключам вокселей. Код возврата 1 при любом различии |
| `scripts/view_map.py PATH` | Визуализация `.bumpmap` или `.pcd`: режимы `quads` (каждый пиксель — реальный квадрат в плоскости своего вокселя), `points`, `patches`; раскраска `bump`/`bump_raw`/`height`/`weight`, `--crop X,Y,Z,R`, выгрузка в `.ply` через `--save`. Требует Open3D |

## Благодарности

Алгоритм и исходная реализация — [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO),
ETH Zurich Autonomous Systems Lab. Авторы благодарят за открытые публикации
[DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry),
[Wavemap](https://github.com/ethz-asl/wavemap) и
[UGPM](https://github.com/UTS-RI/ugpm), послужившие источником идей, а также
[ascii-image-converter](https://github.com/TheZoraiz/ascii-image-converter).


```bibtex
@article{pfreundschuh2026bievr,
  title        = {BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps},
  author       = {Pfreundschuh, Patrick and Tuna, Turcan and {Le Gentil}, Cedric and Siegwart, Roland and Cadena, Cesar and Oleynikova, Helen},
  year         = 2026,
  journal      = {Robotics: Science and Systems},
}
```

## Лицензия

BSD-3-Clause, см. [LICENSE](LICENSE).
