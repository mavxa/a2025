# А2025 Энергоэстафета

Минимальное решение для отборочного задания без YOLO и без датасета. Цвета зарядных станций распознаются через OpenCV HSV-маски по камере Clover.

## Что Делает Скрипт

- Взлетает с синей светодиодной индикацией.
- Включает `rainbow` во время мониторинга.
- Летит к станции на метке `8`, распознаёт цвет и печатает `red`.
- Летит к станции на метке `33`, распознаёт цвет и печатает `green`.
- При распознавании включает LED цветом найденной станции.
- Садится на зелёную зарядную станцию.

## Файлы

```text
world/                          распакованный Gazebo world, модели и карта 7x7
world/resources/maps/a2025_aruco_7x7.txt карта ArUco 7x7 для Clover
scripts/main.py                 автономная миссия
scripts/install_world.sh        установка мира, моделей и карты 7x7
scripts/restore_world.sh        откат стандартного clover_aruco.world
```

## Подготовка Мира

Папка мира должна называться `world/`. Кириллическое имя `Мир/` лучше не использовать: в shell/VM оно легко ломает пути.

Установить мир, модели и карту ArUco `7x7`:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/catkin_ws/install/setup.bash
scripts/install_world.sh
```

Важно: задание использует поле ArUco `7x7` с маркерами `0..48`. Скрипт ставит одинаковую карту `7x7` в `~/catkin_ws/src/clover/aruco_pose/map/map.txt` и `cmit.txt`, чтобы `aruco_map` совпадал с выданным миром.

Скрипт создаёт backup стандартного мира:

```text
clover_aruco.world.before_a2025.bak
map.txt.before_a2025.bak
cmit.txt.before_a2025.bak
```

Откатить мир после записи видео:

```bash
scripts/restore_world.sh
```

## Запуск Симуляции

В первом терминале:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/catkin_ws/install/setup.bash
roslaunch clover_simulation simulator.launch
```

Если в вашей VM другой launch-файл Clover, используйте его. Важно, чтобы работали сервисы:

```bash
rosservice list | grep -E 'navigate|get_telemetry|land|led/set_effect'
```

## Запуск Миссии

Во втором терминале:

```bash
cd ~/zed/a2025
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/catkin_ws/install/setup.bash
python3 scripts/main.py
```

Ожидаемый вывод в терминале:

```text
Mission started
Frame: aruco_map
Route: marker 8 -> marker 33 -> land on green
red
green
Mission finished
red_station=red
green_station=green
```

## Что Снимать На Видео

- Окно Gazebo с миром, красной и зелёной станцией.
- Терминал с запуском `main.py`.
- Взлёт с синей LED-индикацией.
- Полёт мониторинга с `rainbow`.
- Вывод `red` и `green` в терминал.
- Переключение LED на цвет станции.
- Посадку на зелёную станцию.

Видео по регламенту должно быть не длиннее 4 минут.

## Настройки

Координаты станций соответствуют карте ArUco `7x7`:

```text
red marker 8:     x=1.0 y=5.0
green marker 33:  x=5.0 y=2.0
```
