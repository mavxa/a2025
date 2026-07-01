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
world/                          распакованный Gazebo world и модели
scripts/energy_relay_qual.py    автономная миссия
scripts/install_world.sh        добавление станций в штатный Clover world
scripts/restore_world.sh        откат стандартного clover_aruco.world
```

## Подготовка Мира

Папка мира должна называться `world/`. Кириллическое имя `Мир/` лучше не использовать: в shell/VM оно легко ломает пути.

Установить модели станций и добавить их в штатный `clover_aruco.world`:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/catkin_ws/install/setup.bash
scripts/install_world.sh
```

Важно: ArUco-карту `cmit.txt` и штатную модель `aruco_cmit_txt` не меняем. В этом образе используется стандартная CMIT-карта `10x10`, и навигацию выполняем в её `aruco_map`-frame. Скрипт добавляет только две станции: красная на marker `8` `(8, 9)`, зелёная на marker `33` `(3, 6)`.

Скрипт создаёт backup стандартного мира:

```text
clover_aruco.world.before_a2025.bak
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
python3 scripts/energy_relay_qual.py
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
- Терминал с запуском `energy_relay_qual.py`.
- Взлёт с синей LED-индикацией.
- Полёт мониторинга с `rainbow`.
- Вывод `red` и `green` в терминал.
- Переключение LED на цвет станции.
- Посадку на зелёную станцию.

Видео по регламенту должно быть не длиннее 4 минут.

## Настройки

Координаты станций соответствуют активной CMIT-карте `10x10`:

```text
red marker 8:     x=8.0 y=9.0
green marker 33:  x=3.0 y=6.0
```
