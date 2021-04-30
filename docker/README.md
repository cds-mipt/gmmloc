# Инструкция по запуску

Собрать образ
```bash
bash build.sh
```

Запустить контейнер
```bash
bash start.sh
```

Войти внутрь контейнера
```bash
bash into.sh
```

Подготовить рабочее пространство
```bash
cd catkin_ws/
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
cd src/
wstool init . ./gmmloc/.gmmloc_https.install
wstool update
cd ..
```

Выполнить сборку
```bash

catkin build gmmloc_ros
source devel/setup.bash
```

Заменить [**/PATH/TO/EUROC/DATASET/**](https://github.com/HyHuang1995/gmmloc/blob/770eadc99229eff17f2f613e969e4e9c10499496/gmmloc_ros/launch/v1.launch#L25) в [v1.launch](https://github.com/HyHuang1995/gmmloc/blob/master/gmmloc_ros/launch/v1.launch) на расположение извлеченных данных:
```
<param name="data_path" value="/PATH/TO/EUROC/DATASET/$(arg seq)/mav0/" />
```

Запустить
```bash
roslaunch gmmloc_ros v1.launch seq:=V1_03_difficult
```

# Конвертация PCD в GMM
Для этого можно воспользоваться скриптом [pcd_to_gmm.py](../gmmloc_ros/scripts/pcd_to_gmm.py)
```bash
rosrun gmmloc_ros pcd_to_gmm.py -h
```
Перед запуском, возможно, потребуется отдельно запустить `roscore`.

# Визуализация GMM
Для этого можно воспользоваться скриптом [publish_gmm.py](../gmmloc_ros/scripts/publish_gmm.py)
```bash
rosrun gmmloc_ros publish_gmm.py -h
```
GMM будет опубликовано в топик /gmm, который можно будет отобразить в Rviz.
Перед запуском, возможно, потребуется отдельно запустить `roscore`.