# Calibração de câmeras <!-- omit in toc -->

- [Como usar](#como-usar)
- [Instalação](#instalação)
- [Dados necessários](#dados-necessários)
  - [Crie o alvo de calibração](#crie-o-alvo-de-calibração)
  - [Grave os dados de calibração](#grave-os-dados-de-calibração)
  - [Conversão para ROS1](#conversão-para-ros1)
- [Câmera](#câmera)
- [Câmera + IMU](#câmera--imu)
- [Referências](#referências)

Pacote: camera_calibration

## Como usar

- **`size`**: quantidade de vértices no interior do tabuleiro.
- **`square`**: tamanho do quadrado do tabuleiro em metros.

```shell
ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.025 --ros-args -r image:=/my_camera/image_raw -p camera:=/my_camera
```

## Instalação

Clone o repositório do pacote:

```shell
mkdir <CAMINHO>/calibration_ws/src/
cd <CAMINHO>/calibration_ws/src/
git clone https://github.com/ethz-asl/kalibr.git
```

Faça o build da imagem do Docker:

```shell
cd kalibr
docker build -t kalibr -f Dockerfile_ros1_20_04 .
```

Defina a pasta em que serão armazenados os dados da calibração e inicie o container. Nesse caso, irei usar <CAMINHO>/calibration_ws

```shell
export FOLDER=<CAMINHO>/calibration_ws
xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr
```

Esse processo demorou 12 minutos no meu computador.

## Dados necessários

### Crie o alvo de calibração

Crie o arquivo pdf que será impresso e usado na calibração. O tutorial gera [esse]() arquivo.

```shell
source devel/setup.bash
rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 10 --ny 5 --tspace 0.3
cp target.pdf /data
```

No seu computador, vá até a pasta que armazena os dados e altere as permissões do arquivo:

```shell
sudo chown $USER: target.pdf
```

Na pasta comṕartilhada, crie um arquivo YAML de configuração para esse tabuleiro. Para este tutorial, vou considerar que o nome do arquivo é `april_10x5.yaml`

```shell
target_type: 'aprilgrid' 
tagCols: 10               
tagRows: 5               
tagSize: 0.0456             # modifique essa parte           
tagSpacing: 0.3          
```



Para mais informações de como gerar o pdf, use o seguinte comando:

```shell
rosrun kalibr kalibr_create_target_pdf --h
```

### Grave os dados de calibração

Vá até a pasta onde você está guardando os dados. Comece a gravar os tópicos.

- Para a OAK-D Pro:

```shell
ros2 bag record /oak/left/image_raw /oak/right/image_raw /oak/rgb/image_raw /oak/imu/data -o oakd_vi_calibration
```

- Para a RaspCam:
```shell
ros2 bag record /camera/image_raw -o rasp_v_calibration
```

### Conversão para ROS1

Para converter uma bag do ROS2 para o ROS1, você deve fazer o seguinte.

```shell
pip3 install rosbags>=0.9.12 
rosbags-convert <ROS2_BAG_FOLDER> --dst <FILENAME>.bag --exclude-topic <TOPICOS-EXTRAS>
```

Na calibração da OAK-D, nós criamos duas bags do ROS1 (uma para a calibração RGBD-Inertial e outra para Stereo-Inertial).

```shell
rosbags-convert <ROS2_BAG_FOLDER> --dst oak_rgbd_imu.bag --exclude-topic /oak/left/image_raw /oak/right/image_raw
rosbags-convert <ROS2_BAG_FOLDER> --dst oak_stereo_imu.bag --exclude-topic /oak/rgb/image_raw
```

## Câmera

Para calibrar apenas a câmera, entre no container e rode os comandos:

- Calibração da câmera RGB.
```shell
source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/oak_rgbd_imu.bag \
    --target /data/april_10x5.yaml \
    --models pinhole-radtan\
    --topics /oak/rgb/image_raw
```


- Calibração da câmera stereo.
```shell
source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/oak_stereo_imu.bag \
    --target /data/april_10x5.yaml \
    --models pinhole-radtan pinhole-radtan\
    --topics /oak/left/image_raw /oak/right/image_raw
```

## Câmera + IMU

Antes de começar a calibração, você precisa obter alguns parâmetros dos sensores inerciais. Para isso, deixe o sensor parado em um local sem vibrações por pelo menos **três horas** (💀). Eu gravei esses dados por seis horas e salvei no Drive. Você pode achar em `Drive eVTOL ITA > Time de Drones > Calibração > imu_calibration_data.zip`. Converta a bag do ROS2 para o ROS1.

- (Recomendado) Re-organizar as mensagens por *timestamp*:

```shell
rosrun allan_variance_ros cookbag.py --input original_rosbag --output cooked_rosbag
```

- Calibre os sensores.

```shell
rosrun allan_variance_ros allan_variance [path_to_folder_containing_bag] [path_to_config_file]
```

```shell
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/oakd_vi_calibration.bag \
  --target /data/april_10x5.yaml \
  --cam /data/rgbd-camchain.yaml \
  --imu /data/imu.yaml
```

```shell
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/oakd_vi_calibration.bag \
  --target /data/april_10x5.yaml \
  --cam /data/stereo-camchain.yaml \
  --imu /data/imu.yaml
```

## Referências

[1](https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY) Vídeo exemplo