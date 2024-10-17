# Setup da Rasp Cam

Vamos conectar a Rasp Cam (utilizei a Module 1.3) à Rasp e visualizar o videostream dela.

## Índice
- [Instalação](#instalação)
- [Uso](#uso)
- [Referências](#referências)

## Sensor imx219
```shel
sudo nano /boot/firmware/config.txt

#on config.txt
camera_auto_detect=0
dtoverlay=imx219
```


## Checando suporte de Driver
A Raspberry recentemente mudou para um novo driver (Unicam), que tem suporte limitado a certas funções de imagem. Queremos usar o driver antigo, `bm2835 mmal`.

Vamos checar se sua Rasp está usando o driver Unicam:

```shel
4l2-ctl -D
```

Se o driver é `bm2835 mmal`, pode prosseguir à instalação.

Se for `bcm2835-unicam`, faça o seguinte:

```shel
sudo nano boot/config.txt
```

Dentro do arquivo, escreva:
```bash
camera_autodetect=0
start_x=1
```
## Instalação

No terminal da Rasp, rode:
```shel
sudo apt-get install ros-${ROS_DISTRO}-v4l2-camera
```


## Uso

No terminal da Rasp, rode:

```shel
ros2 run v4l2_camera v4l2_camera_node
```

Agora no computador: vamos usar o rqt para visualizar as imagens. É possível usar o terminal do workspace da competição no vscode ou baixar o rqt por fora.

Para instalar o rqt:

```shel
sudo apt-get install ros-${ROS_DISTRO}-rqt-image-view
```

No terminal do computador:

```shel
ros2 run rqt_image_view rqt_image_view
```

Na aba do RQT, visualize por meio da seleção `/image_raw/compressed`.


Também é possível alterar parâmetros do v4l2_camera. No terminal do computador:

* Listar os params: `ros2 param list v4l2_camera`

* Ver a descrição do param: `ros2 param describe v4l2_camera <param_name>`

  Ex: `ros2 param describe v4l2_camera saturation`

* Mudar os params: `ros2 param set v4l2_camera <param_name> <value>`

  Ex: `ros2 param set v4l2_camera saturation 50`



## Referências



[Github do V4L2-Camera](https://gitlab.com/boldhearts/ros2_v4l2_camera)

[Tutorial usando RaspCam com ROS2 e V4L2-Camera](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304)

[Alternativa: camera_ros](https://github.com/christianrauch/camera_ros)


ros2 run camera_ros camera_node --ros-args -p width:=320 -p height:=180 -p FrameDurationLimits:="[200000,200000]"
