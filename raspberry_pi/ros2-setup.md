# Instalação do ROS2 na Raspberry Pi

## ROS2 Humble

Faça o setup do `locale`.

```shell
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Adicione os repositórios.

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```shell
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Instale a base do Humble.

```shell
sudo apt update && sudo apt install ros-humble-ros-base
```

Instale as ferramentas de desenvolvimento.

```shell
sudo apt install ros-dev-tools
```

Faça o setup do ambiente.

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Pacotes adicionais

### ROS2 Demos

Para realizar alguns testes e configurar o Fast DDS, optamos por instalar um pacote adicional com alguns códigos demonstrativos do ROS2.

```shell
sudo apt install ros-humble-demo-nodes-cpp
```

## Teste

Para testar a instalação (e a conexão SSH por hotspot), abra o terminal do seu computador com o ROS2 Humble instalado.

```shell
ros2 run demo_nodes_cpp talker
```

Abra um novo terminal e faça a conexão ssh. Rode o seguinte:

```shell
ros2 topic echo /chatter
```

## Networking

https://www.youtube.com/watch?v=NW97xLF7CYQ

https://husarion.com/tutorials/ros-tutorials/5-running-ros-on-multiple-machines/

https://variwiki.com/index.php?title=Wifi_NetworkManager

https://answers.ros.org/question/365051/using-ros2-offline-ros_localhost_only1/


## Referências

[1](https://roboticsbackend.com/install-ros2-on-raspberry-pi/) Instalando ROS2 na Raspberry Pi.

[2](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) Instalação do ROS2 Humble.

[3](https://www.ros.org/reps/rep-2001.html) Variantes do ROS2.

## Autores

* Profeta T25
