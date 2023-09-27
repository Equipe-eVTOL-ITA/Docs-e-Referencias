# Configurar Fast DDS Discovery Server <!-- omit in toc -->

**Objetivo:** explicar como configurar o Fast DDS Discovery Server para otimização na comunicação entré nós e uso do ROS2 na ausência de conexão com a internet.

**Índice**
- [Status atual](#status-atual)
- [](#)
  - [Na raspberry](#na-raspberry)
  - [No notebook](#no-notebook)
  - [Arquivos de configuração](#arquivos-de-configuração)
- [IDEIAS - DISCOVERY SERVER / TCP](#ideias---discovery-server--tcp)

sudo  sysctl -w net.ipv4.icmp_echo_ignore_broadcasts=0
sudo sysctl -w net.ipv4.route.flush=0
sudo ip route add 224.0.0.0/4 dev wlx788cb533651e 

## Status atual

- Inicializar o servidor pelo .xml funciona
- Super client deu certo

## 

Criar servidor:
```shell
fastdds discovery -i 0 -x eVTOL/fastdds/simple_server.xml
```

```shell
fastdds discovery -i 0 -p 56542
```

### Na raspberry

```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client.xml
export ROS_DISCOVERY_SERVER="192.168.4.20:56542"
```

ou

```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client.xml
export ROS_DISCOVERY_SERVER="192.168.4.1:24565"
```

### No notebook

```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=eVTOL/fastdds/super_client.xml
export ROS_DISCOVERY_SERVER="192.168.4.20:56542"
```

ou

```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=eVTOL/fastdds/super_client.xml
export ROS_DISCOVERY_SERVER="192.168.4.1:24565"
```

### Arquivos de configuração

server.xml:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="UDP SERVER" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SERVER</discoveryProtocol>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>0.0.0.0</address>
                            <port>56542</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
            <prefix>72.61.73.70.66.61.72.6d.74.65.73.74</prefix>
        </rtps>
    </participant>
</profiles>
```

client.xml
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="UDP CLIENT" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>CLIENT</discoveryProtocol>
                    <discoveryServersList>
                        <RemoteServer prefix="72.61.73.70.66.61.72.6d.74.65.73.74">
                            <metatrafficUnicastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>192.168.4.20</address>
                                        <port>56542</port>
                                    </udpv4>
                                </locator>
                            </metatrafficUnicastLocatorList>
                            <metatrafficMulticastLocatorList>
                                <locator>
                                    <udpv4>
                                        <address>192.168.4.1</address>
                                        <port>24565</port>
                                    </udpv4>
                                </locator>
                            </metatrafficMulticastLocatorList>
                        </RemoteServer>
                    </discoveryServersList>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```


## IDEIAS - DISCOVERY SERVER / TCP

Usando servidor na Rpi/SuperClient 

1. Testar primeiro com chatter
2. Inicio servidor Discovery no RPi1
3. export no Rpi2 e começo o talker
4. Rpi3: export + servidor tcp
5. PC1: export + cliente TCP
6. Rpi4: export + serviço TCP
7. PC2: export + listener

```
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client.xml
```

```
source install/setup.bash
ros2 run tcp_tunnel server
```

```
ros2 run tcp_tunnel client --ros-args -p client_ip:="192.168.4.1"
```

```
source install/setup.bash
ros2 service call /tcp_tunnel_client/add_topic tcp_tunnel/srv/AddTopic "topic:
  data: '/chatter'
tunnel_queue_size:
  data: 2'
server_namespace:
  data: '/tcp'"
```

PASSOS:
```shell
fastdds discovery -i 0 -x server.xml
```
```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=~/super_client.xml
ros2 run camera_ros camera_node --ros-args -p width:=320 -p height:=180 -p FrameDurationLimits:="[200000,200000]"
```
```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=~/super_client.xml
cd ~/ros2_ws
source install/setup.bash
ros2 run tcp_tunnel server
```
```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=~/eVTOL/fastdds/super_client.xml
cd ~/eVTOL/tunnel
source install/setup.bash
ros2 run tcp_tunnel client --ros-args -p client_ip:="192.168.4.20"
```
```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=~/super_client.xml
cd ~/ros2_ws
source install/setup.bash
ros2 service call /tcp_tunnel_client/add_topic tcp_tunnel/srv/AddTopic "topic:
  data: '/camera/image_raw'
tunnel_queue_size:
  data: ''
server_namespace:
  data: '/'"

```
```shell
export FASTRTPS_DEFAULT_PROFILES_FILE=~/eVTOL/fastdds/super_client.xml

```

```
ros2 service call /tcp_tunnel_client/remove_topic tcp_tunnel/srv/RemoveTopic "topic:
  data: '/tcp_tunnel_client/camera/image_raw'"
```
