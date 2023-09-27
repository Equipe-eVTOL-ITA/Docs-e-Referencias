# Configuração do hotspot wifi <!-- omit in toc -->

- [Instalação do NetworkManager](#instalação-do-networkmanager)
- [Instalação do driver do adaptador USB](#instalação-do-driver-do-adaptador-usb)
- [Criação do hotspot](#criação-do-hotspot)
  - [Retirar adaptador USB do contexto do NetworkManager](#retirar-adaptador-usb-do-contexto-do-networkmanager)
  - [Instalação do *dnsmasq*, *hostapd* e *dhcpcd*](#instalação-do-dnsmasq-hostapd-e-dhcpcd)
  - [COnfigure IP estático](#configure-ip-estático)
  - [Configure o servidor DHCP](#configure-o-servidor-dhcp)
  - [Configure o ponto de acesso](#configure-o-ponto-de-acesso)
  - [Inicie o hotspot](#inicie-o-hotspot)
- [Conectando com a Raspberry por SSH com o hotspot](#conectando-com-a-raspberry-por-ssh-com-o-hotspot)
- [Teste de velocidade da conexão](#teste-de-velocidade-da-conexão)
- [Extras](#extras)
- [Local host](#local-host)
- [Referências](#referências)


## Instalação do NetworkManager

O `NetworkManager` é uma pacote que facilita a conexão com redes.

```shell
sudo apt install network-manager
```

Você precisará deletar um arquivo e criar outro:

```shell
sudo rm /etc/netplan/50-cloud-init.yaml
sudo nano /etc/netplan/01-network-manager-all.yaml
```

Dentro do arquivo escreva o seguinte:

```yaml
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: NetworkManager
```

Reinicie a Raspberry. Para testar a instalação, rode `nmcli dev status`. Seu resultado deve ser similar ao seguinte:

```shell
DEVICE      TYPE        STATE           CONNECTION
wlan0       wifi        connected       <NOME-DA-REDE>
p2p         wifi-p2p    disconnected    --
eth0        ethernet    disconnected    --
```

Para obter uma lista das redes Wifi disponíveis, você pode rodar:

```shell
nmcli dev wifi
```

Para se conectar a uma rede Wifi, use o seguinte comando, em que `ifname` é o nome do dispositivo Wifi. Para o módulo Wifi embutido na RPi, o `ifname` é `wlan0`. Para o adaptador USB, `wlx`

```shell
nmcli dev wifi connect <WiFi-SSID> password <password> ifname <ifname>
```

## Instalação do driver do adaptador USB

Instale as ferramentas de compilação necessárias.

```shell
sudo apt update
sudo apt install dkms git
sudo apt install build-essential libelf-dev linux-headers-$(uname -r)
```

Clone e compile o repostório com o driver necessário.

```shell
git clone https://github.com/Equipe-eVTOL-ITA/WiFi-Adapter-Driver.git
cd WiFi-Adapter-Driver
sudo make dkms_install
```

Conecte o adaptador e rode `nmcli dev status`. A nova saída será parecida com a seguinte:

```shell
DEVICE              TYPE        STATE           CONNECTION
wlan0               wifi        connected       <NOME-DA-REDE>
wlx788cb533651e     wifi        disconnected    --
p2p-dev-wlan0       wifi-p2p    disconnected    --
eth0                ethernet    disconnected    --
lo                  loopback    unmanaged       --
```


## Criação do hotspot

### Retirar adaptador USB do contexto do NetworkManager

O NM é muito útil para facilitar a conexão com redes sem fio, porém ele possui poucas opções para a criação do hotspot. Por isso, devemos tirar o adaptador USB da gerência do NM.

```bash
sudo nano /etc/NetworkManager/conf.d/99-unmanaged-devices.conf
```

Dentro do arquivo, escreva o seguinte:

```conf
### /etc/NetworkManager/conf.d/99-unmanaged-devices.conf

[keyfile]
unmanaged-devices=interface-name:wlx788cb533651e
```

### Instalação do *dnsmasq*, *hostapd* e *dhcpcd*

```shell
sudo apt update
sudo apt install dnsmasq hostapd dhcpcd5 
```

```shell
sudo systemctl stop dnsmasq
sudo systemctl stop hostapd
```

### COnfigure IP estático

Edite o seguinte arquivo:

```shell
sudo nano /etc/dhcpcd.conf 
```

Vá até o final do arquivo e adicione as linhas:

```
interface wlx788cb533651e
    static ip_address=192.168.4.1/24
    nohook wpa_supplicant
```

Observe que o IP 192.168.4.1 é para seguir a convenção 192.168.X.X. Você pode alterar 192.168.4.1 para 192.168.X.1. Entretanto, padronizar um IP faciilita a conexão por SSH em todos os projetos.

Caso a interface tenha outro nome, substitua `wlx788cb533651e` pelo devido nome.

Reinicie o dhcpcd.

```shell
sudo service dhcpcd restart
```

### Configure o servidor DHCP

Crie um backup da configuração original antes de criar a nova:

```shell
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
sudo nano /etc/dnsmasq.conf
```

```conf
port=5353
listen-address=127.0.0.1

interface=wlx788cb533651e
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
```

Essa configuração, fará o dnsmasq usar a porta 5353, já que a porta padrão (53) é usada pelo plugin do dnsmasq do NetworkManager. Além disso, será fornecido um endereço IP entre 192.168.4.2 e 192.168.4.20 para uma conexão.

Configure o dnsmasq para inicializar durante o boot:

```shell
sudo systemctl enable dnsmasq
sudo systemctl start dnsmasq
```

### Configure o ponto de acesso

Crie um arquivo de configuração do hostapd.

```shell
sudo nano /etd/hostapd/hostapd.conf
```

Adicione as devidas configurações:

```conf
### /etc/hostapd/hostapd.conf

### Interface
interface=wlx788cb533651e
driver=nl80211

### IEEE 802.11
ssid=DroneMarques
hw_mode=a
channel=36
max_num_sta=128
auth_algs=1

### DFS
country_code=BR
ieee80211d=1
ieee80211h=1

### IEEE 802.11n
ieee80211n=1
ht_capab=[HT40+][SHORT-GI-20][SHORT-GI-40][RX-STBC1][DSSS_CCK-40]

### IEEE 802.11ac
ieee80211ac=1
vht_oper_chwidth=1
vht_capab=[MAX-MPDU-11454][SHORT-GI-80][RX-STBC-1]
vht_oper_centr_freq_seg0_idx=42

### IEEE 802.11i
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_passphrase=<SENHA-DO-HOTSPOT>
wpa_pairwise=TKIP CCMP
rsn_pairwise=CCMP

### hostapd event logger
logger_syslog=-1
logger_syslog_level=2
logger_stdout=-1
logger_stdout_level=2

### WMM
wmm_enabled=1
uapsd_advertisement_enabled=1
wmm_ac_bk_cwmin=4
wmm_ac_bk_cwmax=10
wmm_ac_bk_aifs=7
wmm_ac_bk_txop_limit=0
wmm_ac_bk_acm=0
wmm_ac_be_aifs=3
wmm_ac_be_cwmin=4
wmm_ac_be_cwmax=10
wmm_ac_be_txop_limit=0
wmm_ac_be_acm=0
wmm_ac_vi_aifs=2
wmm_ac_vi_cwmin=3
wmm_ac_vi_cwmax=4
wmm_ac_vi_txop_limit=94
wmm_ac_vi_acm=0
wmm_ac_vo_aifs=2
wmm_ac_vo_cwmin=2
wmm_ac_vo_cwmax=3
wmm_ac_vo_txop_limit=47
wmm_ac_vo_acm=0

### TX queue parameters
tx_queue_data3_aifs=7
tx_queue_data3_cwmin=15
tx_queue_data3_cwmax=1023
tx_queue_data3_burst=0
tx_queue_data2_aifs=3
tx_queue_data2_cwmin=15
tx_queue_data2_cwmax=63
tx_queue_data2_burst=0
tx_queue_data1_aifs=1
tx_queue_data1_cwmin=7
tx_queue_data1_cwmax=15
tx_queue_data1_burst=3.0
tx_queue_data0_aifs=1
tx_queue_data0_cwmin=3
tx_queue_data0_cwmax=7
tx_queue_data0_burst=1.5
```

Substitua <SENHA-DO-HOTSPOT> em *wpa-passphrase* pelo valor desejado.

```shell
sudo nano /etc/default/hostapd
```

Encontre a linha com `#DAEMON_CONF` e substitua com isso:

```shell
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```

### Inicie o hotspot

```shell
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl start hostapd
```


Reinicie a Raspberry.

OBS: como o hotspot está configurado para o adaptador USB, nós ainda podemos nos conectar à internet por meio do módulo wifi ou pelo cabo de ethernet.

## Conectando com a Raspberry por SSH com o hotspot

No seu computador, encontre a rede hotspot. Conecte-se. Agora, para se conectar com a RPi por SSH, faça o seguinte.

```shell
ssh evtol@<IP>
```

Seguindo o tutorial de criação do hotspot, temos `IP = 192.168.4.1`.

## Teste de velocidade da conexão


Para testar a velocidade do hotspot, iremos usar a ferramenta `iperf3`:

```shell
sudo apt install iperf3
```

Na Raspberry Pi:

```shell
iperf3 -s 
```

No notebook:
```
iperf3 -c <IP> -t 30
```

## Extras

Para testar a velocidade da conexão com a internet:

```shell
sudo apt install speedtest-cli
speedtest-cli
```


Para localizar as interfaces Wi-Fi disponíveis:

```shell
iwconfig
```


## Local host

ros2 topic pub -r 1 /msg std_msgs/String "data: 'Bora eVTOL'"

## Referências 

[1](https://huobur.medium.com/how-to-setup-wifi-on-raspberry-pi-4-with-ubuntu-20-04-lts-64-bit-arm-server-ceb02303e49b) Como conectar WiFi sem o NetworkManager.

[2](https://askubuntu.com/questions/842773/ubuntu-server-and-networkmanager) Criar arquivo `01-network-manager-all.yaml`.

[3](https://gist.github.com/narate/d3f001c97e1c981a59f94cd76f041140
) Criar hotspot wifi com NetworkManager.


https://raspberrypi.stackexchange.com/questions/121225/why-pi-4-wifi-speed-is-locked-to-54-0-mbit-s

https://www.ekahau.com/blog/channel-planning-best-practices-for-better-wi-fi/

https://support.qacafe.com/cdrouter/knowledge-base/prevent-network-manager-from-controlling-an-interface/

https://askubuntu.com/questions/191226/dnsmasq-failed-to-create-listening-socket-for-port-53-address-already-in-use

https://raspberrypi-guide.github.io/networking/create-wireless-access-point

http://pisarenko.net/blog/2015/02/01/beginners-guide-to-802-dot-11ac-setup/

## Autores <!-- omit in toc -->

* Profeta T25

https://superuser.com/questions/580311/why-is-my-udp-so-slow