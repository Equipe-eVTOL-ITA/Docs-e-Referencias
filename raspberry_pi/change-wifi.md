# Mudando Wifi da Rasp com Ubuntu Headless <!-- omit in toc -->


Ao instalar o Networkd ou mudando de rede Wifi, pode ser que você tenha configurado um Ubuntu Headless numa rede wifi à qual você não tenha mais acesso. 
Então, você precisa conectar a Rasp a uma nova rede para usar o SSH.


Para mudar a rede wifi, você vai precisar ler o cartão sd da Rasp no seu computador.

 
Abra o terminal na pasta `/writable/etc/netplan`


Agora mude o arquivo que está dentro dessa pasta:


```shell
sudo nano 50-cloud-init.yaml
```

Você vai ver que abaixo de accesspoints vai estar a rede wifi previamente conectada e, abaixo dela, a senha.
Mude a rede e a senha para as atuais. OBS: Há um espaço depois do ':' de password.


O mesmo vale caso você esteja mudando de ethernet para wifi e vice-versa.
