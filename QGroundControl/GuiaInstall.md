# Este guia busca explicar como instalar e rodar o QGroundControl em Ubuntu

## Parte 1: Preparo do Ubuntu

Abra um terminal e digite os seguintes comandos:

    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libqt5gui5 -y
    sudo apt install libfuse2 -y

Para ativar essas mudanças, deslogue e logue novamente no seu Ubuntu.

## Parte 2: Instalando o QGC

Baixe o "https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage";

No terminal, vá para o caminho do QGC.AppImage usando o comando: 

    cd *CAMINHO*

A partir daí, basta instalar o QGC com o comando:

    chmod +x ./QGroundControl.AppImage

Para inicializar o QGC, faça o comando nesse mesmo terminal (SEMPRE NO TERMINAL DO QGC.AppImage):

    ./QGroundControl.AppImage
    