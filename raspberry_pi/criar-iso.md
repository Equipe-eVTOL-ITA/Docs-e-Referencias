# Como criar imagem personalizada para RPi

## Copiando cartão SD

Após fazer modificações no sistema operacional da Raspberry Pi, você pode conectar o cartão SD com seu computador e copiar essa imagem.

Primeiro, encontre o nome do dispositivo. Geralmente, cartões SD tem o nome como `/dev/sdX` (por exemplo, `/dev/sda`).

```shell
sudo fdisk -l
```

Para criar a imagem, rode o comando abaixo. Certifique-se que há espaço suficiente no seu sistema, pois esse comando cria uma cópia do cartão, o que geralmente significa 32GB.

```shell
sudo dd bs=16M if=/dev/sdX of=/path/to/file/image-name.img status=progress
```

## Reduzindo o tamanho da imagem

Como a iamgem imagem anterior tem o tamanho do SD todo, precisamos reduzir todo esse espaço ocupado. Para isso usamos o PiShrink.

```shell
sudo pishrink.sh /path/to/original/image.img /path/to/new/image.img -d
```


## Referências

[1](https://raspberrytips.com/create-image-sd-card/) Criar imagem do cartão SD usando o comando `dd`.

[2](https://github.com/Drewsif/PiShrink) PiShrink.

## Autores

* Profeta T25