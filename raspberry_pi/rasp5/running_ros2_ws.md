# Running ros2 workspace on Cosmo&Wanda Drones

### Comando para rodar o container com pix conectada:

```bash
docker run -it --device=/dev/ttyACM0  --network host depois_buildar:v2.0
```

### Comando para rodar mavlink-router:

```bash
mavlink-routerd -c /mavlink-router/pix-uart.conf
```
