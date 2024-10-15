# Running ros2 workspace on Cosmo&Wanda Drones

### Comando para rodar o container com pix conectada:

```bash
docker run -it --device=/dev/ttyACM0  --network host depois_buildar:v2.0
```

### Comando para rodar mavlink-router:

```bash
mavlink-routerd -c /mavlink-router/pix-uart.conf
```

### Proximos passos

1. Configurar o agente DDS na Pixhawk (atualmente esta como mavlink)
2. Certificar de que a porta 8888 esta acessivel (provavelmente deve estar porque estamos rodando com opcao de network host)
3. Rodar o agente MicroXRCE-DDS e rodar `ros2 topic echo /fmu/out/vehicle_odommetry` pra testar se a conexao deu certo
4. Dar pull `frtl_2024` para pegar o codigo da maquina de estados de gestos, alem dos pacotes `camera_publisher` e `gesture_classifier`
5. Testar se a camera esta funcionando dentro do container, rodando o no `camera_publisher`
   
