# Configuração do MTF-01 no QGroundControl para Pixhawk (Sem usar MicoAir)

## Hardware

Conecte o MTF-01 em uma porta serial da Pixhawk.

### Ligação recomendada (TELEM3)

| MTF-01 | Pixhawk |
|---|---|
| TX | RX |
| RX | TX |
| 5V | 5V |
| GND | GND |

---

# Configuração no QGroundControl (PX4)

Abra:

- Vehicle Setup
- Parameters

---

# 1. Configurar a serial

```ini
MAV_2_CONFIG = TELEM 3
SER_TEL3_BAUD = 115200
```

Depois:
- reinicie a Pixhawk.

---

# 2. Ativar Optical Flow

Configure:

```ini
EKF2_OF_CTRL = Enabled
EKF2_RNG_CTRL = Enabled
```

---

# 3. Definir referência de altura

```ini
EKF2_HGT_REF = Range sensor
```

Isso faz o PX4 usar o lidar do MTF-01 para altitude indoor.

---

# 4. Rotação do sensor

Se o sensor estiver alinhado para frente:

```ini
SENS_FLOW_ROT = 0
```

## Tabela de rotação

| Valor | Rotação |
|---|---|
| 0 | Frente |
| 1 | 90° |
| 2 | 180° |
| 3 | 270° |

---

# 5. Reiniciar a controladora

Muito importante.

Depois do reboot o PX4 tenta detectar automaticamente:
- optical flow
- rangefinder

---

# 6. Verificar se detectou

No QGroundControl:

- Analyze Tools
- MAVLink Inspector

Procure mensagens:
- `OPTICAL_FLOW`
- `DISTANCE_SENSOR`

Ao mover o drone:
- flow_x
- flow_y
- distance

devem mudar.

---

# 7. Teste de voo

Primeiro teste:
- Stabilized
- Altitude

Depois:
- Position Mode indoor

O sensor funciona melhor:
- abaixo de 3 metros
- em piso com textura
- com boa iluminação

---

# Resumo rápido dos parâmetros

```ini
MAV_2_CONFIG = TELEM 3
SER_TEL3_BAUD = 115200

EKF2_OF_CTRL = Enabled
EKF2_RNG_CTRL = Enabled
EKF2_HGT_REF = Range sensor

SENS_FLOW_ROT = 0
```

---

# Documentação

## QGroundControl

https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html

## PX4 Optical Flow

https://docs.px4.io/main/en/sensor/optical_flow.html
