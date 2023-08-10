# Baterias

Nessa página, serão apresentadas algumas ideias de como escolher a bateria para o drone.

## Cálculo de carga

### Corrente média exigida pelos motores

A potência de um motor é dada por:
$$P = V \cdot I = D \cdot \omega = k \cdot \omega ^3$$
O empuxo pode ser calculado com:
$$T = k \cdot \omega ^2$$

O motor que nós usamos é o BL 2215/20, com uma hélice 10x4.5. Usando a tabela *Test Data* encontrada nesse [site](https://www.flybrushless.com/motor/view/31), conseguimos obter o valor de $P_{max}$ e $T_{max}$. Assim, é possível calcular a constante $k$.

$$P_{max} = \sqrt{\frac{T_{max}^3}{k}}$$
$$k = \frac{T_{max}^3}{P_{max}²}$$

Nesse caso, as unidades apropriadas são $W$ para a potência e $g$ para o empuxo.

Para calcular a corrente média, devemos determinar a força média que cada motor deverá oferecer. Essa força depende da massa do drone e da inclinação média durante o voo. Para a FRTL, consideramos uma inclinação de 10°.

$$m = 4T' \cos10°$$
$$T' = \frac{m}{4 \cos10°}$$
$$k \cdot \omega ^2 = \frac{m}{4 \cos10°}$$
$$\omega = \sqrt{\frac{m}{4 k \cos10°}}$$

Agora, para encontrar a corrente média, precisamos saber a massa $m$ do nosso *drone*. Assim, a fórmula para a corrente média de cada motor pode ser deduzida.

$$P = V \cdot I = k \cdot (\frac{m}{4 k \cos10°})^{\frac{3}{2}}$$
$$I = \frac{1}{V \cdot k ^\frac{1}{2}} \cdot (\frac{m}{4 \cos10°})^{\frac{3}{2}}$$

### Cálculo da massa de acordo com a bateria

Em geral, usamos que a energia específica de uma bateria é 150 ${Wh}/{kg}$. Para determinar a massa de uma bateria com voltagem $V$ e carga $Q$. Precisamos calcular a carga específica:

$$q = \frac{150}{V} $$

Assim, a massa fica:

$$m_{bat} = \frac{Q}{q} = \frac{QV}{150}$$

### Corrente máxima exigida pelos motores

Essa especificação é dada pelo fabricante. É a corrente que o motor usa quando a potência é máxima.

### Cálculo da carga com base na corrente média

Para calcular a corrente total, precisamos levar em conta todos os componentes eletrônicos (quatro motores, uma *Raspberry Pi*, uma controladora de voo, um servo, etc.), o tempo de voo e a descarga máxima.

Abaixo, temos a listagem de alguns componentes que sempre usamos e cuja corrente é constante.

| Componente | Corrente | Voltagem | Tempo de uso | Energia (kWh) |
|----|----|----|----|----|
|*Raspberry Pi*|3|5,1|10|2550|
|*OAK-D Lite*|0,9|5|10|750|

OBS: A *Pixhawk* exige uma corrente desprezível.

Nós somamos a energia demanda por todos os componentes, resultando um total de $E_{total}$. A fim de converter para $mAh$, devemos fazer:

$$Carga = \frac{E_{total}}{V}$$

### Margem de segurança

Em geral, considera-se seguro esvaziar a bateria até 20 a 30% de sua carga nominal.

## Como usar o script

## Autores

* Profeta T25

