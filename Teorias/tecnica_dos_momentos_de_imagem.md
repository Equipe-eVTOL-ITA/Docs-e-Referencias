# Técnica dos Momentos de Imagem

**Autor:** Ângelo Marconi Pavan

**Diretoria/Função:** Navegação/Membro

**Data:** 08/03/2025

## Abstract

Os Momentos de Imagem constituem uma técnica fundamental em visão computacional que descreve a distribuição dos pixels de uma forma através de um conjunto de descritores numéricos. Ao tratar a intensidade dos pixels como uma distribuição de massa, a técnica permite a extração robusta de propriedades geométricas essenciais. Mais especificamente, os momentos de ordem inferior são utilizados para calcular a área do contorno (análoga à massa total) e seu centroide (o centro de massa). Os momentos de segunda ordem, por sua vez, são empregados para determinar a orientação do eixo principal da forma. Essa caracterização é estabelecida através de uma elegante analogia com a mecânica clássica e a estatística, onde a covariância entre as coordenadas dos pixels (μ₁₁) revela a inclinação do objeto, e a orientação é o ângulo que anula essa covariância.

## Ideia Geral

A ideia central por trás da técnica dos Momentos de Imagem é traduzir a geometria de uma forma em um conjunto de números descritivos. Para isso, tratamos a imagem como uma função de intensidade 2D, `$I(x,y)$`, e definimos uma nova grandeza chamada **momento espacial**.

Intuitivamente, um momento funciona como uma **soma ponderada** das intensidades de todos os pixels da imagem. A fórmula geral para o momento espacial de ordem `($p+q$)` é:

$$M_{pq} = \sum_{x} \sum_{y} x^p y^q \cdot I(x, y)$$

Vamos analisar cada componente:

* **`$I(x,y)$`**: É a intensidade do pixel na posição `(x,y)`. Para simplificar, podemos pensar em uma imagem em tons de cinza, onde `$I$` pode variar (e.g., de 0 para preto a 255 para branco). Em imagens binárias, o valor é simplesmente `1` para a forma e `0` para o fundo.

* **`$x^p y^q$`**: É a **função de ponderação** que determina qual propriedade geométrica estamos medindo. Ao variar os valores de `$p$` e `$q$` (que são inteiros não negativos, ou seja, `$p, q \ge 0$`), nós "destacamos" diferentes características da distribuição dos pixels.

* **`$\sum\sum$`**: Simboliza a soma de todos os valores calculados para cada pixel `(x,y)` da imagem.

A soma `$p+q$` é definida como a **ordem do momento**. Momentos de ordens diferentes capturam propriedades diferentes: a ordem zero nos dá a área, a ordem um nos dá o centroide, e a ordem dois nos dá a orientação.

## Ordem zero

O momento de ordem mais baixa e fundamental é o de ordem zero. Para este caso, a condição da ordem é `$p+q = 0$`. Como `$p$` e `$q$` são inteiros não negativos, a única solução possível é `$p=0$` e `$q=0$`.

Substituindo esses valores na fórmula geral do momento espacial, obtemos:

$$M_{00} = \sum_{x} \sum_{y} x^0 y^0 \cdot I(x, y) = \sum_{x} \sum_{y} I(x, y)$$

A forma mais intuitiva de entender o `$M_{00}$` é através da analogia física da **massa**. A interpretação do resultado, no entanto, depende do tipo de imagem com que estamos trabalhando.

### Caso 1: Imagem Binária

Em uma imagem binária, como uma máscara de contorno, a intensidade `$I(x,y)$` assume apenas dois valores:
* `$I(x,y) = 1$` para pixels que pertencem à forma (branco).
* `$I(x,y) = 0$` para pixels que pertencem ao fundo (preto).

Neste cenário, a fórmula `$M_{00} = \sum \sum I(x,y)$` está, na prática, somando o valor `1` para cada pixel que compõe a forma. Consequentemente, `$M_{00}$` é literalmente a **contagem de pixels** do objeto, o que corresponde diretamente à sua **área**.

### Caso 2: Imagem em Tons de Cinza

Em uma imagem em tons de cinza, a intensidade `$I(x,y)$` pode variar (e.g., de 0 a 255). Aqui, a soma representa a "energia" ou "luminosidade" total da imagem.

Seguindo a analogia física, `$M_{00}$` é análogo à **massa total** de um objeto 2D cuja densidade não é uniforme. Regiões mais brilhantes (maior `$I(x,y)$`) são como partes mais "densas" ou "pesadas", contribuindo mais para a massa total.

**Em resumo, para a maioria das aplicações de análise de formas, trabalhamos com máscaras binárias, e o momento `$M_{00}$` é a ferramenta direta e eficiente para calcular a área do contorno.**

## Ordem 1

Avançando para a próxima ordem, consideramos os momentos de primeira ordem, onde a condição é `$p+q = 1$`. Sendo `$p$` e `$q$` inteiros não negativos, temos duas possibilidades que satisfazem essa condição:

* **Para `$p=1, q=0$`:**
    $$M_{10} = \sum_{x} \sum_{y} x^1 y^0 \cdot I(x, y) = \sum_{x} \sum_{y} x \cdot I(x, y)$$

* **Para `$p=0, q=1$`:**
    $$M_{01} = \sum_{x} \sum_{y} x^0 y^1 \cdot I(x, y) = \sum_{x} \sum_{y} y \cdot I(x, y)$$

Vamos interpretar o que essas somas significam. Na analogia física, onde `$I(x,y)$` é a massa em cada ponto, as fórmulas representam:

* **`$M_{10}$`**: A soma de todas as coordenadas `$x$` de cada "partícula de massa" da forma. É uma medida da distribuição da massa ao longo do eixo X.
* **`$M_{01}$`**: A soma de todas as coordenadas `$y$` de cada "partícula de massa". É uma medida da distribuição da massa ao longo do eixo Y.

Esses valores, `$M_{10}$` e `$M_{01}$`, podem ser entendidos como os **componentes do centro de massa ainda não normalizados**. Eles nos dão a soma ponderada das posições, mas para encontrar o *ponto de equilíbrio* (o centroide), precisamos calcular a posição média.

### A Fórmula do Centroide

O centroide (`$c_x$`, `$c_y$`) de uma forma é a sua posição média, ponderada pela intensidade (ou massa) de cada pixel. A fórmula para uma média é sempre a *soma total dos valores* dividida pelo *número de elementos*.

Aplicando essa lógica:

* **Coordenada `$c_x$` (Média em X):** É a soma de todas as coordenadas `$x$` ponderadas pela massa (`$M_{10}$`), dividida pela massa total (`$M_{00}$`).
* **Coordenada `$c_y$` (Média em Y):** É a soma de todas as coordenadas `$y$` ponderadas pela massa (`$M_{01}$`), dividida pela massa total (`$M_{00}$`).

Isso nos leva diretamente às fórmulas do centroide:

$$c_x = \frac{\sum_{x} \sum_{y} x \cdot I(x, y)}{\sum_{x} \sum_{y} I(x, y)} = \frac{M_{10}}{M_{00}}$$

$$c_y = \frac{\sum_{x} \sum_{y} y \cdot I(x, y)}{\sum_{x} \sum_{y} I(x, y)} = \frac{M_{01}}{M_{00}}$$

## Covariância

Para avançarmos para o caso de momento de ordem 2, é requisito *sine-qua-non* a compreensão do conceito de **covariância**.
Covariância é a medida da tendência de duas variáveis de variarem juntas.

No cerne da questão, a covariância responde a uma pergunta simples: "Quando a variável `X` se desvia de sua média, o que a variável `Y` tende a fazer em relação à sua própria média?". A resposta para essa pergunta pode se manifestar de três formas distintas, revelando a natureza da relação linear entre as duas variáveis.

### Covariância Positiva

Uma covariância positiva indica que as variáveis se movem **na mesma direção**. Quando uma variável está acima de sua média, a outra também tende a estar acima de sua média. Da mesma forma, quando uma está abaixo da média, a outra tende a acompanhá-la.

* **Exemplo Prático:** Altura e peso. Pessoas mais altas (`X` acima da média de altura) tendem a ser mais pesadas (`Y` acima da média de peso).
* **Visualização:** Em um gráfico de dispersão, os pontos formam uma nuvem que se estende do canto inferior esquerdo ao superior direito.

### Covariância Negativa

Uma covariância negativa indica que as variáveis se movem em **direções opostas**. Quando uma variável está acima de sua média, a outra tende a estar *abaixo* de sua média.

* **Exemplo Prático:** Horas de estudo e horas de lazer. Quanto mais tempo se dedica aos estudos (`X` acima da média), menos tempo tende a sobrar para o lazer (`Y` abaixo da média).
* **Visualização:** A nuvem de pontos em um gráfico de dispersão se estende do canto superior esquerdo ao inferior direito.

### Covariância Nula (ou Próxima de Zero)

Uma covariância nula ou próxima de zero sugere que **não há uma tendência linear** entre as variáveis. O valor de uma variável não nos dá uma pista clara sobre onde a outra variável estará em relação à sua média.

* **Exemplo Prático:** Número do sapato e QI. Não há relação linear discernível entre o tamanho do pé de uma pessoa e sua inteligência.
* **Visualização:** A nuvem de pontos é disforme, sem uma direção clara, parecendo um círculo ou uma nuvem aleatória.

### O Formalismo: A Contabilidade nos Quadrantes

A intuição acima é capturada matematicamente pela fórmula da covariância:

$$Cov(X, Y) = \frac{\sum_{i=1}^{n}(X_i - \bar{X})(Y_i - \bar{Y})}{n-1}$$

A parte crucial é o produto `$(X_i - \bar{X})(Y_i - \bar{Y})$`. Como você mencionou, a análise desse produto é a chave para o entendimento. Para visualizar isso, imagine que traçamos um novo sistema de eixos centrado na média dos dados `($\bar{X}$, $\bar{Y}$)`, dividindo o espaço amostral em quatro quadrantes.

O sinal do produto em cada quadrante nos diz tudo:

* **Quadrante 1 (Superior Direito):** Aqui, `$X > \bar{X}$` e `$Y > \bar{Y}$`. Ambos os desvios são positivos. O produto `$(+) \cdot (+)$` é **positivo**.
* **Quadrante 2 (Superior Esquerdo):** Aqui, `$X < \bar{X}$` e `$Y > \bar{Y}$`. O primeiro desvio é negativo e o segundo é positivo. O produto `$(-) \cdot (+)$` é **negativo**.
* **Quadrante 3 (Inferior Esquerdo):** Aqui, `$X < \bar{X}$` e `$Y < \bar{Y}$`. Ambos os desvios são negativos. O produto `$(-) \cdot (-)$` é **positivo**.
* **Quadrante 4 (Inferior Direito):** Aqui, `$X > \bar{X}$` e `$Y < \bar{Y}$`. O primeiro desvio é positivo e o segundo é negativo. O produto `$(+) \cdot (-)$` é **negativo**.

A covariância final é a soma de todos esses produtos (dividida por `$n-1$` para obter a média).
* Se a maioria dos pontos estiver nos quadrantes 1 e 3, a soma dos produtos será **positiva**.
* Se a maioria dos pontos estiver nos quadrantes 2 e 4, a soma dos produtos será **negativa**.
* Se os pontos estiverem espalhados uniformemente, os produtos positivos e negativos se cancelarão, resultando em um valor **próximo de zero**.

### O Sinal Importa Mais que a Magnitude

É fundamental entender que, para a covariância, a informação mais valiosa é o seu **sinal**.

* O **sinal (`+`, `-`, ou `0`)** nos diz a **direção** da relação linear.
* A **magnitude** (o valor numérico) é difícil de interpretar isoladamente, pois ela depende das unidades das variáveis. Uma covariância de `5000` pode ser fraca se estivermos medindo salários anuais em Reais, mas extremamente forte se estivermos medindo alturas em metros.

Por causa dessa limitação, para medir a *força* de uma relação, geralmente normalizamos a covariância para obter o **coeficiente de correlação**, que sempre varia entre -1 e 1.

## Momentos centralizados

Até agora, discutimos os **momentos espaciais brutos** (`$M_{pq}$`), que são calculados em relação à origem da imagem (o canto superior esquerdo, `(0,0)`). Embora úteis, eles possuem uma limitação fundamental: seus valores são altamente dependentes da **posição** do objeto na imagem.

Imagine uma imagem com um quadrado no canto esquerdo. O momento `$M_{10}$` (a soma das coordenadas `$x$`) será um valor baixo. Se movermos exatamente o mesmo quadrado para o canto direito da imagem, sua forma não muda, mas seu `$M_{10}$` se tornará um valor muito mais alto, simplesmente porque todas as suas coordenadas `$x$` aumentaram.

Isso é um problema. Para uma análise de formas robusta, precisamos de descritores que descrevam a geometria intrínseca do objeto, independentemente de onde ele esteja. A solução para isso são os **momentos centralizados**.

### A Ideia Central

A lógica é simples e elegante: em vez de medir as propriedades da forma em relação à origem arbitrária da imagem, vamos medi-las em relação a um ponto que pertence à própria forma e que é invariante à sua posição: o seu **centroide (`$c_x, c_y$`)**.

### O Formalismo

O momento central de ordem `($p+q$)`, denotado pela letra grega `μ` (mu), é definido como:

$$ \mu_{pq} = \sum_{x} \sum_{y} (x - c_x)^p (y - c_y)^q \cdot I(x, y) $$

A única, porém crucial, diferença na fórmula é a substituição de `$x$` por `$(x - c_x)$` e de `$y$` por `$(y - c_y)$`. Agora, estamos ponderando a intensidade de cada pixel pela sua distância relativa ao centroide da forma, e não mais pela sua distância à origem da imagem.

### A Vantagem Principal: Invariância à Translação

Ao fazer essa mudança de referência, os momentos centralizados se tornam **invariantes à translação**.

Isso significa que, se você mover o objeto para qualquer lugar dentro da imagem, os valores dos seus momentos centralizados (`$\mu_{pq}$`) **não mudarão**. A distância de cada pixel do objeto ao centroide do objeto permanece constante, não importa onde o conjunto (objeto + centroide) esteja localizado.

Essa propriedade é a principal vantagem de usar momentos centralizados. Eles nos dão uma "assinatura" da forma pura, permitindo comparar dois objetos e dizer se eles têm a mesma geometria, mesmo que estejam em posições completamente diferentes.

### Propriedades dos Momentos Centralizados

* **`$\mu_{00}$`**: O momento central de ordem zero é igual ao momento bruto de ordem zero (`$\mu_{00} = M_{00}$`). A área, ou massa total, de um objeto não muda quando o movemos.

* **`$\mu_{10}$` e `$\mu_{01}$`**: Os momentos centralizados de primeira ordem são **sempre iguais a zero**. Isso faz sentido intuitivo: o centroide é o "ponto de equilíbrio". A soma das distâncias ponderadas de todas as partículas de massa em relação ao seu próprio centro de massa é, por definição, nula.

* **Momentos de Ordem Superior (`$\mu_{20}, \mu_{02}, \mu_{11}, ...$`):** São estes os momentos que carregam a informação rica sobre a geometria da forma. Eles descrevem a "dispersão", a elongação, a inclinação e outras propriedades da distribuição dos pixels ao redor do centroide, de forma independente da posição. É por isso que a fórmula da orientação (`$\theta$`) utiliza os momentos centralizados de segunda ordem.

## Ordem 2

Após determinar a área (ordem 0) e a localização (ordem 1) de uma forma, o próximo passo é descrever sua geometria intrínseca: como seus pixels estão distribuídos ou "espalhados" em torno de seu centroide. É aqui que entram os **momentos de segunda ordem** (`$p+q=2$`).

Para garantir que estamos descrevendo a forma pura, independente de sua posição, utilizamos os **momentos centralizados** de segunda ordem:
* `$\mu_{20}$` (onde `$p=2, q=0$`)
* `$\mu_{02}$` (onde `$p=0, q=2$`)
* `$\mu_{11}$` (onde `$p=1, q=1$`)

### Interpretação Física: Momento de Inércia

A analogia mais poderosa para os momentos de segunda ordem vem da mecânica clássica: o **Momento de Inércia**. Em termos simples, o momento de inércia é a medida da resistência de um objeto a uma mudança em seu estado de rotação.

Pense em uma patinadora no gelo:
* Quando ela gira com os braços esticados, sua massa está longe do eixo de rotação. Seu momento de inércia é **alto**, e ela gira devagar.
* Quando ela encolhe os braços, sua massa se concentra perto do eixo. Seu momento de inércia é **baixo**, e ela gira muito mais rápido.

Os momentos centralizados `$\mu_{20}$` e `$\mu_{02}$` são análogos diretos ao momento de inércia da nossa forma 2D em relação aos eixos que passam por seu centroide.

* **`$\mu_{20} = \sum \sum (x - c_x)^2 \cdot I(x, y)$`**
    O termo `$(x - c_x)^2$` representa o quadrado da distância perpendicular de um pixel ao eixo vertical que passa pelo centroide. Portanto, `$\mu_{20}$` mede a dispersão da "massa" de pixels ao redor do eixo Y, ou seja, é o **momento de inérc a em relação ao eixo Y**. Uma forma "larga" terá um `$\mu_{20}$` alto.

* **`$\mu_{02} = \sum \sum (y - c_y)^2 \cdot I(x, y)$`**
    Similarmente, `$\mu_{02}$` mede a dispersão da massa ao redor do eixo X, sendo o **momento de inércia em relação ao eixo X**. Uma forma "alta" terá um `$\mu_{02}$` alto.

### Momento Misto (`$\mu_{11}$`) e Covariância

O momento `$\mu_{11}$` é conhecido como o momento misto ou produto de inércia.
$$\mu_{11} = \sum \sum (x - c_x)(y - c_y) \cdot I(x, y)$$

**A relação é de equivalência conceitual:** O momento central `$\mu_{11}$` **é** a **covariância** da distribuição espacial dos pixels da forma.

Como vimos anteriormente, a covariância mede a tendência de duas variáveis (`$x$` e `$y$`) de variarem juntas.
* Se `$\mu_{11}$` é diferente de zero, significa que a massa da forma está concentrada nos quadrantes diagonais em relação ao centroide. Isso é uma assinatura matemática de que a forma está **inclinada** ou **desalinhada** com os eixos `$x,y$`.
* Se `$\mu_{11}$` é zero, significa que a forma é simétrica em relação aos eixos `$x,y$`, ou seja, está perfeitamente alinhada (não inclinada).

### Orientação da Forma

O objetivo final é encontrar o ângulo `$\theta$` do eixo principal da forma (seu eixo de maior elongação). Este é o ângulo para o qual precisaríamos rotacionar os eixos coordenados para que a forma fique "alinhada", ou seja, para que sua covariância se torne nula.

A fórmula que relaciona os momentos de segunda ordem para encontrar este ângulo é:

$$ \theta = \frac{1}{2} \arctan\left(\frac{2\mu_{11}}{\mu_{20} - \mu_{02}}\right) $$

Esta fórmula utiliza a medida de "inclinação" (`$\mu_{11}$`) e as medidas de "dispersão" (`$\mu_{20}$`, `$\mu_{02}$`) para calcular o ângulo de correção exato que define a orientação do objeto.

### Demonstração

**Objetivo:** Encontrar o ângulo de rotação `$\theta$` para um novo sistema de coordenadas `$(u,v)$` no qual o momento misto `$\mu'_{11}$` é zero.

**1. Equações de Rotação:**
Dado um sistema `$(x', y')$` centrado no centroide (`$x' = x-c_x$`, `$y' = y-c_y$`), rotacionamos por `$\theta$` para obter `$(u,v)$`:
$$ u = x' \cos\theta + y' \sin\theta $$
$$ v = -x' \sin\theta + y' \cos\theta $$

**2. Definição do Novo Momento Misto:**
Queremos encontrar o `$\theta$` que satisfaz a condição:
$$ \mu'_{11} = \sum \sum u \cdot v \cdot I(x, y) = 0 $$

**3. Substituição e Expansão:**
Substituímos `u` e `v` na equação:
$$ \mu'_{11} = \sum \sum (x' \cos\theta + y' \sin\theta)(-x' \sin\theta + y' \cos\theta) \cdot I(x, y) $$
Expandindo o produto, obtemos:
$$ \mu'_{11} = \sum \sum [x'y'(\cos^2\theta - \sin^2\theta) + (y'^2 - x'^2)\sin\theta \cos\theta] \cdot I(x, y) $$
Distribuindo a soma, reconhecemos os momentos centralizados originais:
$$ \mu'_{11} = \mu_{11}(\cos^2\theta - \sin^2\theta) + (\mu_{02} - \mu_{20})\sin\theta \cos\theta $$

**4. Simplificação com Identidades Trigonométricas:**
Usando as identidades de ângulo duplo, `$\cos(2\theta) = \cos^2\theta - \sin^2\theta$` e `$\sin(2\theta) = 2\sin\theta\cos\theta$`, a expressão se torna:
$$ \mu'_{11} = \mu_{11}\cos(2\theta) + \frac{1}{2}(\mu_{02} - \mu_{20})\sin(2\theta) $$

**5. Resolvendo para `$\theta$`:**
Definimos `$\mu'_{11} = 0$` e resolvemos a equação:
$$ 0 = \mu_{11}\cos(2\theta) + \frac{1}{2}(\mu_{02} - \mu_{20})\sin(2\theta) $$
$$ -\mu_{11}\cos(2\theta) = \frac{1}{2}(\mu_{02} - \mu_{20})\sin(2\theta) $$
Rearranjando para `$\tan(2\theta) = \sin(2\theta)/\cos(2\theta)$`:
$$ \tan(2\theta) = \frac{-2\mu_{11}}{\mu_{02} - \mu_{20}} = \frac{2\mu_{11}}{\mu_{20} - \mu_{02}} $$
Finalmente, isolando `$\theta$`:
$$ \theta = \frac{1}{2} \arctan\left(\frac{2\mu_{11}}{\mu_{20} - \mu_{02}}\right) $$

Isso completa a demonstração, mostrando que a orientação é o ângulo que diagonaliza a matriz de covariância da forma, alinhando os eixos de análise com a geometria intrínseca do objeto.

### Exemplo em Python (detecção de um caminho azul no chão)

```python
class LaneDetector(Node):
	def __init__(self):
		super().__init__('lane_detector')

		self._centroid_publisher = self.create_publisher(Point, 'centroid', 10)
		self._threshold_publisher = self.create_publisher(UInt8, 'threshold', 10)
		self._subscriber = self.create_subscription(
			CompressedImage,
			'camera/compressed',
			self.lane_detection_callback,
			10
		)
		self.bridge = CvBridge()

		self.threshold: UInt8 = UInt8()
		self.threshold.data = int(8) # valor do threshold
	
	def lane_detection_callback(self, msg):
		np_arr = np.frombuffer(msg.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		if cv_image is None:
			self.get_logger().error('Failed to decode compressed color image.')
			return

		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		lower_blue = np.array([100, 150, 50])
		upper_blue = np.array([140, 255, 255])

		binary_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

		contornos, hierarquia = cv2.findContours(
            binary_mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

		output_image = cv_image.copy()

		if contornos:
			# filtra por tamanho (a faixa deve ser o maior contorno na imagem)
			maior_contorno = max(contornos, key=cv2.contourArea)
			area = cv2.contourArea(maior_contorno)

			if area > 200:
				cv2.drawContours(output_image, [maior_contorno], 0, (0, 255, 0), 3)

				# técnica dos momentos de imagem
				M = cv2.moments(maior_contorno)
				A = M['m00']
				if A > 0:
					# centroide
					cx = int(M['m10']/A)
					cy = int(M['m01']/A)
					cv2.circle(output_image, (cx, cy), 7, (0, 0, 255), -1)

					# orientacao
					mu20 = M['mu20']
					mu02 = M['mu02']
					mu11 = M['mu11']
					theta = 0.5*math.atan2(2*mu11, mu20 - mu02)

					# desenha a seta da orientação
					tamanho = 100
					start_point = (cx, cy)
					end_point = (
						int(cx + tamanho*math.cos(theta)),
						int(cy + tamanho*math.sin(theta))
					)
					cv2.arrowedLine(output_image, start_point, end_point, (0, 0, 255), 3)

		#cv2.imshow('mascara', binary_mask)
		cv2.imshow('Lane Detection', output_image)
		cv2.waitKey(1)
```