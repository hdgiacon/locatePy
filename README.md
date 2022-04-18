# locatePy

locatePy é uma biblioteca Python de código aberto voltada para a implementação de algoritmos para localização de robôs móveis para simuladores. Na versão atual há somente suporte para o simulador *CoppeliaSim*.

Esta biblioteca possui a licença GNU GPL v3, com um intuito de promover o reúso de código livre bem como a manutenção e melhoria
por parte da comunidade de software.

<br>

<br>

## Primeiros Passos

Para ter locatePy em sua máquina basta baixar este repositório compactado e utilizar todos os comandos disponíveis através do arquivo principal **locatePy.py**. Neste repositório há tambem um arquivo *main* que pode ser utilizado como base para conexão com
o simulador *CoppeliaSim*.

<br>

A lista de funções principais abaixo possuem uma breve descrição da sua funcionalidade, bem como dos seus parâmetros. Cada parâmetro possui o seu tipo e uma breve descrição, como mostra o exemplo a seguir:

<br>

> `atributo <`*`tipo`*`>` : descrição

<br>

<br>

<br>

## Funções Principais

Nesta seção serão mostrados os algoritmos disponíveis neste repositório, com uma breve descrição da sua 
utilidade bem como os parametros necessários para o seu funcionamento.

<br>

### Occupance Grid

Algoritmo probabilistico de mapeamento de ambiente que produz uma matriz de crênça a respeito da localização dos obstáculos no ambiente simulado.

> **ocuppance_grid**`(raw_range_data, raw_angle_data, theta, posX, posY, RANGE_MAX, RANGE_LIMIT, RESOLUCAO, LARG_GRID, ALT_GRID, posXGrid, posYGrid, m)`

<br>

> `raw_range_data <`*`list`*`>` : lista com o tamanho de cada feixe de laser.

> `raw_angle_data <`*`list`*`>` : lista com a angulação de cada feixe de laser.

> `theta <`*`int`*`>` : angulo a respeito da orientação do robô.

> `posX <`*`int`*`>` : posição do robô no mapa do simulador em relação ao eixo *x*.

> `posY <`*`int`*`>` : posição do robô no mapa do simulador em relação ao eixo *y*.

> `RANGE_MAX <`*`int`*`>` : constante com o valor maximo que um feixe de laser pode ter.

> `RESOLUCAO <`*`int`*`>` : é o coeficiente de proporção entre o mapa do ambiente e a *grid*.

> `LARG_GRID <`*`int`*`>` : largura da matriz de *grid*.

> `ALT_GRID <`*`int`*`>` : altura da matriz de *grid*.

> `posXGrid <`*`int`*`>` : posição *X* onde o robô está na *grid*.

> `posYGrid <`*`int`*`>` : posição *Y* onde o robô está na *grid*.

> `m: <`*`np.float`*`>` : matriz *grid*.

<br>