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

> `atributo `*`<tipo>`* : descrição

<br>

<br>

<br>

## Funções Principais

Nesta seção serão mostrados os algoritmos disponíveis neste repositório, com uma breve descrição da sua 
utilidade bem como os parametros necessários para o seu funcionamento.

<br>

### Occupance Grid

Algoritmo probabilistico de mapeamento de ambiente que produz uma matriz de crênça a respeito da localização dos obstáculos no ambiente simulado.

> **ocuppance_grid** `(raw_range_data, raw_angle_data, theta, posX, posY, RANGE_MAX, RANGE_LIMIT, RESOLUCAO, LARG_GRID, ALT_GRID, posXGrid, posYGrid, m)`

<br>

> `raw_range_data `*`<list>`* : lista com o tamanho de cada feixe de laser.

> `raw_angle_data `*`<list>`* : lista com a angulação de cada feixe de laser.

> `theta `*`<int>`* : angulo a respeito da orientação do robô.

> `posX `*`<int>`* : posição do robô no mapa do simulador em relação ao eixo *x*.

> `posY `*`<int>`* : posição do robô no mapa do simulador em relação ao eixo *y*.

> `RANGE_MAX `*`<int>`* : constante com o valor maximo que um feixe de laser pode ter.

> `RESOLUCAO `*`<int>`* : coeficiente de proporção entre o mapa do ambiente e a *grid*.

> `LARG_GRID `*`<int>`* : largura da matriz *grid*.

> `ALT_GRID `*`<int>`* : altura da matriz *grid*.

> `posXGrid `*`<int>`* : posição *X* onde o robô está na *grid*.

> `posYGrid `*`<int>`* : posição *Y* onde o robô está na *grid*.

> `m: `*`<np.float>`* : matriz *grid*.

<br>

<br>

<br>

### Monte Carlo

Algoritmo de localização de robôs baseado no espalhamento de partículas pelo mapa a fim de encontrar aquela que mais se assemelha à sua leitura real.

> **monteCarlo** `(conjAmostrasX, num_particles, raw_range_data, raw_angle_data, LARG_GRID, ALT_GRID,       COEF_PROP, grid, RANGE_MAX)`

<br>

> `conjAmostrasX `*`<list>`* : conjunto que contém todas as partículas.

> `num_particles `*`<int>`* : número de partículas.

> `raw_range_data `*`<list>`* : lista com o tamanho de cada feixe de laser do robô real vindo do simulador.

> `raw_angle_data `*`<list>`* : lista com a angulação de cada feixe de laser do robô real vindo do simulador.

> `LARG_GRID `*`<int>`* : largura da matriz *grid*.

> `ALT_GRID `*`<int>`* : altura da matriz *grid*.

> `COEF_PROP `*`<float>`* : coeficiente de proporção entre o mapa do ambiente e a *grid*.

> `grid `*`<np.array>`* : matriz *grid*

> `RANGE_MAX `*`<int>`* : constante com o valor maximo que um feixe de laser pode ter.

<br>

<br>

<br>

### Navegação Base

Modelo de movimentação do robô real no simulador.

> **navegacao_base** `(laser_data, clientID, i, r, L, l_wheel, r_wheel)`

<br>

> `laser_data`*`<list>`* : lista com o tamanho e angulação de cada feixe de laser do robô real.

> `clientId `*`<...>`* : ...

> `i`*`<...> `* : ...

> `r`*`<...> `* : ...

> `L`*`<...> `* : ...

> `l_wheel `*`<float>`* : variável que referencia a roda esquerda do robô real no simulador.

> `r_wheel `*`<float>`* : variável que referencia a roda direita do robô real no simulador.

<br>

<br>

<br>

### Navegação Partícula Base

Modelo de movimentação das partículas baseado no ambiente do simulador.

> **navegacao_particula_base** `(particula, LARG_GRID, ALT_GRID)`

<br>

> `particula `*`<RoboVirtual>`* : robô virtual.

> `LARG_GRID `*`<int>`* : largura da matriz *grid*.

> `ALT_GRID `*`<int>`* : altura da matriz *grid*.

<br>

<br>

<br>

### Get Line

Aplica o método de Linha de Bresenham, criando uma lista de pontos, de uma origem até o fim.

> **get_line** `(start, end)`

<br>

> `start `*`<int>`* : ponto de inicio.

> `end `*`<int>`* : ponto final.

<br>

<br>

<br>

### Convertion Points

Converte valores em termos do simulador para termos da *grid*.

> **convertion_points** `(raw_range_data, raw_angle_data, theta, posX, posY, k, RESOLUCAO, ALT_GRID,LARG_GRID, posXGrid, posYGrid)`

<br>

> `raw_range_data `*`<list>`* : lista com o tamanho de cada feixe de laser do robô real vindo do simulador.

> `raw_angle_data `*`<list>`* : lista com a angulação de cada feixe de laser do robô real vindo do simulador.

> `theta `*`<float>`* : angulação do robô real no simulador.

> `posX `*`<int>`* : posição *X* do robô real no simulador.

> `posY `*`<int>`* : posição *Y* do robô real no simulador.

> `k `*`<int>`* : indice para acessar uma posição de *raw_range_data* e *raw_angle_data*.

> `RESOLUCAO `*`<float>`* : coeficiente de proporção entre o mapa do ambiente e a *grid*.

> `ALT_GRID `*`<list>`* : altura da matriz *grid*.

> `LARG_GRID `*`<list>`* : largura da matriz *grid*.

> `posXGrid `*`<list>`* : posição *X* onde o robô está na *grid*.

> `posYGrid `*`<list>`* : posição *Y* onde o robô está na *grid*.

<br>

<br>

<br>

### Convertion Points Particle

Converte os atributos de uma partícula para termos do simulador.

> **convertion_points_particle** `(raw_angle_data, posX, posY, theta, k, RESOLUCAO, ALT_GRID, LARG_GRID, posXGrid, posYGrid)`

> `raw_angle_data `*`<list>`* : lista com a angulação de cada feixe de laser do robô real vindo do simulador.

> `posX `*`<int>`* : posição *X* do robô virtual em termos do simulador.

> `posY `*`<int>`* : posição *Y* do robô virtual em termos do simulador.

> `theta `*`<float>`* : angulação do robô virtual.

> `k `*`<int>`* : indice para acessar uma posição de *raw_range_data* e *raw_angle_data*.

> `RESOLUCAO `*`<float>`* : coeficiente de proporção entre o mapa do ambiente e a *grid*.



> `posXGrid `*`<list>`* : posição *X* onde o robô está na *grid*.

> `posYGrid `*`<list>`* : posição *Y* onde o robô está na *grid*.

<br>

<br>

<br>

### Create Virtual Robot

Cria uma partícula e a adiciona em *conjuntoAmostrasX*.

> **create_virtual_robot** `(conjAmostrasX, LARG_GRID , ALT_GRID, grid)`

<br>

> `conjAmostrasX `*`<list>`* : conjunto que contém todas as partículas.

> `ALT_GRID `*`<list>`* : largura da matriz *grid*.

> `LARG_GRID `*`<list>`* : altura da matriz *grid*.

> `grid `*`<np.array>`* : matriz *grid*