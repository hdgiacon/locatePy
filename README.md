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

<br>

<br>

## Funções Principais

Nesta seção serão mostrados os algoritmos disponíveis neste repositório, com uma breve descrição da sua 
utilidade bem como os parametros necessários para o seu funcionamento.

<br>

### Occupance Grid

Algoritmo probabilistico de mapeamento de ambiente que produz uma matriz de crênça a respeito da localização dos obstáculos no ambiente simulado.

> `ocuppance_grid(raw_range_data: list, raw_angle_data: list, theta: int, posX: int, posY: int, RANGE_MAX: int, RANGE_LIMIT: int, 
    RESOLUCAO: float, LARG_GRID: int, ALT_GRID: int, posXGrid: int, posYGrid: int, m: np.float) -> None`

<br>

> `raw_range_data:` lista com o tamanho de cada feixe de laser

> `raw_angle_data:` lista com a angulação de cada feixe de laser

> `theta:` angulo a respeito da orientação do robô

> `posX:` posição do robô no mapa do simulador em relação ao eixo *x*

> `posY:` posição do robô no mapa do simulador em relação ao eixo *y*

> `RANGE_MAX:` constante com o valor maximo que um feixe de laser pode ter

> `RANGE_LIMIT:` constante com o valor minimo que um feixe laser pode ter

<br>