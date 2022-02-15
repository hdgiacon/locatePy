from random import randint
from operator import attrgetter
from auxiliares import get_line, convertion_points

class RoboVirtual:
    ''' comentario sobre a classe '''
    def __init__(self, _posX: int, _posY: int, _pesoGlobal: float) -> None:
        self.posX = _posX
        self.posY = _posY
        self.pesoGlobal = _pesoGlobal


def create_virtual_robot(conjAmostrasX: 'list[RoboVirtual]', raw_range_data: list, raw_angle_data: list, theta: int, 
    k: int, RES: float, ALT_GRID: int, LARG_GRID: int, posXGrid: int, posYGrid: int, larg_grid:int , alt_grid: int, 
    posX: int, posY: int) -> 'list[RoboVirtual]':
    ''' comentario sobre a função '''
    
    pesosLocais: list[float] = []

    conjAmostrasX.append(
        RoboVirtual(
            randint(0, larg_grid), 
            randint(0, alt_grid),
            0.0
        )
    )

    # fazer isso pro numero de feixes de lasers
    for _ in range(len(raw_range_data)):
        # bresenham para um feixe de laser do robo real
        point1_r, point2_r = convertion_points(raw_range_data, raw_angle_data, theta, posX, 
        posY, k, RES, ALT_GRID, LARG_GRID, posXGrid, posYGrid)

        path_r = get_line(point1_r, point2_r)
        
        # bresenham para um feixe de laser do robo virtual
        point1_v, point2_v = convertion_points(raw_range_data, raw_angle_data, theta, conjAmostrasX[k].posX, 
        conjAmostrasX[k].posY, k, RES, ALT_GRID, LARG_GRID, posXGrid, posYGrid)

        path_v = get_line(point1_v, point2_v)

        contPesoLocal: int = 0
        for m in path_v:
            contPesoLocal += 1
            if m >= 0.8:
                break

        if contPesoLocal == len(path_r):
            # atribuir peso caso forem iguais
            pesosLocais.append(1.0)
        else:
            # calculo do peso local caso não sejam iguais
            pesosLocais.append(1.0 - (abs(contPesoLocal - len(path_r)) / 10))

    # calcular o peso global - media (quanto mais proximo de 0 mais proximo o robo virtual está de um robo real)
    conjAmostrasX[k].pesoGlobal = sum(pesosLocais) / len(pesosLocais)

    pesosLocais.clear

    return conjAmostrasX
    

def monteCarlo(num_particles: int, alt_grid: int, larg_grid: int, raw_range_data: list, raw_angle_data: list, 
    theta: int, RES: float, LARG_GRID: int, ALT_GRID: int, posXGrid: int, posYGrid: int, posX: int, 
    posY: int) -> 'list[RoboVirtual]':
    ''' comentario sobre monte carlo '''

    # TODO: verificar se o numero de particulas é par

    conjAmostrasX: list[RoboVirtual] = []
    
    k: int = 0

    #for k in num_particles:
        # predição
        # 𝑥_𝑡 ← 𝑠𝑎𝑚𝑝𝑙𝑒_𝑚𝑜𝑡𝑖𝑜𝑛_𝑚𝑜𝑑𝑒𝑙(𝑢_𝑡 , 𝑥_{𝑡−1}) // aplica a movimentação em cada amostra 𝑥^𝑘_𝑡

        # atualização
        # 𝑚𝑒𝑎𝑠𝑢𝑟𝑒𝑚𝑒𝑛𝑡_𝑚𝑜𝑑𝑒𝑙(𝑧_𝑡 , 𝑥_𝑡 , 𝑚) // aplica o modelo de observação atribuindo peso 𝜔_𝑡
        # 𝑋_𝑡 ← 𝑋_𝑡 + ⟨𝑥_𝑡 , 𝑤_𝑡 ⟩ // o conjunto das probabilidades das amostras 𝑋_𝑡 é gerado
    #    pass

    for k in range(num_particles):
        conjAmostrasX = create_virtual_robot(conjAmostrasX, raw_range_data, raw_angle_data, theta, k, RES, ALT_GRID, LARG_GRID, 
            posXGrid, posYGrid, larg_grid, alt_grid, posX, posY)
        
    k = 0

    for k in range(num_particles):
        # reamostragem
        # 𝑑𝑟𝑎𝑤 𝑖 𝑤𝑖𝑡ℎ 𝑝𝑟𝑜𝑏𝑎𝑏𝑖𝑙𝑖𝑡𝑦 ∝ 𝑤_𝑡^{[i]} selecionar as amostras 𝑥^𝑘_𝑡 que possuem maior peso 𝜔_𝑡^𝑘
        # 𝑋_𝑡 ∪ 𝑥_𝑡 // adiciona a 𝑋_𝑡 as amostras de maior peso

        # remove os elementos de menor peso -> quanto menor, mais diferente é do feixe original
        for _ in range(int(num_particles / 4)):
            conjAmostrasX.pop(min(range(len(conjAmostrasX)), key = attrgetter('pesoGlobal')))

        # adicionar na lista n/8 particulas mediante as boas (esquema da roleta)
        for _ in range(int(num_particles / 8)):
            num_aleatorio = randint(0, 99)
            conjAmostrasX.push(next((x for x in conjAmostrasX if x.pesoGlobal == num_aleatorio), None))

        # adicionar na lista de particulas n/8 particulas aleatorias
        for _ in range(int(num_particles / 8)):
            conjAmostrasX = create_virtual_robot(conjAmostrasX, raw_range_data, raw_angle_data, theta, k, RES, ALT_GRID, LARG_GRID, 
                posXGrid, posYGrid, larg_grid, alt_grid)

    # 𝑟𝑒𝑡𝑜𝑟𝑛𝑒 𝑋_𝑡
    return conjAmostrasX 