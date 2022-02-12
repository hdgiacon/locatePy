from random import randint
import typing as ty
from auxiliares import get_line, convertion_points

class RoboVirtual:
    def __init__(self, _posX: int, _posY: int, _pesoGlobal: float) -> None:
        self.posX = _posX
        self.posY = _posY
        self.pesoGlobal = _pesoGlobal
        

def monteCarlo(num_particles: int, alt_grid: int, larg_grid: int, raw_range_data: ty.List, raw_angle_data: ty.List, 
    theta: int, RES: float, LARG_GRID: int, ALT_GRID: int, posXGrid: int, posYGrid: int, rows: int, cols: int) -> ty.List[RoboVirtual]:
    ''' comentario sobre monte carlo '''

    conjAmostrasX: ty.List[RoboVirtual] = []
    pesosLocais: ty.List[int] = []
    k: int = 0

    #for k in num_particles:
        # predição
        # 𝑥_𝑡 ← 𝑠𝑎𝑚𝑝𝑙𝑒_𝑚𝑜𝑡𝑖𝑜𝑛_𝑚𝑜𝑑𝑒𝑙(𝑢_𝑡 , 𝑥_{𝑡−1} ) // aplica a movimentação em cada amostra 𝑥^𝑘_𝑡

        # atualização
        # 𝑚𝑒𝑎𝑠𝑢𝑟𝑒𝑚𝑒𝑛𝑡_𝑚𝑜𝑑𝑒𝑙(𝑧_𝑡 , 𝑥_𝑡 , 𝑚) // aplica o modelo de observação atribuindo peso 𝜔_𝑡
        # 𝑋_𝑡 ← 𝑋_𝑡 + ⟨𝑥_𝑡 , 𝑤_𝑡 ⟩ // o conjunto das probabilidades das amostras 𝑋_𝑡 é gerado
    #    pass

    for k in num_particles:
        conjAmostrasX.append(
            RoboVirtual(
                randint(0, larg_grid), 
                randint(0, alt_grid),
                [],
                0.0
            )
        )

        # fazer isso pro numero de feixes de lasers
        for _ in range(len(raw_range_data)):
            # bresenham para um feixe de laser do robo real
            point1_r, point2_r = convertion_points()

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
                pesosLocais.append(0)
            else:
                # calculo do peso local caso não sejam iguais
                pesosLocais.append(abs(contPesoLocal - len(path_r)))

        # calcular o peso global - media (quanto mais proximo de 0 mais proximo o robo virtual está de um robo real)
        conjAmostrasX[k].pesoGlobal = sum(pesosLocais) / len(pesosLocais)

        pesosLocais.clear


    for k in num_particles:
        # reamostragem
        # 𝑑𝑟𝑎𝑤 𝑖 𝑤𝑖𝑡ℎ 𝑝𝑟𝑜𝑏𝑎𝑏𝑖𝑙𝑖𝑡𝑦 ∝ 𝑤_𝑡^{[i]} selecionar as amostras 𝑥^𝑘_𝑡 que possuem maior peso 𝜔_𝑡^𝑘
        # 𝑋_𝑡 ∪ 𝑥_𝑡 // adiciona a 𝑋_𝑡 as amostras de maior peso
        pass

    # 𝑟𝑒𝑡𝑜𝑟𝑛𝑒 𝑋_𝑡
    return conjAmostrasX 