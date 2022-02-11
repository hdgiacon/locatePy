from random import randint
import typing as ty
import math
from auxiliares import get_line

class RoboVirtual:
    def __init__(self, posX: int, posY: int) -> None:
        self.posX = posX
        self.posY = posY
        

def monteCarlo(num_particles: int, alt_grid: int, larg_grid: int, raw_range_data: ty.List, raw_angle_data: ty.List, 
    theta: int,) -> ty.List:
    ''' comentario sobre monte carlo '''

    conjAmostrasX: ty.List = []
    k: int = 0

    #for k in num_particles:
        # predição
        # 𝑥_𝑡 ← 𝑠𝑎𝑚𝑝𝑙𝑒_𝑚𝑜𝑡𝑖𝑜𝑛_𝑚𝑜𝑑𝑒𝑙(𝑢_𝑡 , 𝑥_{𝑡−1} ) // aplica a movimentação em cada amostra 𝑥^𝑘_𝑡

        # atualização
        # 𝑚𝑒𝑎𝑠𝑢𝑟𝑒𝑚𝑒𝑛𝑡_𝑚𝑜𝑑𝑒𝑙(𝑧_𝑡 , 𝑥_𝑡 , 𝑚) // aplica o modelo de observação atribuindo peso 𝜔_𝑡
        # 𝑋_𝑡 ← 𝑋_𝑡 + ⟨𝑥_𝑡 , 𝑤_𝑡 ⟩ // o conjunto das probabilidades das amostras 𝑋_𝑡 é gerado
    #    pass

    for k in num_particles:
        conjAmostrasX.append(RoboVirtual(randint(0, larg_grid), randint(0, alt_grid)))

        xL = math.cos(raw_angle_data[k] + theta) * raw_range_data[k] + conjAmostrasX[k].posX
        yL = math.sin(raw_angle_data[k] + theta) * raw_range_data[k] + conjAmostrasX[k].posY

        path = get_line()


    for k in num_particles:
        # reamostragem
        # 𝑑𝑟𝑎𝑤 𝑖 𝑤𝑖𝑡ℎ 𝑝𝑟𝑜𝑏𝑎𝑏𝑖𝑙𝑖𝑡𝑦 ∝ 𝑤_𝑡^{[i]} selecionar as amostras 𝑥^𝑘_𝑡 que possuem maior peso 𝜔_𝑡^𝑘
        # 𝑋_𝑡 ∪ 𝑥_𝑡 // adiciona a 𝑋_𝑡 as amostras de maior peso
        pass

    # 𝑟𝑒𝑡𝑜𝑟𝑛𝑒 𝑋_𝑡
    return conjAmostrasX 