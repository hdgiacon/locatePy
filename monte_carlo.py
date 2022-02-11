from random import randint
import typing as ty
import numpy as np
import math
from auxiliares import get_line

class RoboVirtual:
    def __init__(self, _posX: int, _posY: int, _pesosLocais: ty.List, _pesoGlobal: float) -> None:
        self.posX = _posX
        self.posY = _posY
        self.pesosLocais = _pesosLocais
        self.pesoGlobal = _pesoGlobal
        

def monteCarlo(num_particles: int, alt_grid: int, larg_grid: int, raw_range_data: ty.List, raw_angle_data: ty.List, 
    theta: int, RES: float, LARG_GRID: int, ALT_GRID: int, posXGrid: int, posYGrid: int, rows: int, cols: int) -> ty.List:
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
        conjAmostrasX.append(
            RoboVirtual(
                randint(0, larg_grid), 
                randint(0, alt_grid),
                [],
                0.0
            )
        )

        # fazer isso pro numero de feixes de lasers
        for n in len(raw_range_data):
            xL = math.cos(raw_angle_data[k] + theta) * raw_range_data[k] + conjAmostrasX[k].posX
            yL = math.sin(raw_angle_data[k] + theta) * raw_range_data[k] + conjAmostrasX[k].posY


            # Conversão da posição de onde o laser bateu no ambiente para a Grid
            xLGrid = int((xL / RES) + (LARG_GRID / 2))
            yLGrid = int(ALT_GRID - ((yL / RES) + (ALT_GRID / 2)))

            if xLGrid < 0:
                xLGrid = 0
            elif xLGrid >= LARG_GRID:
                xLGrid = LARG_GRID-1

            if yLGrid < 0:
                yLGrid = 0
            elif yLGrid >= ALT_GRID:
                yLGrid = ALT_GRID-1

            # Cálculo de todas as células de acordo com o algoritmo de Bresenham
            #line_bresenham = np.zeros((rows, cols), dtype=np.uint8)

            xi = posXGrid
            yi = posYGrid
            xoi = xLGrid
            yoi = yLGrid
            # x é coluna; y é linha
            # rr, cc = line(yi, xi, yoi, xoi)  # r0, c0, r1, c1
            point1 = (yi, xi)
            point2 = (yoi, xoi)
            #cells = get_line(point1, point2)

            path = get_line(point1, point2)

            contPesoLocal: int = 0
            for m in path:
                contPesoLocal += 1
                if m >= 0.8:
                    break

            if contPesoLocal == len(path):
                # atribuir peso caso forem iguais
                pass
            else:
                # calculo do peso local caso não sejam iguais
                pass


    for k in num_particles:
        # reamostragem
        # 𝑑𝑟𝑎𝑤 𝑖 𝑤𝑖𝑡ℎ 𝑝𝑟𝑜𝑏𝑎𝑏𝑖𝑙𝑖𝑡𝑦 ∝ 𝑤_𝑡^{[i]} selecionar as amostras 𝑥^𝑘_𝑡 que possuem maior peso 𝜔_𝑡^𝑘
        # 𝑋_𝑡 ∪ 𝑥_𝑡 // adiciona a 𝑋_𝑡 as amostras de maior peso
        pass

    # 𝑟𝑒𝑡𝑜𝑟𝑛𝑒 𝑋_𝑡
    return conjAmostrasX 