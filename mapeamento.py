import math
import numpy as np
import typing as ty

from auxiliares import get_line


def ocuppance_grid(raw_range_data: ty.List, raw_angle_data: ty.List, theta: int, posX: int, posY: int, 
    RANGE_MAX: int, RANGE_LIMIT: int, RES: float, LARGGRID: int, ALTGRID: int, rows: int, 
    cols: int, posXGrid: int, posYGrid: int, m: np.float, PRIORI: float) -> None:
    ''' comentario sobre a função '''

    for i in range(len(raw_range_data)):
        # inverse sensor model
        if raw_range_data[i] < RANGE_MAX * RANGE_LIMIT:
            taxaOC = 0.9
        else:
            taxaOC = 0.48

        # Cálculo da posição xL, yL de onde o laser bateu no simulador
        xL = math.cos(raw_angle_data[i] + theta) * raw_range_data[i] + posX
        yL = math.sin(raw_angle_data[i] + theta) * raw_range_data[i] + posY
        # posX e posY são as coordenadas do robô no ambiente

        # Conversão da posição de onde o laser bateu no ambiente para a Grid
        xLGrid = int((xL / RES) + (LARGGRID / 2))
        yLGrid = int(ALTGRID - ((yL / RES) + (ALTGRID / 2)))

        if xLGrid < 0:
            xLGrid = 0
        elif xLGrid >= LARGGRID:
            xLGrid = LARGGRID-1

        if yLGrid < 0:
            yLGrid = 0
        elif yLGrid >= ALTGRID:
            yLGrid = ALTGRID-1

        # Cálculo de todas as células de acordo com o algoritmo de Bresenham
        line_bresenham = np.zeros((rows, cols), dtype=np.uint8)
        xi = posXGrid
        yi = posYGrid
        xoi = xLGrid
        yoi = yLGrid
        # x é coluna; y é linha
        # rr, cc = line(yi, xi, yoi, xoi)  # r0, c0, r1, c1
        point1 = (yi, xi)
        point2 = (yoi, xoi)
        cells = get_line(point1, point2)
        #print("Celulas")
        #print(cells)
        for j in range(len(cells)):
            linha, coluna = cells[j]
            linha = int(linha)
            coluna = int(coluna)

            if linha < 0:
                linha = 0
            elif linha >= LARGGRID:
                linha = LARGGRID-1

            if coluna < 0:
                coluna = 0
            elif coluna >= ALTGRID:
                coluna = ALTGRID-1

            m[linha, coluna] = 1 - pow(1 + (taxaOC/(1 - taxaOC)) * ((1 - PRIORI)/PRIORI) * (m[linha, coluna]/(1 - m[linha, coluna] + 0.00001)), -1) + 0.00001

            if taxaOC > 0.5:
                taxaOC = 0.48
            else:
                taxaOC = 0.95