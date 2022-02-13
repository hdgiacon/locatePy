import numpy as np

from auxiliares import get_line, convertion_points


def ocuppance_grid(raw_range_data: list, raw_angle_data: list, theta: int, posX: int, posY: int, 
    RANGE_MAX: int, RANGE_LIMIT: int, RES: float, LARG_GRID: int, ALT_GRID: int, rows: int, 
    cols: int, posXGrid: int, posYGrid: int, m: np.float, PRIORI: float) -> None:
    ''' comentario sobre a função '''

    for i in range(len(raw_range_data)):
        # inverse sensor model
        if raw_range_data[i] < RANGE_MAX * RANGE_LIMIT:
            taxaOC = 0.9
        else:
            taxaOC = 0.48

        # <codigo retirado>

        point1, point2 = convertion_points(raw_range_data, raw_angle_data, theta, posX, posY, i, RES, ALT_GRID, LARG_GRID, 
        posXGrid, posYGrid)

        cells = get_line(point1, point2)
        #print("Celulas")
        #print(cells)
        for j in range(len(cells)):
            linha, coluna = cells[j]
            linha = int(linha)
            coluna = int(coluna)

            if linha < 0:
                linha = 0
            elif linha >= LARG_GRID:
                linha = LARG_GRID-1

            if coluna < 0:
                coluna = 0
            elif coluna >= ALT_GRID:
                coluna = ALT_GRID-1

            m[linha, coluna] = 1 - pow(1 + (taxaOC/(1 - taxaOC)) * ((1 - PRIORI)/PRIORI) * (m[linha, coluna]/(1 - m[linha, coluna] + 0.00001)), -1) + 0.00001

            if taxaOC > 0.5:
                taxaOC = 0.48
            else:
                taxaOC = 0.95


'''

        # Cálculo da posição xL, yL de onde o laser bateu no simulador
        xL = math.cos(raw_angle_data[i] + theta) * raw_range_data[i] + posX
        yL = math.sin(raw_angle_data[i] + theta) * raw_range_data[i] + posY
        # posX e posY são as coordenadas do robô no ambiente

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
        line_bresenham = np.zeros((rows, cols), dtype=np.uint8)
        xi = posXGrid
        yi = posYGrid
        xoi = xLGrid
        yoi = yLGrid
        # x é coluna; y é linha
        # rr, cc = line(yi, xi, yoi, xoi)  # r0, c0, r1, c1
        point1 = (yi, xi)
        point2 = (yoi, xoi)
'''