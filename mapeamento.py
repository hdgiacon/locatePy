import numpy as np
from auxiliares import get_line, convertion_points


def ocuppance_grid(raw_range_data: list, raw_angle_data: list, theta: int, posX: int, posY: int, RANGE_MAX: int, RESOLUCAO: float, 
    LARG_GRID: int, ALT_GRID: int, posXGrid: int, posYGrid: int, m: np.float) -> None:
    ''' Algoritmo probabilistico de mapeamento de ambiente que produz uma matriz de crênça a respeito da localização dos obstáculos no ambiente simulado. '''

    for i in range(len(raw_range_data)):
        # inverse sensor model

        point1, point2, xLGrid, yLGrid = convertion_points(raw_range_data, raw_angle_data, theta, posX, posY, i, RESOLUCAO, ALT_GRID, LARG_GRID, 
        posXGrid, posYGrid)

        #TODO: posição onde o robo está (convertida pra grid)
        #auxXGrid = int((posX / RESOLUCAO) + (LARG_GRID / 2))
        #auxYGrid = int(ALT_GRID - ((posY / RESOLUCAO) + (ALT_GRID / 2)))

        #m[auxYGrid][auxXGrid] = 0.0

        cells = get_line(point1, point2)

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

            m[linha, coluna] = 0.75


        if raw_range_data[i] < RANGE_MAX * 0.95:
            m[yLGrid, xLGrid] = 1.0

            if xLGrid < 0:
                xLGrid = 0
            elif xLGrid > LARG_GRID:
                xLGrid = LARG_GRID

            if yLGrid < 0:
                yLGrid = 0
            elif yLGrid > LARG_GRID:
                yLGrid = LARG_GRID

            m[yLGrid, xLGrid - 1 if xLGrid > 0 else 0] = 1.0
            m[yLGrid, xLGrid + 1 if xLGrid < LARG_GRID-1 else 0] = 1.0

            m[yLGrid - 1 if yLGrid > 0 else 0, xLGrid] = 1.0
            m[yLGrid - 1 if yLGrid > 0 else 0, xLGrid - 1 if xLGrid > 0 else 0] = 1.0
            m[yLGrid - 1 if yLGrid > 0 else 0, xLGrid + 1 if xLGrid < LARG_GRID-1 else 0] = 1.0

            m[yLGrid + 1 if yLGrid < LARG_GRID-1 else 0, xLGrid] = 1.0
            m[yLGrid + 1 if yLGrid < LARG_GRID - 1 else 0, xLGrid - 1 if xLGrid > 0 else 0] = 1.0
            m[yLGrid + 1 if yLGrid < LARG_GRID - 1 else 0, xLGrid + 1 if xLGrid < LARG_GRID-1 else 0] = 1.0