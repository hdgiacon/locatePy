import numpy as np
from auxiliares import get_line, convertion_points


def ocuppance_grid(raw_range_data: list, raw_angle_data: list, theta: int, posX: int, posY: int, RANGE_MAX: int, RANGE_LIMIT: int, 
    RESOLUCAO: float, LARG_GRID: int, ALT_GRID: int, posXGrid: int, posYGrid: int, m: np.float, PRIORI: float) -> None:
    ''' comentario sobre a função '''

    for i in range(len(raw_range_data)):
        # inverse sensor model
        if raw_range_data[i] < RANGE_MAX * RANGE_LIMIT:
            taxaOC = 0.9
        else:
            taxaOC = 0.48

        point1, point2 = convertion_points(raw_range_data, raw_angle_data, theta, posX, posY, i, RESOLUCAO, ALT_GRID, LARG_GRID, 
        posXGrid, posYGrid)

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

            m[linha, coluna] = 1 - pow(1 + (taxaOC/(1 - taxaOC)) * ((1 - PRIORI)/PRIORI) * (m[linha, coluna]/(1 - m[linha, coluna] \
                + 0.00001)), -1) + 0.00001

            if taxaOC > 0.5:
                taxaOC = 0.48
            else:
                taxaOC = 0.95
