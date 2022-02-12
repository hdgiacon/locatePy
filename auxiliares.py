try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')



import typing as ty
import math

### 
def readSensorData(clientId=-1, range_data_signal_id: str = "hokuyo_range_data", 
    angle_data_signal_id: str = "hokuyo_angle_data") -> 'tuple[ty.List, ty.list]' or None:
    ''' retorna a distancia e o angulo de cada sensor do robô '''

    # a primeira chamada deve ser sem blocking para evitar a obtenção de dados de ângulo fora de sincronia
    returnCodeRanges, string_range_data = sim.simxGetStringSignal(clientId, range_data_signal_id, sim.simx_opmode_streaming)

    # a segunda chamada deve ter blocking para evitar cenários fora de sincronia
    # between your python script and the simulator's main loop
    # (your script may be slower than the simulator's main loop, thus
    # slowing down data processing)
    returnCodeAngles, string_angle_data = sim.simxGetStringSignal(clientId, angle_data_signal_id, sim.simx_opmode_blocking)

    # caso a distancia e o algulo tenham sido retornados do simulador
    if returnCodeRanges == 0 and returnCodeAngles == 0:
        # passa o dado de string para float
        raw_range_data = sim.simxUnpackFloats(string_range_data)
        raw_angle_data = sim.simxUnpackFloats(string_angle_data)

        return raw_range_data, raw_angle_data

    # Nulo caso não haja retorno do simulador
    return None



### retorna uma lista de tuplas com as coordenadas de um ponto inicial a um ponto final
def get_line(start: int, end: int) -> ty.List[ty.Tuple[int, int]]:
    '''
    Bresenham's Line Algorithm
    
    Produces a list of tuples from start and end

    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]

    '''
    

    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    x1 = int(x1)
    x2 = int(x2)
    for x in range(x1, x2+1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    return points

def convertion_points(raw_range_data: ty.List, raw_angle_data: ty.List, theta: float, posX: int, posY: int, k: int, RES: float, 
    ALT_GRID: int, LARG_GRID: int, posXGrid: int, posYGrid: int) -> int and int:
    ''' comentario sobre a função '''
    
    xL = math.cos(raw_angle_data[k] + theta) * raw_range_data[k] + posX
    yL = math.sin(raw_angle_data[k] + theta) * raw_range_data[k] + posY


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

    return point1, point2