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



import math
import numpy as np
from random import randint
import sys


def readSensorData(clientId=-1, range_data_signal_id: str = "hokuyo_range_data", 
    angle_data_signal_id: str = "hokuyo_angle_data") -> 'tuple[list, list]' or None:
    ''' retorna a distancia e o angulo de cada sensor do robô '''


    returnCodeRanges, string_range_data = sim.simxGetStringSignal(clientId, range_data_signal_id, sim.simx_opmode_streaming)

    returnCodeAngles, string_angle_data = sim.simxGetStringSignal(clientId, angle_data_signal_id, sim.simx_opmode_blocking)


    if returnCodeRanges == 0 and returnCodeAngles == 0:
        raw_range_data = sim.simxUnpackFloats(string_range_data)
        raw_angle_data = sim.simxUnpackFloats(string_angle_data)

        return raw_range_data, raw_angle_data

    return None



def get_line(start: int, end: int) -> 'list[tuple[int, int]]':
    ''' Aplica o método de Linha de Bresenham, criando uma lista de pontos, de uma origem até o fim '''
    
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    is_steep = abs(dy) > abs(dx)

    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    dx = x2 - x1
    dy = y2 - y1

    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

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

    if swapped:
        points.reverse()

    return points

def convertion_points(raw_range_data: list, raw_angle_data: list, theta: float, posX: int, posY: int, k: int, RESOLUCAO: float, 
    ALT_GRID: int, LARG_GRID: int, posXGrid: int, posYGrid: int) -> int and int and int and int:
    ''' Converte valores em termos do simulador para termos da grid '''
    
    xL = math.cos(raw_angle_data[k] + theta) * raw_range_data[k] + posX
    yL = math.sin(raw_angle_data[k] + theta) * raw_range_data[k] + posY

    xLGrid = int((xL / RESOLUCAO) + (LARG_GRID / 2))
    yLGrid = int(ALT_GRID - ((yL / RESOLUCAO) + (ALT_GRID / 2)))

    if xLGrid < 0:
        xLGrid = 0
    elif xLGrid >= LARG_GRID:
        xLGrid = LARG_GRID-1

    if yLGrid < 0:
        yLGrid = 0
    elif yLGrid >= ALT_GRID:
        yLGrid = ALT_GRID-1

    xi = posXGrid
    yi = posYGrid
    xoi = xLGrid
    yoi = yLGrid

    point1 = (yi, xi)
    point2 = (yoi, xoi)

    return point1, point2, xLGrid, yLGrid


def convertion_points_particle(raw_angle_data: list, posX: int, posY: int, theta: float, k: int, 
    RESOLUCAO: float, ALT_GRID: int, LARG_GRID: int, posXGrid: int, 
    posYGrid: int) -> int and int and int and int:
    ''' Converte os atributos de uma partícula para termos do simulador '''
    
    xL = math.cos(raw_angle_data[k] + theta) * 5 + posX
    yL = math.sin(raw_angle_data[k] + theta) * 5 + posY

    xLGrid = int((xL / RESOLUCAO) + (LARG_GRID / 2))
    yLGrid = int(ALT_GRID - ((yL / RESOLUCAO) + (ALT_GRID / 2)))

    if xLGrid < 0:
        xLGrid = 0
    elif xLGrid >= LARG_GRID:
        xLGrid = LARG_GRID-1

    if yLGrid < 0:
        yLGrid = 0
    elif yLGrid >= ALT_GRID:
        yLGrid = ALT_GRID-1

    xi = posXGrid 
    yi = posYGrid
    xoi = xLGrid
    yoi = yLGrid

    point1 = (yi, xi)
    point2 = (yoi, xoi)

    return point1, point2, xLGrid, yLGrid



class RoboVirtual:
    ''' Classe para uma partícula '''
    def __init__(self, _posX: int, _posY: int, _posXReal: int, _posYReal: int, _theta: float, _pesoParticula: float, 
        _pesoGlobal: float, _pesoRoleta: int, _range_data: list, _laser_data: list) -> None:
        self.posX: int = _posX                      # em relação a grid
        self.posY: int = _posY                      # em relação a grid
        self.posXReal: int = _posXReal              # em relação ao mapa coppelia
        self.posYReal: int = _posYReal              # em relação ao mapa coppelia
        self.theta: float = _theta                  # angulo do robo no mapa
        self.pesoParticula: float = _pesoParticula  # peso local -> media dos pesos dos feixes
        self.pesoGlobal: float = _pesoGlobal        # peso global -> peso em relação à todas as partículas
        self.pesoRoleta: int = _pesoRoleta          # peso roleta -> peso acumulado em relação à todas as partículas
        self.range_data: list = _range_data         # para modelo de observação e movimentação da particula
        self.laser_data: list = _laser_data         # tupla de range_data da particula com angle data


def create_virtual_robot(conjAmostrasX: 'list[RoboVirtual]', larg_grid:int , alt_grid: int, grid: np.array) -> 'list[RoboVirtual]':
    ''' Cria uma partícula e a adiciona em conjuntoAmostrasX '''

    aux_theta = randint(0, 360 - 1)

    while True:
        auxX = randint(0, larg_grid - 1)
        auxY = randint(0, alt_grid - 1)

        # espaço conhecido e não é um obstáculo
        if grid[auxX][auxY] == 0.75:
            conjAmostrasX.append(
                RoboVirtual(
                    auxX, 
                    auxY,
                    0,
                    0,
                    aux_theta,
                    0.0,
                    0.0,
                    0,
                    [],
                    []
                )
            )
            break

    return conjAmostrasX




def ocuppancy_grid(raw_range_data: list, raw_angle_data: list, theta: int, posX: int, posY: int, RANGE_MAX: int, RESOLUCAO: float, 
    LARG_GRID: int, ALT_GRID: int, posXGrid: int, posYGrid: int, m: np.float) -> None:
    ''' Algoritmo probabilistico de mapeamento de ambiente que produz uma matriz de crênça a respeito da localização dos obstáculos no ambiente simulado. '''

    for i in range(len(raw_range_data)):
        # inverse sensor model

        point1, point2, xLGrid, yLGrid = convertion_points(raw_range_data, raw_angle_data, theta, posX, posY, i, RESOLUCAO, ALT_GRID, LARG_GRID, 
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


    

def monteCarlo(conjAmostrasX: 'list[RoboVirtual]', num_particles: int, raw_range_data: list, 
    raw_angle_data: list, LARG_GRID: int, ALT_GRID: int, COEF_PROP: float,
    grid: np.array, RANGE_MAX: int) -> 'list[RoboVirtual]':
    ''' Algoritmo de localização de robôs baseado no espalhamento de partículas pelo mapa a fim de encontrar aquela que mais se assemelha à sua leitura real. '''


    if num_particles % 4 == 0 and num_particles % 8 == 0:

        pesosLocais: list[float] = []
        contPesoLocal: int = 0

        for particle in conjAmostrasX:

            particle.range_data.clear()

            for k in range(len(raw_range_data)):

                particle.posXReal = int((COEF_PROP * (2 * particle.posX - LARG_GRID)) / 2)
                particle.posYReal = int((COEF_PROP * (2 * particle.posY - LARG_GRID)) / 2)

                point1_v, point2_v, _, _ = convertion_points_particle(raw_angle_data, particle.posXReal, particle.posYReal, 
                particle.theta, k, COEF_PROP, ALT_GRID, LARG_GRID, particle.posX, particle.posY)

                path_v = get_line(point1_v, point2_v)

                contPesoLocal = 0
                for m in path_v:
                    contPesoLocal += 1   

                    if grid[m[1]][m[0]] == 1.0:     
                        break

                particle.range_data.append(path_v[contPesoLocal-1])

                if contPesoLocal == len(path_v):
                    pesosLocais.append(1.0)
                    
                else:
                    pesosLocais.append(1.0 - (abs(contPesoLocal - len(path_v)) / int(RANGE_MAX / COEF_PROP)))

            particle.pesoParticula = sum(pesosLocais) / len(pesosLocais)
            
            pesosLocais.clear


        soma_peso_particula: float = 0.0
        for particle in conjAmostrasX:
            soma_peso_particula = soma_peso_particula + particle.pesoParticula

        for particle in conjAmostrasX:
            particle.pesoGlobal = particle.pesoParticula / soma_peso_particula

            if conjAmostrasX.index(particle) == 0:
                particle.pesoRoleta = particle.pesoGlobal * 100
            else:
                particle.pesoRoleta = conjAmostrasX[conjAmostrasX.index(particle) - 1].pesoRoleta + (particle.pesoGlobal * 100)


        for _ in range(int(num_particles / 4)):             
            conjAmostrasX.pop(conjAmostrasX.index(min(conjAmostrasX, key=lambda x: x.pesoGlobal)))

        for _ in range(int(num_particles / 8)):
            num_aleatorio = randint(0, 99)
            for particle in conjAmostrasX:
                if num_aleatorio >= particle.pesoRoleta:
                    conjAmostrasX.append(particle)
                    break

        for _ in range(int(num_particles / 8)):
            conjAmostrasX = create_virtual_robot(conjAmostrasX, ALT_GRID, LARG_GRID, grid)

            for k in range(len(raw_range_data)):

                conjAmostrasX[len(conjAmostrasX)-1].posXReal = int((COEF_PROP * (2 * conjAmostrasX[len(conjAmostrasX)-1].posX - LARG_GRID)) / 2)
                conjAmostrasX[len(conjAmostrasX)-1].posYReal = int((COEF_PROP * (2 * conjAmostrasX[len(conjAmostrasX)-1].posY - LARG_GRID)) / 2)

                point1_v, point2_v, _, _ = convertion_points_particle(raw_angle_data, conjAmostrasX[len(conjAmostrasX)-1].posXReal, 
                conjAmostrasX[len(conjAmostrasX)-1].posYReal, conjAmostrasX[len(conjAmostrasX)-1].theta, k, COEF_PROP, ALT_GRID, 
                LARG_GRID, conjAmostrasX[len(conjAmostrasX)-1].posX, conjAmostrasX[len(conjAmostrasX)-1].posY)

                path_v = get_line(point1_v, point2_v)

                contPesoLocal = 0
                for m in path_v:
                    contPesoLocal += 1
                    if grid[m[1]][m[0]] == 1.0:
                        break

                conjAmostrasX[len(conjAmostrasX)-1].range_data.append(path_v[contPesoLocal-1])

                if contPesoLocal == len(path_v):
                    pesosLocais.append(1.0)
                    
                else:
                    pesosLocais.append(1.0 - (abs(contPesoLocal - len(path_v)) / int(RANGE_MAX / COEF_PROP)))

            conjAmostrasX[len(conjAmostrasX)-1].pesoParticula = sum(pesosLocais) / len(pesosLocais)

        soma_peso_particula: float = 0.0
        for particle in conjAmostrasX:
            soma_peso_particula = soma_peso_particula + particle.pesoParticula

        for particle in conjAmostrasX:
            particle.pesoGlobal = particle.pesoParticula / soma_peso_particula

            if conjAmostrasX.index(particle) == 0:
                particle.pesoRoleta = particle.pesoGlobal * 100
            else:
                particle.pesoRoleta = conjAmostrasX[conjAmostrasX.index(particle) - 1].pesoRoleta + (particle.pesoGlobal * 100)

        

        # 𝑟𝑒𝑡𝑜𝑟𝑛𝑒 𝑋_𝑡
        return conjAmostrasX

    else:
        print("O numero de partículas deve ser divisível por 4 e por 8!!! Simulação encerrada...")
        sys.exit(0)




def navegacao_base(laser_data, clientID, i, r, L, l_wheel, r_wheel) -> None:
    ''' Modelo de movimentação do robô real no simulador '''

    v = 0
    w = np.deg2rad(0)

    frente = int(len(laser_data) / 2)
    lado_direito = int(len(laser_data) * 1 / 4)
    lado_esquerdo = int(len(laser_data) * 3 / 4)

    if laser_data[frente, 1] < 1:
        v = 0.05
        w = np.deg2rad(-20)
    elif laser_data[lado_direito, 1] < 1:
        v = 0.05
        w = np.deg2rad(20)
    elif laser_data[lado_esquerdo, 1] < 1:
        v = 0.05
        w = np.deg2rad(-20)
    else:
        v = 0.5
        w = 0

    sim.simxAddStatusbarMessage(clientID, str(i) + '- Frente: ' + str(laser_data[frente, 1]) + ' - Direito: ' + str(
        laser_data[lado_direito, 1]) + ' - Esquerdo: ' + str(laser_data[lado_esquerdo, 1]),
                                sim.simx_opmode_oneshot_wait)

    wl = v / r - (w * L) / (2 * r)
    wr = v / r + (w * L) / (2 * r)

    sim.simxSetJointTargetVelocity(clientID, l_wheel, wl, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, wr, sim.simx_opmode_streaming + 5)


def navegacao_particula_base(particula: RoboVirtual, LARG_GRID: int, ALT_GRID: int) -> None:
    ''' Modelo de movimentação das partículas baseado no ambiente do simulador '''

    laser_data = particula.laser_data

    frente = int(len(laser_data) / 2)
    lado_direito = int(len(laser_data) * 1 / 4)
    lado_esquerdo = int(len(laser_data) * 3 / 4)

    if laser_data[frente, 1] < 1:
        if particula.theta <= 90:     
            particula.posX -= 2
            particula.posY += 2

        elif particula.theta <= 180:  
            particula.posX += 2
            particula.posY += 2

        elif particula.theta <= 270:  
            particula.posX += 2
            particula.posY -= 2
            
        elif particula.theta <= 360: 
            particula.posX -= 2
            particula.posY -= 2

    elif laser_data[lado_direito, 1] < 1:
        if particula.theta <= 90:
            particula.posX -= 2
            particula.posY += 2

        elif particula.theta <= 180:
            particula.posX += 2
            particula.posY += 2

        elif particula.theta <= 270:
            particula.posX += 2
            particula.posY -= 2

        elif particula.theta <= 360:
            particula.posX -= 2
            particula.posY -= 2

    elif laser_data[lado_esquerdo, 1] < 1:
        if particula.theta <= 90:
            particula.posX += 2
            particula.posY -= 2

        elif particula.theta <= 180:
            particula.posX -= 2
            particula.posY -= 2

        elif particula.theta <= 270:
            particula.posX -= 2
            particula.posY += 2

        elif particula.theta <= 360:
            particula.posX += 2
            particula.posY += 2

    else:
        if particula.theta <= 90:
            particula.posX += 2
            particula.posY -= 2

        elif particula.theta <= 180:
            particula.posX -= 2
            particula.posY -= 2

        elif particula.theta <= 270:
            particula.posX -= 2
            particula.posY += 2

        elif particula.theta <= 360:
            particula.posX += 2
            particula.posY += 2


    if particula.posX >= LARG_GRID:
        particula.posX = LARG_GRID - 1

    if particula.posY >= ALT_GRID:
        particula.posY = ALT_GRID - 1

    if particula.posX < 0:
        particula.posX = 0

    if particula.posY < 0:
        particula.posY = 0
    