# coding=utf-8
# Insert in a script in Coppelia
# simRemoteApi.start(19999)
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



import numpy as np
import matplotlib.pyplot as plt
import time
import typing as ty

from mapeamento import ocuppance_grid
from navegacao import navegacao_base
from auxiliares import readSensorData



# duvidas:

    # plotar o trajeto do monte carlo e do robo real (coppelia)
    # plotar um grafico de linha sobre a diferença da distancia do MT e a distancia real



'''     Inicio definição das constantes, parâmetros e do mapa<Grid>     '''

LARG_GRID: ty.Final = 1000
ALT_GRID: ty.Final = 1000
COEF_PROPOR: ty.Final = 0.015
RANGE_MAX: ty.Final = 5
RANGE_LIMIT: ty.Final = 0.3
PRIORI: ty.Final = 0.5
CELL_SIZE: ty.Final = 1

fig = plt.figure(figsize=(8, 8), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

map_size = np.array([LARG_GRID, ALT_GRID])


rows, cols = (map_size / CELL_SIZE).astype(int)

map = np.random.uniform(low=0.0, high=1.0, size=(rows, cols))
# inicialização das células da grid com valor de probabilidade desconhecido
map[::, ::] = PRIORI

# a primeira e última célula são inicializadas para que o valor 0.5 seja o intermediário e a matriz toda cinza
map[0, 0] = 1
map[LARG_GRID-1, ALT_GRID-1] = 0



def main() -> None:

    print('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if clientID != -1:
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Number of objects in the scene: ', (objs))
        else:
            print('Remote API function call returned with errlenor code: ', res)

        # Iniciando a simulação
        # Deve usar a porta do 'continuous remote API server services' (remoteApiConnections.txt)
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

        print('Connected to remote API server')
        sim.simxAddStatusbarMessage(clientID, 'Iniciando...', sim.simx_opmode_oneshot_wait)
        time.sleep(0.02)

        # Handle para o Robô
        robotname = 'Pioneer_p3dx'
        erro, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)

        # Handle para as juntas das RODAS
        returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
        returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

        # Criar stream de dados
        [returnCode, positionrobot] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming)
        [returnCode, orientationrobot] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_streaming)
        time.sleep(2)

        # Handle para os dados do LASER
        laser_range_data = "hokuyo_range_data"
        laser_angle_data = "hokuyo_angle_data"

        # Geralmente a primeira leitura é inválida (atenção ao Operation Mode)
        # Em loop até garantir que as leituras serão válidas
        returnCode = 1
        while returnCode != 0:
            returnCode, range_data = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10)

        # Prosseguindo com as leituras
        raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
        laser_data = np.array([raw_angle_data, raw_range_data]).T

        # print(laser_data)
        # # draw_laser_data(laser_data, 5)
        # # print('QUANTIDADE DE LEITURAS: ', len(laser_data))

        # Dados do Pioneer
        L = 0.381  # Metros
        r = 0.0975  # Metros

        tempo = 0
        # Lembrar de habilitar o 'Real-time mode'
        startTime = time.time()
        lastTime = startTime
        dt = 0
        i = 0
        while tempo < 240:
            now = time.time()
            dt = now - lastTime

            # Fazendo leitura do laser
            raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
            laser_data = np.array([raw_angle_data, raw_range_data]).T
            # print('Range: ', raw_range_data)

            returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            posX, posY, posZ = pos
            print('Pos: ', pos)

            # Conversão da posição do robô no ambiente para a Grid
            posXGrid = int((posX / COEF_PROPOR) + (LARG_GRID / 2))
            posYGrid = int(ALT_GRID - ((posY / COEF_PROPOR) + (ALT_GRID / 2)))
            print('PosGRID: ', posXGrid, posYGrid)

            returnCode, th = sim.simxGetObjectOrientation(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            ty, tz, theta = th
            #print('Orientation: ', theta)


            '''     Mapeamento -> Occupance Grid     '''
            
            ocuppance_grid(raw_range_data, raw_angle_data, theta, posX, posY, RANGE_MAX, RANGE_LIMIT, 
                COEF_PROPOR, LARG_GRID, ALT_GRID, rows, cols, posXGrid, posYGrid, map, PRIORI)


            '''     Navegação -> base    '''

            navegacao_base(laser_data, clientID, i, r, L, l_wheel, r_wheel)


            tempo = tempo + dt
            i = i + 1
            lastTime = now

        # end while

        # Parando o robô
        sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

        # Parando a simulação
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)

        plt.imshow(map, cmap='Greys', origin='upper', extent=(0, cols, rows, 0))
        plt.show(10)

    else:
        print('Failed connecting to remote API server')

    print('Program ended')


main()