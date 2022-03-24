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



# duvidas:

    # plotar o trajeto do monte carlo e do robo real (coppelia)
    # plotar um grafico de linha sobre a diferença da distancia do MT e a distancia real



from math import sin
import numpy as np
import matplotlib.pyplot as plt
import time

from mapeamento import ocuppance_grid
from monte_carlo import RoboVirtual, monteCarlo
from navegacao import navegacao_base, navegacao_particula_base
from auxiliares import readSensorData
from monte_carlo import create_virtual_robot



def main(map_dimension: int, numParticles: int, numReamostragens: int) -> None:

    import typing as ty

    '''     Inicio definição das constantes, parâmetros e do mapa<Grid>     '''

    LARG_GRID: ty.Final and int = map_dimension
    ALT_GRID: ty.Final and int = map_dimension

    # coeficiente de proporção
    RESOLUCAO: ty.Final and float = 15 / map_dimension
    RANGE_MAX: ty.Final and int = 5
    RANGE_LIMIT: ty.Final and float = 0.3
    PRIORI: ty.Final and float = 0.5
    CELL_SIZE: ty.Final and int = 1

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
        L: ty.Final and float = 0.381  # Metros
        r: ty.Final and float = 0.0975  # Metros

        tempo: int = 0
        # Lembrar de habilitar o 'Real-time mode'
        startTime = time.time()
        lastTime = startTime
        dt = 0
        i = 0
        while tempo < numReamostragens:
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
            posXGrid = int((posX / RESOLUCAO) + (LARG_GRID / 2))
            posYGrid = int(ALT_GRID - ((posY / RESOLUCAO) + (ALT_GRID / 2)))
            print('PosGRID: ', posXGrid, posYGrid)

            returnCode, th = sim.simxGetObjectOrientation(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            ty, tz, theta = th
            #print('Orientation: ', theta)


            '''     Mapeamento -> Occupance Grid     '''
            # res
            ocuppance_grid(raw_range_data, raw_angle_data, theta, posX, posY, RANGE_MAX, RANGE_LIMIT, 
                RESOLUCAO, LARG_GRID, ALT_GRID, posXGrid, posYGrid, map, PRIORI)


            '''     Navegação -> base    '''

            navegacao_base(laser_data, clientID, i, r, L, l_wheel, r_wheel)



            tempo = tempo + dt
            i = i + 1
            lastTime = now

        # end while <geração do mapa>

        print("\n Fim do mapeamento, inicio da localização \n")

        '''     Criação das Partículas      '''

        #TODO: acho que o tempo gasto esta aqui, o robo continua a andar enquanto as particulas são geradas

        # criar n particulas
        conjAmostrasX: list[RoboVirtual] = []
        for _ in range(numParticles):
            conjAmostrasX = create_virtual_robot(conjAmostrasX, ALT_GRID, LARG_GRID, map)


        path_real: list[tuple[int,int]] = []
        path_monte_carlo: list[tuple[int,int]] = []

        tempo = 0
        # Lembrar de habilitar o 'Real-time mode'
        startTime = time.time()
        lastTime = startTime
        dt = 0
        i = 0
        while tempo < numReamostragens:
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
            posXGrid = int((posX / RESOLUCAO) + (LARG_GRID / 2))
            posYGrid = int(ALT_GRID - ((posY / RESOLUCAO) + (ALT_GRID / 2)))
            print('PosGRID: ', posXGrid, posYGrid)

            returnCode, th = sim.simxGetObjectOrientation(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            ty, tz, theta = th
            #print('Orientation: ', theta)


            '''     Monte Carlo     '''

            conjAmostrasX = monteCarlo(conjAmostrasX, numParticles, ALT_GRID, LARG_GRID, raw_range_data, 
                raw_angle_data, theta, LARG_GRID, ALT_GRID, posXGrid, RESOLUCAO, posYGrid, posX, posY, map)

            # pegar a coodenada da particula de maior peso
            conjAmostrasX.sort(key = lambda x: x.pesoGlobal)
            path_monte_carlo.append([conjAmostrasX[len(conjAmostrasX)-1].posX,conjAmostrasX[len(conjAmostrasX)-1].posY])

            # pegar a coordenada real do simulador
            path_real.append([posXGrid,posYGrid])


            '''     Navegação -> base    '''

            navegacao_base(laser_data, clientID, i, r, L, l_wheel, r_wheel)


            # ponto na grid de onde o feixe da particula bateu (fazer isso para todos os feixes)        feito

            # converter o ponto xLGrid e yLGrid para xL e yL (pontos no mapa do coppelia)               feito

            # com xL e yL da pra saber o tamanho do feixe do robo virtual desde que a posição da particula virtual é conhecida      feito

            # inserir o tamanho do feixe encontrado no range_data desta particula       feito

            aux_range_data: list[int] = []
            index_angle: int = 0

            for particle in conjAmostrasX:
                for data in particle.range_data:

                    # pro alpha (valor que vem do angle data) criar um contador e incrementar dentro desse for interno
                    alpha = raw_angle_data[index_angle]

                    xL = int((RESOLUCAO * (2 * data[0] - LARG_GRID)) / 2)
                    yL = int((RESOLUCAO * (2 * data[1] - ALT_GRID)) / 2)

                    # converter a posição da particula para mapa coppelia tb

                    posXParticle = int((RESOLUCAO * (2 * particle.posX - LARG_GRID)) / 2)
                    posYParticle = int((RESOLUCAO * (2 * particle.posy - LARG_GRID)) / 2)

                    cat_opos = abs(yL - posYParticle)

                    # if externo -> verifica o quadrante do feixe de laser
                        # if interno -> verifica a direção em que o robo aponta

                    if xL > posXParticle and yL > posYParticle:     # primeiro quadrante 
                        if theta < 90:
                            beta = alpha + theta
                            pass
                        elif theta < 180:
                            beta = theta - alpha
                            pass
                        elif theta >= 270:
                            beta = alpha - (360 - theta)
                            pass
                    elif xL < posXParticle and yL > posYParticle:   # segundo quadrante
                        if theta < 90:
                            beta = 180 - (alpha + theta)
                            pass
                        elif theta < 180:
                            beta = 180 - theta
                            pass
                        elif theta < 270:
                            beta = alpha - (theta - 180)
                            pass
                    elif xL < posXParticle and yL < posYParticle:   # terceiro quadrante
                        if theta < 180 and theta > 90:
                            beta = alpha - (180 - theta)
                            pass
                        elif theta < 270:
                            beta = theta - 180 + alpha
                            pass
                        elif theta < 360:
                            beta = 180 - 360 - theta       #TODO ta certo essa parte?
                            pass
                    else:                                           # quarto quadrante
                        if theta < 90:
                            beta = alpha - theta
                            pass
                        elif theta < 180:
                            beta = 360 - theta - alpha
                            pass
                        elif theta < 360:
                            beta = 360 - theta + alpha
                            pass

                    index_angle += 1


                    #hipotenusa = sen(beta) * CO
                    tam_feixe = sin(beta) * cat_opos

                    aux_range_data.append(tam_feixe)

                # todos os valores de tam_feixe dos estão no atributo da particula
                particle.range_data = aux_range_data

                # construir o laser data a partir do range_data e do angle_data de cada particula (angle_data não muda)         feito

                #TODO: esse range data que eu tenho aqui corresponde ao raw_range_data?
                #TODO: ta certo essa parte?
                particle.laser_data = np.array([particle.range_data, raw_angle_data]).T
            

                # aplicar a navegação_base (versão mais simples do que a do robo real) em cada particula

                # atualizar a posição da particula apos a movimentação

                navegacao_particula_base(particle)


            tempo = tempo + dt
            i = i + 1
            lastTime = now

        # end while <reamostragem>


        # Parando o robô
        sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

        # Parando a simulação
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)

        # desenhar os path's no mapa do occupance grid
        print("\n Path monte Carlo: " + str(path_monte_carlo) + "\n")
        print("\n Path real: " + str(path_real) + "\n")

        plt.imshow(map, cmap='Greys', origin='upper', extent=(0, cols, rows, 0))
        plt.show(10)

    else:
        print('Failed connecting to remote API server')

    print('Program ended')


# main(dimensão, particulas, numReamostragens/tempo)

#main(100, 96, 10)

main(1000, 96, 120)
