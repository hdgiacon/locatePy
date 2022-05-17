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

from math import sin
import numpy as np
import matplotlib.pyplot as plt
import time
import copy
from locatePy import *



def main(map_dimension: int, numParticles: int, numReamostragens: int, nomePlot, nomeGrid, final) -> None:

    import typing as ty

    '''     Inicio definição das constantes, parâmetros e do mapa<Grid>     '''

    LARG_GRID: ty.Final and int = map_dimension
    ALT_GRID: ty.Final and int = map_dimension

    # coeficiente de proporção
    RESOLUCAO: ty.Final and float = 7.5 / (map_dimension / 2)
    RANGE_MAX: ty.Final and float = 5
    RANGE_LIMIT: ty.Final and float = 0.3
    PRIORI: ty.Final and float = 0.5
    CELL_SIZE: ty.Final and int = 1

    fig = plt.figure(figsize=(8, 8), dpi=100)
    _ = fig.add_subplot(111, aspect='equal')

    map_size = np.array([LARG_GRID, ALT_GRID])


    rows, cols = (map_size / CELL_SIZE).astype(int)

    mapa = np.random.uniform(low=0.0, high=1.0, size=(rows, cols))
    
    mapa[::, ::] = PRIORI

    mapa[0, 0] = 1
    mapa[LARG_GRID-1, ALT_GRID-1] = 0


    print('Program started')
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if clientID != -1:
        
        res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Number of objects in the scene: ', (objs))
        else:
            print('Remote API function call returned with errlenor code: ', res)

        
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

        print('Connected to remote API server')
        sim.simxAddStatusbarMessage(clientID, 'Iniciando...', sim.simx_opmode_oneshot_wait)
        time.sleep(0.02)

        
        robotname = 'Pioneer_p3dx'
        erro, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)

        
        returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
        returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

        
        [returnCode, positionrobot] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming)
        [returnCode, orientationrobot] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_streaming)
        time.sleep(2)

        
        laser_range_data = "hokuyo_range_data"
        laser_angle_data = "hokuyo_angle_data"

        
        returnCode = 1
        while returnCode != 0:
            returnCode, range_data = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10)

        
        raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
        laser_data = np.array([raw_angle_data, raw_range_data]).T

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

            raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
            laser_data = np.array([raw_angle_data, raw_range_data]).T

            returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            posX, posY, posZ = pos
            print('Pos: ', pos)

            # Conversão da posição do robô no ambiente para a Grid
            posXGrid = int((posX / RESOLUCAO) + (LARG_GRID / 2))
            posYGrid = int(ALT_GRID - ((posY / RESOLUCAO) + (ALT_GRID / 2)))
            print('PosGRID: ', posXGrid, posYGrid)

            # gera o caminho do robo
            #mapa[posYGrid][posXGrid] = 0.0    
            #mapa[posYGrid-1][posXGrid] = 0.0
            #mapa[posYGrid+1][posXGrid] = 0.0
            #mapa[posYGrid][posXGrid-1] = 0.0
            #mapa[posYGrid][posXGrid+1] = 0.0
            #mapa[posYGrid-1][posXGrid-1] = 0.0
            #mapa[posYGrid-1][posXGrid+1] = 0.0
            #mapa[posYGrid+1][posXGrid-1] = 0.0
            #mapa[posYGrid+1][posXGrid+1] = 0.0

            returnCode, th = sim.simxGetObjectOrientation(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            ty, tz, theta = th


            '''     Mapeamento -> Occupancy Grid     '''

            ocuppancy_grid(raw_range_data, raw_angle_data, theta, posX, posY, RANGE_MAX, 
                RESOLUCAO, LARG_GRID, ALT_GRID, posXGrid, posYGrid, mapa)


            '''     Navegação -> base    '''

            navegacao_base(laser_data, clientID, i, r, L, l_wheel, r_wheel)



            tempo = tempo + dt
            i = i + 1
            lastTime = now

        # end while <geração do mapa>

        print("\n Fim do mapeamento, inicio da localização \n")

        '''     Criação das Partículas      '''
        

        conjAmostrasX: list[RoboVirtual] = []
        for _ in range(numParticles):
            conjAmostrasX = create_virtual_robot(conjAmostrasX, ALT_GRID, LARG_GRID, mapa)


        path_real: list[tuple[int,int]] = []
        path_monte_carlo: list[tuple[int,int]] = []

        tempo = 0

        startTime = time.time()
        lastTime = startTime
        dt = 0
        i = 0
        aux_range_data: list[int] = []
        index_angle: int = 0
        while tempo < numReamostragens:
            now = time.time()
            dt = now - lastTime

            raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
            laser_data = np.array([raw_angle_data, raw_range_data]).T

            returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            posX, posY, posZ = pos
            print('Pos: ', pos)

            posXGrid = int((posX / RESOLUCAO) + (LARG_GRID / 2))
            posYGrid = int(ALT_GRID - ((posY / RESOLUCAO) + (ALT_GRID / 2)))
            print('PosGRID: ', posXGrid, posYGrid)

            returnCode, th = sim.simxGetObjectOrientation(clientID, robotHandle, -1, 
                sim.simx_opmode_oneshot_wait)
            ty, _, theta = th

            '''     Monte Carlo     '''

            conjAmostrasX = monteCarlo(conjAmostrasX, numParticles, raw_range_data, 
                raw_angle_data, LARG_GRID, ALT_GRID, RESOLUCAO, mapa, 
                RANGE_MAX)

            conjAmostrasX.sort(key = lambda x: x.pesoGlobal)
            path_monte_carlo.append([conjAmostrasX[len(conjAmostrasX)-1].posX,conjAmostrasX[len(conjAmostrasX)-1].posY])

            path_real.append([posXGrid,posYGrid])

            
            '''     Navegação -> base    '''

            navegacao_base(laser_data, clientID, i, r, L, l_wheel, r_wheel)


            for particle in conjAmostrasX:
                index_angle = 0

                for data in particle.range_data:    

                    alpha = raw_angle_data[index_angle]

                    xL = int((RESOLUCAO * (2 * data[0] - LARG_GRID)) / 2)
                    yL = int((RESOLUCAO * (2 * data[1] - ALT_GRID)) / 2)


                    posXParticle = int((RESOLUCAO * (2 * particle.posX - LARG_GRID)) / 2)
                    posYParticle = int((RESOLUCAO * (2 * particle.posY - LARG_GRID)) / 2)

                    cat_opos = abs(yL - posYParticle)

                    # if externo -> verifica o quadrante do feixe de laser
                        # if interno -> verifica a direção em que o robo aponta

                    if xL > posXParticle and yL > posYParticle:     
                        if theta < 90:
                            beta = alpha + theta
                            
                        elif theta < 180:
                            beta = theta - alpha
                            
                        elif theta >= 270:
                            beta = alpha - (360 - theta)
                            
                    elif xL < posXParticle and yL > posYParticle:   
                        if theta < 90:
                            beta = 180 - (alpha + theta)
                            
                        elif theta < 180:
                            beta = 180 - theta
                            
                        elif theta < 270:
                            beta = alpha - (theta - 180)
                            
                    elif xL < posXParticle and yL < posYParticle:   
                        if theta < 180 and theta > 90:
                            beta = alpha - (180 - theta)
                            
                        elif theta < 270:
                            beta = theta - 180 + alpha
                            
                        elif theta < 360:
                            beta = 180 - 360 - theta
                            
                    else:                                           
                        if theta < 90:
                            beta = alpha - theta
                            
                        elif theta < 180:
                            beta = 360 - theta - alpha
                            
                        elif theta < 360:
                            beta = 360 - theta + alpha
                            

                    index_angle += 1


                    #hipotenusa = sen(beta) * CO
                    tam_feixe = sin(beta) * cat_opos

                    aux_range_data.append(tam_feixe)

                particle.laser_data = np.array([copy.deepcopy(aux_range_data), raw_angle_data]).T
            
                aux_range_data.clear()

                navegacao_particula_base(particle, LARG_GRID, ALT_GRID)


            tempo = tempo + dt
            i = i + 1
            lastTime = now

        # end while <reamostragem>

        if final == True:
            sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
            sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

            sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)

            sim.simxFinish(clientID)

        print("\n Path monte Carlo: " + str(path_monte_carlo) + "\n")
        print("\n Path real: " + str(path_real) + "\n")

        vet_plot1: list = []
        vet_plot2: list = []
        aux: tuple
        for k in range(len(path_monte_carlo)):
            vet_plot1.append(abs(path_monte_carlo[k][0] - path_real[k][0]))
            vet_plot2.append(abs(path_monte_carlo[k][1] - path_real[k][1]))

        plt.figure(figsize=(8, 8), dpi=100)
        plt.plot(list(range(len(vet_plot1))), vet_plot1, color = 'blue')
        plt.plot(list(range(len(vet_plot2))), vet_plot2, color = 'red')
        plt.legend(['Diferença dos valores de X','Diferença dos valores de Y'], fontsize=14)

        plt.savefig(nomePlot, bbox_inches='tight')

        plt.imshow(mapa, cmap='Greys', origin='upper', extent=(0, cols, rows, 0))
        plt.savefig(nomeGrid, bbox_inches='tight')

    else:
        print('Failed connecting to remote API server')

    print('Program ended')


# main(dimensão, particulas, numReamostragens/tempo)

main(500, 96, 900, "dif_96_cenario1", "grid_96_cenario1", False)

main(500, 192, 900, "dif_192_cenario1", "grid_192_cenario1", False)

main(500, 480, 900, "dif_480_cenario1", "grid_480_cenario1", False)

main(500, 960, 900, "dif_960_cenario1", "grid_960_cenario1", True)