# Velocidade básica (linear, angular)
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

from auxiliares import RoboVirtual

def navegacao_base(laser_data, clientID, i, r, L, l_wheel, r_wheel) -> None:
    ''' comentario sobre a função '''

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

    # Isso é o modelo cinemático
    wl = v / r - (w * L) / (2 * r)
    wr = v / r + (w * L) / (2 * r)

    # Enviando velocidades
    sim.simxSetJointTargetVelocity(clientID, l_wheel, wl, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, wr, sim.simx_opmode_streaming + 5)


def navegacao_particula_base(particula: RoboVirtual, theta: int):

    laser_data = particula.laser_data

    frente = int(len(laser_data) / 2)
    lado_direito = int(len(laser_data) * 1 / 4)
    lado_esquerdo = int(len(laser_data) * 3 / 4)

    if laser_data[frente, 1] < 1:
        if theta <= 90:
            pass
        elif theta <= 180:
            pass
        elif theta <= 270:
            pass
        elif theta <= 360:
            pass

        # verificar se esta no 1º quadrante
        # verificar se esta no 2º quadrante
        # verificar se esta no 3º quadrante
        # verificar se esta no 4º quadrante
    elif laser_data[lado_direito, 1] < 1:
        # verificar se esta no 1º quadrante
        # verificar se esta no 2º quadrante
        # verificar se esta no 3º quadrante
        # verificar se esta no 4º quadrante

        #particula.posX -= 2
        #particula.posY += 2
        pass
    elif laser_data[lado_esquerdo, 1] < 1:
        # verificar se esta no 1º quadrante
        # verificar se esta no 2º quadrante
        # verificar se esta no 3º quadrante
        # verificar se esta no 4º quadrante

        #particula.posX += 2
        #particula.posY += 2
        pass
    else:
        pass

    #TODO: como fazer a movimentação da particula? atualizar a sua posição no mapa, precisa converter pra grid? a posição da 
        # particula esta dada em termos da grid e nao do mapa do coppelia
        # seria aumentar ou decrementar os valores de X e Y?