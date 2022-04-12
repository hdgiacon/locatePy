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


def navegacao_particula_base(particula: RoboVirtual, LARG_GRID: int, ALT_GRID: int) -> None:

    laser_data = particula.laser_data

    frente = int(len(laser_data) / 2)
    lado_direito = int(len(laser_data) * 1 / 4)
    lado_esquerdo = int(len(laser_data) * 3 / 4)

    # se estiver a frente ou a direita, virar para a esquerda

    if laser_data[frente, 1] < 1:
        if particula.theta <= 90:     # 1º quadrante
            particula.posX -= 2
            particula.posY += 2

        elif particula.theta <= 180:  # 2º quadrante
            particula.posX += 2
            particula.posY += 2

        elif particula.theta <= 270:  # 3º quadrante
            particula.posX += 2
            particula.posY -= 2
            
        elif particula.theta <= 360:  # 4º quadrante
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
    