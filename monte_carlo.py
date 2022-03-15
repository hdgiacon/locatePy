from random import randint
import numpy as np

import sys

from auxiliares import RoboVirtual, create_virtual_robot, convertion_points, get_line

    

def monteCarlo(conjAmostrasX: 'list[RoboVirtual]', num_particles: int, alt_grid: int, larg_grid: int, raw_range_data: list, 
    raw_angle_data: list, theta: int, LARG_GRID: int, ALT_GRID: int, posXGrid: int, COEF_PROP: float, posYGrid: int,
    posX: int, posY: int, grid: np.array) -> 'list[RoboVirtual]':
    ''' comentario sobre monte carlo '''


    if num_particles % 4 == 0 and num_particles % 8 == 0:

        #for k in num_particles:
            # predição
            # 𝑥_𝑡 ← 𝑠𝑎𝑚𝑝𝑙𝑒_𝑚𝑜𝑡𝑖𝑜𝑛_𝑚𝑜𝑑𝑒𝑙(𝑢_𝑡 , 𝑥_{𝑡−1}) // aplica a movimentação em cada amostra 𝑥^𝑘_𝑡

            # atualização
            # 𝑚𝑒𝑎𝑠𝑢𝑟𝑒𝑚𝑒𝑛𝑡_𝑚𝑜𝑑𝑒𝑙(𝑧_𝑡 , 𝑥_𝑡 , 𝑚) // aplica o modelo de observação atribuindo peso 𝜔_𝑡
            # 𝑋_𝑡 ← 𝑋_𝑡 + ⟨𝑥_𝑡 , 𝑤_𝑡 ⟩ // o conjunto das probabilidades das amostras 𝑋_𝑡 é gerado
        
        #TODO: recalcular os pesos das particulas do 0 (verificar se esta tudo certo aqui)

        # peso local
        pesosLocais: list[float] = []

        for t in conjAmostrasX:

            # fazer isso pro numero de feixes de lasers
            for k in range(len(raw_range_data)):
                # bresenham para um feixe de laser do robo real
                point1_r, point2_r = convertion_points(raw_range_data, raw_angle_data, theta, posX, 
                posY, k, COEF_PROP, ALT_GRID, LARG_GRID, posXGrid, posYGrid)

                path_r = get_line(point1_r, point2_r)
                
                # bresenham para um feixe de laser do robo virtual
                point1_v, point2_v = convertion_points(raw_range_data, raw_angle_data, theta, conjAmostrasX[len(conjAmostrasX)-1].posX, 
                conjAmostrasX[len(conjAmostrasX)-1].posY, k, COEF_PROP, ALT_GRID, LARG_GRID, posXGrid, posYGrid)

                path_v = get_line(point1_v, point2_v)

                contPesoLocal: int = 0
                for m in path_v:
                    contPesoLocal += 1
                    if grid[m[0]][m[1]] >= 0.999:
                        break

                if contPesoLocal == len(path_r):
                    # atribuir peso caso forem iguais
                    pesosLocais.append(1.0)
                    
                else:
                    # calculo do peso local caso não sejam iguais
                    pesosLocais.append(1.0 - (abs(contPesoLocal - len(path_r)) / 10))

            # calcular o peso particula - media (quanto mais proximo de 0 mais proximo o robo virtual está de um robo real)
            #conjAmostrasX[len(conjAmostrasX)-1].pesoParticula = sum(pesosLocais) / len(pesosLocais)
            t.pesoParticula = sum(pesosLocais) / len(pesosLocais)
            #TODO: os pesos particulas estão dando valores negativos
            pesosLocais.clear


        # peso global

        # soma todos os pesos particula (peso particula = media dos pesos dos feixes de laser)
        soma_peso_particula: float = 0.0
        for particle in conjAmostrasX:
            soma_peso_particula = soma_peso_particula + particle.pesoParticula

        # calcula o peso global
        for particle in conjAmostrasX:
            particle.pesoGlobal = particle.pesoParticula / soma_peso_particula

            # se for a primeira particula, pesoRoleta = pesoGlobal * 100
            # os demais pesoRoleta anterior + (pesoGlobal * 100)
            if conjAmostrasX.index(particle) == 0:
                particle.pesoRoleta = particle.pesoGlobal * 100
            else:
                particle.pesoRoleta = conjAmostrasX[conjAmostrasX.index(particle) - 1].pesoRoleta + (particle.pesoGlobal * 100)



        # reamostragem
        # 𝑑𝑟𝑎𝑤 𝑖 𝑤𝑖𝑡ℎ 𝑝𝑟𝑜𝑏𝑎𝑏𝑖𝑙𝑖𝑡𝑦 ∝ 𝑤_𝑡^{[i]} selecionar as amostras 𝑥^𝑘_𝑡 que possuem maior peso 𝜔_𝑡^𝑘
        # 𝑋_𝑡 ∪ 𝑥_𝑡 // adiciona a 𝑋_𝑡 as amostras de maior peso


        # remove os elementos de menor peso -> quanto menor, mais diferente é do feixe original
        for _ in range(int(num_particles / 4)):
            conjAmostrasX.pop(conjAmostrasX.index(min(conjAmostrasX, key=lambda x: x.pesoParticula)))

        # adicionar na lista n/8 particulas mediante as boas (esquema da roleta)
        for _ in range(int(num_particles / 8)):
            num_aleatorio = randint(0, 99)
            for particle in conjAmostrasX:
                # verificar se particle é uma copia ou uma referencia do objeto verificado
                if num_aleatorio >= particle.pesoRoleta:
                    conjAmostrasX.append(particle)
                    break


        # adicionar na lista de particulas n/8 particulas aleatorias
        for _ in range(int(num_particles / 8)):
            conjAmostrasX = create_virtual_robot(conjAmostrasX, ALT_GRID, LARG_GRID, grid)

            # calculo do peso da nova particula
            for k in range(len(raw_range_data)):
                # bresenham para um feixe de laser do robo real
                point1_r, point2_r = convertion_points(raw_range_data, raw_angle_data, theta, posX, 
                posY, k, COEF_PROP, ALT_GRID, LARG_GRID, posXGrid, posYGrid)

                path_r = get_line(point1_r, point2_r)
                
                # bresenham para um feixe de laser do robo virtual
                point1_v, point2_v = convertion_points(raw_range_data, raw_angle_data, theta, conjAmostrasX[len(conjAmostrasX)-1].posX, 
                conjAmostrasX[len(conjAmostrasX)-1].posY, k, COEF_PROP, ALT_GRID, LARG_GRID, posXGrid, posYGrid)

                path_v = get_line(point1_v, point2_v)

                contPesoLocal: int = 0
                for m in path_v:
                    contPesoLocal += 1
                    if grid[m[0]][m[1]] >= 0.999:
                        break

                if contPesoLocal == len(path_r):
                    # atribuir peso caso forem iguais
                    pesosLocais.append(1.0)
                    
                else:
                    # calculo do peso local caso não sejam iguais
                    pesosLocais.append(1.0 - (abs(contPesoLocal - len(path_r)) / 10))

            # calcular o peso particula - media (quanto mais proximo de 0 mais proximo o robo virtual está de um robo real)
            conjAmostrasX[len(conjAmostrasX)-1].pesoParticula = sum(pesosLocais) / len(pesosLocais)

        soma_peso_particula: float = 0.0
        for particle in conjAmostrasX:
            soma_peso_particula = soma_peso_particula + particle.pesoParticula

        for particle in conjAmostrasX:
            particle.pesoGlobal = particle.pesoParticula / soma_peso_particula

            # se for a primeira particula, pesoRoleta = pesoGlobal * 100
            # os demais pesoRoleta anterior + (pesoGlobal * 100)
            if conjAmostrasX.index(particle) == 0:
                particle.pesoRoleta = particle.pesoGlobal * 100
            else:
                particle.pesoRoleta = conjAmostrasX[conjAmostrasX.index(particle) - 1].pesoRoleta + (particle.pesoGlobal * 100)

        

        # 𝑟𝑒𝑡𝑜𝑟𝑛𝑒 𝑋_𝑡
        return conjAmostrasX

    else:
        print("O numero de partículas deve ser divisível por 4 e por 8!!! Simulação encerrada...")
        sys.exit(0)