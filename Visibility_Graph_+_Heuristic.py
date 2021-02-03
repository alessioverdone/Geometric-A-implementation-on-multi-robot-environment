## HERE I IMPLEMENT AN ADDITIONAL FEATURE TO GEOMETRIC A*
## I BUILD THE VISIBILITY GRAPH AND THEN FIND THE SHORTER PATH
## BASED ON THIS GRAPH. AT THE END I USE THIS REFERENCE AS
## AN HEURISTIC FOR MY MULTI-ROBOT SYSTEM

import pygame
import numpy as np
import math
maze = [[0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 1, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 1, 0]]

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BROWN = (150,100,50)
BLUE = (127,194,188)
VIOLET = (92,46,145)
pygame.init()
# Set the width and height of the screen [width, height]
size = (400, 400)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("A* variant with visibility graph and heuristic")
# Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
pygame.font.init() # you have to call this at the start, 
myfont = pygame.font.SysFont('Comic Sans MS', 30)
#Text
width=40
height=40
margin=0
#Here i define the heuristic function applied to the 'main' robot in the function astar2
def heur4(l,p):
    min =0
    for elem in l :
        if (elem[0]-p[0]) + (elem[1]-p[1]) < min:
            min = (elem[0]-p[0]) + (elem[1]-p[1])
    return min

class Node2():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

#This version of astar is used on final computation 
def astar2(maze, start, end,flag,lista_eur_gen):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node2(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node2(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []
    
    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    q = 0   
    all_moves = []
    possible = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    for i in range(8):
        move1 = (possible[i][0],possible[i][1],0,0,0,0)
        move2 = (0,0,possible[i][0],possible[i][1],0,0)
        move3 = (0,0,0,0,possible[i][0],possible[i][1])
        move4 = (possible[i][0],possible[i][1],possible[i][0],possible[i][1],0,0)
        move5 = (possible[i][0],possible[i][1],0,0,possible[i][0],possible[i][1])
        move6 = (0,0,possible[i][0],possible[i][1],possible[i][0],possible[i][1])
        move7 = (possible[i][0],possible[i][1],possible[i][0],possible[i][1],possible[i][0],possible[i][1])
        all_moves.append(move1)
        all_moves.append(move2)
        all_moves.append(move3)
        all_moves.append(move4)
        all_moves.append(move5)
        all_moves.append(move6)
        all_moves.append(move7)
            
    while len(open_list) > 0:
        q+=1
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node.position[0] == end_node.position[0] and current_node.position[1] == end_node.position[1]:#Maybe three end nodes
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            print('Path found')
            return path[::-1] # Return reversed path

        # Generate children
        children = []                    
        for new_position in all_moves: # Adjacent squares

            # Get node position
            a = np.array(current_node.position)
            b = np.array(new_position)
            node_positio = a+b
            node_position = tuple(node_positio)

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue
            if node_position[2] > (len(maze) - 1) or node_position[2] < 0 or node_position[3] > (len(maze[len(maze)-1]) -1) or node_position[3] < 0:
                continue
            if node_position[4] > (len(maze) - 1) or node_position[4] < 0 or node_position[5] > (len(maze[len(maze)-1]) -1) or node_position[5] < 0:
                continue
            
            #Constraint condition 
            if node_position[0] > (node_position[2]+1) or node_position[0] < (node_position[2]-1):
                continue
            if node_position[4] > (node_position[2]+1) or node_position[4] < (node_position[2]-1):
                continue
            if node_position[1] > (node_position[3]+1) or node_position[1] < (node_position[3]-1):
                continue
            if node_position[5] > (node_position[3]+1) or node_position[5] < (node_position[3]-1):
                continue
            if node_position[0] == node_position[2] and node_position[1] == node_position[3]:
                continue
            if node_position[2] == node_position[4] and node_position[3] ==node_position[5]:
                continue
            if node_position[0] == node_position[4] and node_position[1] ==node_position[5]:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue
            if maze[node_position[2]][node_position[3]] != 0:
                continue
            if maze[node_position[4]][node_position[5]] != 0:
                continue

            # Create new node
            new_node = Node2(current_node, node_position)
            # Append
            children.append(new_node)
        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            if flag == 0:
                heur1 = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                heur2 = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                heur3 = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.h = heur1 + heur2 + heur3 
            else:#Considering the distance heuristic among robot and best path on visibility graph
                heur1 = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                heuri4 = heur4(lista_eur_gen,[child.position[0],child.position[1]]) 
                child.h = heur1 + heuri4 
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            if child not in open_list:#####QUesto non c'era prima
                open_list.append(child)
            #print(child.position)
#This class is used in the astar_map
class Node3():
    def __init__(self, parent=None, position=None,cost = None):
        self.parent = parent
        self.position = position
        self.cost = cost

        self.g = 0
        self.h = 0
        self.f = 0
#Here i implement a classical version of A* that will be used on the visibility graph to compute best graph
def astar_map(map, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node3(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node3(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    x = 0
    while len(open_list) > 0:
        x+=1
        # Get the current node
        current_node = open_list[0]
        
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node.position == end_node.position:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for key in map:
            if key == tuple(current_node.position):
                for elem in map[key]:
                    new_node = Node3(current_node, elem[0],elem[1])
                    children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values           
            child.g = current_node.g + child.cost
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            # Add the child to the open list
            open_list.append(child)


def main():
    #Extended state for multi-robot application
    start2 = (1,0,2,0,3,0)
    done = False
    cont = -1
    stop = False
    lista_prova = []
    vertici_ostacoli = []
    lista_ok = []
    elementi_uguali=0
    map_distance = {}
    map = {}
    end2 = (5,8)
    while not done:
        cont +=1;
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        screen.fill(BLACK)        
        for column in range(len(maze)):
            for row in range(len(maze)):
                if maze[row][column] == 1:
                    #in lista_prova inserisco tutti gli ostacoli(oggetti rect)
                    obs = pygame.draw.rect(screen,BROWN,[column*(width+margin),row*(width+margin),width,height])
                    if cont == 0:#Questa operazione la faccio solo all'inizio
                        lista_prova.append(obs)
#                        #Disegna i vertici sullo schermo
#                        pygame.draw.rect(screen,RED,[column*(width+margin),row*(width+margin),5,5])
#                        pygame.draw.rect(screen,RED,[column*(width+margin)+width-5,row*(width+margin)+width-5,5,5])
#                        pygame.draw.rect(screen,RED,[column*(width+margin),row*(width+margin)+width-5,5,5])
#                        pygame.draw.rect(screen,RED,[column*(width+margin)+width-5,row*(width+margin),5,5])
                        #Inserisce le posizioni dei vertici dei rettangoli in una lista
                        vertici_ostacoli.append([column*(width+margin)-2,row*(width+margin)-2])
                        vertici_ostacoli.append([column*(width+margin)+width+2,row*(width+margin)+width+2])
                        vertici_ostacoli.append([column*(width+margin)-2,row*(width+margin)+width+2])
                        vertici_ostacoli.append([column*(width+margin)+width+2,row*(width+margin)-2])
                else:
                    pygame.draw.rect(screen,WHITE,[column*(width+margin),row*(width+margin),width,height])
        
        
        if cont == 0:
            #Inserisco in vertici ostacoli anche le tre posizioni iniziali
            init1 = [10,50]
            init2 = [10,90]
            init3 = [10,130]
            
            vertici_ostacoli.append(init1)
            vertici_ostacoli.append(init2)  
            vertici_ostacoli.append(init3)
            #Verifico se tutte i segmenti possibili per il visibility graph
            #siano ammissibili, ossia non intersechino ostacoli
            for elem1 in vertici_ostacoli:
                for elem2 in vertici_ostacoli:
                    if elem1 == elem2 :
                        elementi_uguali +=1
                        continue
                    for column in range(len(maze)):
                        for row in range(len(maze)):
                            if maze[row][column] == 1:
                                obs = pygame.draw.rect(screen,BROWN,[column*(width+margin),row*(width+margin),width,height])
                            else:
                                pygame.draw.rect(screen,WHITE,[column*(width+margin),row*(width+margin),width,height])                                        
                    a = pygame.draw.rect(screen,BLACK,[elem1[0],elem1[1], (elem2[0]-elem1[0])*1, (elem2[1]-elem1[1])*1])                    
                    sol = a.collidelist(lista_prova)
                    if sol == -1:
                        #Se il segmento che ha come estremi due vertici degli ostacoli non interseca
                        #un ostacolo,allora aggiungo il segmento in una lista
                        lista_ok.append([elem1,elem2])
                    screen.fill(BLACK)
            #Ricreo l'ambiente per il prossimo calcolo
            for column in range(len(maze)):
                for row in range(len(maze)):
                    if maze[row][column] == 1:
                        obs = pygame.draw.rect(screen,BROWN,[column*(width+margin),row*(width+margin),width,height])
                    else:
                        pygame.draw.rect(screen,WHITE,[column*(width+margin),row*(width+margin),width,height])
        
            z = 0
            max_dist =0
            #Calcoliamo le distanze tra i vari punti ammissibili
            for coppia in lista_ok:
                quad1 = (coppia[1][0]-coppia[0][0])**2
                quad2 = (coppia[1][1]-coppia[0][1])**2
                dist = math.sqrt(quad1+quad2)
                if dist > max_dist:
                    max_dist = dist
                map_distance[z] = dist
                z+=1
            z = 0
            #Crea mappa tra i nodi con le distanze tra di loro come chiavi
            for coppia in lista_ok:
                if tuple(coppia[0]) in map : 
                    old = map[tuple(coppia[0])]
                    old.append([coppia[1],map_distance[z]])
                    map[tuple(coppia[0])] = old
                else :
                    map[tuple(coppia[0])] = [[coppia[1],map_distance[z]]]
                z +=1
            
            VIOLET = (92,46,145)    
            path_vg3 = []
            start = [10,90]
            end = [362, 238]
            path_vg = astar_map(map,start,end)#Percorso migliore espresso in sequenza di stati
            path_vg2 = astar2(maze,start2,end2,1,path_vg)
            #print(path_vg)
            for i in range(len(path_vg)-1):
                path_vg3.append([path_vg[i],path_vg[i+1]])#INserisco segmenti del percorso migliore
            print(path_vg)
            print(path_vg2)
            print(path_vg3)
        for coppia_vert in lista_ok:
            if coppia_vert in path_vg3:
                #Final path for scout robot
                pygame.draw.line(screen,VIOLET,coppia_vert[0],coppia_vert[1],5)
            else:
                #all other arcs of final graph
                pygame.draw.line(screen,BLUE,coppia_vert[0],coppia_vert[1],1)
                
        if cont == len(path_vg2):
            cont = len(path_vg2)-1
            stop = True
        #Display the robots
        pygame.draw.circle(screen,BLACK,[path_vg2[cont][1]*(width+margin)+10,path_vg2[cont][0]*(width+margin)+10], 10)
        pygame.draw.circle(screen,BLACK,[path_vg2[cont][3]*(width+margin)+10,path_vg2[cont][2]*(width+margin)+10], 10)
        pygame.draw.circle(screen,BLACK,[path_vg2[cont][5]*(width+margin)+10,path_vg2[cont][4]*(width+margin)+10], 10)        
        pygame.time.wait(300)
        if stop == True:
            done = True 
            pygame.time.wait(600)   
        pygame.display.flip()
        pygame.display.update()
        # --- Limit to 60 frames per second
        clock.tick(60)           
    pygame.quit()
if __name__ == '__main__':
    main()
