###In this implemetation first i compute path with original a-star,after i smooth it geometrically
### and recompute the a-star with this path as reference 
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
pygame.display.set_caption("A* variant with smoothing")
# Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
pygame.font.init() # you have to call this at the start, 
myfont = pygame.font.SysFont('Comic Sans MS', 30)
#Screen
width=40
height=40
margin=0#1
cont =0
#Heuristic function among robot and smoothed path
def heur4(l,p):
    minimum =0
    for elem in l :
        k = (elem[0]-p[0]) + (elem[1]-p[1])
        if k <= minimum:
            minimum = k       
    return minimum

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


def astar2(maze, start, end,flag,lista_finale):
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
            #print('Ã¨ entrato')
            while current is not None:
                path.append(current.position)
                current = current.parent
            
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
            heur1 = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            heur2 = ((child.position[2] - end_node.position[0]) ** 2) + ((child.position[3] - end_node.position[1]) ** 2)
            heur3 = ((child.position[4] - end_node.position[0]) ** 2) + ((child.position[5] - end_node.position[1]) ** 2) 
             
            if flag == 0:#Prima esecuzioe dell'algoritmo senza post-smoothing
                child.h = heur1 + heur2 + heur3
            else:#Seconda esecuzione dell'algoritmo considerando il percorso generato dal post-smoothing
                heuri4 = heur4(lista_finale,[child.position[2],child.position[3]])
                child.h = heur1 + heuri4
                
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            # Add the child to the open list
            if child not in open_list:#####QUesto non c'era prima
                open_list.append(child)

def main():

    start2 = (1,0,2,0,3,0)
    done = False
    cont = -1
    stop = False
    end2 = (5,8)
    lista1 = []
    lista2 = []
    lista3 = []
    lista_prova = []
    path = astar2(maze, start2, end2,0,[])#Prima esecuzione dell'astar
    path2 = path
    for nodi in path :
        l1 = list(nodi[0:2])
        l1 = l1[-2:]
        l1.reverse()
        lista1.append(l1)
        l2 = list(nodi[2:4])
        l2 = l2[-2:]
        l2.reverse()
        lista2.append(l2)
        l3 = list(nodi[4:6])
        l3 = l3[-2:]
        l3.reverse()
        lista3.append(l3)
    lista1 = []
    lista2 = []
    lista3 = []
    vertici_ostacoli = []
    lista_ok = []
    elementi_uguali=0
    map_distance = {}
    map = {}  
    lista_punti=[]
    #inserire in una lista tutti i punti degli spigoli degli ostacoli,punto iniziale e finale
    while not done:
        cont +=1;
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        screen.fill(BLACK)        
        for column in range(len(maze)):
            for row in range(len(maze)):
                if maze[row][column] == 1:
                    #in lista_prova inserisco tutti i rettangoli
                    obs = pygame.draw.rect(screen,BROWN,[column*(width+margin),row*(width+margin),width,height])
                    if cont == 0:
                        lista_prova.append(obs)
                    if cont == 0 :
                        vertici_ostacoli.append([column*(width+margin)-2,row*(width+margin)-2])
                        vertici_ostacoli.append([column*(width+margin)+width+2,row*(width+margin)+width+2])
                        vertici_ostacoli.append([column*(width+margin)-2,row*(width+margin)+width+2])
                        vertici_ostacoli.append([column*(width+margin)+width+2,row*(width+margin)-2])
                else:
                    pygame.draw.rect(screen,WHITE,[column*(width+margin),row*(width+margin),width,height])
        
        
        if cont == 0:
            #Inserisco in vertici ostacoli anche le tre posizioni iniziali
            vertici_ostacoli.append([path[cont][1]*(width+margin)+10,path[cont][0]*(width+margin)+10]) 
            vertici_ostacoli.append([path[cont][3]*(width+margin)+10,path[cont][2]*(width+margin)+10])  
            vertici_ostacoli.append([path[cont][5]*(width+margin)+10,path[cont][4]*(width+margin)+10])
        
        
            for elem in path2:
               elem = list(elem)
               while len(elem) != 0:
                   new_elem = elem[0:2]
                   #pygame.draw.rect(screen,BLUE,[new_elem[1]*(width+margin),new_elem[0]*(width+margin),10,10])
                   lista_punti.append([new_elem[1]*width,new_elem[0]*width])
                   elem[0:2]=[]
            
            for elem in path:
                lista1.append([elem[1]*width,elem[0]*width])
                lista2.append([elem[3]*width,elem[2]*width])
                lista3.append([elem[5]*width,elem[4]*width])

            for elem1 in lista_punti:
                for elem2 in lista_punti:
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
                        lista_ok.append([elem1,elem2])
                    screen.fill(BLACK)
            for column in range(len(maze)):
                for row in range(len(maze)):
                    if maze[row][column] == 1:
                        obs = pygame.draw.rect(screen,BROWN,[column*(width+margin),row*(width+margin),width,height])
                    else:
                        pygame.draw.rect(screen,WHITE,[column*(width+margin),row*(width+margin),width,height])
            #lista_ok è la lista con i collegamenti tra tutti i punti che non passano tra gli ostacoli
        
            z = 0
            max_dist =0
            dict_dist = {}
            #Calcoliamo le distanze
            for coppia in lista_ok:
                quad1 = (coppia[1][0]-coppia[0][0])**2
                quad2 = (coppia[1][1]-coppia[0][1])**2
                dist = math.sqrt(quad1+quad2)
                c1 = tuple(coppia[0])
                c2 = tuple(coppia[1])
                c = (c1,c2)
                dict_dist[c] = dist
                if dist > max_dist:
                    max_dist = dist
                map_distance[z] = dist
                z+=1
            z = 0
            #Crea mappa tra i nodi con le distanze tra di loro
            for coppia in lista_ok:
                if tuple(coppia[0]) in map : 
                    old = map[tuple(coppia[0])]
                    old.append([coppia[1],map_distance[z]])
                    map[tuple(coppia[0])] = old
                else :
                    map[tuple(coppia[0])] = [[coppia[1],map_distance[z]]]
                z +=1

            VIOLET = (92,46,145)
            lista_fin = []
            lc = []
            for i in lista1:
                for j in lista1:
                    if i != j:
                        if [i,j] in lista_ok:
                            lc.append([i,j])

            flag = False
            pi = lista1[0]#Punto iniziale 
            pf = lista1[-2]#Punto finale
            seg_aux = [[0,0],[0,0]]
            lista_prova= []
            p=0
            for elem in lc:
               for elem2 in lc:
                   if elem[0] == elem2[1] and elem[1] == elem2[0]:
                       lc.remove(elem2)
            
            while flag != True :
                if p == 20:
                    flag = True
                p+=1
                lnls=[]  
                lnls2=[]                 
                for segmento in lc:                    
                    if segmento[0] == pi:                        
                        lnls.append(segmento)                        
                    if segmento[1] == pi :                        
                        lnls2.append(segmento)

                for elem in lnls :
                    for elem2 in lc :
                        if elem == elem2:
                            lc.remove(elem2)
                for elem in lnls2 :
                    for elem2 in lc :
                        if elem[1] == elem2[0] or elem[1] == elem2[1]:
                            lc.remove(elem2)                        
 
                max = 0
                for elem in lnls:
                    if (tuple(elem[0]),tuple(elem[1])) in dict_dist:
                        if dict_dist[tuple(elem[0]),tuple(elem[1])] >= max:                            
                            max = dict_dist[tuple(elem[0]),tuple(elem[1])]
                            seg_aux = elem
                            pi = seg_aux[1]
                          
                lista_fin.append(seg_aux)                
                #print(pi,)                
                if pi == pf :
                    flag = True
                else:
                    for h in lc :
                        if h[0] == seg_aux[0] or h[1] == seg_aux[0]:
                            lc.remove(h)
           
            c = lista_fin #Percorso ottimo per un singolo robot
            lista_punti = []
            for coppia in c:
                for punto in coppia:
                    if punto not in lista_punti:
                        lista_punti.append(punto)
        
        print(lista_punti)
        path_fin = astar2(maze,start2,end2,1,lista_punti)#Seconda implementazione dell'a-star 
        if c != None:                                    #utilizzando come euristica il post-smoothing
            for i in lista_fin:
                    pygame.draw.line(screen,VIOLET,i[0],i[1],3)
           
        if cont == len(path_fin):
            cont = len(path_fin)-1
            stop = True
        pygame.draw.circle(screen,RED,[path_fin[cont][1]*(width+margin)+10,path_fin[cont][0]*(width+margin)+10], 10)
        pygame.draw.circle(screen,RED,[path_fin[cont][3]*(width+margin)+10,path_fin[cont][2]*(width+margin)+10], 10)
        pygame.draw.circle(screen,RED,[path_fin[cont][5]*(width+margin)+10,path_fin[cont][4]*(width+margin)+10], 10)
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
    


