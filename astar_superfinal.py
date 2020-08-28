from __future__ import division #para determinar si el resultado de una division es int o long
from AriaPy import * #Importemos todas las herramientas de AriaPy
import sys #Importemos el modulo sys
import numpy #Importemos el modulo Numpy
import math as m #Importar el modulo math para utilizar algunas funciones a implementar en el algoritmo
from heapq import * #Modulo que implementa el algoritmo headpq

debug = 1 # Si es verdadero se imprime el arreglo del mapa

# Establezcamos parametros de Aria

numpy.set_printoptions(threshold=numpy.inf) #Define las posiciones de un arreglo como +inf

Aria_init()
parser = ArArgumentParser(sys.argv) #se analizan argumentos
parser.loadDefaultArguments()
robot = ArRobot() #Clase para comunicar y operar el robot
conn = ArRobotConnector(parser, robot)
sonar = ArSonarDevice() #Esta clase se encarga de las lecturas del sonar
if not conn.connectRobot():
  print "No se pudo conectar con el robot, EXIT"
  Aria_exit(1)

if not Aria_parseArgs(): #Si no se pueden analizar los argumentos entonces se sale del programa
  Aria_logOptions() #Registra las opciones de los argumentos
  Aria_exit(1) #Sale del programa


# Representacion del mapa


print "Inicializando programa"

GoalState = {'x': 1700, 'y': 18320} #Coordenadas en mm, diccionario, elementos en forma clave-valor
gridSize = 510 # medida de cada elemento del grid
mapSize = int(45 * 1000 / gridSize) # 45m max de longitud del mapa
mapOffset = (mapSize / 2) #Se emplea para calcular las coordenadas del mapa y las reales
explored = numpy.zeros(shape=(mapSize,mapSize)) #crea una matriz de orden mapSize*mapSize, que sirve para representar el mapa

print "Realizado con exito"


def exploredInsert(x, y): 
    vecindad = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1),
                (0,2),(0,-2),(1,2),(1,-2),(2,0),(2,-1),(2,-2),(-2,-2),
                (-2,2),(-2,1),(-2,0),(-1,2),(-1,-2),(-2,-1),(2,2),(2,1)] #Esto es similar a lo que ocurre en la vecindad de Moore pero tambien considera el borde, se toman dos unidades de distancia como referencia entre las casillas de la vecindad porque distancia (2r + 1)^2 - 1 al usar 24 celdas r es 2

    for i, j in vecindad:
        #print "(%d, %d)" % (i, j)
        explored[int((x+i) / gridSize + mapOffset)][int((y+j) / gridSize + mapOffset)] = 1 #marca las celdas de la vecindad como visitadas, por ello el 1

#Se obtiene la coordenada real
def realcoords(x, y):
    return ((x - mapOffset) * gridSize, (y - mapOffset) * gridSize)

def mapcoords(x, y): # se obtiene la coordenada real pero redondeada
    return (int(round(x / gridSize + mapOffset)), int(round(y / gridSize + mapOffset)))

robot.addRangeDevice(sonar) #Obtiene la lectura mas cercana al robot dentro de un area especifica nxn
robot.runAsync(True)  #Continuar con la operacion del robot


# Comportamiento del agente

recover = ArActionStallRecover() #accion para solucionar algun problema en una llanta
robot.addAction(recover, 100) #agrega una accion a la lista de acciones por jerarqia (accion    ,int jerarquia)

gotoPoseAction = ArActionGotoStraight("goto") #esta accion dirige al agente en linea recta
robot.addAction(gotoPoseAction, 50)

stopAction = ArActionStop ("stop") #Accion para detener al agente
robot.addAction(stopAction, 40) 

robot.enableMotors() #este metodo de class ArRobot habilita los motores del robot

timer = ArTime() #Se inicializa el tiempo en el instante actual
timer.setToNow() #este metodo de la clase ArTime Restablece el tiempo

#
# Implementacion de algoritmo A estrella
#

path = [] #Creamos una lista para el path
goal = mapcoords(GoalState['x'], GoalState['y']); #obtenemos las coordenadas del mapa (los valores dados en el diccionario GoalState definido inicialmente)

def calcDistance(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def h(a, b):
    return m.sqrt(calcDistance(a, b))

def aStar(start):

    nearby = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)] #Posibilidades de movimiento en el grid (algo como distancia de manhatan)

    conjunto = set() #metodo que convierte un conjunto de iterables en una secuencia de iterables, admite listas, tuplas o diccionarios
    conjuntoaux= [] #lista abierta

    cameFrom = {} #diccionario haciendo referencia al nodo anterior
    G = {start: 0} #diccionario para el Costo
    fScore = {start: h(start, goal)} #diccionario de clave start valor heuristica con estados inicial y meta

    heappush(conjuntoaux, (fScore[start], start)) #El padre siempre tiene un valor menor o igual que el hijo, agrega el valor de fscore a conjuntoaux
    #agrega el valor recorrido (la distancia que ya se recorrio) porque se tiene submetas para alcanzar el goalstate

    while conjuntoaux: #mientras conjuntoaux no este vacio

        actual = heappop(conjuntoaux)[1] #Devuelve el nodo con menor heuristica

        if actual == goal: #si se ha alcanzado la meta
            totalPath = [] #ruta desde el goal hasta start
            while actual in cameFrom: #mientras el valor actual del nodo anterior
                totalPath.append(actual) #agrega esa coordenada a la lista de total path
                actual = cameFrom[actual]  #se tiene un vector anterior al actual para pasar a la siguiente posicion
            return totalPath #si no, retorna total path

        conjunto.add(actual) #agrega el actual a la lista cerrada
        for i, j in nearby: #para cada vecino del nodo actual
            neighbor = actual[0] + i, actual[1] + j # se obtienen las coordenadas
            tentativeGScore = G[actual] + h(actual, neighbor) #costo del nodo en el que estamos mas el de la heuristica con el actual y el vecino, costo total acumulado
            if 0 <= neighbor[0] < explored.shape[0]: #si se esta dentro de los limites del mapa para x
                if 0 <= neighbor[1] < explored.shape[1]: #si se esta dentro de los limites del mapa para y
                    if explored[neighbor[0]][neighbor[1]] == 1: #si el nodo vecino actual ya fue visitado, entonces se ignora
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in conjunto and tentativeGScore >= G.get(neighbor, 0): #si el nodo vecino ya esta en la lista conjunto y su tentativeGScore es peor, entonces se ignora
                continue

            if  tentativeGScore < G.get(neighbor, 0) or neighbor not in [i[1]for i in conjuntoaux]:#si tentativeGScore es mejor o el vecino no ha sido agregado a la lista abierta
                cameFrom[neighbor] = actual
                G[neighbor] = tentativeGScore
                fScore[neighbor] = tentativeGScore + h(neighbor, goal)
                heappush(conjuntoaux, (fScore[neighbor], neighbor))

    return False

#
# Main
#

reroute = True #parametro de aria que se usara para evaluar si se recalcula una ruta o se ha alcanzado la meta

while Aria.getRunning(): #Mientras Aria se encuentre en ejecucion
    robot.lock() #este metodo de la clase ArRobot, bloque la instancia robot

    poses = sonar.getCurrentBufferAsVector() #Este metodo de la clase ArRangeDevice obtiene las lecturas del bufer como vector
    for p in poses:
        try: #intenta agregar las coordenadas excepto si detecta pared
            exploredInsert(p.x, p.y)
        except Exception, e:
            print "Error al intentar insertar x: %d, y: %d -> x: %d, y: %d" % (p.x, p.y, int(p.x / gridSize + mapOffset), int(y / gridSize + mapOffset))
            print e

    if (reroute or gotoPoseAction.haveAchievedGoal()):  #Verifica si ha llegado al estado meta o hay que rebuscar
        reroute = False #para parar
        myPos = robot.getPose() #Obtiene la posicion global actual de agente
        path = aStar(mapcoords(myPos.x, myPos.y)) #el camino lo manda de acuerdo a x y y

        # Si se llega al tile Grid final
        if not path: #si ya no hay camino que calcular que obtiene de astar
            ArLog.log(ArLog.Normal, "GoalState Alcanzado"); #Registra un mensaje
            print explored
            robot.unlock() #Desbloquea el robot
            break

        # Si no, se evalua otro paso
        nextTile = path[-1]
        nextPos = realcoords(nextTile[0], nextTile[1]);
        nextPosEase = (nextPos[0] + (myPos.x % gridSize), nextPos[1] + (myPos.x % gridSize))
        gotoPoseAction.setGoal(ArPose(nextPos[0], nextPos[1])); #Define una nueva meta y las acciones para llegar a ella
        #Registramos un nuevo mensaje
        ArLog.log(ArLog.Normal, "El agente se dirige a la siguiente meta: %.0f %.0f" % (gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY()) );

    # Se imprime el arreglo del mapa
    if (debug and timer.mSecSince() >= 5000): #Obtiene el num de mseg transcurrido desde el ultimo reseteo
        print explored
        print "\n\n\n"
        timer.setToNow() #Restablecemos el tiempo
 
    distance = robot.findDistanceTo(ArPose(nextPos[0], nextPos[1])) #Obtiene la distancia hacia un punto desde la posicion actual

    # En caso de aproximarse a un grid o a un obstaculo
    if distance < gridSize / 2 or explored[nextTile[0]][nextTile[1]] == 1:
        gotoPoseAction.cancelGoal() #Eliminar la meta actual
        reroute = True

    robot.unlock() #Desbloqueamos el robot
    ArUtil.sleep(100) #Desactivamos el agente

Aria_exit(0) #Finaliza la simulacion
