import heapq
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
import math

class AStarPathfinder:
    def __init__(self, map_array: np.array, start: tuple, goal: tuple, wall_influence=5.0, buffer_factor=2.0):
        """
        Inicializa o A* com mapa, ponto inicial, objetivo e parâmetros de influência.

        Args:
            map_array (np.array): Mapa binário (obstáculos e caminho livre).
            start (tuple): Ponto inicial (linha, coluna).
            goal (tuple): Ponto objetivo (linha, coluna).
            wall_influence (float): Peso da proximidade das paredes.
            buffer_factor (float): Escala da influência das paredes.
        """
        self.start = start
        self.goal = goal
        self.wall_influence = wall_influence
        self.buffer_factor = buffer_factor
        self.GOAL_REACHEABLE = False  
        
        # Prepara o mapa, expandindo suas bordas e ajustando o array.
        self.map = map_array.copy()                                   #copia o mapa
        self.map_array = self.preprocess_map(map_array)               #guarda uma cópia do mapa original 

        # Cria um campo potencial baseado no mapa para influenciar o caminho.
        self.potential_field = self.create_potential_field()
        

    def preprocess_map(self, map_array: np.array) -> np.array:
        """
        Ajusta o mapa, convertendo valores intermediários para obstáculos.

        Args:
            map_array (np.array): Mapa original.

        Returns:
            np.array: Mapa processado.
        """

        processed_map = map_array.copy()
        for i in range(map_array.shape[0]):
            for j in range(map_array.shape[1]):
                valor = map_array[i][j]
                #valor menor que 60 vira obstaculo
                if valor < 60:
                    processed_map[i][j] = 0
                #valor entre 60 e 150 vira desconhecido
                elif valor == 128 or valor == 205:
                    processed_map[i][j] = 128
                #valor maior ou igual a 150 vira livre
                elif valor >= 150:
                    processed_map[i][j] = 255
                #resto vira obstaculo
                else:
                    processed_map[i][j] = 0

        return processed_map

    def create_potential_field(self) -> np.array:
        
        """
        Gera campo potencial com base na distância de obstáculos.

        Returns:
            np.array: Campo potencial.
        """
        
        vazio = self.map_array != 0

        #distância até o obstáculo mais próximo
        distancias = distance_transform_edt(vazio)

        #Quanto maior o valor, mais distante a célula está das paredes
        #todas as células cuja distância até um obstáculo seja menor que buffer_factor tornam-se obstáculos if bufffactor=3
        perto = distancias < self.buffer_factor
        self.map_array[perto] = 0

        #isso deixa a parede mais "espessa"

        #cria uma matriz de números reais, inicialmente preenchida com zero.
        potencial = np.zeros_like(distancias, dtype=float)

        #define até que distância a parede influencia o custo.
        distancia_influencia = self.buffer_factor * 5


        #percorre o mapa
        for i in range(self.map_array.shape[0]):
            for j in range(self.map_array.shape[1]):
                #obstáculos recebem custo infinito
                if self.map_array[i][j] == 0:
                    potencial[i][j] = np.inf
                #células próximas de obstáculos recebem uma penalização
                elif distancias[i][j] < distancia_influencia:
                    potencial[i][j] = self.wall_influence * (
                        distancia_influencia - distancias[i][j]
                    )

        #formula é: custo potencial = wall_influence * (distancia_influencia - distancia_ate_obstaculo)

        return potencial

    def heuristic(self, a: tuple, b: tuple) -> float:
        """
        Calcula a heurística entre dois pontos.

        Args:
            a (tuple): Ponto A.
            b (tuple): Ponto B.

        Returns:
            float: Resultado da heurística.
        """
        #calcula as diferenças entre linhas e colunas
        dist_x = a[0] - b[0]
        dist_y = a[1] - b[1]

        #aplica a distancia euclidiana
        dist = ((dist_x ** 2) + (dist_y ** 2)) ** 0.5

        #daria para usar a distancia de manhattan pq é permitido apenas movimentos verticais e horizontais
        return dist

    def find_path(self):
        """
        Executa o algoritmo A* para encontrar caminho até o objetivo.

        Returns:
            dict: Predecessores dos nós no caminho. Se o caminho não for encontrado, retorna None.
            tuple: O ponto final (objetivo) ou None se não encontrado.
        """

        #cria uma fila de prioridade e insere o ponto inicial
        #na teoria ele usa sistema de fila, ignora a nomeclatura 
        pilha = []
        heapq.heappush(pilha, (0, self.start))

        #O A* prioriza os nós com menor: f(n) = g(n) + h(n)

        #cada item possui um (f_score, posição)
        # O menor f_score é retirado primeiro.

        # g_score: custo real acumulado desde o início;
        # heuristic: estimativa até o objetivo;
        # f_score: soma dos dois.
        # Além disso, o g_score inclui o custo de proximidade das paredes.
                
        came_from = {}   #Armazena de onde cada célula veio.
        
        g_score = {}
        g_score[self.start] = 0
        #g_score armazena o menor custo conhecido entre o início e cada célula (o custo do ponto inicial é zero)
        
        #O robô pode movimentar-se em quatro direções
        direcoes = [
            (-1, 0),  # cima
            (1, 0),   # baixo
            (0, -1),  # esquerda
            (0, 1)    # direita
        ]

        #Armazena as dimensões do mapa.
        linhas = self.map_array.shape[0]
        colunas = self.map_array.shape[1]

        #continua enquanto existir algum ponto a explorar
        while pilha:
            #remove o item com menor prioridade e pega sua posição
            current = heapq.heappop(pilha)[1]

            #Se a posição atual for o objetivo
            if current == self.goal:
                #marca que o objetivo foi alcançado;
                #retorna o dicionário de predecessores;
                #retorna o nó final.
                self.GOAL_REACHEABLE = True
                return came_from, current

            #geração dos vizinhos
            for direcao in direcoes:
                vizinho = (
                    current[0] + direcao[0],
                    current[1] + direcao[1]
                )
                
                # pra nao sair do mapa
                if vizinho[0] < 0 or vizinho[0] >= linhas:
                    continue
                
                if vizinho[1] < 0 or vizinho[1] >= colunas:
                    continue
                
                #se for uma parede/obstaculo
                if self.map_array[vizinho] == 0:
                    continue

                #Calcula o custo do movimento
                move_cost = self.heuristic(current, vizinho)
                #obtem a penalização por proximidade de paredes
                wall_cost = self.potential_field[vizinho]

                #calcula o possível novo custo para chegar ao vizinho
                tentative_g_score = (
                    g_score[current]
                    + move_cost
                    + wall_cost
                )
                #g(vizinho) = g(atual) + custo do movimento + custo da parede
                
                #atualiza vizinho se ainda não tiver sido visitado ou o novo caminho até ele for mais barato
                if vizinho not in g_score or tentative_g_score < g_score[vizinho]:
                    #registra que o vizinho foi alcançado a partir da posição atual
                    came_from[vizinho] = current

                    #salva o novo custo
                    g_score[vizinho] = tentative_g_score

                    #calcula f_score = g_score + heurística até o objetivo
                    f_score = tentative_g_score + self.heuristic(vizinho, self.goal)

                    #insere o vizinho na fila de prioridade
                    heapq.heappush(pilha, (f_score, vizinho))
                    
        #se a fila esvaziar sem alcançar o objetivo
        print("Caminho não encontrado")
        return None, None

    def reconstruct_path(self, came_from: dict, current: tuple) -> list:
        #O A* não constrói diretamente uma lista ordenada do caminho. Ele guarda o predecessor de cada posição
        """
        Reconstrói o caminho a partir do ponto final até o inicial.
        
        Args:
            came_from (dict): O dicionário de predecessores no caminho.
            current (tuple): O ponto final (objetivo).
        
        Returns:
            list: Lista de tuplas com caminho reconstruído.
        """
        #Inicializa o caminho com o nó final
        list = [current]
        
        #o while segue a linha de pai para filho dentro do dicionario, 
        #conectando ao caminho encontrado de trás pra frente
        while current in came_from:
            current = came_from[current]
            list.append(current)
        
        #inverte o caminho para a sequência correta
        list.reverse()

        return list

    def know_path(self, path: list) -> list:
        #Essa função corta o caminho quando encontra uma região desconhecida
        """
        Remove trechos desconhecidos e ajusta o caminho, se necessário.

        Args:
            path (list): Caminho completo.

        Returns:
            list: Caminho ajustado.
        """

        #cria uma lista para armazenar apenas a parte conhecida
        caminho_conhecido = []
        #percorre as células do caminho
        for celula in path:
            linha = celula[0]
            coluna = celula[1]
            #se encontrar uma célula desconhecida, interrompe o percurso
            if self.map_array[linha][coluna] == 128: # checa se o caminho nao é desconhecido
                break
            #se a célula não for desconhecida, adiciona-a ao resultado
            caminho_conhecido.append(celula)
        
        return caminho_conhecido

    def simplify_path(self, path: list) -> list:
        #Essa função remove pontos intermediários em trechos retos, mantendo:
        #o ponto inicial;
        #pontos onde ocorre mudança de direção;
        #o ponto final.
        """
        Simplifica o caminho removendo direções repetidas.
        
        Args:
            path (list): Caminho completo.

        Returns:
            list: Caminho simplificado.
        """
        #Se a lista estiver vazia, retorna outra lista vazia
        if not path:
            return []

        #Se houver apenas um ou dois pontos, não existe nenhum ponto intermediário a remover
        if len(path) <= 2:
            return path.copy()

        #O ponto inicial sempre é preservado
        caminho_simplificado = [path[0]]

        # direcao do primeiro segmento
        #calcula a direção entre o primeiro e o segundo ponto
        direcao_anterior = (
            path[1][0] - path[0][0],
            path[1][1] - path[0][1]
        )

        #O laço começa em 2, pois a direção entre os pontos 0 e 1 já foi calculada
        for i in range(2, len(path)):
            #calcula a direção do segmento atual
            direcao_atual = (
                path[i][0] - path[i - 1][0],
                path[i][1] - path[i - 1][1]
            )

            # se mudou a direcao, o ponto anterior é um canto importante
            #se a direção mudou, significa que houve uma curva
            if direcao_atual != direcao_anterior:
                #adiciona o ponto imediatamente anterior, pois ele é o ponto da curva
                caminho_simplificado.append(path[i - 1])
                #atualiza a direção de referência
                direcao_anterior = direcao_atual

        # garante que o ultimo ponto entre no caminho
        #o último ponto sempre é adicionado
        caminho_simplificado.append(path[-1])

        #retorna somente os pontos essenciais
        return caminho_simplificado

    def plot_path(self, path: list, simplified_path: list):
        """
        Exibe o mapa com o caminho completo e o simplificado.

        Args:
            path (list): O caminho completo encontrado.
            simplified_path (list): O caminho simplificado encontrado.
        """
        simplified_path = self.simplify_path(path)

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray')
        plt.scatter(self.start[1], self.start[0], color='green', s=100, label='Início')
        plt.scatter(self.goal[1], self.goal[0], color='blue', s=100, label='Objetivo')

        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_y, path_x, color='magenta', linewidth=1, label='Caminho Completo')
            simp_x, simp_y = zip(*simplified_path)
            plt.plot(simp_y, simp_x, color='red', linewidth=2, linestyle='--', label='Caminho Simplificado')
        else:
            plt.title("Caminho não encontrado")

        plt.legend()
        plt.axis('equal')
        plt.show()

    def run(self, show_path=True):
        """
        Essa função é chamada pelo navegador para executar o algoritmo A* e gerar o caminho.
        Executa o processo completo: busca, reconstrução, simplificação e visualização do caminho.
        
        Args:
            show_path (bool): Se True, exibe o caminho graficamente.

        Returns:
            list or None: Caminho simplificado ou None se não encontrado.
        """
        print("Iniciando busca pelo caminho...")
        came_from, final_node = self.find_path()

        if final_node:
            print("Reconstruindo caminho...")
            path = self.reconstruct_path(came_from, final_node)
            
            print("Robo não anda no disconhecido")
            path = self.know_path(path)

            print("Caminho encontrado, simplificando...")
            simplified_path = self.simplify_path(path)

            print("Plotando o caminho...")
            if show_path:
                self.plot_path(path, simplified_path)
            
            return simplified_path
        else:
            print("Nenhum caminho pôde ser encontrado.")
            return None


def prep_map(map_path: str) -> np.array:
    """
    Prepara o mapa carregando e processando a imagem de entrada.

    Args:
        map_path (str): O caminho do arquivo do mapa.

    Returns:
        np.array: O mapa processado como um array numpy.
    """
    map_array = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    map_array[map_array == 0] = 0
    map_array[map_array == 205] = 128
    map_array[map_array == 254] = 255
    map_array[(map_array >= 60) & (map_array != 128) & (map_array != 255)] = 0
    map_array = map_array.astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    map_array = cv2.morphologyEx(map_array, cv2.MORPH_OPEN, kernel)
    map_array = np.flipud(map_array)
    map_array = np.pad(map_array, ((0, 200), (0, 200)), 'constant', constant_values=128)
    return map_array


def main():
    map_array = prep_map('map5.pgm')
    astar = AStarPathfinder(map_array, (60, 20), (60, 120), wall_influence=10.0, buffer_factor=3.0)
    astar.run()


if __name__ == '__main__':
    main()