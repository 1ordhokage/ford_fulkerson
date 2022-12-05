import time
import numpy as np


class Graph:
    def __init__(self, graph):
        self.graph = graph
        self.ROW = len(graph)

    def bfs(self, s, t, parent):
        """Алгоритм поиска в ширину."""

        visited = [False] * self.ROW
        queue = [s]

        # Отмечаем исток посещенным
        visited[s] = True

        # поиск в ширину
        while queue:
            u = queue.pop(0)
            for ind, val in enumerate(self.graph[u]):
                if visited[ind] is False and val > 0:
                    queue.append(ind)
                    visited[ind] = True
                    parent[ind] = u
                    if ind == t:
                        return True
        # если не дошли до стока
        return False

    # Алгоритм Форда-Фалкерсона
    def ford_fulkerson(self, source, sink):
        parent = [-1] * self.ROW
        max_flow = 0
        while self.bfs(source, sink, parent):
            path_flow = float("Inf")
            s = sink
            while s != source:
                path_flow = min(path_flow, self.graph[parent[s]][s])
                s = parent[s]
            max_flow += path_flow
            v = sink
            while v != source:
                u = parent[v]
                self.graph[u][v] -= path_flow
                self.graph[v][u] += path_flow
                v = parent[v]

        return max_flow


def generate_graph(t):
    """Генерация матрицы графа заданного размера n x n, n = 2^t"""
    n = 2 ** t
    g = np.random.randint(100, size=(n, n))

    for i in range(n):
        g[i][i] = 0  # обнуляем главную диагональ.
    return n, np.count_nonzero(g), g  # Размерность матрицы (количество вершин), кол-во ребер, граф


def calculate(t):
    start = time.perf_counter()

    data = generate_graph(t)
    v = data[0]
    e = data[1]
    graph = data[2]
    source = 0
    sink = 2 ** t - 1
    res = Graph(graph).ford_fulkerson(source, sink)

    elapsed = time.perf_counter() - start

    return v, e, res, elapsed  # Размерность матрицы (количество вершин), кол-во ребер, максимальный поток, время работы


def main():
    time_array = []
    for i in range(1, 11):
        with open(f"iter{i}.txt", "w") as f:
            for t in range(4, 11):
                data = calculate(t)
                f.write(
                    f'Размерность матрицы (количество вершин): {data[0]}\nКоличество ребер: {data[1]}\nМаксимальный поток: {data[2]}\nВремя работы: {1000 * data[3]:0.7f} мс\n\n'
                )
                time_array.append(data[3])
    avg_time_array = []
    for j in range(0, 7):
        avg_time_array.append(
            sum(time_array[j::7]) / 10.0
        )
    print(avg_time_array)  # [0.0004256832997675519, 0.0021258206994389183, 0.013238470799842616, 0.10457553330052179, 0.7332554123990122, 6.7829942331001805, 54.254011233199705]

