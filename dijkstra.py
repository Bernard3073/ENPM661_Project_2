import matplotlib.pyplot as plt
import numpy as np
import heapq

map_width = 400
map_height = 300


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index


def motion_model():
    model = {'Right': (1, 0, 1),
             'Left': (-1, 0, 1),
             'Up': (0, 1, 1),
             'Down': (0, -1, 1),
             'UpRight': (1, 1, np.sqrt(2)),
             'DownRight': (1, -1, np.sqrt(2)),
             'UpLeft': (-1, 1, np.sqrt(2)),
             'DownLeft': (-1, -1, np.sqrt(2))}

    return model


def dijkstra(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y):
    start_node = Node(start_x, start_y, 0, -1)
    goal_node = Node(goal_x, goal_y, 0, -1)

    motion = motion_model()
    queue, visited = dict(), dict()
    queue[(start_node.x, start_node.y)] = start_node  # dictionary
    while True:
        cur_index = min(queue, key=lambda o: queue[o].cost)
        # print(cur_id)
        cur = queue[cur_index]
        if cur.x == goal_node.x and cur.y == goal_node.y:
            print("Goal!!!")
            break
        del queue[cur_index]

        for value in motion.items():
            node = Node(cur + value[0], cur.y + value[1], cur.cost + value[2], cur_index)
            node_index = (node.x, node.y)
            if node_index in queue:
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:
                queue[node_index] = node



def main():
    path = dijkstra()
if __name__ == '__main__':
    main()