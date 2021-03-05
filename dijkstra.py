import cv2
import numpy as np
from obstacle import ObstacleMap


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index


def motion_model():
    model = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, np.sqrt(2)],
              [-1, 1, np.sqrt(2)],
              [1, -1, np.sqrt(2)],
              [1, 1, np.sqrt(2)]]

    return model


def dijkstra(start_x, start_y, goal_x, goal_y):
    start_node = Node(start_x, start_y, 0, -1)
    goal_node = Node(goal_x, goal_y, 0, -1)

    obstacle_map = ObstacleMap((start_x, start_y), (goal_x, goal_y))
    path, distance, queue, visited = dict(), dict(), dict(), dict()



    motion = motion_model()
    explored_map = []
    queue[(start_node.x, start_node.y)] = start_node  # dictionary
    distance[(start_node.x, start_node.y)] = 0

    while True:
        cur_index = min(queue, key=lambda o: queue[o].cost)
        cur = queue[cur_index]
        if cur.x == goal_node.x and cur.y == goal_node.y:
            print("Goal!!!")
            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            break

        del queue[cur_index]
        visited[cur_index] = cur

        explored_map.append((cur.x, cur.y))

        for i in range(len(motion)):
            node = Node(cur.x + motion[i][0], cur.y + motion[i][1], cur.cost + motion[i][2], cur_index)
            node_index = (node.x, node.y)

            # Check if it is a obstacle
            if obstacle_map.is_obstacle(obstacle_map, node.x, node.y):
                continue
            # If the node is already visited, skip it
            if node_index in visited:
                continue

            if node_index in queue:
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:
                queue[node_index] = node

    # Generate Path to Goal
    path = []
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = visited[parent_index]
        path.append((n.x, n.y))
        parent_index = n.parent_index

    path = list(reversed(path))
    path.append((goal_x, goal_y))
    return explored_map, path

# def animation(self, )

def main():
    start = (5, 5)
    goal = (10, 10)
    path, explored_map = dijkstra(start[0], start[1], goal[0], goal[1])
    print(path)
    print(explored_map)
    # for point1, point2 in zip(a, a[1:]):
    
if __name__ == '__main__':
    main()