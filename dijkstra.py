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
    queue[(start_node.x, start_node.y)] = start_node
    distance[(start_node.x, start_node.y)] = 0

    while True:
        cur_index = min(queue, key=lambda o: queue[o].cost)
        cur = queue[cur_index]
        if cur.x == goal_node.x and cur.y == goal_node.y:
            # print("Goal!!!")
            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            break

        del queue[cur_index]
        visited[cur_index] = cur

        explored_map.append((cur.x, cur.y))

        for i in range(len(motion)):
            node = Node(cur.x + motion[i][0], cur.y + motion[i][1], cur.cost + motion[i][2], cur_index)
            node_index = (node.x, node.y)

            # Check if the next node is valid (Out of boundary)
            if not obstacle_map.is_valid(node.x, node.y):
                continue

            # Check if the next node is a obstacle
            if obstacle_map.is_obstacle(node.x, node.y):
                continue

            # If the next node is already visited, skip it
            if node_index in visited:
                continue

            if node_index in queue:
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:
                queue[node_index] = node

    # Backtrack the path from Goal to Start
    path = []
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = visited[parent_index]
        path.append((n.x, n.y))
        parent_index = n.parent_index

    path = list(reversed(path))
    path.append((goal_x, goal_y))
    return explored_map, path


def animation(start, goal, explored_map, path, file_dir):
    ob_map = ObstacleMap(start, goal)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(str(file_dir), fourcc, 20.0, (ob_map.numCols, ob_map.numRows))
    image = np.zeros((ob_map.numRows, ob_map.numCols, 3), dtype=np.uint8)

    for row in range(ob_map.numRows):
        for col in range(ob_map.numCols):
            if (image[int(ob_map.numRows - row - 1), int(col), 0] == 0
                    and image[int(ob_map.numRows - row - 1), int(col), 1] == 0
                    and image[int(ob_map.numRows - row - 1), int(col), 2] == 0):
                if ob_map.is_valid(row, col) and not ob_map.is_obstacle(row, col):
                    image[int(ob_map.numRows - row - 1), int(col)] = (154, 250, 0)
                    out.write(image)

    count = 0
    for state in explored_map:
        image[int(ob_map.numRows - 1 - state[0]), int(state[1])] = (255, 255, 0)
        if count % 10 == 0:
            out.write(image)
            cv2.imshow('dijkstra', image)
            cv2.waitKey(1)
        count = count + 1

    if len(path) > 0:
        for state in path:
            image[int(ob_map.numRows - 1 - state[0]), int(state[1])] = (0, 0, 255)
            out.write(image)
            cv2.imshow('dijkstra', image)
            cv2.waitKey(100)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

    out.release()

def is_valid(currRow, currCol):
    return 1 <= currRow <= 300 and 1 <= currCol <= 400

def main():
    start_row = int(input("Enter the row coordinate for start node (between 1 and 300) : "))
    start_col = int(input("Enter the column coordinate for start node (between 1 and 400) : "))
    goal_row = int(input("Enter the row coordinate for goal node (between 1 and 300) : "))
    goal_col = int(input("Enter the column coordinate for goal node (between 1 and 400) : "))
    start = (start_row, start_col)
    goal = (goal_row, goal_col)
    if is_valid(start[0], start[1]) and is_valid(goal[0], goal[1]):
        explored_map, path = dijkstra(start[0], start[1], goal[0], goal[1])
        animation(start, goal, explored_map, path, "./dijkstra.avi")


if __name__ == '__main__':
    main()
