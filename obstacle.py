class ObstacleMap:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.numRows = 300
        self.numCols = 400

    # check if the move is valid
    def is_valid(self, currRow, currCol):
        return 1 <= currRow <= self.numRows and 1 <= currCol <= self.numCols

    @staticmethod
    # check if it is an obstacle
    def is_obstacle(self, row, col):
        def check_grad(a1, b1, a2, b2):
            return ((row - b1) * (a2 - a1)) - ((b2 - b1) * (col - a1))

        # Coordinate: (col, row)

        # check circle
        dist1 = ((row - 70) ** 2 + (col - 90) ** 2) - (35 ** 2)

        # check eclipse
        dist2 = ((((row - 145) ** 2) / 30 ** 2) + (
                ((col - 246) ** 2) / (60 ** 2))) - 1

        # 0: Obstacle 1: No Obstacle
        if 200 <= col <= 210 and 230 <= row <= 280:
            dist3 = 0
        elif 210 <= col <= 230 and 270 <= row <= 280:
            dist3 = 0
        elif 210 <= col <= 230 and 230 <= row <= 240:
            dist3 = 0
        else:
            dist3 = 1

        # Eclipse
        (x1, y1) = (36.53, 124.38)
        (x2, y2) = (48, 108)
        (x3, y3) = (170.8728, 194.0364655)
        (x4, y4) = (159.4, 210.4195)
        first = check_grad(x1, y1, x2, y2)
        second = ((row - y2) * (x3 - x2)) - ((y3 - y2) * (col - x2))
        third = ((row - y3) * (x4 - x3)) - ((y4 - y3) * (col - x3))
        fourth = ((row - y4) * (x1 - x4)) - ((y1 - y4) * (col - x4))
        dist4 = 1
        if first >= 0 and second >= 0 and third >= 0 and fourth >= 0:
            dist4 = 0

        # Polygon
        (p1_x, p1_y) = (285.5736, 105.4264)
        (p2_x, p2_y) = (328, 63)
        (p3_x, p3_y) = (381.033, 116.033)
        (p4_x, p4_y) = (381.033, 171.033)
        (p5_x, p5_y) = (354, 138)
        (p6_x, p6_y) = (325.17158, 145.02438)
        first = check_grad(p1_x, p1_y, p2_x, p2_y)
        second = check_grad(p2_x, p2_y, p3_x, p3_y)
        third = check_grad(p3_x, p3_y, p4_x, p4_y)
        fourth = check_grad(p5_x, p5_y, p1_x, p1_y)
        dist5 = 1
        if first >= 0 and second >= 0 and third >= 0 and fourth >= 0:
            dist5 = 0
        first = check_grad(p1_x, p1_y, p5_x, p5_y)
        second = check_grad(p5_x, p5_y, p6_x, p6_y)
        third = check_grad(p6_x, p6_y, p1_x, p1_y)
        dist6 = 1
        if first >= 0 and second >= 0 and third >= 0:
            dist6 = 0

        if dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0:
            return True
        return False
