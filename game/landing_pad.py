from collections import deque
import random

class LandingPad():
    def __init__(self, r, c, width):
        self.row = r
        self.col = c
        self.width = width

    def is_carrying(self, obj_row, obj_col, width):
        if self.col <= obj_col <= self.col + self.width or self.col <= obj_col + width <= self.col + self.width:
            return abs(self.row - obj_row) <= 1
                
        return False

class LandingPadCollection():
    def __init__(self, rows, cols, start_row, start_col):
        self.collection = deque()
        self.row_limit = rows
        self.col_limit = cols
        self.width = 60
        self.max_row_distance_between_pads = 20
        self.start_row = start_row
        self.start_col = start_col
        self.initialize()
        random.seed(10)

    def get_all(self):
        return self.collection

    def initialize(self):
        self.collection.append(LandingPad(self.start_row, self.start_col, self.width))

        for i in range(self.max_row_distance_between_pads // 2, self.row_limit, self.max_row_distance_between_pads):
            r = random.randint(i - self.max_row_distance_between_pads // 2, i + self.max_row_distance_between_pads // 2)
            c = random.randint(0, self.col_limit)
            self.collection.append(LandingPad(r, c, self.width))

    def shift_downward(self, shift):
        for i in range(len(self.collection)):
            self.collection[i].row += shift

    def prune(self):
        if self.collection[-1].row > self.row_limit:
            self.collection.pop()
            self.collection.appendleft(self.spawn_new(self.collection[0].row))

    def scroll(self, shift):
        self.shift_downward(shift)
        self.prune()

    def spawn_new(self, row):
        return LandingPad(random.randint(0, row), random.randint(0, self.col_limit), self.width)
    
    def is_on_pad(self, row, col, width):
        for i in range(len(self.collection)):
            if self.collection[i].is_carrying(row, col, width):
                return True
            
        return False