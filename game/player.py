class Player():
    def __init__(self, row, col, fall_rate, default_fall_rate):
        self.row = row
        self.col = col
        self.up_distance = 0
        self.command_left_right = 0
        self.fall_rate = fall_rate
        self.default_fall_rate = default_fall_rate
        self.used_jump = False
        self.mode = 'EASY'

    def process_command(self, command_lateral, command_up_y, command_up_z):
        self.command_left_right += command_lateral

        if self.mode == 'EASY':
            if (command_up_y != 0 or command_up_z != 0) and not self.used_jump:
                self.used_jump = True
                self.up_distance = -250
        elif self.mode == 'REGULAR':
            if command_up_z != 0 and not self.used_jump:
                self.used_jump = True
                self.up_distance = -250

    def update_location(self, max_cols):
        # Update horizontal location
        self.col += self.command_left_right
        if self.col < 0:
            self.col = 0
        elif self.col > max_cols:
            self.col = max_cols
        self.command_left_right = 0

        # Update vertical location
        if self.up_distance < 0:
            self.row -= 10
            self.up_distance += 10

    def constant_fall(self, on_pad):
        if on_pad:
            self.row += self.default_fall_rate
            self.used_jump = False
        else:
            self.row += self.fall_rate

        if self.mode == 'EASY':
            if self.row + 60 >= 960:
                self.row = 960 - 60

    def has_died(self, death_row):
        if self.mode == 'EASY':
            return False
        
        return self.row > death_row