import os
import pygame
from landing_pad import LandingPadCollection
from player import Player
import socket

ROWS = 960
COLUMNS = 576
WIDTH = 40
HEIGHT = 60

UDP_IP = "127.0.0.1"
UDP_PORT = 12345

os.environ['SDL_VIDEO_CENTERED'] = '1'

class Button:
    def __init__(self, text, pos, size):
        font = pygame.font.SysFont("Arial", 40)
        self.text = text
        self.rect = pygame.Rect(pos, size)
        self.color = (200, 200, 200)
        self.text_surf = font.render(text, True, (0, 0, 0))
        self.text_rect = self.text_surf.get_rect(center = self.rect.center)

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect)
        screen.blit(self.text_surf, self.text_rect)

    def is_hovered(self, mouse_pos):
        return self.rect.collidepoint(mouse_pos)

    def update(self, mouse_pos):
        self.color = (100, 100, 100) if self.is_hovered(mouse_pos) else (200, 200, 200)

class Game():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)

        print(f"Listening on {UDP_IP}:{UDP_PORT}")
        pygame.init()
        self.window = pygame.display.set_mode((COLUMNS, ROWS))
        pygame.display.set_caption("Petr Jump Game")

        self.background = pygame.image.load("sky.jpg")
        self.background = pygame.transform.scale(self.background, (COLUMNS, ROWS))

        self.shift = 1
        self.clock = pygame.time.Clock()
        start_row, start_col = ROWS // 2, COLUMNS / 2 - WIDTH / 2

        self.player_icon = pygame.image.load("ptr.png")
        self.player_icon = pygame.transform.scale(self.player_icon, (WIDTH, HEIGHT))  

        self.player = Player(start_row, start_col, 3, self.shift)
        self.landing_pads = LandingPadCollection(ROWS, COLUMNS, start_row + HEIGHT, start_col)
        self.player_died = False
        self.running = True

        self.command_lateral = 0
        self.command_up_y = 0
        self.command_up_z = 0

    # def processInput(self):
    #     self.command_lateral = 0
    #     self.command_up = 0

    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             self.running = False
    #             break
    #         elif event.type == pygame.KEYDOWN:
    #             if event.key == pygame.K_RIGHT:
    #                 self.command_lateral = 10
    #             elif event.key == pygame.K_LEFT:
    #                 self.command_lateral = -10
    #             elif event.key == pygame.K_UP:
    #                 self.command_up = 1

    #     self.player.process_command(self.command_lateral, self.command_up)

    def update(self):
        self.landing_pads.scroll(self.shift)
        
        self.player.update_location(COLUMNS - WIDTH)
        on_pad = self.landing_pads.is_on_pad(self.player.row + HEIGHT, self.player.col, WIDTH) and self.player.up_distance == 0
        self.player.constant_fall(on_pad)

        if self.player.has_died(ROWS):
            self.player_died = True
            self.running = False

    def draw_landing_pads(self):
        for p in self.landing_pads.get_all():
            pygame.draw.rect(self.window, (0, 0, 0), (p.col, p.row, p.width, 10))

    def draw_player(self):
        self.window.blit(self.player_icon, (self.player.col, self.player.row))

    def render(self):
        self.window.blit(self.background, (0, 0))
        self.draw_landing_pads()
        self.draw_player()
        pygame.display.update()

    def game_over(self):
        self.window.fill((0, 0, 0))
        font = pygame.font.SysFont(None, 70)
        img = font.render('GAME OVER', True, (255, 255, 255))
        self.window.blit(img, (COLUMNS // 4, ROWS // 2))
        pygame.display.update()

        exit = False
        while not exit:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit = True
                    break

    def run(self):
        easy_button = Button("Easy Mode", (COLUMNS // 4, ROWS // 3), (COLUMNS // 2, ROWS // 5))
        regular_button = Button("Regular Mode", (COLUMNS // 4, 2 * ROWS // 3), (COLUMNS // 2, ROWS // 5))

        mode_selected = False

        while not mode_selected:
            self.window.blit(self.background, (0, 0))
            mouse_pos = pygame.mouse.get_pos()

            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if easy_button.is_hovered(mouse_pos):
                        self.player.mode = 'EASY'
                        mode_selected = True
                    elif regular_button.is_hovered(mouse_pos):
                        self.player.mode = 'REGULAR'
                        mode_selected = True

            for button in [easy_button, regular_button]:
                button.update(mouse_pos)
                button.draw(self.window)

            pygame.display.update()

        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                message = data.decode()
                self.command_lateral, self.command_up_y, self.command_up_z = message.split(';')
                # print("Received:", data.decode())
            except BlockingIOError:
                pass
            
            # self.processInput()
            self.player.process_command(int(self.command_lateral), int(self.command_up_y), int(self.command_up_z))

            self.update()
            self.render()
            self.clock.tick(60)

        self.game_over()

        pygame.quit()