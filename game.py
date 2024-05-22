import pygame

import time

pygame.init()

WIDTH =800
HEIGHT = 600

screen = pygame.display.set_mode((WIDTH,HEIGHT))
player = pygame.Rect((300,250,10,10))

run = True

while run:
    screen.fill((0,0,0))
    pygame.draw.rect(screen,(250,0,0),player)

    key = pygame.key.get_pressed()

    if key[pygame.K_a] ==True:
        player.move_ip(-1,0)
    elif key[pygame.K_d] ==True:
        player.move_ip(1,0)
    elif key[pygame.K_w] ==True:
        player.move_ip(0,-1)
    elif key[pygame.K_s] ==True:
        player.move_ip(0,1)
    elif key[pygame.K_0]== True and key[pygame.K_1]==True:
        player.move_ip(1,-1)
    for event in pygame.event.get():
        if event.type ==pygame.QUIT:
            run =False
    pygame.display.update()
pygame.quit()