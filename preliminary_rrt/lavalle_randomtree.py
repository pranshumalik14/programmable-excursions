#!/usr/bin/env python

import random, math, pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2, pi

# constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000


def main():
    # initialize and prepare screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption("Random Tree      S. LaValle    Nov 2020")
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(black)

    nodes = []

    nodes.append((XDIM / 2.0, YDIM / 2.0))  # Start in the center

    i = 0
    while i < NUMNODES:
        ri = random.randint(0, len(nodes) - 1)
        rtheta = random.random() * 2.0 * pi
        nn = nodes[ri]
        newnode = (nn[0] + 5.0 * cos(rtheta), nn[1] + 5.0 * sin(rtheta))
        nodes.append(newnode)
        pygame.draw.line(screen, white, nn, newnode)
        pygame.display.update()
        i += 1

    done = 0
    while not done:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                print "Leaving because you said so\n"
                done = 1
                break


# if python says run, then we should run
if __name__ == "__main__":
    main()