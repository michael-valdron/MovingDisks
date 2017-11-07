# Assignment #2: Moving Disks
# Student Name: Michael Valdron
# Student ID: 100487615
# Date: November 7, 2017

import pygame, sys, os
import numpy as np
import random as rand
from math import floor
from scipy.integrate import ode

# Model Scale
SCALE = 128

class Disk(pygame.sprite.Sprite):
    
    def __init__(self, imgfile, radius, mass=1.0):
        pygame.sprite.Sprite.__init__(self)

        self.image = pygame.image.load(imgfile)
        self.image = pygame.transform.scale(self.image, (int(floor(radius*SCALE*2)), int(floor(radius*SCALE*2))))
        self.state = [0, 0, 0, 0]
        self.mass = mass
        self.t = 0
        self.radius = radius
        self.solver = ode(self.f)
        self.solver.set_integrator('dop853')
        self.solver.set_initial_value(self.state, self.t)

    def f(self, t, y):
        return [y[2], y[3], 0, 0]

    def set_pos(self, pos):
        self.state[0:2] = pos
        self.solver.set_initial_value(self.state, self.t)
        return self

    def set_vel(self, vel):
        self.state[2:] = vel
        self.solver.set_initial_value(self.state, self.t)
        return self

    def update(self, dt):
        self.t += dt
        self.state = self.solver.integrate(self.t)

    def draw(self, surface):
        rect = self.image.get_rect()
        rect.center = (self.state[0]*SCALE, 640-(self.state[1]*SCALE)) # Flipping y
        surface.blit(self.image, rect)

class World:

    def __init__(self, dt, w, h):
        self.disks = []
        self.e = 1. # Coefficient of restitution
        self.dt = dt
        self.tol_distance = 0.00001
        self.win_width = w
        self.win_height = h

    def add(self, imgfile, radius, mass=1.0):
        disk = Disk(imgfile, radius, mass)
        self.disks.append(disk)
        return disk

    def draw(self, screen):
        for d in self.disks:
            d.draw(screen)

    def update(self):
        self.check_for_collisions()

        for d in self.disks:
            d.update(self.dt)
    
    def binary_search(self, d, i, b):
        dt_frac = self.dt
        dt = self.dt
        new_state = d.state
        if b == 0:
            while True:
                dt_frac *= 0.5 
                if (new_state[i] - d.radius) <= self.tol_distance and new_state[i] >= b:
                    break
                elif (new_state[i] - d.radius) > self.tol_distance:
                    dt += dt_frac
                else:
                    dt -= dt_frac
                d.solver.set_initial_value(d.state, d.t)
                new_state = d.solver.integrate(d.t + dt)
        else:
            while True:
                dt_frac *= 0.5
                if (new_state[i] + d.radius) >= (b - self.tol_distance) and new_state[i] <= b:
                    break
                elif (new_state[i] + d.radius) < (b - self.tol_distance):
                    dt += dt_frac
                else:
                    dt -= dt_frac
                d.solver.set_initial_value(d.state, d.t)
                new_state = d.solver.integrate(d.t + dt)
        d.t += dt
        new_state[i+2] *= -1
        d.set_pos(new_state[0:2])
        d.set_vel(new_state[2:])

    def check_for_collisions(self):
        for i in range(0, len(self.disks)):
            self.check_for_boundary_collisions(self.disks[i])
            for j in range(i+1, len(self.disks)):
                self.check_for_disk_collisions(self.disks[i], self.disks[j])
    
    def check_for_disk_collisions(self, disk1, disk2):
        PA = np.asarray(disk1.state[0:2])
        PB = np.asarray(disk2.state[0:2])
        rA = disk1.radius
        rB = disk2.radius
        if np.linalg.norm(PA - PB) <= (rA + rB):
            vA = np.asarray(disk1.state[2:4])
            vB = np.asarray(disk2.state[2:4])
            mA = disk1.mass
            mB = disk2.mass
            VAB = vA - vB
            n = (PA - PB) / np.linalg.norm(PA - PB)
            if np.dot(VAB, n) < 0:
                j = np.dot(np.dot(-(1+self.e), VAB),n) / ((1/mA)+(1/mB))
                vAF = vA + (np.dot(j, n) / mA)
                vBF = vB - (np.dot(j, n) / mB)
                disk1.set_vel(vAF[0:2])
                disk2.set_vel(vBF[0:2])
    
    def check_for_boundary_collisions(self, d):
        if (d.state[0] - d.radius) <= 0:
            self.binary_search(d, 0, 0)
        elif (d.state[0] + d.radius) >= (self.win_width/SCALE):
            self.binary_search(d, 0, (self.win_width/SCALE))
        elif (d.state[1] - d.radius) <= 0:
            self.binary_search(d, 1, 0)
        elif (d.state[1] + d.radius) >= (self.win_height/SCALE):
            self.binary_search(d, 1, (self.win_height/SCALE))
        
def is_pos_ok(cur_pos, cur_vel, pos, d_tol):
    b = True
    if not cur_pos in pos and (cur_vel[0] != 0 and cur_vel[1] != 0):
        for p in pos:
            if not abs(cur_pos[0] - p[0]) > d_tol or not abs(cur_pos[1] - p[1]) > d_tol:
                b = False
                break
    else:
        b = False
    return b

def main():

   # initializing pygame
    pygame.init()

    clock = pygame.time.Clock()

    # top left corner is (0,0)
    win_width = 640
    win_height = 640
    screen = pygame.display.set_mode((win_width, win_height))
    pygame.display.set_caption('Moving Disks')
    
    img_dir = './img/'
    files = os.listdir(img_dir)
    disk_imgfile = []
    for f in files:
        disk_imgfile.append(img_dir + f)
    n_disks = 10
    
    sel_pos = []
    world = World(0.0033, win_width, win_height)
    i = 0
    while i < n_disks:
        pos = [rand.uniform(0.5,((win_width/SCALE)-0.5)),rand.uniform(0.5,((win_height/SCALE)-0.5))]
        vel = [rand.uniform(-10,10),rand.uniform(-10,10)]
        r = rand.uniform(0.1, 0.2)
        if is_pos_ok(pos, vel, sel_pos, (r + 0.1)):
            world.add(rand.choice(disk_imgfile), r, rand.uniform(1,5)).set_pos(pos).set_vel(vel)
            sel_pos.append(pos)
            i += 1
    
    print '--------------------------------'
    print 'Usage:'
    print 'Press (q) to quit simulation'
    print '--------------------------------'
    
    while True:
        # 30 fps
        clock.tick(30)

        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit(0)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            pygame.quit()
            sys.exit(0)
        else:
            pass

        # Clear the background, and draw the sprites
        screen.fill((255,255,255))
        world.draw(screen)
        world.update()

        pygame.display.update()

if __name__ == '__main__':
    main()
