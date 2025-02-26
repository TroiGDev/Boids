import pygame
import random
import math
import numpy as np

pygame.init()
screenWidth = 1000
screenHeight = 500
screen = pygame.display.set_mode((screenWidth, screenHeight))
pygame.display.set_caption('Boids')

def rotatePointAroundCenter(c, p, angle):
    angleRad = math.radians(angle)
    #translate center (get vector from center to point)
    t = (p[0] - c[0], p[1] - c[1])
    #do some math magic
    rotatedP = (t[0] * math.cos(angleRad) - t[1] * math.sin(angleRad), t[0] * math.sin(angleRad) + t[1] * math.cos(angleRad))
    #move center back
    finalP = (rotatedP[0] + c[0] , rotatedP[1] + c[1])
    
    return finalP

def GetClockwiseAngle(frm, to):
    #get signed angle, get absolute angle
    signedAngle = np.arctan2(frm[1], to[0]) - np.arctan2(frm[1], to[0])
    unsignedAngle = abs(signedAngle)

    #normalize unsigned between 0 and 360
    if unsignedAngle > 180:
        unsignedAngle = 360 - unsignedAngle

    #get cross product
    crossP = frm[0] * to[1] - frm[1] * to[0]

    #update to clockwise if the angle was counter clockwise
    if crossP < 0:
        unsignedAngle = 360 - unsignedAngle

    return unsignedAngle

class Boid:
    def __init__(self, x, y, angleDeg):
        boids.append(self)
        self.x = x
        self.y = y

        self.angleDeg = angleDeg
        self.speed = 60 + random.uniform(-1, 1) * 20
        self.rotationSpeed = 180 + random.uniform(-1, 1) * 60
        self.viewDistance = 250 + random.uniform(-1, 1) * 50
        self.size = 0.5 + random.uniform(0, 1)

    def draw(self):
        drawOffsets = [(self.x, self.y - 10 * self.size), (self.x - 4 * self.size, self.y + 2 * self.size) , (self.x + 4 * self.size, self.y + 2 * self.size)]
        rotatedVerts = [
            rotatePointAroundCenter((self.x, self.y), drawOffsets[0], self.angleDeg),
            rotatePointAroundCenter((self.x, self.y), drawOffsets[1], self.angleDeg),
            rotatePointAroundCenter((self.x, self.y), drawOffsets[2], self.angleDeg)
        ]

        pygame.draw.polygon(screen, (255, 255, 255), rotatedVerts, 1)

    def move(self, deltaTime):
        #get direction vector from angle
        angleRad = math.radians(self.angleDeg - 90)
        dirX = math.cos(angleRad)
        dirY = math.sin(angleRad)
        self.x += dirX * self.speed * deltaTime
        self.y += dirY * self.speed * deltaTime

        #loop on edges
        self.x = self.x % screenWidth
        self.y = self.y % screenHeight

    def steer(self, seperationWeight, alignmentWeight, cohesionWeight, fearWeight, deltaTime):
        #normalize weights
        mag = seperationWeight + alignmentWeight + cohesionWeight + fearWeight
        sepW = seperationWeight / mag
        aliW = alignmentWeight / mag
        cohW = cohesionWeight / mag
        feaW = fearWeight / mag

        finalSteer = 0

        #get boids in range
        localBoids = []
        for boid in boids:
            dist = math.sqrt((boid.x - self.x) ** 2 + (boid.y - self.y) ** 2)
            if dist < self.viewDistance:
                localBoids. append(boid)

        #separation----------------------------------------------------------------------------------------------------------------
        #get avarage vector to local boids
        s_v = (0, 0)
        for boid in localBoids:
            s_v = (s_v[0] + (boid.x - self.x), s_v[1] + (boid.y - self.y))
        #steer towards opposite
        s_vo = (s_v[0] * -1, s_v[1] * -1)

        #get clockwise angle towards opposite
        angleRad = math.radians(self.angleDeg - 90)
        selfDir = (math.cos(angleRad), math.sin(angleRad))
        s_clockwiseAngle = GetClockwiseAngle(selfDir, s_vo)

        if s_clockwiseAngle > 180:
            #turn left
            finalSteer += sepW * self.rotationSpeed * -1
        else:
            #turn right
            finalSteer += sepW * self.rotationSpeed
        #alignments------------------------------------------------------------------------------------------------------------------
        a_avarageAngle = 0
        for boid in localBoids:
            a_avarageAngle += boid.angleDeg
        
        #get shortest turn towards avarage direction of heading
        # Calculate raw difference
        a_diff = (a_avarageAngle - self.angleDeg) % 360
        #if diff more than 180, flip
        if a_diff > 180:
            a_diff = -(360 - a_diff)
        
        #return "left" if diff > 0 else "right", abs(diff)
        if a_diff > 0:
            #turn left
            finalSteer += aliW * self.rotationSpeed
        else:
            #turn right
            finalSteer += aliW * self.rotationSpeed * -1
        #cohesion---------------------------------------------------------------------------------------------------
        #get avarage local boid pos
        c_avaragePos = (0, 0)
        for boid in localBoids:
            c_avaragePos = (c_avaragePos[0] + boid.x, c_avaragePos[1] + boid.y)
        c_avaragePos = (c_avaragePos[0] / len(localBoids), c_avaragePos[1] / len(localBoids))

        #get vector to avarage center and rotate towards it
        c_v = (c_avaragePos[0] - self.x, c_avaragePos[1] - self.y)
        c_clockwiseAngle = GetClockwiseAngle(selfDir, c_v)

        if c_clockwiseAngle > 180:
            #turn left
            finalSteer += cohW * self.rotationSpeed * -1
        else:
            #turn right
            finalSteer += cohW * self.rotationSpeed
        #fear------------------------------------------------------------------------------------------------------
        #get closest predator
        closestPred = predators[0]
        closestDist = math.sqrt((predators[0].x - self.x) ** 2 + (predators[0].y - self.y) ** 2)
        for pred in predators:
            #get distance towards pred
            dist = math.sqrt((pred.x - self.x) ** 2 + (pred.y - self.y) ** 2)
            if dist < closestDist:
                closestDist = dist
                closestPred = pred
        
        #steer away from closest pred if it is in range
        if closestDist < self.viewDistance:
            f_v = (closestPred.x - self.x, closestPred.y - self.y)
            f_clockwiseAngle = GetClockwiseAngle(selfDir, f_v)

            if f_clockwiseAngle > 180:
                #turn left
                finalSteer += feaW * self.rotationSpeed
            else:
                #turn right
                finalSteer += feaW * self.rotationSpeed * -1
        #----------------------------------------------------------------------------------------------------------

        #finaly steer
        self.angleDeg += finalSteer * deltaTime

class Predator:
    def __init__(self, x, y, angleDeg):
        predators.append(self)
        self.x = x
        self.y = y

        self.angleDeg = angleDeg
        self.speed = 80
        self.rotationSpeed = 120
        self.viewDistance = 200

    def draw(self):
        drawOffsets = [(self.x, self.y-20), (self.x-8, self.y+4) , (self.x+8, self.y+4)]
        rotatedVerts = [
            rotatePointAroundCenter((self.x, self.y), drawOffsets[0], self.angleDeg),
            rotatePointAroundCenter((self.x, self.y), drawOffsets[1], self.angleDeg),
            rotatePointAroundCenter((self.x, self.y), drawOffsets[2], self.angleDeg)
        ]

        pygame.draw.polygon(screen, (255, 0, 0), rotatedVerts, 2)

    def move(self, deltaTime):
        #get direction vector from angle
        angleRad = math.radians(self.angleDeg - 90)
        dirX = math.cos(angleRad)
        dirY = math.sin(angleRad)
        self.x += dirX * self.speed * deltaTime
        self.y += dirY * self.speed * deltaTime

        #loop on edges
        self.x = self.x % screenWidth
        self.y = self.y % screenHeight

    def steer(self, deltaTime):
        #steer towards local center of mass of boids, cohesion

        #get boids in range
        localBoids = []
        for boid in boids:
            dist = math.sqrt((boid.x - self.x) ** 2 + (boid.y - self.y) ** 2)
            if dist < self.viewDistance:
                localBoids. append(boid)

        finalSteer = 0

        #get avarage center of mass
        c_avaragePos = (0, 0)
        for boid in localBoids:
            c_avaragePos = (c_avaragePos[0] + boid.x, c_avaragePos[1] + boid.y)
        if len(localBoids) != 0:
            c_avaragePos = (c_avaragePos[0] / len(localBoids), c_avaragePos[1] / len(localBoids))

        #get vector to avarage center and rotate towards it
        c_v = (c_avaragePos[0] - self.x, c_avaragePos[1] - self.y)
        angleRad = math.radians(self.angleDeg - 90)
        selfDir = (math.cos(angleRad), math.sin(angleRad))
        c_clockwiseAngle = GetClockwiseAngle(selfDir, c_v)

        if c_clockwiseAngle > 180:
            #turn left
            finalSteer += self.rotationSpeed * -1
        else:
            #turn right
            finalSteer += self.rotationSpeed

        #apply final steer, only if has target, otherwise continue moving forward instead of rotating in a circle
        if c_avaragePos != (0, 0):
            self.angleDeg += finalSteer * deltaTime


seperationWeight = 0.2
alignmentWeight = 0.4
cohesionWeight = 0.8
fearWeight = 1.2

#generate boids
boids = []
boidNum = 10
for i in range(boidNum):
    boid = Boid(random.uniform(0, 1) * screenWidth, random.uniform(0, 1) * screenHeight, random.uniform(0, 1) * 360)

#generate predaors
predators = []
predatorNum = 1
for i in range(predatorNum):
    pred = Predator(random.uniform(0, 1) * screenWidth, random.uniform(0, 1) * screenHeight, random.uniform(0, 1) * 360)

#get delta time
prevT = pygame.time.get_ticks()

running = True
while running:

    #update delta time
    currT = pygame.time.get_ticks()
    dTms = currT - prevT
    dTs = dTms / 1000.0

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Fill screen
    screen.fill((0, 0, 0))

    #update boids
    for boid in boids:
        boid.steer(seperationWeight, alignmentWeight, cohesionWeight, fearWeight, dTs)
        boid.move(dTs)
        boid.draw()

    #update predators
    for pred in predators:
        pred.steer(dTs)
        pred.move(dTs)
        pred.draw()


    #draw range around first boid
    """pygame.draw.circle(screen, (255, 255, 255), (boids[0].x, boids[0].y), boids[0].viewDistance, 1)
    #draw range around first pred
    pygame.draw.circle(screen, (255, 0, 0), (predators[0].x, predators[0].y), predators[0].viewDistance, 1)"""

    # Update the display
    pygame.display.flip()

    #update delta time
    prevT = currT

# Quit Pygame
pygame.quit()
