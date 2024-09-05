import serial.tools.list_ports
import time
import pygame
ports = serial.tools.list_ports.comports()
pygame.display.init()
# enter on terminal: python3 serialcomm_pid_final.py
ser = serial.Serial('/dev/ttyUSB0',9600, timeout=1.0)
time.sleep(0.003)
q = 1
screen = pygame.display.set_mode((800,600))
WHITE = (255,255,255)
screen.fill(WHITE)
pygame.display.set_caption("vehicle control")
while q == 1:
    for event in pygame.event.get():
    
        if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
            ser.write(b'wl:0.55, wr:0.55')
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
            ser.write(b'wl:-0.4, wr:-0.4')
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
            ser.write(b'wl:0.2, wr:0.5')
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
            ser.write(b'wl:0.5, wr:0.2')
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            ser.write(b'wl:0, wr:0')
        elif event.type == pygame.KEYUP:
            ser.write(b'wl:0, wr:0')
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            q == 0

    time.sleep(0.003)
ser.close()
