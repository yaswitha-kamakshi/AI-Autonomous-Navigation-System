import sys
import numpy as np
import cv2
import pygame
import matplotlib
import pandas as pd
import imageio

print("Python     :", sys.version.split()[0])
print("NumPy      :", np.__version__)
print("OpenCV     :", cv2.__version__)
print("Pygame     :", pygame.version.ver)
print("Matplotlib :", matplotlib.__version__)
print("Pandas     :", pd.__version__)
print("ImageIO    :", imageio.__version__)

pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Environment Check")
screen.fill((30, 144, 255))
pygame.display.flip()
pygame.time.wait(1500)
pygame.quit()

print("\nAll good. Environment is ready.")