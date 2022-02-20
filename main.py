import numpy as np
import math
from boat_model import Boat
import matplotlib.pyplot as plt

boat = Boat(0, 0, 5, 0, 0, 0, 0, 1.024, 0.05068)
for i in range(0, 10000):
    Boat.step(10, 0.001)
print(Boat.pos)