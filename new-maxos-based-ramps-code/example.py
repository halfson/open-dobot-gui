from Dobot import Dobot
import time
import math
# import numpy as np
# import matplotlib.pyplot as plt

dobot = Dobot('COM9', 115200)
dobot.Open()
time.sleep(1)
successes = 0
i = 0
while True:
	ret = dobot.isReady()
	if ret[0] and ret[1]:
		successes += 1
	if successes > 10:
		print ("Dobot ready!")
		break
	if i > 100:
		raise Exception('Comm problem')




for k in range(0,3200):
	dobot.Steps(30, 30, 30,30, 0, 0, 0, False)
print ('Adding fast section')

