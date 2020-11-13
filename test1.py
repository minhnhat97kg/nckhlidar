import numpy as np
import random
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter
kf = KalmanFilter()

kf2 = KalmanFilter()
current = []
after = []
predict = []

current2 = []
after2 = []
predict2 = []

start = [0,0]
current.append(start)

for i in range(100):
    r1 = random.randint(-100,100)
    r2 = random.randint(-100,100)

    _current = [i,current[-1][1]+r2]
    current.append(_current)

    _predict = kf.predict()
    predict.append([i,_predict[0][0]])

    _after = kf.correct(_current[1],1)[0]
    after.append([i,_after[0]])


    _after2 = kf2.correct(_current[1],1)[0]
    _after2 = kf2.correct(_after2[0],1)[0]
    after2.append([i,_after2[0]])


current=np.array(current)
after=np.array(after)
after2=np.array(after2)
predict=np.array(predict)

plt.figure(figsize=(11,6))
plt.plot(current[:,0],current[:,1], linewidth=2, linestyle="-", c="b")  # it include some noise
plt.plot(after[:,0],after[:,1], linewidth=2, linestyle="-", c="r")  # it include some noise
plt.plot(after2[:,0],after2[:,1], linewidth=2, linestyle="-", c="yellow")  # it include some noise
#plt.plot(predict[:,0],predict[:,1], linewidth=2, linestyle="-", c="black")  # it include some noise
plt.show()


