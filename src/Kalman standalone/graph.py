import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from ast import literal_eval


plt.style.use('fivethirtyeight')

# TODO
# Eliminate these ugly if conditions


cnt=0
with open('Vectors.txt') as inp:
    for line in inp:
        vec = literal_eval(line)
        if cnt == 0:
            xt = vec
        elif cnt == 1:
            yt = vec
        elif cnt == 2:
            xgps = vec
        elif cnt == 3:
            ygps = vec
        elif cnt == 4:
            x_filter = vec
        elif cnt == 5:
            y_filter = vec
        cnt = cnt + 1


plt.plot(xt, yt, xgps, ygps, 'bo', x_filter, y_filter, 'yo')



plt.tight_layout()
plt.show()


