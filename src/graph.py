#---------------------------------------------------------------------------------------------------------------------
#  
#---------------------------------------------------------------------------------------------------------------------
#  Copyright 2020 Luis Marzo Román a.k.a Luichi,  luismarzor@gmail.com
#---------------------------------------------------------------------------------------------------------------------
#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#  and associated documentation files (the "Software"), to deal in the Software without restriction,
#  including without limitation the rights to use, copy, modify, merge, publish, distribute,
#  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all copies or substantial
#  portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
#  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
#  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
#  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#---------------------------------------------------------------------------------------------------------------------

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


