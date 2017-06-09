import numpy as np
from time import time

def test(m,n):
    start = time()
    a=np.zeros((m,n))
    b=np.zeros(m*n)
    k = 0
    for i in range(m):
        for j in range(n):
            if a[i,j]>=0:
             b[k]=a[i,j]
             k+=1
    b = b[:k]
    end = time()
    return (end - start)

                
        

if __name__ == '__main__':
    m=1000
    n=1000
    start = time()
    a=np.random.rand(m,n)
    b=a.ravel()
    b = b[b >=0.5]
    end = time()
    print "python can do test in %f seconds" % (end - start)