#!/usr/bin/env python3
import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt
import subprocess
files = ['press','noise', 'oscillate']
Fs = 200                        # sampling frequency in Hz
f_0 = 60                        # target frequency in Hz
N = 20                          # "bins" in the DFT
k = f_0*N/Fs                 # the bin number to extract
assert round(k) == k            # k has to be an integer

# python implementation of magnitude goertzel
def goertzel(x,k):
    N = len(x)
    w = 2*np.pi/N*k
    coeff = 2*np.cos(w)
    Q1,Q2 = (0.0,0.0)
    for i in range(N):
        Q0 = coeff*Q1-Q2+x[i]
        Q2=Q1
        Q1=Q0
    return Q1**2 + Q2**2 -Q1*Q2*coeff


# call the rust version of goertzel
def rs_humbutton(fn):
    with open(fn, "rb") as f:
        cmd = "cargo run --example humbutton"
        output=subprocess.Popen(cmd, shell=True ,stdin=f,text=True,
                                stdout=subprocess.PIPE)   
        out, error = output.communicate()
        return np.fromstring(out,dtype=float,sep="\n")


# call the rust version of goertzel
def rs_goertzel(fn,n=20,k=6):
    with open(fn, "rb") as f:
        cmd = "cargo run --example goertzel {} {}".format( n,int(k))
        output=subprocess.Popen(cmd, shell=True ,stdin=f,text=True,
                                stdout=subprocess.PIPE)   
        out, error = output.communicate()
        return np.fromstring(out,dtype=float,sep="\n")

# SM to filter for button presses
def filter_out(X):
    y = np.zeros(len(X))

    X_max = X[0]
    X_min = X[0]

    for i in range(len(X)):
        if X[i] > X_max:
            X_max = X[i]
        if X[i] < X_min:
            X_min = X[i]
        threshold = (X_max+X_min)/2

        if y[i-1] == 0:
            if X[i] > threshold:
                y[i]=1
            else:
                y[i] = 0
        else:
            if X[i] < threshold:
                y[i] = 0
            else:
                y[i] = 1
    return y



# test suite runner
for fn in files:

    x = np.genfromtxt(fn+'.csv',dtype=float)
    L = len(x)
    L = L - L%N
    x = x[:L]

#    x = x / (2**12-1)
    
    assert int(L/N)==L/N


    X = np.zeros(int(L/N))
    for i,j in enumerate(range(0,L,N)):
        X[i] = goertzel(x[j:j+N],k)

    y = filter_out(X)
    r_X = rs_goertzel(fn+'.csv', N,k)
    r_y = rs_humbutton(fn+'.csv')
    fig,ax = plt.subplots(3,1)
    ax[0].plot(x)
    ax[1].plot(r_X)
    ax[2].plot(r_y)
    fig.savefig('{}.png'.format(fn))
