import random
import numpy as np
import matplotlib.pyplot as plt


# Sample i.i.d samples M = z1...z(m)
def generateSamples():
    M = []
    i = 0
    while i < 200:
        sample = random.uniform(0,15)
        M.append(sample)
        i += 1
    return M
    
M = generateSamples()
## Compute weights

### Step2

def normalDistribution(x,mu,sigma):
    firstpart = 1 / (np.sqrt(2 * np.pi) * sigma)
    secondpart = np.exp(
        (-0.5 * (x - mu)**2) / (sigma**2)
    )
    return firstpart * secondpart


def calcpxi(x):
    return 0.3 * normalDistribution(x, 2.0,1.0) + 0.4 * normalDistribution(x, 5.0, 2.0) + 0.3 * normalDistribution(x, 9.0, 1.0)


Mw = []
for m in M:
    Mw.append(calcpxi(m) * 15)
Mww = []
for i in range(len(Mw)):
    Mww.append(Mw[i] / np.sum(Mw))

print(np.sum(Mww))
## Resampling: sample with replacement from M based on weights W(m)
## Reduce to N <= M samples





## Generate a sample computed by P(x^i) = w(i), i in {1,...,N}

## Use random number z in [0,1] from a uniform distribution use np.randon.rand()
hi = []
hi = np.cumsum(Mww)



H20 = []
for k in range(20):
    z = np.random.rand()
    for i in range(len(hi)):
        if z <= hi[i]:
            H20.append(M[i])
            print("\n")
            break

H100 = []
for k in range(100):
    z = np.random.rand()
    for i in range(len(hi)):
        if z <= hi[i]:
            H100.append(M[i])
            print("\n")
            break

H1000 = []
for k in range(1000):
    z = np.random.rand()
    for i in range(len(hi)):
        if z <= hi[i]:
            H1000.append(M[i])
            print("\n")
            break

hist20 = np.histogram(H20)
hist100 = np.histogram(H100)
hist1000 = np.histogram(H1000)
plt.hist(hist20)
plt.show()
plt.hist(hist100)
plt.show()
plt.hist(hist1000)
plt.show()



## p(x) = 0.3 · N (x; 2.0, 1.0) + 0.4 · N (x; 5.0, 2.0) + 0.3 · N (x; 9.0, 1.0)

## Compute Cumsum



## Lookup in H [0,h(x^1)],..h(x^{N-1}) and find the smalled index i such that z >= Hi.
## The corresponding particle xi is our sample
# Important: Remove any particles with weight Wi = 0 prior to constructing table H



