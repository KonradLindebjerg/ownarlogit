import random
import numpy as np
import matplotlib.pyplot as plt


# Proposal distribution q(x) = N(5, 4): mean 5, variance 4 (std = 2)
q_mu = 5.0
q_sigma = 2.0

# Sample i.i.d samples M = z1...z(m) from the proposal N(5,4)
def generateSamples():
    M = []
    i = 0
    while i < 200:
        sample = random.gauss(q_mu, q_sigma)
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
    Mw.append(calcpxi(m) / normalDistribution(m, q_mu, q_sigma))
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

plt.hist(H20, bins = 10, density=True)
plt.title("histogram k = 20")
xs = np.linspace(0, 15, 500)
plt.plot(xs, [calcpxi(x) for x in xs])
plt.show()
plt.hist(H100, bins = 10, density=True)
xs = np.linspace(0, 15, 500)
plt.plot(xs, [calcpxi(x) for x in xs])
plt.title("histogram k = 100")
plt.show()
plt.hist(H1000, bins = 10, density=True)
xs = np.linspace(0, 15, 500)
plt.plot(xs, [calcpxi(x) for x in xs])
plt.title("histogram k = 1000")
plt.show()



## p(x) = 0.3 · N (x; 2.0, 1.0) + 0.4 · N (x; 5.0, 2.0) + 0.3 · N (x; 9.0, 1.0)

## Compute Cumsum



## Lookup in H [0,h(x^1)],..h(x^{N-1}) and find the smalled index i such that z >= Hi.
## The corresponding particle xi is our sample
# Important: Remove any particles with weight Wi = 0 prior to constructing table H



