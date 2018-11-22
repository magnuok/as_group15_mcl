import random


#def low_var_sample(X_bar,W):
X_bar = [1, 0.7, 3, 4]
W = [0.1, 0.2, 0.1, 0.9]

X = []
r = random.random()*(1./(len(X_bar)))
c = W[0]
i = 0

for m in range(1,len(X_bar)+1):
    u = r + (m-1)*(1./(len(X_bar)))
    while u > c:
      i += 1
      c += W[i]
    X.append(X_bar[i])

print(X)