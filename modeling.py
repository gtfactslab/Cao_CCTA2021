from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse

A = sparse.csc_matrix([
  [-1.,      0.,     0., 0., 0., 1],
  [1.,      -1.,     0., 0., 0., 0],
  [0.,      1.,     -1., 0., 0., 0],
  [0.,       0.,    1., -1., 0., 0],
  [0.,      0,      0., 1., -1., 0],
  [0.,      0,      0., 0., 0., -1]])

B = sparse.csc_matrix([
  [100.],
  [0.],
  [0.],
  [0.],
  [0.],
  [150.]])
[r, c] = A.shape

# reference values for
# supply
w_list = np.array([-100, -100, -100, -100, -100])
x_jam_list = np.array([600, 600, 600, 600, 600])
# demand
v_list = np.array([100, 100, 100, 100, 0])

# modeling horizon
N = 10

# initial state
x0 = np.zeros(c)

# Define problem
f = Variable((c, N)) # f represents the flow rate exiting each cell/onramp
x = Variable((c, N+1)) # x represents the density of each cell/onramp

x_init = Parameter(c)

objective = 0
constraints = [x[:,0] == x_init]
for k in range(N):
    # objective: minimize the sum of all cell/onramp densities over time
    objective += sum(x[:,k])
    # calculating density at each time step using values from previous timestep
    constraints += [x[:,k+1] == np.eye(c) *x[:, k] + A * f[:, k]]
    # constraints for demand & supply
    constraints += [f[:-1,k] <= np.multiply(x[:,k], v_list[:, np.newaxis]), f[:-2,k] <= np.multiply(x[1:-1,k], w_list[1:, np.newaxis]) + x_jam_list[1:].transpose()]
prob = Problem(Minimize(objective), constraints)

# Simulate
prob.solve()
print(prob.value)