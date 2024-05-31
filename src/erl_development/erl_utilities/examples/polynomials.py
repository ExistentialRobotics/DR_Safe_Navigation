import numpy as np
from pyUtils import Polynomial
from pyUtils import PiecewisePolynomial
import matplotlib.pyplot as plt

# Create Simple Polynomials
t = Polynomial("t", 1)
x1 = t ** 2
x2 = -1 * t ** 3 + t ** 2

y1 = 3 * t ** 3 - 2 * t ** 2 - t / 2 - 2
y2 = -2 * y1

# Create Piecewise Polynomial
breaks = [0, 1, 1.5]
x_t = PiecewisePolynomial([x1, x2], breaks)
y_t = PiecewisePolynomial([y1, y2], breaks)

print("Trajectory Start: ", x_t.start_time())
print("Trajectory Initial Condition: ", x_t.value(x_t.start_time()))
print("Trajectory End: ", x_t.end_time())
print("Trajectory Terminal Condition: ", x_t.value(x_t.end_time()))

# Plot Piecewise Polynomial
tspan = np.linspace(x_t.start_time(), x_t.end_time(), 100)
xspan = []
for t in tspan:
    xspan.append(x_t.scalarValue(t, 0, 0))

# Plot Velocity
v_t = x_t.derivative(1)
vspan = []
for t in tspan:
    vspan.append(v_t.value(t))


fig, axes = plt.subplots(2, 1)
axes[0].plot(tspan, xspan)
axes[0].set_title('Position')

axes[1].scatter(tspan, vspan)
axes[1].set_title('Velocity')


plt.show()
