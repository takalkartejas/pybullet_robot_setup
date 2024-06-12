import pybullet as p
import math

# Example quaternion
quaternion = [-0.14744917191414036, -0.14744918872535126, -0.6915624951477904, 0.69156257399532]  # Example quaternion (w, x, y, z)

# Convert quaternion to Euler angles
ex,ey,ez= p.getEulerFromQuaternion(quaternion)

if abs(ex) < 0.01:
    ex = 0
if abs(ey) < 0.01:
    ey = 0
if abs(ez) < 0.01:
    ez = 0

euler = [ex,ey,ez]
# Print the Euler angles
print(euler)

