from sympy import *
from time import time
from mpmath import radians
import tf

## Insert IK code here!

### Your FK code here
# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')   #theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Create Modified DH parameters
dh = {alpha0:      0,    a0:      0,   d1:  0.75,       
      alpha1: -pi/2.,    a1:   0.35,   d2:     0,   q2:q2-pi/2,
      alpha2:      0,    a2:   1.25,   d3:     0,
      alpha3: -pi/2.,    a3: -0.054,   d4:   1.5,
      alpha4:  pi/2.,    a4:      0,   d5:     0,
      alpha5: -pi/2.,    a5:      0,   d6:     0,
      alpha6:      0,    a6:      0,   d7: 0.303,   q7:0}
#
# Define Modified DH Transformation matrix
def TF_matrix(alpha, a, d, q):
    TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [                0,                 0,           0,             1]])
    return TF


#
# Create individual transformation matrices
T0_1 = TF_matrix(alpha0, a0, d1, q1).subs(dh)
T1_2 = TF_matrix(alpha1, a1, d2, q2).subs(dh)
T2_3 = TF_matrix(alpha2, a2, d3, q3).subs(dh)
T3_4 = TF_matrix(alpha3, a3, d4, q4).subs(dh)
T4_5 = TF_matrix(alpha4, a4, d5, q5).subs(dh)
T5_6 = TF_matrix(alpha5, a5, d6, q6).subs(dh)
T6_G = TF_matrix(alpha6, a6, d7, q7).subs(dh)

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

print(T0_G.evalf(subs={q1:-0.65, q2:0.45, q3:-0.36, q4:0.95, q5:0.79, q6:0.49}))

