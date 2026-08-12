from dqrobotics import *

# Camera 2 to marker 0:
x2 = 2.09640572699
y2 = 0.0224121394825
z2 = 0.445883576398

wx2 = -0.0480832715622
wy2 = -0.822635562395
wz2 = -0.0739944212306
w2 = 0.561679228836

p20 = DQ([0, x2, y2, z2])
r20 = DQ([w2, wx2, wy2, wz2])
x20 = r20 + DQ.E * 0.5 * p20 * r20

# Camera 3 to marker 0:
x3 = 2.33111687325
y3 = 0.293148241786
z3 = -0.633457472327

wx3 = 0.0135030843062
wy3 = -0.291468068668
wz3 = 0.0209307822412
w3 = 0.956256207309

p30 = DQ([0, x3, y3, z3])
r30 = DQ([w3, wx3, wy3, wz3])
x30 = r30 + DQ.E * 0.5 * p30 * r30

x23 = x20 * x30.conj()
print(x23)
print(norm(x23))
print("Translation: ",x23.translation())
print("Rotation: ", x23.P())

