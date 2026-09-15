# OpenManipulatorX_kinematics

## 1. Understand the OpenMANIPULATOR-X structure

The current ROBOTIS URDF describes the arm as:

$$
q =
\begin{bmatrix}
q_1&q_2&q_3&q_4
\end{bmatrix}^T
$$

with:

$$
q_1:\text{ rotation about Z}
$$

and

$$
q_2,q_3,q_4:\text{ rotation about Y}.
$$

The current official Xacro uses these transforms: 

$$
p_1 =
\begin{bmatrix}
0.012\\0\\0
\end{bmatrix}
$$

$$
p_2 =
\begin{bmatrix}
0\\0\\0.0595
\end{bmatrix}
$$

$$
p_3 =
\begin{bmatrix}
0.024\\0\\0.128
\end{bmatrix}
$$

$$
p_4 =
\begin{bmatrix}
0.124\\0\\0
\end{bmatrix}
$$

and the tool offset is

$$
p_e =
\begin{bmatrix}
0.126\\0\\0
\end{bmatrix}.
$$

Therefore the complete FK is

$$
\boxed{
{}^0T_E =
T(p_1)
R_z(q_1)
T(p_2)
R_y(q_2)
T(p_3)
R_y(q_3)
T(p_4)
R_y(q_4)
T(p_e)
}
$$

This is better than inventing a DH table because it directly follows the official URDF convention.

---

# 2. Important property of this robot

Because joints 2, 3, and 4 all rotate around Y,

$$
R_E =
R_z(q_1)
R_y(q_2+q_3+q_4).
$$

Define

$$
\phi = q_2+q_3+q_4.
$$

Then the end-effector orientation is basically controlled by:

$$
q_1 = \text{yaw}
$$

and

$$
\phi = \text{pitch}.
$$

This also explains why OpenMANIPULATOR-X cannot independently control arbitrary

$$
[x,y,z,roll,pitch,yaw].
$$

It only has four arm DOF.

For our IK function, a very natural command is therefore:

```cpp
IK(x, y, z, pitch)
```

instead of attempting arbitrary 6-DOF IK.

---

# 3. Class structure

I would organize it as:

```text
open_manipulator_x_kinematics/
├── include/
│   └── open_manipulator_x_kinematics.hpp
├── src/
│   ├── open_manipulator_x_kinematics.cpp
│   └── test_kinematics.cpp
└── CMakeLists.txt
```

For

$$
q=[0,0,0,0]
$$

we can calculate it by hand.

The X position is

$$
x =
0.012
+
0.024
+
0.124
+
0.126
$$

so

$$
\boxed{x=0.286\,m}
$$

and

$$
z =
0.0595+0.128
$$

so

$$
\boxed{z=0.1875\,m}.
$$

Therefore:

```text
x = 0.286
y = 0
z = 0.1875
```

This is a very useful first unit test.

---

# 4. Now inverse kinematics

IK is more interesting.

Suppose we want

$$
P_d =
\begin{bmatrix}
x_d\\y_d\\z_d
\end{bmatrix}
$$

and end-effector pitch

$$
\phi_d.
$$

Remember:

$$
\phi_d=q_2+q_3+q_4.
$$

---

# 5. Solve joint 1

First remove the fixed 12 mm base offset:

$$
x' = x_d-0.012
$$

$$
y'=y_d.
$$

Because joint 1 rotates around Z,

$$
\boxed{
q_1=\operatorname{atan2}(y',x')
}
$$

and the radial distance from joint 1 is

$$
r=\sqrt{x'^2+y'^2}.
$$

After solving \(q_1\), the remaining problem becomes a **2D planar problem**.

This is the nice property of OpenMANIPULATOR-X.

---

# 6. Remove the end-effector/tool length

The tool has length

$$
L_t=0.126.
$$

Since the desired end-effector pitch is \(\phi\), its contribution is

$$
x_t=L_t\cos\phi
$$

and because of the Y-axis rotation convention,

$$
z_t=-L_t\sin\phi.
$$

Therefore the wrist position becomes

$$
r_w =
r-L_t\cos\phi
$$

and

$$
z_w =
(z_d-0.0595)
+
L_t\sin\phi.
$$

This gives us the location of joint 4.

---

# 7. Geometry of joints 2 and 3

There is an interesting detail here.

The first planar link isn't simply 0.128 m.

Its vector is

$$
\begin{bmatrix}
0.024\\
0.128
\end{bmatrix}.
$$

Therefore its effective length is

$$
L_1=
\sqrt{0.024^2+0.128^2}.
$$

Numerically,

$$
\boxed{
L_1\approx0.13023\,m
}
$$

and its fixed geometric angle is

$$
\beta =
\operatorname{atan2}(0.128,0.024)
$$

so

$$
\boxed{
\beta\approx1.38545\ rad
}
$$

or approximately

$$
79.38^\circ.
$$

The next link is

$$
L_2=0.124.
$$

So now we have essentially a standard 2-link planar IK problem.

---

# 8. Law of cosines

Compute

$$
D=
\frac{
r_w^2+z_w^2-L_1^2-L_2^2
}{
2L_1L_2
}.
$$

If

$$
|D|>1
$$

then the target is outside the workspace.

Otherwise

$$
\delta
=
\operatorname{atan2}
\left(
\pm\sqrt{1-D^2},
D
\right).
$$

The \(+\) and \(-\) give the two possible configurations:

```text
elbow up
elbow down
```

Then

$$
\theta_1 =
\operatorname{atan2}(z_w,r_w)
-
\operatorname{atan2}
\left(
L_2\sin\delta,
L_1+L_2\cos\delta
\right).
$$

Because of the OpenMANIPULATOR joint convention,

$$
\boxed{
q_2=\beta-\theta_1
}
$$

and

$$
\boxed{
q_3=-\delta-\beta
}
$$

Finally,

$$
\boxed{
q_4=\phi_d-q_2-q_3
}
$$

because

$$
\phi_d=q_2+q_3+q_4.
$$


---

# 9. Select the best IK solution

There may be multiple IK solutions.

For a real robot, don't randomly choose one.

Suppose the robot currently has

$$
q_{current}.
$$

We should choose

$$
q^* =
\arg\min_q
\|q-q_{current}\|.
$$

This prevents the arm from suddenly changing to another elbow configuration.


Then:

```bash
mkdir build
cd build

cmake ..
make

./test_kinematics
```

---

# 10. The most important idea to understand

Don't think of FK as one huge mysterious equation.

Think:

```text
Base
 |
 | fixed translation
 v
Joint 1
 |
 | Rz(q1)
 v
Joint 2 location
 |
 | translation
 | Ry(q2)
 v
Joint 3
 |
 | translation
 | Ry(q3)
 v
Joint 4
 |
 | translation
 | Ry(q4)
 v
Tool
 |
 | translation
 v
End Effector
```

Mathematically:

$$
T_{EE}
=
T_1
R_1
T_2
R_2
T_3
R_3
T_4
R_4
T_E.
$$

That is essentially what ROS `robot_state_publisher`, MoveIt, KDL, Pinocchio, and other robotics libraries are doing internally.

---

# 11. FK and IK relationship

You should always validate IK using FK:

$$
q_{IK}
=
IK(P_d)
$$

then calculate

$$
P_{check}=FK(q_{IK}).
$$

Finally check

$$
e_p =
P_d-P_{check}.
$$

The error should be approximately zero.

Your workflow should therefore be:

```text
Joint angles
     q
     |
     v
+----------+
|    FK    |
+----------+
     |
     v
[x y z pitch]
     |
     v
+----------+
|    IK    |
+----------+
     |
     v
    q_ik
     |
     v
+----------+
|    FK    |
+----------+
     |
     v
Compare with original pose
```
