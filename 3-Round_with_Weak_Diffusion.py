import numpy
import gurobipy
import turtle
import ModelingFuntions as MFs
import VisualizeFuntions as VFs
from gurobipy import *
from gurobipy import GRB

X = 4
Y = 3
Z = 32

ini_DoF_B = 12

# Model
m = gurobipy.Model("m-model")

# Create variables
# Objection
DoF_R = m.addVar(lb=0, ub=128, vtype=GRB.INTEGER, name="DoF_R")
DoF_B = m.addVar(lb=0, ub=128, vtype=GRB.INTEGER, name="DoF_B")
DoM = m.addVar(lb=0, ub=128, vtype=GRB.INTEGER, name="DoM")
Vector_obj = m.addVar(vtype=GRB.INTEGER, name="Vector_obj")

# Middle States
R0_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "R0_State")
C0_State = MFs.creat_color_variable_of_middle_state(X, Z, m, "C0_State")
D0_State = MFs.creat_color_variable_of_middle_state(X, Z, m, "D0_State")
Theta0_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Theta0_State")
Rho0_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Rho0_State")
Kai0_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Kai0_State")
R1_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "R1_State")
C1_State = MFs.creat_color_variable_of_middle_state(X, Z, m, "C1_State")
D1_State = MFs.creat_color_variable_of_middle_state(X, Z, m, "D1_State")
Theta1_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Theta1_State")
Rho1_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Rho1_State")
Kai1_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Kai1_State")
R2_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "R2_State")
C2_State = MFs.creat_color_variable_of_middle_state(X, Z, m, "C2_State")
D2_State = MFs.creat_color_variable_of_middle_state(X, Z, m, "D2_State")
Theta2_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Theta2_State")
Rho2_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Rho2_State")
Kai2_State = MFs.creat_color_variable_of_state(X, Y, Z, m, "Kai2_State")

# Initial DoF
Lambda_B = m.addVar(vtype=GRB.INTEGER, name=f'Lambda_B')
m.addConstr(Lambda_B == ini_DoF_B, f'Ini_Lambda_B')
Lambda_R = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "Lambda_R")

# Consumed DoF of C0 State
Delta_R_C0 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_R_C0")
Delta_B_C0 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_B_C0")

# Consumed DoF of D0 State
Delta_R_D0 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_R_D0")
Delta_B_D0 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_B_D0")

# Consumed DoF of Theta0 State
Delta_R_Theta0 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "Delta_R_Theta0")
Delta_B_Theta0 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "Delta_B_Theta0")

# SBox-0 Conditions
CondS0 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "CondS0")

# SBox-0 Conditions Value
ValueS0 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "ValueS0")

# Consumed DoF of C1 State
Delta_R_C1 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_R_C1")
Delta_B_C1 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_B_C1")

# Consumed DoF of D1 State
Delta_R_D1 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_R_D1")
Delta_B_D1 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_B_D1")

# Consumed DoF of Theta1 State
Delta_R_Theta1 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "Delta_R_Theta1")
Delta_B_Theta1 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "Delta_B_Theta1")

# Consumed DoF of C2 State
Delta_R_C2 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_R_C2")
Delta_B_C2 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_B_C2")

# Consumed DoF of D2 State
Delta_R_D2 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_R_D2")
Delta_B_D2 = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_B_D2")

# Consumed DoF of Theta2 State
Delta_R_Theta2 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "Delta_R_Theta2")
Delta_B_Theta2 = MFs.creat_consumed_dof_variable_of_state(X, Y, Z, m, "Delta_B_Theta2")

# DeltaM
Delta_M = MFs.creat_consume_dof_of_middle_state(X, Z, m, "Delta_M")

# Conditions
Cond_Num = m.addVar(lb=0, ub=384, vtype=GRB.INTEGER, name="Cond_Num")
m.addConstr(Cond_Num == gurobipy.quicksum(CondS0[i][j][k] for i in range(X) for j in range(Y) for k in range(Z)), "Condition_Number")

# Lambda_R
Lambda_R_Num = m.addVar(lb=0, ub=128, vtype=GRB.INTEGER, name="Lambda_R_Num")
m.addConstr(Lambda_R_Num == gurobipy.quicksum(Lambda_R[i][j][k] for i in range(X) for j in range(Y) for k in range(Z)), "Lambda_R_Number")

# Set objective
m.setObjective(Vector_obj, GRB.MAXIMIZE)

# Add constraint
m.addConstr(Vector_obj <= DoF_R, "c0")
m.addConstr(Vector_obj <= DoF_B, "c1")
m.addConstr(Vector_obj <= DoM, "c2")

# Initial State
MFs.partial_fixed_ini_state_12(m, R0_State, Lambda_R)

# XOR constraint: A0 --> C0
for i in range(Z):
    for j in range(X):
        input_set = []
        for k in range(Y):
            input_set.append(R0_State[j][k][i])
        v_variable = MFs.calculate_v_variable(input_set, m, "v_variable_C0")
        MFs.xor_rules(v_variable, C0_State[j][i], Delta_R_C0[j][i], Delta_B_C0[j][i], m, "C0")

# XOR constraint: C0 --> D0
for i in range(Z):
    for j in range(X):
        input_set = [C0_State[(j - 1) % X][(i - 5) % Z], C0_State[(j - 1) % X][(i - 14) % Z]]
        v_variable = MFs.calculate_v_variable(input_set, m, "v_variable_D0")
        MFs.xor_rules(v_variable, D0_State[j][i], Delta_R_D0[j][i], Delta_B_D0[j][i], m, "D0")

# XOR constraint: D0 --> Theta0
for k in range(Z):
    for j in range(Y):
        for i in range(X):
            input_set = [R0_State[i][j][k], D0_State[i][k]]
            v_variable = MFs.calculate_v_variable(input_set, m, "v_variable_Theta0")
            MFs.xor_rules(v_variable, Theta0_State[i][j][k], Delta_R_Theta0[i][j][k], Delta_B_Theta0[i][j][k], m,
                          "Theta0")

# # Rho0 constraint
MFs.rho_west(X, Z, Theta0_State, Rho0_State, m, "RhoWest0_Cond")

# SBox with condition
for k in range(Z):
    for i in range(X):
        Rho0_variable0 = MFs.calculate_v_variable([R0_State[i][0][k], R0_State[(i-1) % X][2][(k-5) % Z], R0_State[(i-1) % X][2][(k-14) % Z]], m, "Rho0_variable_0")
        Rho0_variable1 = MFs.calculate_v_variable([R0_State[(i - 1) % X][1][k], R0_State[(i - 2) % X][2][(k - 5) % Z], R0_State[(i - 2) % X][2][(k - 14) % Z]], m, "Rho0_variable_1")
        Rho0_variable2 = MFs.calculate_v_variable([R0_State[i][2][(k - 11) % Z], R0_State[(i - 1) % X][2][(k - 16) % Z], R0_State[(i - 1) % X][2][(k - 25) % Z]], m, "Rho0_variable_2")
        Bit_In_0 = [Rho0_variable0[0], Rho0_variable0[2], CondS0[i][0][k]]
        Bit_In_1 = [Rho0_variable1[0], Rho0_variable1[2], CondS0[i][1][k]]
        Bit_In_2 = [Rho0_variable2[0], Rho0_variable2[2], CondS0[i][2][k]]
        MFs.bit_cond(m, Bit_In_0, "CondBit0")
        MFs.bit_cond(m, Bit_In_1, "CondBit1")
        MFs.bit_cond(m, Bit_In_2, "CondBit2")

for k in range(Z):
    for j in range(Y):
        for i in range(X):
            CondSBox_In = [Rho0_State[i][j][k][0], Rho0_State[i][j][k][2], Rho0_State[i][(j + 1) % Y][k][0], Rho0_State[i][(j + 1) % Y][k][2], CondS0[i][(j + 1) % Y][k], ValueS0[i][(j + 1) % Y][k], Rho0_State[i][(j + 2) % Y][k][0], Rho0_State[i][(j + 2) % Y][k][2], CondS0[i][(j + 2) % Y][k], ValueS0[i][(j + 2) % Y][k], Kai0_State[i][j][k][0], Kai0_State[i][j][k][1], Kai0_State[i][j][k][2]]
            MFs.cond_sbox(m, CondSBox_In, "CondSBox")

# Rho0 constraint
MFs.rho_east(X, Z, Kai0_State, R1_State, m, "RhoEast0_Cond")

# XOR constraint: A1 --> C1
for i in range(Z):
    for j in range(X):
        input_set = []
        for k in range(Y):
            input_set.append(R1_State[j][k][i])
        v_variable = MFs.calculate_v_variable(input_set, m, "v_variable_C1")
        MFs.xor_rules(v_variable, C1_State[j][i], Delta_R_C1[j][i], Delta_B_C1[j][i], m, "C1")

# XOR constraint: C1 --> D1
for i in range(Z):
    for j in range(X):
        input_set = [C1_State[(j - 1) % X][(i - 5) % Z], C1_State[(j - 1) % X][(i - 14) % Z]]
        v_variable = MFs.calculate_v_variable(input_set, m, "v_variable_D1")
        MFs.xor_rules(v_variable, D1_State[j][i], Delta_R_D1[j][i], Delta_B_D1[j][i], m, "D1")

# XOR constraint: D1 --> Theta1
for k in range(Z):
    for j in range(Y):
        for i in range(X):
            input_set = [R1_State[i][j][k], D1_State[i][k]]
            v_variable = MFs.calculate_v_variable(input_set, m, "v_variable_Theta1")
            MFs.xor_rules(v_variable, Theta1_State[i][j][k], Delta_R_Theta1[i][j][k], Delta_B_Theta1[i][j][k], m,
                          "Theta1")

# Rho1 constraint
MFs.rho_west(X, Z, Theta1_State, Rho1_State, m, "RhoWest1_Cond")

# SBox
for k in range(Z):
    for j in range(Y):
        for i in range(X):
            in_points = [Rho1_State[i][j][k], Rho1_State[i][(j + 1) % Y][k], Rho1_State[i][(j + 2) % Y][k]]
            MFs.sbox_with_no_conditions(m, in_points, Kai1_State[i][j][k], "SBox_R1")

# Rho1 constraint
MFs.rho_east(X, Z, Kai1_State, R2_State, m, "RhoEast1_Cond")

# XOR constraint: A2 --> C2
for i in range(Z):
    for j in range(X):
        input_set = []
        for k in range(Y):
            input_set.append(R2_State[j][k][i])
        v_variable = MFs.calculate_v_variable(input_set, m, f"v_variable_C2_{j}_{i}")
        MFs.xor_rules(v_variable, C2_State[j][i], Delta_R_C2[j][i], Delta_B_C2[j][i], m, "C2")

# XOR constraint: C2 --> D2
for i in range(Z):
    for j in range(X):
        input_set = [C2_State[(j - 1) % X][(i - 5) % Z], C2_State[(j - 1) % X][(i - 14) % Z]]
        v_variable = MFs.calculate_v_variable(input_set, m, f"v_variable_D2_{j}_{i}")
        MFs.xor_rules(v_variable, D2_State[j][i], Delta_R_D2[j][i], Delta_B_D2[j][i], m, "D2")

# XOR constraint: D2 --> Theta2
for k in range(Z):
    for j in range(Y):
        for i in range(X):
            input_set = [R2_State[i][j][k], D2_State[i][k]]
            v_variable = MFs.calculate_v_variable(input_set, m, f"v_variable_Theta2_{i}_{k}")
            MFs.xor_rules(v_variable, Theta2_State[i][j][k], Delta_R_Theta2[i][j][k], Delta_B_Theta2[i][j][k], m,
                          "Theta2")

# Rho1 constraint
MFs.rho_west(X, Z, Theta2_State, Rho2_State, m, "RhoWest2_Cond")

# SBox
for k in range(Z):
    for j in range(Y):
        for i in range(X):
            in_points = [Rho2_State[i][j][k], Rho2_State[i][(j + 1) % Y][k], Rho2_State[i][(j + 2) % Y][k]]
            MFs.sbox_with_no_conditions(m, in_points, Kai2_State[i][j][k], "SBox_R2")

# def matching_conditions(in_points, delta_m, model, varname):
for k in range(Z):
    for i in range(X):
        Matching_In = [Rho2_State[i][j][k] for j in range(Y)]
        MFs.new_matching_conditions(Matching_In, Delta_M[i][k], m, "MatchingCond")

# DoF_B
m.addConstr(DoF_B == Lambda_B - (gurobipy.quicksum(Delta_B_C0[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_B_C1[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_B_C2[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_B_D0[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_B_D1[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_B_D2[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_B_Theta0[i][j][k] for i in range(X) for j in range(Y) for k in range(Z)) + gurobipy.quicksum(Delta_B_Theta1[i][j][k] for i in range(X) for j in range(Y) for k in range(Z)) + gurobipy.quicksum(Delta_B_Theta2[i][j][k] for i in range(X) for j in range(Y) for k in range(Z))), 'DoF_B_Cond')
m.addConstr(DoF_B == ini_DoF_B, "DoF_B")

# DoF_R
m.addConstr(DoF_R == gurobipy.quicksum(Lambda_R[i][j][k] for i in range(X) for j in range(Y) for k in range(Z)) - (gurobipy.quicksum(Delta_R_C0[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_R_C1[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_R_C2[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_R_D0[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_R_D1[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_R_D2[i][k] for i in range(X) for k in range(Z)) + gurobipy.quicksum(Delta_R_Theta0[i][j][k] for i in range(X) for j in range(Y) for k in range(Z)) + gurobipy.quicksum(Delta_R_Theta1[i][j][k] for i in range(X) for j in range(Y) for k in range(Z)) + gurobipy.quicksum(Delta_R_Theta2[i][j][k] for i in range(X) for j in range(Y) for k in range(Z))), 'DoF_R_Cond')
# m.addConstr(DoF_R == 8, "DoF_R")

# DoM
m.addConstr(DoM == 2 * gurobipy.quicksum(Delta_M[i][k] for i in range(X) for k in range(Z)), 'DoM_Cond')
# m.addConstr(DoM == 7, "DoM")

# Optimize model
m.optimize()

if m.SolCount > 0:
    # Print Result
    for v in m.getVars():
        print('%s: %g' % (v.VarName, v.X))
    print('Obj: %g' % m.ObjVal)

    # Print Picture
    turtle.hideturtle()
    # Round0
    VFs.visualize_the_state(R0_State, X, Y, Z, 0)
    VFs.visualize_the_middle_state_dof(C0_State, X, Z, 15, Delta_R_C0, Delta_B_C0)
    VFs.visualize_the_middle_state_dof(D0_State, X, Z, 25, Delta_R_D0, Delta_B_D0)
    VFs.visualize_the_state_dof(Theta0_State, X, Y, Z, 50, Delta_R_Theta0, Delta_B_Theta0)
    VFs.visualize_the_state(Rho0_State, X, Y, Z, 70)
    VFs.visualize_the_state(Kai0_State, X, Y, Z, 90)
    # Round1
    VFs.visualize_the_state(R1_State, X, Y, Z, 130)
    VFs.visualize_the_middle_state_dof(C1_State, X, Z, 145, Delta_R_C1, Delta_B_C1)
    VFs.visualize_the_middle_state_dof(D1_State, X, Z, 155, Delta_R_D1, Delta_B_D1)
    VFs.visualize_the_state_dof(Theta1_State, X, Y, Z, 180, Delta_R_Theta1, Delta_B_Theta1)
    VFs.visualize_the_state(Rho1_State, X, Y, Z, 200)
    VFs.visualize_the_state(Kai1_State, X, Y, Z, 220)
    # Round2
    VFs.visualize_the_state(R2_State, X, Y, Z, 260)
    VFs.visualize_the_middle_state_dof(C2_State, X, Z, 275, Delta_R_C2, Delta_B_C2)
    VFs.visualize_the_middle_state_dof(D2_State, X, Z, 285, Delta_R_D2, Delta_B_D2)
    VFs.visualize_the_state_dof(Theta2_State, X, Y, Z, 310, Delta_R_Theta2, Delta_B_Theta2)
    VFs.visualize_the_state(Rho2_State, X, Y, Z, 330)
    VFs.visualize_the_state(Kai2_State, X, Y, Z, 350)
    turtle.done()
