import numpy
import gurobipy
from gurobipy import *
from gurobipy import GRB

X = 4
Y = 3
Z = 32


# 定义算法中间状态颜色矩阵
def creat_color_variable_of_state(x, y, z, model, varname):
    color_variable_array = []
    for i in range(x):
        row = []
        for j in range(y):
            column = []
            for k in range(z):
                var_array = [model.addVar(vtype=GRB.BINARY, name=f'{varname}_{i}_{j}_{k}_{l}') for l in range(3)]
                column.append(var_array)
            row.append(column)
        color_variable_array.append(row)
    model.update()
    return color_variable_array


# 定义拆解的中间状态（C、D）颜色矩阵
def creat_color_variable_of_middle_state(x, z, model, varname):
    color_variable_array = []
    for j in range(x):
        column = []
        for k in range(z):
            var_array = [model.addVar(vtype=GRB.BINARY, name=f'{varname}_{j}_{k}_{l}') for l in range(3)]
            column.append(var_array)
        color_variable_array.append(column)
    model.update()
    return color_variable_array


# 定义算法中间状态自由度的消耗矩阵
def creat_consumed_dof_variable_of_state(x, y, z, model, varname):
    color_variable_array = []
    for i in range(x):
        row = []
        for j in range(y):
            column = []
            for k in range(z):
                var = model.addVar(vtype=GRB.BINARY, name=f'{varname}_{i}_{j}_{k}')
                column.append(var)
            row.append(column)
        color_variable_array.append(row)
    model.update()
    return color_variable_array


# 定义拆解的中间状态（C、D）自由度的消耗矩阵
def creat_consume_dof_of_middle_state(x, z, model, varname):
    dof_variable_array = []
    for j in range(x):
        column = []
        for k in range(z):
            var = model.addVar(vtype=GRB.BINARY, name=f'{varname}_{j}_{k}')
            column.append(var)
        dof_variable_array.append(column)
    model.update()
    return dof_variable_array


# 对初始状态的限制性描述
def initial_state_cond(x, z, model, ini_state, lamda_r, lamda_b):
    for k in range(z):
        for i in range(x):
            model.addConstr(ini_state[i][0][k][0] + ini_state[i][0][k][1] + ini_state[i][0][k][2] == 3,
                            f"IniCons0_{i}_{k}")
            model.addConstr(ini_state[i][1][k][0] + ini_state[i][1][k][1] + ini_state[i][1][k][2] == 3,
                            f"IniCons1_{i}_{k}")
            model.addConstr(ini_state[i][2][k][1] == 1, f"IniCons2_{i}_{k}")
            model.addConstr(ini_state[i][2][k][0] + ini_state[i][2][k][2] >= 1, f"IniCons3_{i}_{k}")
    for k in range(z):
        for i in range(x):
            model.addConstr(lamda_b[i][0][k] == 0, f"CondLamda_B_{i}_0_{k}")
            model.addConstr(lamda_b[i][1][k] == 0, f"CondLamda_B_{i}_1_{k}")
            model.addConstr(lamda_b[i][2][k] == ini_state[i][2][k][0], f"CondLamda_B_{i}_2_{k}")
            model.addConstr(lamda_r[i][0][k] == 0, f"CondLamda_R_{i}_0_{k}")
            model.addConstr(lamda_r[i][1][k] == 0, f"CondLamda_R_{i}_1_{k}")
            model.addConstr(lamda_r[i][2][k] == ini_state[i][2][k][2], f"CondLamda_R_{i}_2_{k}")


# 计算XOR运算过程任意输入比特的描述变量v
def calculate_v_variable(color_point_set, model, varname):
    out_variable = [model.addVar(vtype=GRB.BINARY, name=f'{varname}_{i}') for i in range(3)]
    and0 = []
    and1 = []
    and2 = []
    for data in color_point_set:
        and0.append(data[0])
        and1.append(data[1])
        and2.append(data[2])
    model.addGenConstrAnd(out_variable[0], and0, f"{varname}and_constr0")
    model.addGenConstrAnd(out_variable[1], and1, f"{varname}and_constr1")
    model.addGenConstrAnd(out_variable[2], and2, f"{varname}and_constr2")
    return out_variable


# XOR运算的限制性描述
def xor_rules(v_variable, out_variable, delta_r, delta_b, model, varname):
    model.addConstr(out_variable[0] - v_variable[0] >= 0, f"{varname}_xor_condition0")
    model.addConstr(v_variable[1] - out_variable[0] >= 0, f"{varname}_xor_condition1")
    model.addConstr(out_variable[1] - v_variable[1] == 0, f"{varname}_xor_condition2")
    model.addConstr(out_variable[2] - v_variable[2] >= 0, f"{varname}_xor_condition4")
    model.addConstr(v_variable[1] - out_variable[2] >= 0, f"{varname}_xor_condition5")
    model.addConstr(delta_r - out_variable[0] + v_variable[0] == 0, f"{varname}_xor_condition6")
    model.addConstr(delta_b - out_variable[2] + v_variable[2] == 0, f"{varname}_xor_condition3")
    model.update()


# 计算自由度的消耗（可描述初始自由度）
def count_consumed_dof(delta):
    count = 0
    for var in delta:
        if isinstance(var, gurobipy.Var) and var.getAttr('VType') == gurobipy.GRB.BINARY:
            if var.X == 1:
                count += 1
    return count


# Rho_west的描述
def rho_west(x, z, theta, rho, model, varname):
    for k in range(z):
        for i in range(x):
            for j in range(3):
                model.addConstr(rho[i][0][k][j] == theta[i][0][k][j], f"{varname}_Rho_west{i}_{k}_{j}")
                model.addConstr(rho[i][1][k][j] == theta[(i - 1) % X][1][k][j], f"{varname}_Rho_west{i}_{k}_{j}")
                model.addConstr(rho[i][2][k][j] == theta[i][2][(k - 11) % Z][j], f"{varname}_Rho_west{i}_{k}_{j}")


# Rho_east的描述
def rho_east(x, z, kai, a_r, model, varname):
    for k in range(z):
        for i in range(x):
            for j in range(3):
                model.addConstr(a_r[i][0][k][j] == kai[i][0][k][j], f"{varname}_Rho_east{i}_{k}")
                model.addConstr(a_r[i][1][k][j] == kai[i][1][(k - 1) % Z][j], f"{varname}_Rho_east{i}_{k}")
                model.addConstr(a_r[i][2][k][j] == kai[(i - 2) % X][2][(k - 8) % Z][j], f"{varname}_Rho_east{i}_{k}")


# S-box无条件传播限制
def sbox_with_no_conditions(model, in_points, out_point, varname):
    model.addConstr(- in_points[0][1] + 1 >= 0, f'{varname}_SBox_0')
    model.addConstr(- in_points[1][1] + 1 >= 0, f'{varname}_SBox_1')
    model.addConstr(- in_points[0][0] - in_points[1][0] - in_points[2][0] + out_point[0] + 2 >= 0, f'{varname}_SBox_2')
    model.addConstr(- in_points[0][0] + in_points[0][1] >= 0, f'{varname}_SBox_3')
    model.addConstr(- in_points[1][0] + in_points[1][1] >= 0, f'{varname}_SBox_4')
    model.addConstr(in_points[2][0] - out_point[0] >= 0, f'{varname}_SBox_5')
    model.addConstr(
        - in_points[0][0] + in_points[0][1] - in_points[1][0] + in_points[1][1] - in_points[2][0] + in_points[2][1] +
        out_point[0] - out_point[1] >= 0, f'{varname}_SBox_6')
    model.addConstr(in_points[2][1] - out_point[1] >= 0, f'{varname}_SBox_7')
    model.addConstr(
        in_points[0][1] - in_points[0][2] + in_points[1][1] - in_points[1][2] + in_points[2][1] - in_points[2][2] -
        out_point[1] + out_point[2] >= 0, f'{varname}_SBox_8')
    model.addConstr(- in_points[0][2] - in_points[1][2] - in_points[2][2] + out_point[2] + 2 >= 0, f'{varname}_SBox_9')
    model.addConstr(in_points[2][2] - out_point[2] >= 0, f'{varname}_SBox_10')
    model.addConstr(in_points[0][1] - in_points[0][2] >= 0, f'{varname}_SBox_11')
    model.addConstr(in_points[1][1] - in_points[1][2] >= 0, f'{varname}_SBox_12')
    model.addConstr(
        - in_points[0][1] - in_points[1][0] + in_points[1][1] - in_points[1][2] - in_points[2][1] + out_point[
            1] + 2 >= 0, f'{varname}_SBox_13')
    model.addConstr(in_points[0][1] - out_point[1] >= 0, f'{varname}_SBox_14')
    model.addConstr(in_points[1][1] - out_point[1] >= 0, f'{varname}_SBox_15')
    model.addConstr(- in_points[2][1] + 1 >= 0, f'{varname}_SBox_16')
    model.addConstr(in_points[0][0] - out_point[0] >= 0, f'{varname}_SBox_17')
    model.addConstr(- in_points[0][1] - in_points[1][0] - in_points[2][0] + out_point[1] + 2 >= 0, f'{varname}_SBox_18')
    model.addConstr(- in_points[2][0] + in_points[2][1] >= 0, f'{varname}_SBox_19')
    model.addConstr(in_points[1][0] - out_point[0] >= 0, f'{varname}_SBox_20')
    model.addConstr(- in_points[0][1] - in_points[1][2] - in_points[2][2] + out_point[1] + 2 >= 0, f'{varname}_SBox_21')
    model.addConstr(in_points[1][2] - out_point[2] >= 0, f'{varname}_SBox_22')
    model.addConstr(in_points[2][1] - in_points[2][2] >= 0, f'{varname}_SBox_23')
    model.addConstr(in_points[0][2] - out_point[2] >= 0, f'{varname}_SBox_24')
    model.addConstr(
        - in_points[0][1] - in_points[1][1] - in_points[2][0] + in_points[2][1] - in_points[2][2] + out_point[
            1] + 2 >= 0, f'{varname}_SBox_25')
    model.addConstr(in_points[1][0] + in_points[2][2] - out_point[1] >= 0, f'{varname}_SBox_26')
    model.addConstr(
        - in_points[0][0] + in_points[0][1] - in_points[2][0] + in_points[2][1] + in_points[2][2] + out_point[0] -
        out_point[1] >= 0, f'{varname}_SBox_27')
    model.addConstr(
        in_points[0][1] - in_points[0][2] + in_points[2][0] + in_points[2][1] - in_points[2][2] - out_point[1] +
        out_point[2] >= 0, f'{varname}_SBox_28')
    model.addConstr(
        - in_points[0][0] + in_points[0][1] - in_points[1][0] + in_points[1][1] + in_points[1][2] + out_point[0] -
        out_point[1] >= 0, f'{varname}_SBox_29')
    model.addConstr(in_points[1][2] + in_points[2][0] - out_point[1] >= 0, f'{varname}_SBox_30')
    model.addConstr(
        in_points[0][1] - in_points[0][2] + in_points[1][0] + in_points[2][0] - out_point[0] - out_point[1] + out_point[
            2] >= 0, f'{varname}_SBox_31')
    model.addConstr(
        - in_points[0][0] + in_points[0][1] + in_points[1][2] + in_points[2][2] + out_point[0] - out_point[1] -
        out_point[2] >= 0, f'{varname}_SBox_32')
    model.addConstr(
        - in_points[0][0] + 2 * in_points[0][1] - in_points[0][2] - in_points[1][0] + in_points[1][1] + in_points[2][
            1] - in_points[2][2] + out_point[0] - out_point[1] + out_point[2] >= 0, f'{varname}_SBox_33')
    model.addConstr(out_point[0] >= 0, f'{varname}_SBox_34')
    model.addConstr(out_point[2] >= 0, f'{varname}_SBox_35')
    model.addConstr(
        in_points[0][1] - in_points[0][2] + in_points[1][0] + in_points[1][1] - in_points[1][2] - out_point[1] +
        out_point[2] >= 0, f'{varname}_SBox_36')
    model.addConstr(
        - in_points[0][0] + 2 * in_points[0][1] - in_points[0][2] + in_points[1][1] - in_points[1][2] - in_points[2][
            0] + in_points[2][1] + out_point[0] - out_point[1] + out_point[2] >= 0, f'{varname}_SBox_37')
    model.addConstr(- out_point[0] + out_point[1] >= 0, f'{varname}_SBox_38')
    model.addConstr(out_point[1] - out_point[2] >= 0, f'{varname}_SBox_39')


# S-box带条件传播
def cond_sbox(model, point, varname):
    model.addConstr(- point[0] + 1 >= 0, f'{varname}_CondBit_0')
    model.addConstr(- point[1] + 1 >= 0, f'{varname}_CondBit_1')
    model.addConstr(- point[2] + 1 >= 0, f'{varname}_CondBit_2')
    model.addConstr(- point[11] + 1 >= 0, f'{varname}_CondBit_3')
    model.addConstr(- point[3] + 1 >= 0, f'{varname}_CondBit_4')
    model.addConstr(- point[2] - point[3] + point[11] + 1 >= 0, f'{varname}_CondBit_5')
    model.addConstr(point[8] >= 0, f'{varname}_CondBit_6')
    model.addConstr(- point[0] - point[1] - point[2] - point[3] - point[8] + point[11] + 3 >= 0, f'{varname}_CondBit_7')
    model.addConstr(point[3] - point[4] >= 0, f'{varname}_CondBit_8')
    model.addConstr(- point[4] + point[11] >= 0, f'{varname}_CondBit_9')
    model.addConstr(point[2] - point[4] >= 0, f'{varname}_CondBit_10')
    model.addConstr(- point[0] - point[1] - point[4] - point[8] + point[11] + 2 >= 0, f'{varname}_CondBit_11')
    model.addConstr(- point[0] + point[3] - point[4] - point[5] + point[10] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_12')
    model.addConstr(- point[0] - point[4] - point[5] + point[10] + 2 >= 0, f'{varname}_CondBit_13')
    model.addConstr(- point[0] - point[4] - point[8] + 2 * point[11] - point[12] + 1 >= 0, f'{varname}_CondBit_14')
    model.addConstr(- point[1] - point[2] - point[3] - point[8] - point[10] + 2 * point[11] + 2 >= 0,
                    f'{varname}_CondBit_15')
    model.addConstr(point[2] + point[8] - point[10] >= 0, f'{varname}_CondBit_16')
    model.addConstr(point[0] - point[10] >= 0, f'{varname}_CondBit_17')
    model.addConstr(- point[10] + point[11] >= 0, f'{varname}_CondBit_18')
    model.addConstr(- point[1] + point[2] - point[3] - point[4] - point[5] - point[11] + point[12] + 3 >= 0,
                    f'{varname}_CondBit_19')
    model.addConstr(- point[1] - point[4] - point[5] + point[12] + 2 >= 0, f'{varname}_CondBit_20')
    model.addConstr(
        - 2 * point[1] + point[2] - 2 * point[4] - point[5] - point[6] - point[7] - point[10] + point[11] + point[
            12] + 4 >= 0, f'{varname}_CondBit_21')
    model.addConstr(- point[1] + point[2] - point[4] - point[5] - point[10] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_22')
    model.addConstr(- point[1] - point[4] - point[8] - point[10] + 2 * point[11] + 1 >= 0, f'{varname}_CondBit_23')
    model.addConstr(- point[0] - point[2] + point[3] - point[4] - point[5] + point[10] - point[11] + 3 >= 0,
                    f'{varname}_CondBit_24')
    model.addConstr(point[11] - point[12] >= 0, f'{varname}_CondBit_25')
    model.addConstr(point[1] - point[12] >= 0, f'{varname}_CondBit_26')
    model.addConstr(- point[0] - point[2] - point[3] - point[8] + 2 * point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_27')
    model.addConstr(- point[5] + 1 >= 0, f'{varname}_CondBit_28')
    model.addConstr(point[3] + point[8] - point[12] >= 0, f'{varname}_CondBit_29')
    model.addConstr(
        - 2 * point[0] + point[3] - 2 * point[4] - point[5] - point[8] + point[10] + point[11] - point[12] + 3 >= 0,
        f'{varname}_CondBit_30')
    model.addConstr(
        - 2 * point[0] + point[3] - 2 * point[4] - point[5] - point[6] - point[7] + point[10] + point[11] - point[
            12] + 4 >= 0, f'{varname}_CondBit_31')
    model.addConstr(
        - 2 * point[0] - 2 * point[1] - 3 * point[4] - point[5] - point[6] - point[7] + point[10] + point[11] + point[
            12] + 6 >= 0, f'{varname}_CondBit_32')
    model.addConstr(point[2] + point[3] - point[4] - point[6] - point[7] + 2 * point[8] - point[10] + point[11] - point[
        12] + 1 >= 0, f'{varname}_CondBit_33')
    model.addConstr(- point[4] - point[6] - point[7] - point[10] + 3 * point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_34')
    model.addConstr(- point[2] - point[3] - point[6] - point[7] - point[10] + 3 * point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_35')
    model.addConstr(- point[6] - point[7] + point[8] - point[10] + 2 * point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_36')
    model.addConstr(point[2] + point[3] - point[4] + point[5] + 2 * point[8] - point[10] - point[12] >= 0,
                    f'{varname}_CondBit_37')
    model.addConstr(- point[2] - point[3] + point[4] - point[10] + 2 * point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_38')
    model.addConstr(point[4] + point[8] - point[10] + point[11] - point[12] >= 0, f'{varname}_CondBit_39')
    model.addConstr(- point[0] + point[3] + point[7] + point[10] - point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_40')
    model.addConstr(point[7] - point[8] >= 0, f'{varname}_CondBit_41')
    model.addConstr(
        - 2 * point[0] - 2 * point[4] + point[7] - 2 * point[8] + point[10] + 2 * point[11] - point[12] + 2 >= 0,
        f'{varname}_CondBit_42')
    model.addConstr(
        - 2 * point[1] + point[2] - 2 * point[4] - point[5] - point[8] - point[10] + point[11] + point[12] + 3 >= 0,
        f'{varname}_CondBit_43')
    model.addConstr(- point[2] - point[3] - point[8] - point[10] + 3 * point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_44')
    model.addConstr(- point[4] - point[8] - point[10] + 3 * point[11] - point[12] >= 0, f'{varname}_CondBit_45')
    model.addConstr(
        - 3 * point[0] - 3 * point[1] - point[2] - point[3] - 2 * point[4] - 2 * point[5] - 2 * point[8] + point[10] +
        point[11] + point[12] + 9 >= 0, f'{varname}_CondBit_46')
    model.addConstr(point[5] + point[8] - point[10] + point[11] - point[12] >= 0, f'{varname}_CondBit_47')
    model.addConstr(- point[2] - point[3] + point[5] - point[10] + 2 * point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_48')
    model.addConstr(- point[4] + point[5] - point[10] + 2 * point[11] - point[12] >= 0, f'{varname}_CondBit_49')
    model.addConstr(- point[0] - point[4] - point[6] - point[7] + 2 * point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_50')
    model.addConstr(- point[1] - point[2] - point[3] - point[6] - point[7] - point[10] + 2 * point[11] + 3 >= 0,
                    f'{varname}_CondBit_51')
    model.addConstr(- point[0] - point[1] - point[2] - point[3] - point[6] - point[7] + point[11] + 4 >= 0,
                    f'{varname}_CondBit_52')
    model.addConstr(- point[1] + point[2] - point[4] - point[6] - point[7] + point[8] - point[10] + point[11] + 2 >= 0,
                    f'{varname}_CondBit_53')
    model.addConstr(- point[1] - point[4] - point[6] - point[7] - point[10] + 2 * point[11] + 2 >= 0,
                    f'{varname}_CondBit_54')
    model.addConstr(- point[0] - point[2] - point[3] - point[6] - point[7] + 2 * point[11] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_55')
    model.addConstr(- point[0] - point[1] - point[4] - point[6] - point[7] + point[11] + 3 >= 0,
                    f'{varname}_CondBit_56')
    model.addConstr(- point[0] + point[3] - point[4] - point[6] - point[7] + point[8] + point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_57')
    model.addConstr(
        - 4 * point[0] - 2 * point[2] + point[3] - 2 * point[4] - 2 * point[5] + point[7] - 2 * point[8] + 3 * point[
            10] - point[11] - point[12] + 8 >= 0, f'{varname}_CondBit_58')
    model.addConstr(
        - 2 * point[0] - 2 * point[2] - point[3] + point[7] - 2 * point[8] + point[10] + point[11] - point[12] + 4 >= 0,
        f'{varname}_CondBit_59')
    model.addConstr(- 2 * point[0] - point[1] - point[2] - point[4] - point[5] - point[8] + point[10] + 5 >= 0,
                    f'{varname}_CondBit_60')
    model.addConstr(- point[0] - 2 * point[1] - point[3] - point[4] - point[5] - point[8] + point[12] + 5 >= 0,
                    f'{varname}_CondBit_61')
    model.addConstr(
        - 3 * point[1] + point[2] - point[3] - 2 * point[4] - 2 * point[5] - point[8] - point[10] + 2 * point[
            12] + 6 >= 0, f'{varname}_CondBit_62')
    model.addConstr(
        - 3 * point[0] - point[2] + point[3] - 2 * point[4] - 2 * point[5] - point[8] + 2 * point[10] - point[
            12] + 6 >= 0, f'{varname}_CondBit_63')
    model.addConstr(
        - 2 * point[0] + point[3] - 2 * point[4] + point[7] - 2 * point[8] + point[10] + point[11] - point[12] + 2 >= 0,
        f'{varname}_CondBit_64')
    model.addConstr(point[5] + point[7] - point[12] >= 0, f'{varname}_CondBit_65')
    model.addConstr(- point[0] - point[2] + point[7] - point[8] + point[10] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_66')
    model.addConstr(- point[6] + 1 >= 0, f'{varname}_CondBit_67')
    model.addConstr(- point[0] - point[2] - point[6] + point[10] + 2 >= 0, f'{varname}_CondBit_68')
    model.addConstr(- point[0] + point[3] - point[4] + point[7] - point[8] + point[10] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_69')
    model.addConstr(- point[0] + point[7] + point[10] - point[12] + 1 >= 0, f'{varname}_CondBit_70')
    model.addConstr(- point[0] - point[4] + point[5] + point[7] - point[8] + point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_71')
    model.addConstr(- point[0] - point[2] - point[3] + point[7] - point[8] + point[10] + point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_72')
    model.addConstr(- point[0] - point[4] + point[7] - point[8] + point[10] + point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_73')
    model.addConstr(- point[0] - point[2] - point[3] + point[5] + point[7] - point[8] + point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_74')
    model.addConstr(point[2] + point[7] - point[11] >= 0, f'{varname}_CondBit_75')
    model.addConstr(- point[2] - point[6] + point[11] + 1 >= 0, f'{varname}_CondBit_76')
    model.addConstr(- point[0] + point[3] - point[4] - point[5] + point[7] + point[10] - point[11] + 2 >= 0,
                    f'{varname}_CondBit_77')
    model.addConstr(- point[0] - point[2] - point[3] + point[4] + point[7] - point[8] + point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_78')
    model.addConstr(- point[0] - point[2] - point[3] + point[4] - point[6] + point[11] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_79')
    model.addConstr(point[2] + point[3] - point[4] + point[5] + point[7] + point[8] - point[11] - point[12] >= 0,
                    f'{varname}_CondBit_80')
    model.addConstr(
        - 2 * point[0] - point[2] + point[3] - point[4] - point[5] + point[7] - point[8] + 2 * point[10] - point[11] -
        point[12] + 4 >= 0, f'{varname}_CondBit_81')
    model.addConstr(
        - point[0] + point[3] - point[4] + point[5] - point[6] + point[7] + point[10] - point[11] - point[12] + 2 >= 0,
        f'{varname}_CondBit_82')
    model.addConstr(point[4] + point[7] - point[12] >= 0, f'{varname}_CondBit_83')
    model.addConstr(
        - 2 * point[0] - 2 * point[2] - 2 * point[3] + point[7] - 2 * point[8] + point[10] + 2 * point[11] - point[
            12] + 4 >= 0, f'{varname}_CondBit_84')
    model.addConstr(- point[0] + point[3] - point[4] - point[6] + point[10] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_85')
    model.addConstr(- point[0] + point[3] - point[4] - point[8] - point[9] + point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_86')
    model.addConstr(- point[1] + point[2] - point[4] - point[8] - point[9] - point[10] + point[11] + 2 >= 0,
                    f'{varname}_CondBit_87')
    model.addConstr(
        - point[0] - 2 * point[1] - point[3] - point[4] - point[5] - point[6] - point[7] + point[12] + 6 >= 0,
        f'{varname}_CondBit_88')
    model.addConstr(- point[1] - point[3] - point[8] - point[9] - point[10] + point[11] + 3 >= 0,
                    f'{varname}_CondBit_89')
    model.addConstr(point[2] - point[9] - point[10] + 1 >= 0, f'{varname}_CondBit_90')
    model.addConstr(point[5] - point[9] - point[10] + point[11] - point[12] + 1 >= 0, f'{varname}_CondBit_91')
    model.addConstr(- point[9] + 1 >= 0, f'{varname}_CondBit_92')
    model.addConstr(point[3] - point[9] - point[12] + 1 >= 0, f'{varname}_CondBit_93')
    model.addConstr(- point[8] - point[9] - point[10] + 2 * point[11] - point[12] + 1 >= 0, f'{varname}_CondBit_94')
    model.addConstr(point[4] - point[9] - point[10] + point[11] - point[12] + 1 >= 0, f'{varname}_CondBit_95')
    model.addConstr(- point[0] + point[3] - point[4] - point[6] - point[7] - point[9] + point[11] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_96')
    model.addConstr(
        point[2] + point[3] - point[4] - point[8] - 2 * point[9] - point[10] + point[11] - point[12] + 2 >= 0,
        f'{varname}_CondBit_97')
    model.addConstr(- point[1] + point[2] - point[4] - point[6] - point[7] - point[9] - point[10] + point[11] + 3 >= 0,
                    f'{varname}_CondBit_98')
    model.addConstr(
        - point[0] + 2 * point[3] - point[4] + point[5] + 2 * point[7] - point[8] - point[9] + point[10] - point[
            11] - 2 * point[12] + 2 >= 0, f'{varname}_CondBit_99')
    model.addConstr(point[2] + point[3] - point[4] + point[5] + point[7] - point[9] - point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_100')
    model.addConstr(- point[0] + point[3] - point[4] + point[5] + point[7] - point[8] - point[9] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_101')
    model.addConstr(
        - point[0] - point[2] + point[3] + point[7] - point[8] - point[9] + point[10] - point[11] - point[12] + 3 >= 0,
        f'{varname}_CondBit_102')
    model.addConstr(- point[6] - point[7] - point[9] - point[10] + 2 * point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_103')
    model.addConstr(point[2] + point[3] - point[4] - point[6] - point[7] - 2 * point[9] - point[10] + point[11] - point[
        12] + 3 >= 0, f'{varname}_CondBit_104')
    model.addConstr(
        - 2 * point[1] + point[2] - point[3] - point[4] - point[5] - point[8] - point[9] - point[10] + point[
            12] + 5 >= 0, f'{varname}_CondBit_105')
    model.addConstr(point[2] + point[3] - point[4] + point[5] - 2 * point[9] - point[10] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_106')
    model.addConstr(- point[0] + point[3] - point[4] + point[5] - point[6] + point[8] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_107')
    model.addConstr(- point[0] - point[2] - point[3] + point[5] - point[6] + point[11] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_108')
    model.addConstr(- point[0] - point[4] + point[5] - point[6] + point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_109')
    model.addConstr(
        - 2 * point[0] + point[3] - 2 * point[4] + point[5] - point[6] + point[7] - point[8] + point[10] - point[
            12] + 3 >= 0, f'{varname}_CondBit_110')
    model.addConstr(- 2 * point[0] + point[3] - 2 * point[4] + point[5] - 2 * point[6] + point[10] - point[12] + 4 >= 0,
                    f'{varname}_CondBit_111')
    model.addConstr(- point[0] - point[2] - point[6] - point[7] + point[8] + point[11] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_112')
    model.addConstr(
        - 3 * point[0] - point[2] + point[3] - 2 * point[4] - 2 * point[5] - point[6] - point[7] + 2 * point[10] -
        point[12] + 7 >= 0, f'{varname}_CondBit_113')
    model.addConstr(
        - 2 * point[0] - point[2] + point[3] - point[4] - point[5] - point[6] - point[7] + point[8] + point[10] - point[
            12] + 5 >= 0, f'{varname}_CondBit_114')
    model.addConstr(
        - 2 * point[0] - point[1] - point[2] - point[4] - point[5] - point[6] - point[7] + point[10] + 6 >= 0,
        f'{varname}_CondBit_115')
    model.addConstr(
        - 2 * point[0] + point[3] - 2 * point[4] - 2 * point[6] - point[7] + point[10] + point[11] - point[12] + 4 >= 0,
        f'{varname}_CondBit_116')
    model.addConstr(
        - 2 * point[0] - 2 * point[1] - point[2] - point[3] - point[4] - point[5] - point[6] - point[7] + point[10] +
        point[12] + 7 >= 0, f'{varname}_CondBit_117')
    model.addConstr(
        - 3 * point[1] + point[2] - point[3] - 2 * point[4] - 2 * point[5] - point[6] - point[7] - point[10] + 2 *
        point[12] + 7 >= 0, f'{varname}_CondBit_118')
    model.addConstr(
        - 2 * point[1] + point[2] - point[3] - point[4] - point[5] - point[6] - point[7] + point[8] - point[10] + point[
            12] + 5 >= 0, f'{varname}_CondBit_119')
    model.addConstr(- point[1] - point[3] - point[6] - point[7] + point[8] - point[10] + point[11] + 3 >= 0,
                    f'{varname}_CondBit_120')
    model.addConstr(
        - point[1] + point[2] - point[3] + point[6] - point[8] - point[9] - point[10] - point[11] + point[12] + 3 >= 0,
        f'{varname}_CondBit_121')
    model.addConstr(
        - point[1] + point[2] - point[4] - point[5] + point[7] - point[8] + point[9] - point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_122')
    model.addConstr(- 2 * point[1] + point[2] - 2 * point[4] + point[5] - 2 * point[7] - point[10] + point[12] + 4 >= 0,
                    f'{varname}_CondBit_123')
    model.addConstr(
        - 3 * point[0] + point[3] - 2 * point[4] - 2 * point[5] - point[8] + point[9] + 2 * point[10] - point[
            12] + 5 >= 0, f'{varname}_CondBit_124')
    model.addConstr(- 2 * point[0] - point[1] - point[4] - point[5] - point[8] + point[9] + point[10] + 4 >= 0,
                    f'{varname}_CondBit_125')
    model.addConstr(- point[1] + point[2] - point[4] + point[5] + point[6] - point[8] - point[9] - point[10] + 2 >= 0,
                    f'{varname}_CondBit_126')
    model.addConstr(
        - 4 * point[0] - 4 * point[1] - 3 * point[4] - point[5] - point[6] - point[7] - 2 * point[8] + 2 * point[9] +
        point[10] + point[11] + point[12] + 10 >= 0, f'{varname}_CondBit_127')
    model.addConstr(- point[1] - point[2] - point[3] + point[4] - point[7] - point[10] + point[11] + 3 >= 0,
                    f'{varname}_CondBit_128')
    model.addConstr(
        - 3 * point[1] + point[2] - 2 * point[4] - 2 * point[5] - point[8] + point[9] - point[10] + 2 * point[
            12] + 5 >= 0, f'{varname}_CondBit_129')
    model.addConstr(- point[0] + point[7] - point[8] + point[9] + point[10] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_130')
    model.addConstr(
        - 4 * point[0] - 4 * point[1] - point[2] - point[3] - 2 * point[4] - 2 * point[5] - 3 * point[8] + point[9] +
        point[10] + point[11] + point[12] + 11 >= 0, f'{varname}_CondBit_131')
    model.addConstr(
        - 2 * point[1] - point[2] - 2 * point[3] + point[6] - 2 * point[8] - point[10] + point[11] + point[12] + 4 >= 0,
        f'{varname}_CondBit_132')
    model.addConstr(- 2 * point[0] - point[1] - point[4] - point[6] - point[8] + point[9] + point[10] + 4 >= 0,
                    f'{varname}_CondBit_133')
    model.addConstr(
        - 2 * point[1] + point[2] - 2 * point[4] + point[5] + point[6] - point[7] - point[8] - point[10] + point[
            12] + 3 >= 0, f'{varname}_CondBit_134')
    model.addConstr(- point[1] + point[2] - point[4] + point[5] - point[7] + point[8] - point[10] + 2 >= 0,
                    f'{varname}_CondBit_135')
    model.addConstr(
        - 4 * point[1] + point[2] - 2 * point[3] - 2 * point[4] - 2 * point[5] + point[6] - 2 * point[8] - point[10] -
        point[11] + 3 * point[12] + 8 >= 0, f'{varname}_CondBit_136')
    model.addConstr(
        - 3 * point[1] - point[4] - point[5] + point[6] - 2 * point[8] + 2 * point[9] - point[10] + 2 * point[
            12] + 4 >= 0, f'{varname}_CondBit_137')
    model.addConstr(- point[1] - point[4] + point[5] - point[7] - point[10] + point[11] + 2 >= 0,
                    f'{varname}_CondBit_138')
    model.addConstr(
        - 3 * point[1] - point[2] - point[3] + point[6] - point[7] - 2 * point[8] + 2 * point[9] - point[10] + 2 *
        point[12] + 5 >= 0, f'{varname}_CondBit_139')
    model.addConstr(
        - 3 * point[1] - point[4] + point[6] - point[7] - 2 * point[8] + 2 * point[9] - point[10] + 2 * point[
            12] + 4 >= 0, f'{varname}_CondBit_140')
    model.addConstr(
        - 2 * point[1] - point[4] + point[6] - 2 * point[8] + point[9] - point[10] + point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_141')
    model.addConstr(
        - 3 * point[1] + point[2] - 2 * point[4] + point[5] + point[6] - 2 * point[7] - point[8] + point[9] - point[
            10] - point[11] + 2 * point[12] + 5 >= 0, f'{varname}_CondBit_142')
    model.addConstr(- point[0] - 2 * point[1] - point[4] - point[7] - point[8] + point[9] + point[12] + 4 >= 0,
                    f'{varname}_CondBit_143')
    model.addConstr(
        - 2 * point[1] - point[2] - point[3] + point[5] + point[6] - point[7] - point[8] + point[9] - point[10] + point[
            12] + 4 >= 0, f'{varname}_CondBit_144')
    model.addConstr(
        - 2 * point[1] - point[4] + point[5] + point[6] - point[7] - point[8] + point[9] - point[10] + point[
            12] + 3 >= 0, f'{varname}_CondBit_145')
    model.addConstr(
        - 2 * point[1] + point[2] - point[4] + point[6] - point[7] - point[8] + point[9] - point[10] - point[11] + 2 *
        point[12] + 3 >= 0, f'{varname}_CondBit_146')
    model.addConstr(- point[1] + point[6] - point[7] - point[8] + point[9] - point[11] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_147')
    model.addConstr(
        - 2 * point[1] + point[2] - 2 * point[4] - 2 * point[6] - 2 * point[7] - point[10] + 2 * point[11] + point[
            12] + 4 >= 0, f'{varname}_CondBit_148')
    model.addConstr(- point[0] - point[6] + point[7] - point[8] + point[9] + point[10] - point[11] + 2 >= 0,
                    f'{varname}_CondBit_149')
    model.addConstr(
        - point[1] + point[3] - point[4] - point[5] + point[6] - point[8] + point[9] - point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_150')
    model.addConstr(point[3] + point[4] + point[6] + point[7] - point[8] - point[9] - point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_151')
    model.addConstr(
        - 4 * point[1] + point[2] - 2 * point[4] - 2 * point[5] + point[6] - 2 * point[8] + 2 * point[9] - point[10] -
        point[11] + 3 * point[12] + 6 >= 0, f'{varname}_CondBit_152')
    model.addConstr(point[2] + point[5] + point[6] + point[7] - point[8] - point[9] - point[10] - point[11] + 1 >= 0,
                    f'{varname}_CondBit_153')
    model.addConstr(
        point[2] + point[3] - point[4] + 2 * point[5] + point[6] + point[7] - point[8] - 2 * point[9] - point[10] -
        point[11] - point[12] + 2 >= 0, f'{varname}_CondBit_154')
    model.addConstr(- point[0] - point[8] + point[9] + point[10] + 1 >= 0, f'{varname}_CondBit_155')
    model.addConstr(- point[1] - point[8] + point[9] + point[12] + 1 >= 0, f'{varname}_CondBit_156')
    model.addConstr(
        - 2 * point[1] - point[2] - point[3] + point[6] - 2 * point[8] + point[9] - point[10] + point[11] + point[
            12] + 3 >= 0, f'{varname}_CondBit_157')
    model.addConstr(point[2] + point[4] + point[6] + point[7] - point[8] - point[9] - point[10] - point[11] + 1 >= 0,
                    f'{varname}_CondBit_158')
    model.addConstr(
        - 4 * point[0] + point[3] - 2 * point[4] - 2 * point[5] + point[7] - 2 * point[8] + 2 * point[9] + 3 * point[
            10] - point[11] - point[12] + 6 >= 0, f'{varname}_CondBit_159')
    model.addConstr(point[3] + point[5] + point[6] + point[7] - point[8] - point[9] - point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_160')
    model.addConstr(
        - point[1] + point[2] - point[4] - point[5] + point[6] - point[8] + point[9] - point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_161')
    model.addConstr(
        - point[2] - point[3] + 2 * point[4] + point[6] + point[7] - point[8] - point[10] + point[11] - point[
            12] + 1 >= 0, f'{varname}_CondBit_162')
    model.addConstr(
        - 2 * point[1] + point[2] - point[4] - point[5] + point[6] - point[8] + point[9] - point[10] - point[11] + 2 *
        point[12] + 3 >= 0, f'{varname}_CondBit_163')
    model.addConstr(- point[1] + point[2] + point[6] - point[10] - point[11] + point[12] + 1 >= 0,
                    f'{varname}_CondBit_164')
    model.addConstr(- point[0] - 2 * point[1] - point[4] - point[5] - point[8] + point[9] + point[12] + 4 >= 0,
                    f'{varname}_CondBit_165')
    model.addConstr(2 * point[5] + point[6] + point[7] - point[8] - point[9] - point[10] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_166')
    model.addConstr(
        - point[0] + point[3] - point[4] - point[5] + point[7] - point[8] + point[9] + point[10] - point[11] + 2 >= 0,
        f'{varname}_CondBit_167')
    model.addConstr(point[4] + point[6] - point[10] >= 0, f'{varname}_CondBit_168')
    model.addConstr(- point[0] + point[3] - point[4] - point[6] - point[7] + point[10] + point[11] - point[12] + 2 >= 0,
                    f'{varname}_CondBit_169')
    model.addConstr(
        - 2 * point[0] - 2 * point[1] - point[2] - point[3] - 3 * point[8] + point[9] + point[10] + point[11] + point[
            12] + 5 >= 0, f'{varname}_CondBit_170')
    model.addConstr(- point[1] - point[3] + point[6] - point[8] - point[10] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_171')
    model.addConstr(
        - 2 * point[1] - 2 * point[2] - 2 * point[3] + point[6] - 2 * point[8] - point[10] + 2 * point[11] + point[
            12] + 4 >= 0, f'{varname}_CondBit_172')
    model.addConstr(point[6] - point[8] >= 0, f'{varname}_CondBit_173')
    model.addConstr(
        - 2 * point[1] + point[2] - 2 * point[4] - point[6] - 2 * point[7] - point[10] + point[11] + point[12] + 4 >= 0,
        f'{varname}_CondBit_174')
    model.addConstr(point[4] >= 0, f'{varname}_CondBit_175')
    model.addConstr(
        - point[0] - 2 * point[1] - point[2] - point[3] - point[7] - point[8] + point[9] + point[12] + 5 >= 0,
        f'{varname}_CondBit_176')
    model.addConstr(
        - 2 * point[0] - 2 * point[1] - point[2] - point[3] - point[6] - point[7] - point[8] + point[9] + point[10] +
        point[12] + 6 >= 0, f'{varname}_CondBit_177')
    model.addConstr(- point[1] - point[3] + point[4] + point[6] - point[8] - point[9] - point[10] + 3 >= 0,
                    f'{varname}_CondBit_178')
    model.addConstr(- point[1] - point[4] + point[6] - point[8] - point[10] + point[11] + point[12] + 1 >= 0,
                    f'{varname}_CondBit_179')
    model.addConstr(
        - 2 * point[1] - point[2] - point[3] + point[4] + point[6] - point[7] - point[8] + point[9] - point[10] + point[
            12] + 4 >= 0, f'{varname}_CondBit_180')
    model.addConstr(- point[1] - point[2] - point[3] + point[6] - point[8] - point[10] + point[11] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_181')
    model.addConstr(2 * point[4] + point[6] + point[7] - point[8] - point[9] - point[10] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_182')
    model.addConstr(- point[1] + point[6] - point[10] + point[12] + 1 >= 0, f'{varname}_CondBit_183')
    model.addConstr(point[2] + point[3] - point[4] + point[5] + point[6] + point[8] - point[10] - point[11] >= 0,
                    f'{varname}_CondBit_184')
    model.addConstr(- point[1] + point[6] - point[8] + point[9] - point[10] + point[12] + 1 >= 0,
                    f'{varname}_CondBit_185')
    model.addConstr(- point[1] - point[3] + point[5] + point[6] - point[8] - point[9] - point[10] + 3 >= 0,
                    f'{varname}_CondBit_186')
    model.addConstr(- point[0] - point[4] - point[8] + point[10] + point[11] + 1 >= 0, f'{varname}_CondBit_187')
    model.addConstr(- point[0] - point[2] - point[3] - point[8] + point[10] + point[11] + 2 >= 0,
                    f'{varname}_CondBit_188')
    model.addConstr(- point[4] + 2 * point[5] + point[6] + point[7] - point[8] - point[10] + point[11] - point[12] >= 0,
                    f'{varname}_CondBit_189')
    model.addConstr(
        - 2 * point[1] - 2 * point[4] + point[6] - 2 * point[8] - point[10] + 2 * point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_190')
    model.addConstr(- point[1] - point[3] - point[7] + point[12] + 2 >= 0, f'{varname}_CondBit_191')
    model.addConstr(
        - 2 * point[1] + point[2] - 2 * point[4] + point[6] - 2 * point[8] - point[10] + point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_192')
    model.addConstr(- point[1] - point[4] + point[5] + point[6] - point[8] - point[10] + point[11] + 1 >= 0,
                    f'{varname}_CondBit_193')
    model.addConstr(- point[1] + point[2] - point[4] + point[6] - point[8] - point[10] + point[12] + 1 >= 0,
                    f'{varname}_CondBit_194')
    model.addConstr(
        - 2 * point[1] + point[2] - point[3] - point[4] - point[5] + point[6] - point[8] - point[10] - point[11] + 2 *
        point[12] + 4 >= 0, f'{varname}_CondBit_195')
    model.addConstr(- point[1] - point[2] - point[3] - point[8] + point[11] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_196')
    model.addConstr(- point[0] + point[3] + point[10] - point[12] + 1 >= 0, f'{varname}_CondBit_197')
    model.addConstr(- point[1] - point[2] - point[3] + point[5] + point[6] - point[8] - point[10] + point[11] + 2 >= 0,
                    f'{varname}_CondBit_198')
    model.addConstr(point[3] + point[6] - point[11] >= 0, f'{varname}_CondBit_199')
    model.addConstr(- point[1] - point[2] - point[3] + point[4] + point[6] - point[8] - point[10] + point[11] + 2 >= 0,
                    f'{varname}_CondBit_200')
    model.addConstr(
        - 3 * point[0] - 3 * point[1] - 2 * point[4] - point[6] - point[7] - 2 * point[8] + 2 * point[9] + point[10] +
        point[11] + point[12] + 7 >= 0, f'{varname}_CondBit_201')
    model.addConstr(- point[1] + point[2] - point[4] - point[8] - point[10] + point[11] + point[12] + 1 >= 0,
                    f'{varname}_CondBit_202')
    model.addConstr(
        - 2 * point[1] + point[2] - 2 * point[4] - 2 * point[8] - point[10] + 2 * point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_203')
    model.addConstr(point[5] + point[6] - point[10] >= 0, f'{varname}_CondBit_204')
    model.addConstr(
        - point[2] - point[3] + 2 * point[5] + point[6] + point[7] - point[8] - point[10] + point[11] - point[
            12] + 1 >= 0, f'{varname}_CondBit_205')
    model.addConstr(
        - 2 * point[0] + point[3] - 2 * point[4] - 2 * point[6] - 2 * point[7] + point[10] + 2 * point[11] - point[
            12] + 4 >= 0, f'{varname}_CondBit_206')
    model.addConstr(- point[1] + point[2] - point[4] - point[7] - point[10] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_207')
    model.addConstr(- point[1] + point[2] + point[6] - point[8] + point[9] - point[11] + point[12] + 1 >= 0,
                    f'{varname}_CondBit_208')
    model.addConstr(- point[0] + point[3] + point[7] - point[8] + point[9] + point[10] - point[11] + 1 >= 0,
                    f'{varname}_CondBit_209')
    model.addConstr(- point[1] + point[2] - point[10] + point[12] + 1 >= 0, f'{varname}_CondBit_210')
    model.addConstr(- point[3] - point[7] + point[11] + 1 >= 0, f'{varname}_CondBit_211')
    model.addConstr(
        - 3 * point[0] - point[4] - point[5] + point[7] - 2 * point[8] + 2 * point[9] + 2 * point[10] - point[
            12] + 4 >= 0, f'{varname}_CondBit_212')
    model.addConstr(- point[1] - point[4] - point[6] - point[7] + point[11] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_213')
    model.addConstr(point[12] >= 0, f'{varname}_CondBit_214')
    model.addConstr(- point[8] + point[11] >= 0, f'{varname}_CondBit_215')
    model.addConstr(
        - 2 * point[0] + point[3] - point[4] - point[5] + point[7] - point[8] + point[9] + 2 * point[10] - point[11] -
        point[12] + 3 >= 0, f'{varname}_CondBit_216')
    model.addConstr(- point[7] + 1 >= 0, f'{varname}_CondBit_217')
    model.addConstr(
        - point[0] - point[1] - 2 * point[4] - point[6] - point[7] + point[10] + point[11] + point[12] + 3 >= 0,
        f'{varname}_CondBit_218')
    model.addConstr(- point[1] + point[2] - point[4] + point[5] - point[7] - point[9] - point[10] + 3 >= 0,
                    f'{varname}_CondBit_219')
    model.addConstr(point[9] >= 0, f'{varname}_CondBit_220')
    model.addConstr(- point[1] - point[4] - point[8] + point[11] + point[12] + 1 >= 0, f'{varname}_CondBit_221')
    model.addConstr(- point[1] + point[2] - point[4] - point[6] - point[7] - point[10] + point[11] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_222')
    model.addConstr(- point[0] + point[3] - point[4] - point[8] + point[10] + point[11] - point[12] + 1 >= 0,
                    f'{varname}_CondBit_223')
    model.addConstr(
        - 2 * point[1] + point[2] - point[3] - point[4] + point[6] - 2 * point[8] - point[9] - point[10] + point[
            12] + 4 >= 0, f'{varname}_CondBit_224')
    model.addConstr(
        - point[1] + point[2] - point[4] + point[5] + point[6] - point[7] - point[10] - point[11] + point[12] + 2 >= 0,
        f'{varname}_CondBit_225')
    model.addConstr(- point[1] - point[2] - point[3] + point[5] - point[7] - point[10] + point[11] + 3 >= 0,
                    f'{varname}_CondBit_226')
    model.addConstr(- point[6] - point[7] + point[11] + 1 >= 0, f'{varname}_CondBit_227')
    model.addConstr(- point[0] - point[4] - point[6] - point[7] + point[10] + point[11] + 2 >= 0,
                    f'{varname}_CondBit_228')
    model.addConstr(
        - point[0] - point[1] - point[2] - point[3] - 2 * point[8] + point[10] + point[11] + point[12] + 3 >= 0,
        f'{varname}_CondBit_229')
    model.addConstr(
        - 3 * point[1] + point[2] - 2 * point[3] - point[4] - point[5] + point[6] - 2 * point[8] - point[9] - point[
            10] - point[11] + 2 * point[12] + 7 >= 0, f'{varname}_CondBit_230')
    model.addConstr(
        - point[0] + point[3] - point[4] - point[5] + point[6] - point[8] + point[9] + point[10] - point[11] + 2 >= 0,
        f'{varname}_CondBit_231')
    model.addConstr(
        - 2 * point[0] + point[3] - 2 * point[4] - 2 * point[8] + point[10] + 2 * point[11] - point[12] + 2 >= 0,
        f'{varname}_CondBit_232')
    model.addConstr(- 2 * point[1] - 2 * point[3] + point[6] - 2 * point[8] - point[9] - point[10] + point[12] + 5 >= 0,
                    f'{varname}_CondBit_233')
    model.addConstr(
        - 2 * point[0] + point[3] - point[4] - point[6] + point[7] - point[8] + point[9] + 2 * point[10] - point[11] -
        point[12] + 3 >= 0, f'{varname}_CondBit_234')
    model.addConstr(point[10] >= 0, f'{varname}_CondBit_235')
    model.addConstr(
        - point[1] + 2 * point[2] - point[4] + point[5] + 2 * point[6] - point[8] - point[9] - 2 * point[10] - point[
            11] + point[12] + 2 >= 0, f'{varname}_CondBit_236')
    model.addConstr(point[5] >= 0, f'{varname}_CondBit_237')
    model.addConstr(
        - point[0] + point[2] - point[4] - point[5] + point[7] - point[8] + point[9] + point[10] - point[11] + 2 >= 0,
        f'{varname}_CondBit_238')
    model.addConstr(
        - 2 * point[0] - point[4] - point[5] - point[8] + point[9] + point[10] + point[11] - point[12] + 3 >= 0,
        f'{varname}_CondBit_239')
    model.addConstr(
        - 3 * point[0] - 3 * point[1] - point[2] - point[3] - point[4] - point[5] - point[6] - point[7] - point[8] +
        point[9] + point[10] + point[12] + 9 >= 0, f'{varname}_CondBit_240')
    model.addConstr(
        - 2 * point[0] - point[2] - point[3] + point[5] - point[6] + point[7] - point[8] + point[9] + point[10] - point[
            12] + 4 >= 0, f'{varname}_CondBit_241')
    model.addConstr(
        - 3 * point[0] + point[3] - 2 * point[4] + point[5] - 2 * point[6] + point[7] - point[8] + point[9] + 2 * point[
            10] - point[11] - point[12] + 5 >= 0, f'{varname}_CondBit_242')
    model.addConstr(
        - 2 * point[0] - point[1] - point[2] - point[3] - point[6] - point[8] + point[9] + point[10] + 5 >= 0,
        f'{varname}_CondBit_243')
    model.addConstr(
        - 2 * point[0] - point[4] + point[7] - 2 * point[8] + point[9] + point[10] + point[11] - point[12] + 2 >= 0,
        f'{varname}_CondBit_244')
    model.addConstr(
        - 2 * point[0] - point[4] + point[5] - point[6] + point[7] - point[8] + point[9] + point[10] - point[
            12] + 3 >= 0, f'{varname}_CondBit_245')
    model.addConstr(
        - 2 * point[1] - point[4] - point[5] - point[8] + point[9] - point[10] + point[11] + point[12] + 3 >= 0,
        f'{varname}_CondBit_246')
    model.addConstr(
        - 2 * point[0] - point[2] - point[3] + point[7] - 2 * point[8] + point[9] + point[10] + point[11] - point[
            12] + 3 >= 0, f'{varname}_CondBit_247')
    model.addConstr(
        - 3 * point[0] - point[4] - point[6] + point[7] - 2 * point[8] + 2 * point[9] + 2 * point[10] - point[
            12] + 4 >= 0, f'{varname}_CondBit_248')
    model.addConstr(
        - 4 * point[0] + point[3] - 2 * point[4] - 2 * point[6] + point[7] - 2 * point[8] + 2 * point[9] + 3 * point[
            10] - point[11] - point[12] + 6 >= 0, f'{varname}_CondBit_249')
    model.addConstr(
        - 3 * point[0] - point[2] - point[3] - point[6] + point[7] - 2 * point[8] + 2 * point[9] + 2 * point[10] -
        point[12] + 5 >= 0, f'{varname}_CondBit_250')
    model.addConstr(- point[1] + point[2] - point[4] - point[5] + point[6] - point[11] + point[12] + 2 >= 0,
                    f'{varname}_CondBit_251')
    model.addConstr(
        - 4 * point[1] + point[2] - 2 * point[4] + point[6] - 2 * point[7] - 2 * point[8] + 2 * point[9] - point[10] -
        point[11] + 3 * point[12] + 6 >= 0, f'{varname}_CondBit_252')
    model.addConstr(
        - 2 * point[0] - point[2] - point[3] + point[4] - point[6] + point[7] - point[8] + point[9] + point[10] - point[
            12] + 4 >= 0, f'{varname}_CondBit_253')
    model.addConstr(point[2] + point[3] - point[4] + point[5] + point[6] - point[9] - point[10] - point[11] + 1 >= 0,
                    f'{varname}_CondBit_254')
    model.addConstr(- point[0] + point[3] - point[4] + point[5] - point[6] - point[9] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_255')
    model.addConstr(
        - 2 * point[0] - point[2] + point[3] - point[4] - point[5] - point[6] - point[7] - point[9] + point[10] - point[
            12] + 6 >= 0, f'{varname}_CondBit_256')
    model.addConstr(- point[0] - point[2] - point[6] - point[7] - point[9] + point[11] - point[12] + 4 >= 0,
                    f'{varname}_CondBit_257')
    model.addConstr(- point[1] - point[3] - point[6] - point[7] - point[9] - point[10] + point[11] + 4 >= 0,
                    f'{varname}_CondBit_258')
    model.addConstr(
        - 2 * point[1] + point[2] - point[3] - point[4] - point[5] - point[6] - point[7] - point[9] - point[10] + point[
            12] + 6 >= 0, f'{varname}_CondBit_259')
    model.addConstr(- point[0] - point[2] + point[4] + point[7] - point[8] - point[9] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_260')
    model.addConstr(
        - 2 * point[0] - point[2] + point[3] - point[4] - point[5] - point[8] - point[9] + point[10] - point[
            12] + 5 >= 0, f'{varname}_CondBit_261')
    model.addConstr(- point[0] - point[2] - point[8] - point[9] + point[11] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_262')
    model.addConstr(- 2 * point[0] - 2 * point[2] + point[7] - 2 * point[8] - point[9] + point[10] - point[12] + 5 >= 0,
                    f'{varname}_CondBit_263')
    model.addConstr(
        - 2 * point[0] - point[2] + point[3] - point[4] + point[7] - 2 * point[8] - point[9] + point[10] - point[
            12] + 4 >= 0, f'{varname}_CondBit_264')
    model.addConstr(
        - 3 * point[0] - 2 * point[2] + point[3] - point[4] - point[5] + point[7] - 2 * point[8] - point[9] + 2 * point[
            10] - point[11] - point[12] + 7 >= 0, f'{varname}_CondBit_265')
    model.addConstr(- point[0] - point[2] + point[5] + point[7] - point[8] - point[9] - point[12] + 3 >= 0,
                    f'{varname}_CondBit_266')


# Matching条件
def matching_conditions(in_points, delta_m, model, varname):
    model.addConstr(- in_points[0][1] + 1 >= 0, f'{varname}_Matching0')
    model.addConstr(- in_points[0][0] + in_points[0][1] >= 0, f'{varname}_Matching1')
    model.addConstr(in_points[0][1] - in_points[0][2] >= 0, f'{varname}_Matching2')
    model.addConstr(- in_points[1][1] + 1 >= 0, f'{varname}_Matching3')
    model.addConstr(- in_points[1][0] + in_points[1][1] >= 0, f'{varname}_Matching4')
    model.addConstr(in_points[1][1] - in_points[1][2] >= 0, f'{varname}_Matching5')
    model.addConstr(- in_points[2][1] + 1 >= 0, f'{varname}_Matching6')
    model.addConstr(delta_m >= 0, f'{varname}_Matching7')
    model.addConstr(- in_points[2][0] + in_points[2][1] >= 0, f'{varname}_Matching8')
    model.addConstr(- in_points[0][0] + in_points[0][1] - in_points[2][0] + in_points[2][1] - delta_m >= 0,
                    f'{varname}_Matching9')
    model.addConstr(- in_points[1][0] + in_points[1][1] - in_points[2][0] + in_points[2][1] - delta_m >= 0,
                    f'{varname}_Matching10')
    model.addConstr(in_points[1][1] - in_points[1][2] + in_points[2][1] - in_points[2][2] - delta_m >= 0,
                    f'{varname}_Matching11')
    model.addConstr(- in_points[2][0] + 2 * in_points[2][1] - in_points[2][2] - delta_m >= 0, f'{varname}_Matching12')
    model.addConstr(in_points[0][1] - in_points[0][2] + in_points[2][1] - in_points[2][2] - delta_m >= 0,
                    f'{varname}_Matching13')
    model.addConstr(in_points[2][1] - in_points[2][2] >= 0, f'{varname}_Matching14')
    model.addConstr(
        - in_points[0][0] + in_points[0][1] - in_points[0][2] - in_points[1][1] + in_points[2][0] - in_points[2][1] +
        in_points[2][2] + delta_m + 2 >= 0, f'{varname}_Matching15')
    model.addConstr(
        - in_points[0][0] - in_points[1][0] + in_points[2][0] - in_points[2][1] + in_points[2][2] + delta_m + 2 >= 0,
        f'{varname}_Matching16')
    model.addConstr(in_points[2][1] - delta_m >= 0, f'{varname}_Matching17')
    model.addConstr(in_points[0][1] - delta_m >= 0, f'{varname}_Matching18')
    model.addConstr(in_points[1][1] - delta_m >= 0, f'{varname}_Matching19')
    model.addConstr(in_points[2][2] >= 0, f'{varname}_Matching20')
    model.addConstr(in_points[1][0] + in_points[2][1] - in_points[2][2] - delta_m >= 0, f'{varname}_Matching21')
    model.addConstr(
        - in_points[0][1] - in_points[1][0] + in_points[1][1] - in_points[1][2] + in_points[2][0] - in_points[2][1] +
        in_points[2][2] + delta_m + 2 >= 0, f'{varname}_Matching22')
    model.addConstr(
        - in_points[0][2] - in_points[1][2] + in_points[2][0] - in_points[2][1] + in_points[2][2] + delta_m + 2 >= 0,
        f'{varname}_Matching23')
    model.addConstr(in_points[0][0] + in_points[1][2] - delta_m >= 0, f'{varname}_Matching24')
    model.addConstr(
        - in_points[0][0] + in_points[0][2] - in_points[1][0] + in_points[1][2] + in_points[2][0] - in_points[2][
            1] + delta_m + 2 >= 0, f'{varname}_Matching25')
    model.addConstr(in_points[1][0] >= 0, f'{varname}_Matching26')
    model.addConstr(in_points[1][2] - in_points[2][0] + in_points[2][1] - delta_m >= 0, f'{varname}_Matching27')
    model.addConstr(in_points[1][2] >= 0, f'{varname}_Matching28')
    model.addConstr(in_points[2][0] >= 0, f'{varname}_Matching29')
    model.addConstr(in_points[0][0] >= 0, f'{varname}_Matching30')
    model.addConstr(in_points[0][0] + in_points[2][1] - in_points[2][2] - delta_m >= 0, f'{varname}_Matching31')
    model.addConstr(
        in_points[0][0] - in_points[0][2] + in_points[1][0] - in_points[1][2] - in_points[2][1] + in_points[2][
            2] + delta_m + 2 >= 0, f'{varname}_Matching32')
    model.addConstr(in_points[0][2] - in_points[2][0] + in_points[2][1] - delta_m >= 0, f'{varname}_Matching33')
    model.addConstr(in_points[0][2] + in_points[1][0] - delta_m >= 0, f'{varname}_Matching34')
    model.addConstr(in_points[0][2] >= 0, f'{varname}_Matching35')


# 新的Matching
def new_matching_conditions(in_point, delta_m, model, varname):
    model.addConstr(- in_point[0][1] + 1 >= 0, f'{varname}_New_Matching_0')
    model.addConstr(- in_point[0][1] - in_point[1][1] - in_point[2][1] + delta_m + 2 >= 0, f'{varname}_New_Matching_1')
    model.addConstr(in_point[0][1] - in_point[0][2] >= 0, f'{varname}_New_Matching_2')
    model.addConstr(- in_point[0][0] + in_point[0][1] >= 0, f'{varname}_New_Matching_3')
    model.addConstr(in_point[2][1] - delta_m >= 0, f'{varname}_New_Matching_4')
    model.addConstr(- in_point[1][1] + 1 >= 0, f'{varname}_New_Matching_5')
    model.addConstr(in_point[1][1] - in_point[1][2] >= 0, f'{varname}_New_Matching_6')
    model.addConstr(- in_point[1][0] + in_point[1][1] >= 0, f'{varname}_New_Matching_7')
    model.addConstr(in_point[0][1] - delta_m >= 0, f'{varname}_New_Matching_8')
    model.addConstr(- in_point[2][1] + 1 >= 0, f'{varname}_New_Matching_9')
    model.addConstr(in_point[1][1] - delta_m >= 0, f'{varname}_New_Matching_10')
    model.addConstr(- in_point[2][0] + in_point[2][1] >= 0, f'{varname}_New_Matching_11')
    model.addConstr(in_point[2][1] - in_point[2][2] >= 0, f'{varname}_New_Matching_12')
    model.addConstr(delta_m >= 0, f'{varname}_New_Matching_13')
    model.addConstr(in_point[2][0] >= 0, f'{varname}_New_Matching_14')
    model.addConstr(in_point[0][0] >= 0, f'{varname}_New_Matching_15')
    model.addConstr(in_point[1][2] >= 0, f'{varname}_New_Matching_16')
    model.addConstr(in_point[2][2] >= 0, f'{varname}_New_Matching_17')
    model.addConstr(in_point[0][2] >= 0, f'{varname}_New_Matching_18')
    model.addConstr(in_point[1][0] >= 0, f'{varname}_New_Matching_19')


# 固定起始状态
def fixed_ini_state(model, r0_state):
    blue_set = [r0_state[2][2][4], r0_state[3][2][4], r0_state[2][2][13], r0_state[3][2][13], r0_state[2][2][22],
                r0_state[3][2][22], r0_state[2][2][27], r0_state[3][2][27]]
    red_set = [r0_state[0][2][0], r0_state[1][2][0], r0_state[2][2][0], r0_state[3][2][0], r0_state[0][2][1],
               r0_state[1][2][1], r0_state[2][2][1], r0_state[3][2][1], r0_state[0][2][2], r0_state[1][2][2],
               r0_state[2][2][2], r0_state[3][2][2], r0_state[0][2][3], r0_state[1][2][3], r0_state[2][2][3],
               r0_state[3][2][3], r0_state[0][2][4], r0_state[1][2][4], r0_state[0][2][5], r0_state[1][2][5],
               r0_state[2][2][5], r0_state[3][2][5], r0_state[0][2][6], r0_state[1][2][6], r0_state[2][2][6],
               r0_state[3][2][6], r0_state[0][2][7], r0_state[1][2][7], r0_state[2][2][7], r0_state[3][2][7],
               r0_state[0][2][8], r0_state[1][2][8], r0_state[2][2][8], r0_state[3][2][8], r0_state[0][2][9],
               r0_state[1][2][9], r0_state[2][2][9], r0_state[3][2][9], r0_state[0][2][10], r0_state[1][2][10],
               r0_state[2][2][10], r0_state[3][2][10], r0_state[0][2][11], r0_state[1][2][11], r0_state[2][2][11],
               r0_state[3][2][11], r0_state[0][2][12], r0_state[1][2][12], r0_state[2][2][12], r0_state[3][2][12],
               r0_state[0][2][13], r0_state[1][2][13], r0_state[0][2][14], r0_state[1][2][14], r0_state[2][2][14],
               r0_state[3][2][14], r0_state[0][2][15], r0_state[1][2][15], r0_state[2][2][15], r0_state[3][2][15],
               r0_state[0][2][16], r0_state[1][2][16], r0_state[2][2][16], r0_state[3][2][16], r0_state[0][2][17],
               r0_state[1][2][17], r0_state[2][2][17], r0_state[3][2][17], r0_state[0][2][18], r0_state[1][2][18],
               r0_state[2][2][18], r0_state[3][2][18], r0_state[0][2][19], r0_state[1][2][19], r0_state[2][2][19],
               r0_state[3][2][19], r0_state[0][2][20], r0_state[1][2][20], r0_state[2][2][20], r0_state[3][2][20],
               r0_state[0][2][21], r0_state[1][2][21], r0_state[2][2][21], r0_state[3][2][21], r0_state[0][2][22],
               r0_state[1][2][22], r0_state[0][2][23], r0_state[1][2][23], r0_state[2][2][23], r0_state[3][2][23],
               r0_state[0][2][24], r0_state[1][2][24], r0_state[2][2][24], r0_state[3][2][24], r0_state[0][2][25],
               r0_state[1][2][25], r0_state[2][2][25], r0_state[3][2][25], r0_state[0][2][26], r0_state[1][2][26],
               r0_state[2][2][26], r0_state[3][2][26], r0_state[0][2][27], r0_state[1][2][27], r0_state[0][2][28],
               r0_state[1][2][28], r0_state[2][2][28], r0_state[3][2][28], r0_state[0][2][29], r0_state[1][2][29],
               r0_state[2][2][29], r0_state[3][2][29], r0_state[0][2][30], r0_state[1][2][30], r0_state[2][2][30],
               r0_state[3][2][30], r0_state[0][2][31], r0_state[1][2][31]]
    grey_set = [r0_state[0][0][0], r0_state[1][0][0], r0_state[2][0][0], r0_state[3][0][0], r0_state[0][1][0],
                r0_state[1][1][0], r0_state[2][1][0], r0_state[3][1][0], r0_state[0][0][1], r0_state[1][0][1],
                r0_state[2][0][1], r0_state[3][0][1], r0_state[0][1][1], r0_state[1][1][1], r0_state[2][1][1],
                r0_state[3][1][1], r0_state[0][0][2], r0_state[1][0][2], r0_state[2][0][2], r0_state[3][0][2],
                r0_state[0][1][2], r0_state[1][1][2], r0_state[2][1][2], r0_state[3][1][2], r0_state[0][0][3],
                r0_state[1][0][3], r0_state[2][0][3], r0_state[3][0][3], r0_state[0][1][3], r0_state[1][1][3],
                r0_state[2][1][3], r0_state[3][1][3], r0_state[0][0][4], r0_state[1][0][4], r0_state[2][0][4],
                r0_state[3][0][4], r0_state[0][1][4], r0_state[1][1][4], r0_state[2][1][4], r0_state[3][1][4],
                r0_state[0][0][5], r0_state[1][0][5], r0_state[2][0][5], r0_state[3][0][5], r0_state[0][1][5],
                r0_state[1][1][5], r0_state[2][1][5], r0_state[3][1][5], r0_state[0][0][6], r0_state[1][0][6],
                r0_state[2][0][6], r0_state[3][0][6], r0_state[0][1][6], r0_state[1][1][6], r0_state[2][1][6],
                r0_state[3][1][6], r0_state[0][0][7], r0_state[1][0][7], r0_state[2][0][7], r0_state[3][0][7],
                r0_state[0][1][7], r0_state[1][1][7], r0_state[2][1][7], r0_state[3][1][7], r0_state[0][0][8],
                r0_state[1][0][8], r0_state[2][0][8], r0_state[3][0][8], r0_state[0][1][8], r0_state[1][1][8],
                r0_state[2][1][8], r0_state[3][1][8], r0_state[0][0][9], r0_state[1][0][9], r0_state[2][0][9],
                r0_state[3][0][9], r0_state[0][1][9], r0_state[1][1][9], r0_state[2][1][9], r0_state[3][1][9],
                r0_state[0][0][10], r0_state[1][0][10], r0_state[2][0][10], r0_state[3][0][10], r0_state[0][1][10],
                r0_state[1][1][10], r0_state[2][1][10], r0_state[3][1][10], r0_state[0][0][11], r0_state[1][0][11],
                r0_state[2][0][11], r0_state[3][0][11], r0_state[0][1][11], r0_state[1][1][11], r0_state[2][1][11],
                r0_state[3][1][11], r0_state[0][0][12], r0_state[1][0][12], r0_state[2][0][12], r0_state[3][0][12],
                r0_state[0][1][12], r0_state[1][1][12], r0_state[2][1][12], r0_state[3][1][12], r0_state[0][0][13],
                r0_state[1][0][13], r0_state[2][0][13], r0_state[3][0][13], r0_state[0][1][13], r0_state[1][1][13],
                r0_state[2][1][13], r0_state[3][1][13], r0_state[0][0][14], r0_state[1][0][14], r0_state[2][0][14],
                r0_state[3][0][14], r0_state[0][1][14], r0_state[1][1][14], r0_state[2][1][14], r0_state[3][1][14],
                r0_state[0][0][15], r0_state[1][0][15], r0_state[2][0][15], r0_state[3][0][15], r0_state[0][1][15],
                r0_state[1][1][15], r0_state[2][1][15], r0_state[3][1][15], r0_state[0][0][16], r0_state[1][0][16],
                r0_state[2][0][16], r0_state[3][0][16], r0_state[0][1][16], r0_state[1][1][16], r0_state[2][1][16],
                r0_state[3][1][16], r0_state[0][0][17], r0_state[1][0][17], r0_state[2][0][17], r0_state[3][0][17],
                r0_state[0][1][17], r0_state[1][1][17], r0_state[2][1][17], r0_state[3][1][17], r0_state[0][0][18],
                r0_state[1][0][18], r0_state[2][0][18], r0_state[3][0][18], r0_state[0][1][18], r0_state[1][1][18],
                r0_state[2][1][18], r0_state[3][1][18], r0_state[0][0][19], r0_state[1][0][19], r0_state[2][0][19],
                r0_state[3][0][19], r0_state[0][1][19], r0_state[1][1][19], r0_state[2][1][19], r0_state[3][1][19],
                r0_state[0][0][20], r0_state[1][0][20], r0_state[2][0][20], r0_state[3][0][20], r0_state[0][1][20],
                r0_state[1][1][20], r0_state[2][1][20], r0_state[3][1][20], r0_state[0][0][21], r0_state[1][0][21],
                r0_state[2][0][21], r0_state[3][0][21], r0_state[0][1][21], r0_state[1][1][21], r0_state[2][1][21],
                r0_state[3][1][21], r0_state[0][0][22], r0_state[1][0][22], r0_state[2][0][22], r0_state[3][0][22],
                r0_state[0][1][22], r0_state[1][1][22], r0_state[2][1][22], r0_state[3][1][22], r0_state[0][0][23],
                r0_state[1][0][23], r0_state[2][0][23], r0_state[3][0][23], r0_state[0][1][23], r0_state[1][1][23],
                r0_state[2][1][23], r0_state[3][1][23], r0_state[0][0][24], r0_state[1][0][24], r0_state[2][0][24],
                r0_state[3][0][24], r0_state[0][1][24], r0_state[1][1][24], r0_state[2][1][24], r0_state[3][1][24],
                r0_state[0][0][25], r0_state[1][0][25], r0_state[2][0][25], r0_state[3][0][25], r0_state[0][1][25],
                r0_state[1][1][25], r0_state[2][1][25], r0_state[3][1][25], r0_state[0][0][26], r0_state[1][0][26],
                r0_state[2][0][26], r0_state[3][0][26], r0_state[0][1][26], r0_state[1][1][26], r0_state[2][1][26],
                r0_state[3][1][26], r0_state[0][0][27], r0_state[1][0][27], r0_state[2][0][27], r0_state[3][0][27],
                r0_state[0][1][27], r0_state[1][1][27], r0_state[2][1][27], r0_state[3][1][27], r0_state[0][0][28],
                r0_state[1][0][28], r0_state[2][0][28], r0_state[3][0][28], r0_state[0][1][28], r0_state[1][1][28],
                r0_state[2][1][28], r0_state[3][1][28], r0_state[0][0][29], r0_state[1][0][29], r0_state[2][0][29],
                r0_state[3][0][29], r0_state[0][1][29], r0_state[1][1][29], r0_state[2][1][29], r0_state[3][1][29],
                r0_state[0][0][30], r0_state[1][0][30], r0_state[2][0][30], r0_state[3][0][30], r0_state[0][1][30],
                r0_state[1][1][30], r0_state[2][1][30], r0_state[3][1][30], r0_state[0][0][31], r0_state[1][0][31],
                r0_state[2][0][31], r0_state[3][0][31], r0_state[0][1][31], r0_state[1][1][31], r0_state[2][1][31],
                r0_state[3][1][31], r0_state[2][2][31], r0_state[3][2][31]]
    for i in range(len(grey_set)):
        model.addConstr(grey_set[i][0] == 1, f'grey_{i}_cons0')
        model.addConstr(grey_set[i][1] == 1, f'grey_{i}_cons1')
        model.addConstr(grey_set[i][2] == 1, f'grey_{i}_cons2')
    for i in range(len(blue_set)):
        model.addConstr(blue_set[i][0] == 1, f'blue_{i}_cons0')
        model.addConstr(blue_set[i][1] == 1, f'blue_{i}_cons1')
        model.addConstr(blue_set[i][2] == 0, f'blue_{i}_cons2')
    for i in range(len(red_set)):
        model.addConstr(red_set[i][0] == 0, f'red_{i}_cons0')
        model.addConstr(red_set[i][1] == 1, f'red_{i}_cons1')
        model.addConstr(red_set[i][2] == 1, f'red_{i}_cons2')


# CondSBox条件指示
def bit_cond(model, point, varname):
    model.addConstr(point[2] >= 0, f'{varname}_CondBit_0')
    model.addConstr(point[0] - point[2] >= 0, f'{varname}_CondBit_1')
    model.addConstr(point[1] - point[2] >= 0, f'{varname}_CondBit_2')
    model.addConstr(- point[0] + 1 >= 0, f'{varname}_CondBit_3')
    model.addConstr(- point[1] + 1 >= 0, f'{varname}_CondBit_4')


# 多匹配器选择
def fixed_matching(model, delta_m):
    set1 = [delta_m[1][5], delta_m[1][19], delta_m[1][28]]
    for i in range(len(set1)):
        model.addConstr(set1[i] == 0, f'delta_m1_{i}_cond')


def partial_fixed_ini_state_10_dom_10(model, r0_state, lambda_r):
    blue_set = [r0_state[2][2][3], r0_state[3][2][3], r0_state[3][2][8], r0_state[2][2][12], r0_state[3][2][12],
                r0_state[3][2][17], r0_state[2][2][21], r0_state[3][2][21], r0_state[3][2][26], r0_state[2][2][30]]
    free_set = [r0_state[0][2][0], r0_state[1][2][0], r0_state[2][2][0], r0_state[3][2][0], r0_state[0][2][1],
                r0_state[1][2][1], r0_state[2][2][1], r0_state[3][2][1], r0_state[0][2][2], r0_state[1][2][2],
                r0_state[2][2][2], r0_state[3][2][2], r0_state[0][2][3], r0_state[1][2][3], r0_state[0][2][4],
                r0_state[1][2][4], r0_state[2][2][4], r0_state[3][2][4], r0_state[0][2][5], r0_state[1][2][5],
                r0_state[2][2][5], r0_state[3][2][5], r0_state[0][2][6], r0_state[1][2][6], r0_state[2][2][6],
                r0_state[3][2][6], r0_state[0][2][7], r0_state[1][2][7], r0_state[2][2][7], r0_state[3][2][7],
                r0_state[0][2][8], r0_state[1][2][8], r0_state[2][2][8], r0_state[0][2][9], r0_state[1][2][9],
                r0_state[2][2][9], r0_state[3][2][9], r0_state[0][2][10], r0_state[1][2][10], r0_state[2][2][10],
                r0_state[3][2][10], r0_state[0][2][11], r0_state[1][2][11], r0_state[2][2][11], r0_state[3][2][11],
                r0_state[0][2][12], r0_state[1][2][12], r0_state[0][2][13], r0_state[1][2][13], r0_state[2][2][13],
                r0_state[3][2][13], r0_state[0][2][14], r0_state[1][2][14], r0_state[2][2][14], r0_state[3][2][14],
                r0_state[0][2][15], r0_state[1][2][15], r0_state[2][2][15], r0_state[3][2][15], r0_state[0][2][16],
                r0_state[1][2][16], r0_state[2][2][16], r0_state[3][2][16], r0_state[0][2][17], r0_state[1][2][17],
                r0_state[2][2][17], r0_state[0][2][18], r0_state[1][2][18], r0_state[2][2][18], r0_state[3][2][18],
                r0_state[0][2][19], r0_state[1][2][19], r0_state[2][2][19], r0_state[3][2][19], r0_state[0][2][20],
                r0_state[1][2][20], r0_state[2][2][20], r0_state[3][2][20], r0_state[0][2][21], r0_state[1][2][21],
                r0_state[0][2][22], r0_state[1][2][22], r0_state[2][2][22], r0_state[3][2][22], r0_state[0][2][23],
                r0_state[1][2][23], r0_state[2][2][23], r0_state[3][2][23], r0_state[0][2][24], r0_state[1][2][24],
                r0_state[2][2][24], r0_state[3][2][24], r0_state[0][2][25], r0_state[1][2][25], r0_state[2][2][25],
                r0_state[3][2][25], r0_state[0][2][26], r0_state[1][2][26], r0_state[2][2][26], r0_state[0][2][27],
                r0_state[1][2][27], r0_state[2][2][27], r0_state[3][2][27], r0_state[0][2][28], r0_state[1][2][28],
                r0_state[2][2][28], r0_state[3][2][28], r0_state[0][2][29], r0_state[1][2][29], r0_state[2][2][29],
                r0_state[3][2][29], r0_state[0][2][30], r0_state[1][2][30], r0_state[3][2][30], r0_state[0][2][31],
                r0_state[1][2][31], r0_state[2][2][31], r0_state[3][2][31]]
    fixed_lambda_r = [lambda_r[2][2][3], lambda_r[3][2][3], lambda_r[3][2][8], lambda_r[2][2][12], lambda_r[3][2][12],
                lambda_r[3][2][17], lambda_r[2][2][21], lambda_r[3][2][21], lambda_r[3][2][26], lambda_r[2][2][30]]
    free_lambda_r = [lambda_r[0][2][0], lambda_r[1][2][0], lambda_r[2][2][0], lambda_r[3][2][0], lambda_r[0][2][1],
                lambda_r[1][2][1], lambda_r[2][2][1], lambda_r[3][2][1], lambda_r[0][2][2], lambda_r[1][2][2],
                lambda_r[2][2][2], lambda_r[3][2][2], lambda_r[0][2][3], lambda_r[1][2][3], lambda_r[0][2][4],
                lambda_r[1][2][4], lambda_r[2][2][4], lambda_r[3][2][4], lambda_r[0][2][5], lambda_r[1][2][5],
                lambda_r[2][2][5], lambda_r[3][2][5], lambda_r[0][2][6], lambda_r[1][2][6], lambda_r[2][2][6],
                lambda_r[3][2][6], lambda_r[0][2][7], lambda_r[1][2][7], lambda_r[2][2][7], lambda_r[3][2][7],
                lambda_r[0][2][8], lambda_r[1][2][8], lambda_r[2][2][8], lambda_r[0][2][9], lambda_r[1][2][9],
                lambda_r[2][2][9], lambda_r[3][2][9], lambda_r[0][2][10], lambda_r[1][2][10], lambda_r[2][2][10],
                lambda_r[3][2][10], lambda_r[0][2][11], lambda_r[1][2][11], lambda_r[2][2][11], lambda_r[3][2][11],
                lambda_r[0][2][12], lambda_r[1][2][12], lambda_r[0][2][13], lambda_r[1][2][13], lambda_r[2][2][13],
                lambda_r[3][2][13], lambda_r[0][2][14], lambda_r[1][2][14], lambda_r[2][2][14], lambda_r[3][2][14],
                lambda_r[0][2][15], lambda_r[1][2][15], lambda_r[2][2][15], lambda_r[3][2][15], lambda_r[0][2][16],
                lambda_r[1][2][16], lambda_r[2][2][16], lambda_r[3][2][16], lambda_r[0][2][17], lambda_r[1][2][17],
                lambda_r[2][2][17], lambda_r[0][2][18], lambda_r[1][2][18], lambda_r[2][2][18], lambda_r[3][2][18],
                lambda_r[0][2][19], lambda_r[1][2][19], lambda_r[2][2][19], lambda_r[3][2][19], lambda_r[0][2][20],
                lambda_r[1][2][20], lambda_r[2][2][20], lambda_r[3][2][20], lambda_r[0][2][21], lambda_r[1][2][21],
                lambda_r[0][2][22], lambda_r[1][2][22], lambda_r[2][2][22], lambda_r[3][2][22], lambda_r[0][2][23],
                lambda_r[1][2][23], lambda_r[2][2][23], lambda_r[3][2][23], lambda_r[0][2][24], lambda_r[1][2][24],
                lambda_r[2][2][24], lambda_r[3][2][24], lambda_r[0][2][25], lambda_r[1][2][25], lambda_r[2][2][25],
                lambda_r[3][2][25], lambda_r[0][2][26], lambda_r[1][2][26], lambda_r[2][2][26], lambda_r[0][2][27],
                lambda_r[1][2][27], lambda_r[2][2][27], lambda_r[3][2][27], lambda_r[0][2][28], lambda_r[1][2][28],
                lambda_r[2][2][28], lambda_r[3][2][28], lambda_r[0][2][29], lambda_r[1][2][29], lambda_r[2][2][29],
                lambda_r[3][2][29], lambda_r[0][2][30], lambda_r[1][2][30], lambda_r[3][2][30], lambda_r[0][2][31],
                lambda_r[1][2][31], lambda_r[2][2][31], lambda_r[3][2][31]]
    for i in range(len(blue_set)):
        model.addConstr(blue_set[i][0] == 1, f'blue_set_{i}_cons0')
        model.addConstr(blue_set[i][1] == 1, f'blue_set_{i}_cons1')
        model.addConstr(blue_set[i][2] == 0, f'blue_set_{i}_cons2')
        model.addConstr(fixed_lambda_r[i] == 0, f'lambda_r_{i}_cons0')
    for i in range(len(free_set)):
        model.addConstr(free_set[i][0] + free_lambda_r[i] == 1, f'free_set_{i}_cons0')
        model.addConstr(free_set[i][1] == 1, f'free_set_{i}_cons1')
        model.addConstr(free_set[i][2] == 1, f'free_set_{i}_cons2')
    for k in range(Z):
        for i in range(X):
            model.addConstr(r0_state[i][0][k][0] == 1, f'free_set_{i}_cons0')
            model.addConstr(r0_state[i][0][k][1] == 1, f'free_set_{i}_cons1')
            model.addConstr(r0_state[i][0][k][2] == 1, f'free_set_{i}_cons2')
            model.addConstr(r0_state[i][1][k][0] == 1, f'free_set_{i}_cons0')
            model.addConstr(r0_state[i][1][k][1] == 1, f'free_set_{i}_cons1')
            model.addConstr(r0_state[i][1][k][2] == 1, f'free_set_{i}_cons2')
            model.addConstr(lambda_r[i][0][k] == 0, f'lambda_r_{i}_cons1')
            model.addConstr(lambda_r[i][1][k] == 0, f'lambda_r_{i}_cons2')


def fixed_ini_state_12_dom_6(model, r0_state):
    blue_set = [r0_state[1][2][1], r0_state[2][2][1], r0_state[0][2][5], r0_state[1][2][10], r0_state[2][2][10],
                r0_state[2][2][15], r0_state[0][2][19], r0_state[1][2][19], r0_state[1][2][24], r0_state[2][2][24],
                r0_state[0][2][28], r0_state[1][2][28]]
    grey_set = [r0_state[0][0][0], r0_state[1][0][0], r0_state[2][0][0], r0_state[3][0][0], r0_state[0][1][0],
                r0_state[1][1][0], r0_state[2][1][0], r0_state[3][1][0], r0_state[0][0][1], r0_state[1][0][1],
                r0_state[2][0][1], r0_state[3][0][1], r0_state[0][1][1], r0_state[1][1][1], r0_state[2][1][1],
                r0_state[3][1][1], r0_state[0][0][2], r0_state[1][0][2], r0_state[2][0][2], r0_state[3][0][2],
                r0_state[0][1][2], r0_state[1][1][2], r0_state[2][1][2], r0_state[3][1][2], r0_state[0][2][2],
                r0_state[0][0][3], r0_state[1][0][3], r0_state[2][0][3], r0_state[3][0][3], r0_state[0][1][3],
                r0_state[1][1][3], r0_state[2][1][3], r0_state[3][1][3], r0_state[0][2][3], r0_state[1][2][3],
                r0_state[0][0][4], r0_state[1][0][4], r0_state[2][0][4], r0_state[3][0][4], r0_state[0][1][4],
                r0_state[1][1][4], r0_state[2][1][4], r0_state[3][1][4], r0_state[0][2][4], r0_state[0][0][5],
                r0_state[1][0][5], r0_state[2][0][5], r0_state[3][0][5], r0_state[0][1][5], r0_state[1][1][5],
                r0_state[2][1][5], r0_state[3][1][5], r0_state[2][2][5], r0_state[0][0][6], r0_state[1][0][6],
                r0_state[2][0][6], r0_state[3][0][6], r0_state[0][1][6], r0_state[1][1][6], r0_state[2][1][6],
                r0_state[3][1][6], r0_state[0][0][7], r0_state[1][0][7], r0_state[2][0][7], r0_state[3][0][7],
                r0_state[0][1][7], r0_state[1][1][7], r0_state[2][1][7], r0_state[3][1][7], r0_state[0][2][7],
                r0_state[1][2][7], r0_state[0][0][8], r0_state[1][0][8], r0_state[2][0][8], r0_state[3][0][8],
                r0_state[0][1][8], r0_state[1][1][8], r0_state[2][1][8], r0_state[3][1][8], r0_state[2][2][8],
                r0_state[3][2][8], r0_state[0][0][9], r0_state[1][0][9], r0_state[2][0][9], r0_state[3][0][9],
                r0_state[0][1][9], r0_state[1][1][9], r0_state[2][1][9], r0_state[3][1][9], r0_state[0][0][10],
                r0_state[1][0][10], r0_state[2][0][10], r0_state[3][0][10], r0_state[0][1][10], r0_state[1][1][10],
                r0_state[2][1][10], r0_state[3][1][10], r0_state[0][0][11], r0_state[1][0][11], r0_state[2][0][11],
                r0_state[3][0][11], r0_state[0][1][11], r0_state[1][1][11], r0_state[2][1][11], r0_state[3][1][11],
                r0_state[0][0][12], r0_state[1][0][12], r0_state[2][0][12], r0_state[3][0][12], r0_state[0][1][12],
                r0_state[1][1][12], r0_state[2][1][12], r0_state[3][1][12], r0_state[0][2][12], r0_state[1][2][12],
                r0_state[0][0][13], r0_state[1][0][13], r0_state[2][0][13], r0_state[3][0][13], r0_state[0][1][13],
                r0_state[1][1][13], r0_state[2][1][13], r0_state[3][1][13], r0_state[0][2][13], r0_state[0][0][14],
                r0_state[1][0][14], r0_state[2][0][14], r0_state[3][0][14], r0_state[0][1][14], r0_state[1][1][14],
                r0_state[2][1][14], r0_state[3][1][14], r0_state[0][0][15], r0_state[1][0][15], r0_state[2][0][15],
                r0_state[3][0][15], r0_state[0][1][15], r0_state[1][1][15], r0_state[2][1][15], r0_state[3][1][15],
                r0_state[0][0][16], r0_state[1][0][16], r0_state[2][0][16], r0_state[3][0][16], r0_state[0][1][16],
                r0_state[1][1][16], r0_state[2][1][16], r0_state[3][1][16], r0_state[0][2][16], r0_state[1][2][16],
                r0_state[0][0][17], r0_state[1][0][17], r0_state[2][0][17], r0_state[3][0][17], r0_state[0][1][17],
                r0_state[1][1][17], r0_state[2][1][17], r0_state[3][1][17], r0_state[1][2][17], r0_state[2][2][17],
                r0_state[0][0][18], r0_state[1][0][18], r0_state[2][0][18], r0_state[3][0][18], r0_state[0][1][18],
                r0_state[1][1][18], r0_state[2][1][18], r0_state[3][1][18], r0_state[0][0][19], r0_state[1][0][19],
                r0_state[2][0][19], r0_state[3][0][19], r0_state[0][1][19], r0_state[1][1][19], r0_state[2][1][19],
                r0_state[3][1][19], r0_state[2][2][19], r0_state[0][0][20], r0_state[1][0][20], r0_state[2][0][20],
                r0_state[3][0][20], r0_state[0][1][20], r0_state[1][1][20], r0_state[2][1][20], r0_state[3][1][20],
                r0_state[0][0][21], r0_state[1][0][21], r0_state[2][0][21], r0_state[3][0][21], r0_state[0][1][21],
                r0_state[1][1][21], r0_state[2][1][21], r0_state[3][1][21], r0_state[0][2][21], r0_state[1][2][21],
                r0_state[0][0][22], r0_state[1][0][22], r0_state[2][0][22], r0_state[3][0][22], r0_state[0][1][22],
                r0_state[1][1][22], r0_state[2][1][22], r0_state[3][1][22], r0_state[3][2][22], r0_state[0][0][23],
                r0_state[1][0][23], r0_state[2][0][23], r0_state[3][0][23], r0_state[0][1][23], r0_state[1][1][23],
                r0_state[2][1][23], r0_state[3][1][23], r0_state[0][0][24], r0_state[1][0][24], r0_state[2][0][24],
                r0_state[3][0][24], r0_state[0][1][24], r0_state[1][1][24], r0_state[2][1][24], r0_state[3][1][24],
                r0_state[0][0][25], r0_state[1][0][25], r0_state[2][0][25], r0_state[3][0][25], r0_state[0][1][25],
                r0_state[1][1][25], r0_state[2][1][25], r0_state[3][1][25], r0_state[0][2][25], r0_state[0][0][26],
                r0_state[1][0][26], r0_state[2][0][26], r0_state[3][0][26], r0_state[0][1][26], r0_state[1][1][26],
                r0_state[2][1][26], r0_state[3][1][26], r0_state[0][2][26], r0_state[1][2][26], r0_state[2][2][26],
                r0_state[0][0][27], r0_state[1][0][27], r0_state[2][0][27], r0_state[3][0][27], r0_state[0][1][27],
                r0_state[1][1][27], r0_state[2][1][27], r0_state[3][1][27], r0_state[0][0][28], r0_state[1][0][28],
                r0_state[2][0][28], r0_state[3][0][28], r0_state[0][1][28], r0_state[1][1][28], r0_state[2][1][28],
                r0_state[3][1][28], r0_state[2][2][28], r0_state[0][0][29], r0_state[1][0][29], r0_state[2][0][29],
                r0_state[3][0][29], r0_state[0][1][29], r0_state[1][1][29], r0_state[2][1][29], r0_state[3][1][29],
                r0_state[0][0][30], r0_state[1][0][30], r0_state[2][0][30], r0_state[3][0][30], r0_state[0][1][30],
                r0_state[1][1][30], r0_state[2][1][30], r0_state[3][1][30], r0_state[0][2][30], r0_state[1][2][30],
                r0_state[0][0][31], r0_state[1][0][31], r0_state[2][0][31], r0_state[3][0][31], r0_state[0][1][31],
                r0_state[1][1][31], r0_state[2][1][31], r0_state[3][1][31], r0_state[3][2][31]]
    red_set = [r0_state[0][2][0], r0_state[1][2][0], r0_state[2][2][0], r0_state[3][2][0], r0_state[0][2][1],
               r0_state[3][2][1], r0_state[1][2][2], r0_state[2][2][2], r0_state[3][2][2], r0_state[2][2][3],
               r0_state[3][2][3], r0_state[1][2][4], r0_state[2][2][4], r0_state[3][2][4], r0_state[1][2][5],
               r0_state[3][2][5], r0_state[0][2][6], r0_state[1][2][6], r0_state[2][2][6], r0_state[3][2][6],
               r0_state[2][2][7], r0_state[3][2][7], r0_state[0][2][8], r0_state[1][2][8], r0_state[0][2][9],
               r0_state[1][2][9], r0_state[2][2][9], r0_state[3][2][9], r0_state[0][2][10], r0_state[3][2][10],
               r0_state[0][2][11], r0_state[1][2][11], r0_state[2][2][11], r0_state[3][2][11], r0_state[2][2][12],
               r0_state[3][2][12], r0_state[1][2][13], r0_state[2][2][13], r0_state[3][2][13], r0_state[0][2][14],
               r0_state[1][2][14], r0_state[2][2][14], r0_state[3][2][14], r0_state[0][2][15], r0_state[1][2][15],
               r0_state[3][2][15], r0_state[2][2][16], r0_state[3][2][16], r0_state[0][2][17], r0_state[3][2][17],
               r0_state[0][2][18], r0_state[1][2][18], r0_state[2][2][18], r0_state[3][2][18], r0_state[3][2][19],
               r0_state[0][2][20], r0_state[1][2][20], r0_state[2][2][20], r0_state[3][2][20], r0_state[2][2][21],
               r0_state[3][2][21], r0_state[0][2][22], r0_state[1][2][22], r0_state[2][2][22], r0_state[0][2][23],
               r0_state[1][2][23], r0_state[2][2][23], r0_state[3][2][23], r0_state[0][2][24], r0_state[3][2][24],
               r0_state[1][2][25], r0_state[2][2][25], r0_state[3][2][25], r0_state[3][2][26], r0_state[0][2][27],
               r0_state[1][2][27], r0_state[2][2][27], r0_state[3][2][27], r0_state[3][2][28], r0_state[0][2][29],
               r0_state[1][2][29], r0_state[2][2][29], r0_state[3][2][29], r0_state[2][2][30], r0_state[3][2][30],
               r0_state[0][2][31], r0_state[1][2][31], r0_state[2][2][31]]
    for i in range(len(grey_set)):
        model.addConstr(grey_set[i][0] == 1, f'grey_{i}_cons0')
        model.addConstr(grey_set[i][1] == 1, f'grey_{i}_cons1')
        model.addConstr(grey_set[i][2] == 1, f'grey_{i}_cons2')
    for i in range(len(blue_set)):
        model.addConstr(blue_set[i][0] == 1, f'blue_{i}_cons0')
        model.addConstr(blue_set[i][1] == 1, f'blue_{i}_cons1')
        model.addConstr(blue_set[i][2] == 0, f'blue_{i}_cons2')
    for i in range(len(red_set)):
        model.addConstr(red_set[i][0] == 0, f'red_{i}_cons0')
        model.addConstr(red_set[i][1] == 1, f'red_{i}_cons1')
        model.addConstr(red_set[i][2] == 1, f'red_{i}_cons2')


def cond_and_value_conditions_dof_12_dom_6(model, conds0, values0):
    true_set = [conds0[0][2][15], conds0[0][2][24], conds0[1][0][3], conds0[1][0][7], conds0[1][0][12],
                conds0[1][0][21], conds0[1][0][30], conds0[2][0][3], conds0[2][0][8], conds0[2][0][12],
                conds0[2][0][17], conds0[2][0][21], conds0[2][0][26], conds0[2][1][8], conds0[2][1][12],
                conds0[2][1][17], conds0[2][1][21], conds0[2][1][26], conds0[3][0][10], conds0[3][0][31],
                conds0[3][1][8], conds0[3][1][17], conds0[3][1][26], conds0[3][1][31], conds0[3][2][1],
                conds0[3][2][10]]
    false_set = [conds0[0][0][0], conds0[0][0][1], conds0[0][0][2], conds0[0][0][3], conds0[0][0][4], conds0[0][0][5],
                 conds0[0][0][6], conds0[0][0][7], conds0[0][0][8], conds0[0][0][9], conds0[0][0][10], conds0[0][0][11],
                 conds0[0][0][12], conds0[0][0][13], conds0[0][0][14], conds0[0][0][15], conds0[0][0][16],
                 conds0[0][0][17], conds0[0][0][18], conds0[0][0][19], conds0[0][0][20], conds0[0][0][21],
                 conds0[0][0][22], conds0[0][0][23], conds0[0][0][24], conds0[0][0][25], conds0[0][0][26],
                 conds0[0][0][27], conds0[0][0][28], conds0[0][0][29], conds0[0][0][30], conds0[0][0][31],
                 conds0[0][1][0], conds0[0][1][1], conds0[0][1][2], conds0[0][1][3], conds0[0][1][4], conds0[0][1][5],
                 conds0[0][1][6], conds0[0][1][7], conds0[0][1][8], conds0[0][1][9], conds0[0][1][10], conds0[0][1][11],
                 conds0[0][1][12], conds0[0][1][13], conds0[0][1][14], conds0[0][1][15], conds0[0][1][16],
                 conds0[0][1][17], conds0[0][1][18], conds0[0][1][19], conds0[0][1][20], conds0[0][1][21],
                 conds0[0][1][22], conds0[0][1][23], conds0[0][1][24], conds0[0][1][25], conds0[0][1][26],
                 conds0[0][1][27], conds0[0][1][28], conds0[0][1][29], conds0[0][1][30], conds0[0][1][31],
                 conds0[0][2][0], conds0[0][2][1], conds0[0][2][2], conds0[0][2][3], conds0[0][2][4], conds0[0][2][5],
                 conds0[0][2][6], conds0[0][2][7], conds0[0][2][8], conds0[0][2][9], conds0[0][2][10], conds0[0][2][11],
                 conds0[0][2][12], conds0[0][2][13], conds0[0][2][14], conds0[0][2][16], conds0[0][2][17],
                 conds0[0][2][18], conds0[0][2][19], conds0[0][2][20], conds0[0][2][21], conds0[0][2][22],
                 conds0[0][2][23], conds0[0][2][25], conds0[0][2][26], conds0[0][2][27], conds0[0][2][28],
                 conds0[0][2][29], conds0[0][2][30], conds0[0][2][31], conds0[1][0][0], conds0[1][0][1],
                 conds0[1][0][2], conds0[1][0][4], conds0[1][0][5], conds0[1][0][6], conds0[1][0][8], conds0[1][0][9],
                 conds0[1][0][10], conds0[1][0][11], conds0[1][0][13], conds0[1][0][14], conds0[1][0][15],
                 conds0[1][0][16], conds0[1][0][17], conds0[1][0][18], conds0[1][0][19], conds0[1][0][20],
                 conds0[1][0][22], conds0[1][0][23], conds0[1][0][24], conds0[1][0][25], conds0[1][0][26],
                 conds0[1][0][27], conds0[1][0][28], conds0[1][0][29], conds0[1][0][31], conds0[1][1][0],
                 conds0[1][1][1], conds0[1][1][2], conds0[1][1][3], conds0[1][1][4], conds0[1][1][5], conds0[1][1][6],
                 conds0[1][1][7], conds0[1][1][8], conds0[1][1][9], conds0[1][1][10], conds0[1][1][11],
                 conds0[1][1][12], conds0[1][1][13], conds0[1][1][14], conds0[1][1][15], conds0[1][1][16],
                 conds0[1][1][17], conds0[1][1][18], conds0[1][1][19], conds0[1][1][20], conds0[1][1][21],
                 conds0[1][1][22], conds0[1][1][23], conds0[1][1][24], conds0[1][1][25], conds0[1][1][26],
                 conds0[1][1][27], conds0[1][1][28], conds0[1][1][29], conds0[1][1][30], conds0[1][1][31],
                 conds0[1][2][0], conds0[1][2][1], conds0[1][2][2], conds0[1][2][3], conds0[1][2][4], conds0[1][2][5],
                 conds0[1][2][6], conds0[1][2][7], conds0[1][2][8], conds0[1][2][9], conds0[1][2][10], conds0[1][2][11],
                 conds0[1][2][12], conds0[1][2][13], conds0[1][2][14], conds0[1][2][15], conds0[1][2][16],
                 conds0[1][2][17], conds0[1][2][18], conds0[1][2][19], conds0[1][2][20], conds0[1][2][21],
                 conds0[1][2][22], conds0[1][2][23], conds0[1][2][24], conds0[1][2][25], conds0[1][2][26],
                 conds0[1][2][27], conds0[1][2][28], conds0[1][2][29], conds0[1][2][30], conds0[1][2][31],
                 conds0[2][0][0], conds0[2][0][1], conds0[2][0][2], conds0[2][0][4], conds0[2][0][5], conds0[2][0][6],
                 conds0[2][0][7], conds0[2][0][9], conds0[2][0][10], conds0[2][0][11], conds0[2][0][13],
                 conds0[2][0][14], conds0[2][0][15], conds0[2][0][16], conds0[2][0][18], conds0[2][0][19],
                 conds0[2][0][20], conds0[2][0][22], conds0[2][0][23], conds0[2][0][24], conds0[2][0][25],
                 conds0[2][0][27], conds0[2][0][28], conds0[2][0][29], conds0[2][0][30], conds0[2][0][31],
                 conds0[2][1][0], conds0[2][1][1], conds0[2][1][2], conds0[2][1][3], conds0[2][1][4], conds0[2][1][5],
                 conds0[2][1][6], conds0[2][1][7], conds0[2][1][9], conds0[2][1][10], conds0[2][1][11],
                 conds0[2][1][13], conds0[2][1][14], conds0[2][1][15], conds0[2][1][16], conds0[2][1][18],
                 conds0[2][1][19], conds0[2][1][20], conds0[2][1][22], conds0[2][1][23], conds0[2][1][24],
                 conds0[2][1][25], conds0[2][1][27], conds0[2][1][28], conds0[2][1][29], conds0[2][1][30],
                 conds0[2][1][31], conds0[2][2][0], conds0[2][2][1], conds0[2][2][2], conds0[2][2][3], conds0[2][2][4],
                 conds0[2][2][5], conds0[2][2][6], conds0[2][2][7], conds0[2][2][8], conds0[2][2][9], conds0[2][2][10],
                 conds0[2][2][11], conds0[2][2][12], conds0[2][2][13], conds0[2][2][14], conds0[2][2][15],
                 conds0[2][2][16], conds0[2][2][17], conds0[2][2][18], conds0[2][2][19], conds0[2][2][20],
                 conds0[2][2][21], conds0[2][2][22], conds0[2][2][23], conds0[2][2][24], conds0[2][2][25],
                 conds0[2][2][26], conds0[2][2][27], conds0[2][2][28], conds0[2][2][29], conds0[2][2][30],
                 conds0[2][2][31], conds0[3][0][0], conds0[3][0][1], conds0[3][0][2], conds0[3][0][3], conds0[3][0][4],
                 conds0[3][0][5], conds0[3][0][6], conds0[3][0][7], conds0[3][0][8], conds0[3][0][9], conds0[3][0][11],
                 conds0[3][0][12], conds0[3][0][13], conds0[3][0][14], conds0[3][0][15], conds0[3][0][16],
                 conds0[3][0][17], conds0[3][0][18], conds0[3][0][19], conds0[3][0][20], conds0[3][0][21],
                 conds0[3][0][22], conds0[3][0][23], conds0[3][0][24], conds0[3][0][25], conds0[3][0][26],
                 conds0[3][0][27], conds0[3][0][28], conds0[3][0][29], conds0[3][0][30], conds0[3][1][0],
                 conds0[3][1][1], conds0[3][1][2], conds0[3][1][3], conds0[3][1][4], conds0[3][1][5], conds0[3][1][6],
                 conds0[3][1][7], conds0[3][1][9], conds0[3][1][10], conds0[3][1][11], conds0[3][1][12],
                 conds0[3][1][13], conds0[3][1][14], conds0[3][1][15], conds0[3][1][16], conds0[3][1][18],
                 conds0[3][1][19], conds0[3][1][20], conds0[3][1][21], conds0[3][1][22], conds0[3][1][23],
                 conds0[3][1][24], conds0[3][1][25], conds0[3][1][27], conds0[3][1][28], conds0[3][1][29],
                 conds0[3][1][30], conds0[3][2][0], conds0[3][2][2], conds0[3][2][3], conds0[3][2][4], conds0[3][2][5],
                 conds0[3][2][6], conds0[3][2][7], conds0[3][2][8], conds0[3][2][9], conds0[3][2][11], conds0[3][2][12],
                 conds0[3][2][13], conds0[3][2][14], conds0[3][2][15], conds0[3][2][16], conds0[3][2][17],
                 conds0[3][2][18], conds0[3][2][19], conds0[3][2][20], conds0[3][2][21], conds0[3][2][22],
                 conds0[3][2][23], conds0[3][2][24], conds0[3][2][25], conds0[3][2][26], conds0[3][2][27],
                 conds0[3][2][28], conds0[3][2][29], conds0[3][2][30], conds0[3][2][31]]
    value1_set = [values0[2][1][8], values0[2][1][12], values0[2][1][17], values0[2][1][21], values0[2][1][26],
                  values0[3][0][10], values0[3][1][8], values0[3][1][17], values0[3][1][26], values0[3][1][31]]
    value0_set = [values0[0][2][15], values0[0][2][24], values0[1][0][3], values0[1][0][7], values0[1][0][12],
                  values0[1][0][21], values0[1][0][30], values0[2][0][3], values0[2][0][8], values0[2][0][12],
                  values0[2][0][17], values0[2][0][21], values0[2][0][26], values0[3][0][31], values0[3][2][1],
                  values0[3][2][10]]
    for i in range(len(true_set)):
        model.addConstr(true_set[i] == 1, f'true_cond_{i}_cons0')
    for i in range(len(false_set)):
        model.addConstr(false_set[i] == 0, f'false_cond_{i}_cons0')
    for i in range(len(value1_set)):
        model.addConstr(value1_set[i] == 1, f'1_value_{i}_cons0')
    for i in range(len(value0_set)):
        model.addConstr(value0_set[i] == 0, f'0_value_{i}_cons0')


def new_partial_fixed_ini_state_12_dom_6(model, r0_state, lambda_r):
    blue_set = [r0_state[1][2][2], r0_state[2][2][2], r0_state[1][2][4], r0_state[0][2][8], r0_state[1][2][11],
                r0_state[2][2][11], r0_state[1][2][13], r0_state[1][2][20], r0_state[1][2][25], r0_state[2][2][25],
                r0_state[1][2][29], r0_state[0][2][31]]
    free_set = [r0_state[0][2][0], r0_state[1][2][0], r0_state[2][2][0], r0_state[3][2][0], r0_state[0][2][1],
                r0_state[1][2][1], r0_state[2][2][1], r0_state[3][2][1], r0_state[0][2][2], r0_state[3][2][2],
                r0_state[0][2][3], r0_state[1][2][3], r0_state[2][2][3], r0_state[3][2][3], r0_state[0][2][4],
                r0_state[2][2][4], r0_state[3][2][4], r0_state[0][2][5], r0_state[1][2][5], r0_state[2][2][5],
                r0_state[3][2][5], r0_state[0][2][6], r0_state[1][2][6], r0_state[2][2][6], r0_state[3][2][6],
                r0_state[0][2][7], r0_state[1][2][7], r0_state[2][2][7], r0_state[3][2][7], r0_state[1][2][8],
                r0_state[2][2][8], r0_state[3][2][8], r0_state[0][2][9], r0_state[1][2][9], r0_state[2][2][9],
                r0_state[3][2][9], r0_state[0][2][10], r0_state[1][2][10], r0_state[2][2][10], r0_state[3][2][10],
                r0_state[0][2][11], r0_state[3][2][11], r0_state[0][2][12], r0_state[1][2][12], r0_state[2][2][12],
                r0_state[3][2][12], r0_state[0][2][13], r0_state[2][2][13], r0_state[3][2][13], r0_state[0][2][14],
                r0_state[1][2][14], r0_state[2][2][14], r0_state[3][2][14], r0_state[0][2][15], r0_state[1][2][15],
                r0_state[2][2][15], r0_state[3][2][15], r0_state[0][2][16], r0_state[1][2][16], r0_state[2][2][16],
                r0_state[3][2][16], r0_state[0][2][17], r0_state[1][2][17], r0_state[2][2][17], r0_state[3][2][17],
                r0_state[0][2][18], r0_state[1][2][18], r0_state[2][2][18], r0_state[3][2][18], r0_state[0][2][19],
                r0_state[1][2][19], r0_state[2][2][19], r0_state[3][2][19], r0_state[0][2][20], r0_state[2][2][20],
                r0_state[3][2][20], r0_state[0][2][21], r0_state[1][2][21], r0_state[2][2][21], r0_state[3][2][21],
                r0_state[0][2][22], r0_state[1][2][22], r0_state[2][2][22], r0_state[3][2][22], r0_state[0][2][23],
                r0_state[1][2][23], r0_state[2][2][23], r0_state[3][2][23], r0_state[0][2][24], r0_state[1][2][24],
                r0_state[2][2][24], r0_state[3][2][24], r0_state[0][2][25], r0_state[3][2][25], r0_state[0][2][26],
                r0_state[1][2][26], r0_state[2][2][26], r0_state[3][2][26], r0_state[0][2][27], r0_state[1][2][27],
                r0_state[2][2][27], r0_state[3][2][27], r0_state[0][2][28], r0_state[1][2][28], r0_state[2][2][28],
                r0_state[3][2][28], r0_state[0][2][29], r0_state[2][2][29], r0_state[3][2][29], r0_state[0][2][30],
                r0_state[1][2][30], r0_state[2][2][30], r0_state[3][2][30], r0_state[1][2][31], r0_state[2][2][31],
                r0_state[3][2][31]]
    fixed_lambda_r = [lambda_r[1][2][2], lambda_r[2][2][2], lambda_r[1][2][4], lambda_r[0][2][8], lambda_r[1][2][11], lambda_r[2][2][11], lambda_r[1][2][13], lambda_r[1][2][20], lambda_r[1][2][25], lambda_r[2][2][25], lambda_r[1][2][29], lambda_r[0][2][31]]
    free_lambda_r = [lambda_r[0][2][0], lambda_r[1][2][0], lambda_r[2][2][0], lambda_r[3][2][0], lambda_r[0][2][1], lambda_r[1][2][1], lambda_r[2][2][1], lambda_r[3][2][1], lambda_r[0][2][2], lambda_r[3][2][2], lambda_r[0][2][3], lambda_r[1][2][3], lambda_r[2][2][3], lambda_r[3][2][3], lambda_r[0][2][4], lambda_r[2][2][4], lambda_r[3][2][4], lambda_r[0][2][5], lambda_r[1][2][5], lambda_r[2][2][5], lambda_r[3][2][5], lambda_r[0][2][6], lambda_r[1][2][6], lambda_r[2][2][6], lambda_r[3][2][6], lambda_r[0][2][7], lambda_r[1][2][7], lambda_r[2][2][7], lambda_r[3][2][7], lambda_r[1][2][8], lambda_r[2][2][8], lambda_r[3][2][8], lambda_r[0][2][9], lambda_r[1][2][9], lambda_r[2][2][9], lambda_r[3][2][9], lambda_r[0][2][10], lambda_r[1][2][10], lambda_r[2][2][10], lambda_r[3][2][10], lambda_r[0][2][11], lambda_r[3][2][11], lambda_r[0][2][12], lambda_r[1][2][12], lambda_r[2][2][12], lambda_r[3][2][12], lambda_r[0][2][13], lambda_r[2][2][13], lambda_r[3][2][13], lambda_r[0][2][14], lambda_r[1][2][14], lambda_r[2][2][14], lambda_r[3][2][14], lambda_r[0][2][15], lambda_r[1][2][15], lambda_r[2][2][15], lambda_r[3][2][15], lambda_r[0][2][16], lambda_r[1][2][16], lambda_r[2][2][16], lambda_r[3][2][16], lambda_r[0][2][17], lambda_r[1][2][17], lambda_r[2][2][17], lambda_r[3][2][17], lambda_r[0][2][18], lambda_r[1][2][18], lambda_r[2][2][18], lambda_r[3][2][18], lambda_r[0][2][19], lambda_r[1][2][19], lambda_r[2][2][19], lambda_r[3][2][19], lambda_r[0][2][20], lambda_r[2][2][20], lambda_r[3][2][20], lambda_r[0][2][21], lambda_r[1][2][21], lambda_r[2][2][21], lambda_r[3][2][21], lambda_r[0][2][22], lambda_r[1][2][22], lambda_r[2][2][22], lambda_r[3][2][22], lambda_r[0][2][23], lambda_r[1][2][23], lambda_r[2][2][23], lambda_r[3][2][23], lambda_r[0][2][24], lambda_r[1][2][24], lambda_r[2][2][24], lambda_r[3][2][24], lambda_r[0][2][25], lambda_r[3][2][25], lambda_r[0][2][26], lambda_r[1][2][26], lambda_r[2][2][26], lambda_r[3][2][26], lambda_r[0][2][27], lambda_r[1][2][27], lambda_r[2][2][27], lambda_r[3][2][27], lambda_r[0][2][28], lambda_r[1][2][28], lambda_r[2][2][28], lambda_r[3][2][28], lambda_r[0][2][29], lambda_r[2][2][29], lambda_r[3][2][29], lambda_r[0][2][30], lambda_r[1][2][30], lambda_r[2][2][30], lambda_r[3][2][30], lambda_r[1][2][31], lambda_r[2][2][31], lambda_r[3][2][31]]
    for i in range(len(blue_set)):
        model.addConstr(blue_set[i][0] == 1, f'blue_set_{i}_cons0')
        model.addConstr(blue_set[i][1] == 1, f'blue_set_{i}_cons1')
        model.addConstr(blue_set[i][2] == 0, f'blue_set_{i}_cons2')
        model.addConstr(fixed_lambda_r[i] == 0, f'lambda_r_{i}_cons0')
    for i in range(len(free_set)):
        model.addConstr(free_set[i][0] + free_lambda_r[i] == 1, f'free_set_{i}_cons0')
        model.addConstr(free_set[i][1] == 1, f'free_set_{i}_cons1')
        model.addConstr(free_set[i][2] == 1, f'free_set_{i}_cons2')
    for k in range(Z):
        for i in range(X):
            model.addConstr(r0_state[i][0][k][0] == 1, f'free_set_{i}_cons0')
            model.addConstr(r0_state[i][0][k][1] == 1, f'free_set_{i}_cons1')
            model.addConstr(r0_state[i][0][k][2] == 1, f'free_set_{i}_cons2')
            model.addConstr(r0_state[i][1][k][0] == 1, f'free_set_{i}_cons0')
            model.addConstr(r0_state[i][1][k][1] == 1, f'free_set_{i}_cons1')
            model.addConstr(r0_state[i][1][k][2] == 1, f'free_set_{i}_cons2')
            model.addConstr(lambda_r[i][0][k] == 0, f'lambda_r_{i}_cons1')
            model.addConstr(lambda_r[i][1][k] == 0, f'lambda_r_{i}_cons2')