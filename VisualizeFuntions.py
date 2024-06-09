import numpy
import gurobipy
import turtle
from gurobipy import *
from gurobipy import GRB

X = 4
Y = 3
Z = 32

# 颜色矩阵编码维数
Color_Dimension = 3

# 方块填充颜色
Color = '#FFFFFF'

# 画图边长及转角
Len = 5
Angle = 90

# 创建画布
Screen = turtle.Screen()
Width = 1000
Height = 1000
Screen.setup(Width, Height)
Ini_Loc_X = -Width / 2 + 20
Ini_Loc_Y = Height / 2 - 200


# 绘制一个方格
def grid_rendering(len, angle):
    turtle.tracer(False)
    turtle.begin_fill()
    turtle.forward(len)
    turtle.right(angle)
    turtle.forward(len)
    turtle.right(angle)
    turtle.forward(len)
    turtle.right(angle)
    turtle.forward(len)
    turtle.right(angle)
    turtle.penup()
    turtle.forward(len)
    turtle.pendown()


# 根据颜色矩阵选择填充颜色
def select_color(color_array):
    global Color
    if color_array[0].X == 1:
        if color_array[2].X == 1:
            Color = '#BFBFBF'
        else:
            Color = '#0000FF'
    else:
        if color_array[1].X == 0:
            Color = '#FFFFFF'
        else:
            if color_array[2].X == 1:
                Color = '#FF0000'
            else:
                Color = '#66FF66'


# 绘制三维列表color_array_list的颜色阵列
def visualize_the_state(color_array_list, x, y, z, y_axis):
    turtle.penup()
    turtle.goto(-Width / 2 + 20, Height / 2 - 200 - y_axis)
    turtle.pendown()
    for k in range(z):
        for j in range(y):
            for i in range(x):
                turtle.pencolor('#000000')
                select_color(color_array_list[i][j][k])
                turtle.fillcolor(Color)
                grid_rendering(Len, Angle)
                turtle.end_fill()
            turtle.penup()
            turtle.backward(X * Len)
            turtle.left(Angle)
            turtle.forward(Len)
            turtle.right(Angle)
            turtle.pendown()
        turtle.penup()
        turtle.forward((X+2) * Len)
        turtle.right(Angle)
        turtle.forward(Y * Len)
        turtle.left(Angle)
        turtle.pendown()


def select_pencolor(delta_r, delta_b):
    if delta_r.X == 1 and delta_b.X == 0:
        turtle.pencolor('#FF0000')
    elif delta_r.X == 0 and delta_b.X == 1:
        turtle.pencolor('#0000FF')
    elif delta_r.X == 1 and delta_b.X == 1:
        turtle.pencolor('#66FF66')
    else:
        turtle.pencolor('#000000')


def visualize_the_state_dof(color_array_list, x, y, z, y_axis, delta_r, delta_b):
    turtle.penup()
    turtle.goto(-Width / 2 + 20, Height / 2 - 200 - y_axis)
    turtle.pendown()
    for k in range(z):
        for j in range(y):
            for i in range(x):
                select_pencolor(delta_r[i][j][k], delta_b[i][j][k])
                select_color(color_array_list[i][j][k])
                turtle.fillcolor(Color)
                grid_rendering(Len, Angle)
                turtle.end_fill()
            turtle.penup()
            turtle.backward(X * Len)
            turtle.left(Angle)
            turtle.forward(Len)
            turtle.right(Angle)
            turtle.pendown()
        turtle.penup()
        turtle.forward((X+2) * Len)
        turtle.right(Angle)
        turtle.forward(Y * Len)
        turtle.left(Angle)
        turtle.pendown()
    turtle.penup()
    turtle.goto(-Width / 2 + 20, Height / 2 - 200 - y_axis)
    turtle.pendown()
    for k in range(z):
        for j in range(y):
            for i in range(x):
                if delta_r[i][j][k].X != 0 or delta_b[i][j][k].X != 0:
                    select_pencolor(delta_r[i][j][k], delta_b[i][j][k])
                    select_color(color_array_list[i][j][k])
                    turtle.fillcolor(Color)
                    grid_rendering(Len, Angle)
                    turtle.end_fill()
                else:
                    turtle.penup()
                    turtle.forward(Len)
                    turtle.pendown()
            turtle.penup()
            turtle.backward(X * Len)
            turtle.left(Angle)
            turtle.forward(Len)
            turtle.right(Angle)
            turtle.pendown()
        turtle.penup()
        turtle.forward((X + 2) * Len)
        turtle.right(Angle)
        turtle.forward(Y * Len)
        turtle.left(Angle)
        turtle.pendown()


# 绘制二维列表color_array_list的颜色阵列
def visualize_the_middle_state(color_array_list, x, z, y_axis):
    turtle.penup()
    turtle.goto(-Width / 2 + 20, Height / 2 - 200 - y_axis)
    turtle.pendown()
    for k in range(z):
        for i in range(x):
            turtle.pencolor('#000000')
            select_color(color_array_list[i][k])
            turtle.fillcolor(Color)
            grid_rendering(Len, Angle)
            turtle.end_fill()
        turtle.penup()
        turtle.forward(2 * Len)
        turtle.pendown()


def visualize_the_middle_state_dof(color_array_list, x, z, y_axis, delta_r, delta_b):
    turtle.penup()
    turtle.goto(-Width / 2 + 20, Height / 2 - 200 - y_axis)
    turtle.pendown()
    for k in range(z):
        for i in range(x):
            select_pencolor(delta_r[i][k], delta_b[i][k])
            select_color(color_array_list[i][k])
            turtle.fillcolor(Color)
            grid_rendering(Len, Angle)
            turtle.end_fill()
        turtle.penup()
        turtle.forward(2 * Len)
        turtle.pendown()
    turtle.penup()
    turtle.goto(-Width / 2 + 20, Height / 2 - 200 - y_axis)
    turtle.pendown()
    for k in range(z):
        for i in range(x):
            if delta_r[i][k].X != 0 or delta_b[i][k].X != 0:
                select_pencolor(delta_r[i][k], delta_b[i][k])
                select_color(color_array_list[i][k])
                turtle.fillcolor(Color)
                grid_rendering(Len, Angle)
                turtle.end_fill()
            else:
                turtle.penup()
                turtle.forward(Len)
                turtle.pendown()
        turtle.penup()
        turtle.forward(2 * Len)
        turtle.pendown()

