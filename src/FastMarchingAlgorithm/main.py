#from fast_marching_trees_1 import  FMT1
from fast_marching_tree_new import FMTN

'''''
 Input data
'''

x00 = (18,15)
x0 = (18, 16)
x1 = (36, 14)
x2 = (48, 24)
x3 = (52, 42)
x4 = (36, 48)
x5 = (22, 46)
x6 = (8, 36)
x7 = (16, 24)

# f0 = FMT1(x0,x1,x2,x3,x4,x5,x6,x7, 80)
# f0.Planning()

f1 = FMTN(x0,x1,x2,x3,x4,x5,x6,x7,x00,60)
f1.main()

