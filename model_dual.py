#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  2 17:16:36 2019

@author: victorchomel
"""

from docplex.mp.model import Model
from docplex.util.environment import get_environment
from read_files import file_to_data



# ----------------------------------------------------------------------------
# Initialize the problem data
# ----------------------------------------------------------------------------


data = file_to_data['20_USA-road-d.BAY.gr']

n = data['n']
s = data['s']
t = data['t']
S = data['S']
d1 = data['d1']
d2 = data['d2']
p = data['p']
ph = data['ph']
d = data['d']
D = data['D']


print(n)
# ----------------------------------------------------------------------------
# Build the model
# ----------------------------------------------------------------------------

def build_dualized_model(**kwargs):
    
    
    #Model
    mdl = Model(name='dualized', **kwargs)
    
    
    #Decision variable
    mdl.h = mdl.continuous_var(lb = 0)
    mdl.l = mdl.continuous_var_list(n, lb = 0)
    mdl.x = mdl.binary_var_matrix(n, n)
    mdl.y = mdl.binary_var_list(n)
    mdl.g = mdl.continuous_var(lb = 0)
    mdl.f = mdl.continuous_var_matrix(n, n, lb = 0)
    
    
    #Constraints
    for i in range(n):
        for j in range(n):
            if d[i,j] > 0:
                mdl.add_constraint(mdl.f[i, j] + mdl.g >= mdl.x[i, j])

        
    for i in range(n):
        if not i == t:
            somme = 0
            for j in range(n):
                if d[i,j] > 0:
                    somme += mdl.x[i, j]
            mdl.add_constraint(mdl.y[i] == somme)
    
    for j in range(n):
        if not j == s:
            somme = 0
            for i in range(n):
                if d[i,j] > 0:
                    somme += mdl.x[i, j]
            mdl.add_constraint(mdl.y[j] == somme)
    
    mdl.add_constraint(mdl.y[s] == 1)
    mdl.add_constraint(mdl.y[t] == 1)
    
    mdl.add_constraint(mdl.sum(2*mdl.l[i] + mdl.y[i]*p[i] for i in range(n)) + d2*mdl.h <= S)
    
        
    for i in range(n):
        mdl.add_constraint(mdl.l[i] + mdl.h >= mdl.y[i]*ph[i])
    
    ##Problem with the value below 0, it should be 0 in the sum to avoid this pb
    ##On doit bien compter tout 2 fois ? i vers j et j vers i ?
    mdl.minimize(mdl.sum(mdl.x[i, j]*d[i, j] + D[i,j]*mdl.f[i, j] for i in range(n) for j in range(n) if d[i, j] > 0) + d1*mdl.g)
    
    return mdl


if __name__ == '__main__':
    mdl = build_dualized_model()
    mdl.print_information()
    mdl.export_as_lp()
    if mdl.solve():
        mdl.float_precision = 3
        print("* model solved as function:")
        mdl.print_solution()
        mdl.report_kpis()
        # Save the CPLEX solution as "solution.json" program output
#        with get_environment().get_output_stream("solution.json") as fp:
#            mdl.solution.export(fp, "json")
    else:
        print("* model has no solution")
    
