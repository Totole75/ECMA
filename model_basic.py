#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 12:09:47 2019

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

def build_basic_model(**kwargs):
    
    
    #Model
    mdl = Model(name='basic', **kwargs)
    
    
    #Decision variable
    mdl.z = mdl.continuous_var()
    mdl.x = mdl.binary_var_matrix(n, n)
    mdl.y = mdl.binary_var_list(n)
    mdl.delta1 = mdl.continuous_var_matrix(n, n, lb = 0)
    mdl.delta2 = mdl.continuous_var_list(n, lb = 0, ub = 2)
    
    
    #Constraints
    
    ##Problem with the value below 0, it should be 0 in the sum to avoid this pb
    ##On doit bien compter tout 2 fois ? i vers j et j vers i ?
    
    
    mdl.add_constraint(mdl.sum(mdl.x[i, j]*d[i, j]*(1 + mdl.delta1[i, j]) for i in range(n) for j in range(n) if d[i, j] > 0) <= mdl.z)
    
    mdl.add_constraint(mdl.sum(mdl.y[i]*(p[i] + mdl.delta2[i]*ph[i]) for i in range(n)) <= S)
    
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
    
    for i in range(n):
        for j in range(n):
            if d[i, j] > 0:
                mdl.add_constraint(mdl.delta1[i, j] <= D[i, j])
                
    mdl.add_constraint(mdl.sum(mdl.delta1[i, j] for i in range(n) for j in range(n) if d[i, j] > 0) <= d1)
    
    mdl.add_constraint(mdl.sum(mdl.delta2[i] for i in range(n)) <= d2)

    mdl.minimize(mdl.z)
    
    return mdl


if __name__ == '__main__':
    mdl = build_basic_model()
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
    
