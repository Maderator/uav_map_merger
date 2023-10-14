import numpy as np
import sys
import csv
import os
#from sets import Set

class OrienteeringProblemDefinition():
    def __init__(self):
        self.budget = 0
        self.nodes = np.array([])
        self.num_vehicles = 0
        self.start_index = 0
        self.end_index = -1
    
    def load_problem_file(self,problem_file):
        try:
            with open(problem_file,"r")  as file:
                first_line_readed = False
                for line in file:
                    if first_line_readed:
                        node_line = line.split()
                        point_to_add = np.array([float(node_line[0]),float(node_line[1]),float(node_line[2])])
                        if np.size(self.nodes,0)<1:  
                            self.nodes = np.hstack((self.nodes,point_to_add))
                        else:
                            self.nodes = np.vstack((self.nodes,point_to_add))
                        
                    else:
                        budget_line = line.split()
                        first_line_readed = True
                        self.budget = float(budget_line[0])
                        self.num_vehicles = int(budget_line[1])
                        
        except Exception as e:
            print("can not parse op problem file")
            raise


def parse_individual_value(value):
    converted = False
    if not converted:
        try: 
            value = int(value)
            converted = True
        except ValueError:
            pass
    if not converted:
        try:
            value = float(value)
            converted = True
        except ValueError:
            pass
    return value

def parse_op_log(log_file):
    log_lines = []
    delimiterVariable = ';';
    delimiterValue = ':';
    delimiterSubValue = ',';
    #not yet implemented
    delimiterSubSubValue = '|';
    if not os.path.exists(log_file):
        print("file does not exists "+log_file)
        return log_lines
    with open(log_file,"r")  as file:
        for line in file:
            var_values = line.split(delimiterVariable)
            variables = []
            values = []
            for single_var_value in var_values:
                if delimiterValue in single_var_value: 
                    splited_var_value = single_var_value.split(delimiterValue)
                    #print("splited_var_value",splited_var_value)
                    variable = splited_var_value[0]
                    value = splited_var_value[1]
                    
                   
                    if delimiterSubValue in value or delimiterSubSubValue in value:
                        #contains delimiterSubValue
                        splited_sub_value = value.split(delimiterSubValue)
                        subvalue_array = []
                        for subvalue in splited_sub_value:
                                
                            if delimiterSubSubValue in subvalue:
                                #contains delimiterSubSubValue
                                splited_sub_sub_value = subvalue.split(delimiterSubSubValue)
                                sub_subvalue_array = []
                                for sub_subvalue in splited_sub_sub_value:
                                    sub_subvalue = parse_individual_value(sub_subvalue)
                                    sub_subvalue_array.append(sub_subvalue)
                                subvalue_array.append(sub_subvalue_array)
                            else:
                                #does not contains delimiterSubSubValue
                                subvalue = parse_individual_value(subvalue)
                                subvalue_array.append(subvalue)
                        value = subvalue_array
                    else:
                        #does not contains delimiterSubValue
                        value = parse_individual_value(value)    
                    #convert normal value ints
                        
                    
                    variables.append(variable)
                    values.append(value)
            #print("variables",variables)
            #print("values",values)
            single_log = dict(zip(variables, values))
            log_lines.append(single_log)
    #sys.exit()
    return log_lines

def load_sampled_path(sampled_path_file):
    sampled_path = None
    with open(sampled_path_file, 'rt') as csvfile:
        scv_reader = csv.reader(csvfile)
        for row in scv_reader:
            to_add = np.array([float(row[0]),float(row[1]),                   float(row[2])])
            if(sampled_path is None):
                sampled_path = to_add
            else:
                sampled_path = np.vstack((sampled_path,to_add))
    #print(sampled_path)
    return sampled_path

def getUniqueVeluesOfKey(data_to_process,key):
    unique_value_set = set([])
    for idx in range(len(data_to_process)):
        unique_value_set.add(data_to_process[idx][key])
    return list(unique_value_set)

def getDataWithKeyValue(data_to_process,key,value):
    data_subset = []
    for idx in range(len(data_to_process)):
        if data_to_process[idx][key] == value:
            data_subset.append(data_to_process[idx])
    return data_subset

def getMaxValueInData(data_to_process,key):
    max = data_to_process[0][key]
    for idx in range(1,len(data_to_process)):
        if data_to_process[idx][key] > max:
            max = data_to_process[idx][key]
    return max

def getMinValueInData(data_to_process,key):
    min = data_to_process[0][key]
    for idx in range(1,len(data_to_process)):
        if data_to_process[idx][key] < min:
            min = data_to_process[idx][key]
    return min

def getAverageValueInData(data_to_process,key):
    sum = 0
    num_tested = 0
    for idx in range(len(data_to_process)):
        num_tested +=1
        sum += data_to_process[idx][key]
    return sum / num_tested
