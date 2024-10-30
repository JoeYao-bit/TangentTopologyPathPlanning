import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
import os

def loadDataFromfile(file_path):
    data_list = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                splited_line = line.split()
                
                head_split = splited_line[0].split('_')
                
                new_data = LineData()
                
                new_data.method = head_split[0]
                new_data.path_count = int(head_split[1])
                
                new_data.mean_path_length = float(splited_line[5])
                
                new_data.time_cost = float(splited_line[7]) + float(splited_line[8])
                
                new_data.success = int(splited_line[9]) >= new_data.path_count
                
                new_data.inflation_ratio = float(splited_line[11])
                
                data_list.append(new_data)
            #print(new_data.method, ' ', new_data.path_count, ' ', new_data.real_path_count, ' ', new_data.time_cost)
    except Exception as e:            
        print(e)             
    return data_list

class LineData:
    method = ''
    path_count = 0
    inflation_ratio = 0 

    time_cost = 0
    success = 0
    mean_path_length = 0  
        

# all data in a txt file
class SingleTestData:
    data_list = list()
    map_name = ''
    
    
def drawMethodMaps(all_data_map, xlable, ylable, title, is_percentage=False):    
    for method_key, method_value in all_data_map.items():
        fig=plt.figure(figsize=(5,3.5)) #添加绘图框
        for map_key, map_value in all_data_map[method_key].items():
            x = list()
            y = list()
            sorted_keys = sorted(all_data_map[method_key][map_key].keys())
            
            splited_method_name = method_key.split('_')
            for path_size_key in sorted_keys:
                x.append(path_size_key)
                if len(all_data_map[method_key][map_key][path_size_key]) > 0:
                    y.append(np.mean(all_data_map[method_key][map_key][path_size_key]))
                else:
                    y.append(0)
                    
            plt.errorbar(x, y, fmt=map_list[map_key], markersize=14, label=map_key, linewidth=2, elinewidth=4, capsize=4)
            #plt.errorbar(x, y,markersize=14, label=map_key, linewidth=2, elinewidth=4, capsize=4)

                
        plt.tick_params(axis='both', labelsize=18)
        formater = ticker.ScalarFormatter(useMathText=True) 
        formater.set_scientific(True)
        formater.set_powerlimits((0,0))
        ax = plt.gca()
        ax.yaxis.offsetText.set_fontsize(18)
        ax.yaxis.set_major_formatter(formater) 
        
        y_range = plt.ylim()      
        # if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
        #     ax.set_yscale('log')  
        #     plt.ylim(1, y_range[1]*10)  
        # else:    
        plt.ylim(0, y_range[1])
            
        #plt.title(ylable)
        plt.legend(ncol=2)
        if is_percentage:
            plt.ylim(0, 1)
                
        plt.tight_layout()
        save_path = '../test/pic/'+title
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
            
        save_path = save_path + "/" + method_key
        plt.savefig(save_path, dpi = 400, bbox_inches='tight')   
        plt.close()
        print("save path to " + save_path)    
    

def drawSummaryOfAllInflationRatio(all_inflation_map, xlable, ylable, title, is_percentage=False):
    for method_key in method_list:
        data_dict = dict()
        for inflation_key, inflation_value in all_inflation_map.items():
            # print('inflation_key 1 ='+str(inflation_key))

            if data_dict.get(inflation_key) == None:
                data_dict[inflation_key] = dict()
                
                
            # print(all_inflation_map[inflation_key])    
                
            for map_key, map_value in all_inflation_map[inflation_key][method_key].items():
                for path_count_key, path_count_value in all_inflation_map[inflation_key][method_key][map_key].items():

                    if data_dict[inflation_key].get(path_count_key) == None:
                        data_dict[inflation_key][path_count_key] = list()
                        
                    data_dict[inflation_key][path_count_key].append( \
                        all_inflation_map[inflation_key][method_key][map_key][path_count_key])
        
        #print(data_dict.items()) # only contain 3.0
                        
        fig=plt.figure(figsize=(5,3.5)) #添加绘图框
        for inflation_key2, inflation_value in data_dict.items():
            # print('inflation_key 2 ='+str(inflation_key2))
            x = list()
            y = list()
            sorted_keys = sorted(data_dict[inflation_key2].keys())
            
            for path_size_key2 in sorted_keys:
                x.append(path_size_key2)
                if len(data_dict[inflation_key2][path_size_key2]) > 0:
                    y.append(np.mean(data_dict[inflation_key2][path_size_key2]))
                else:
                    y.append(0)
                    
            plt.errorbar(x, y, fmt='o-', label=inflation_key2, markersize=14, linewidth=2, elinewidth=4, capsize=4)
            #plt.errorbar(x, y,markersize=14, label=map_key, linewidth=2, elinewidth=4, capsize=4)

                
        plt.tick_params(axis='both', labelsize=18)
        formater = ticker.ScalarFormatter(useMathText=True) 
        formater.set_scientific(True)
        formater.set_powerlimits((0,0))
        ax = plt.gca()
        ax.yaxis.offsetText.set_fontsize(18)
        ax.yaxis.set_major_formatter(formater) 
        
        y_range = plt.ylim()      
        # if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
        #     ax.set_yscale('log')  
        #     plt.ylim(1, y_range[1]*10)  
        # else:    
        plt.ylim(0, y_range[1])
            
        #plt.title(ylable)
        plt.legend(ncol=2)
        if is_percentage:
            plt.ylim(0, 1)
                
        plt.tight_layout()
        save_path = '../test/pic/'+title
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
            
        save_path = save_path + "/" + method_key
        plt.savefig(save_path, dpi = 400, bbox_inches='tight')   
        plt.close()
        print("save path to " + save_path)   
            
data_path_dir = '../test/test_data/'

map_list = {"Berlin_1_256":'o-',
            "Denver_2_256":'*-',
            "Boston_2_256":'v-',
            "Milan_2_256":'<-',
            "Moscow_2_256":'H-',
            "London_2_256":'D-',
            "Sydney_1_256":'X-',
            "Paris_0_256":'+-'}

method_list = [
    'RJ', 'RHCF', 'HsAs', 'HsTs'
]

all_single_data = list()    

# 1, load all data
for map_name_key, map_name_vvalue in map_list.items():
    data_file_path = data_path_dir + map_name_key + '.txt'
    print('load data from', data_file_path)
    single = SingleTestData() # 不带括号则均指向同一元素
    single.map_name = map_name_key
    single.data_list = loadDataFromfile(data_file_path)
    all_single_data.append(single)
    
all_method_time_cost_map = dict()

all_method_mean_path_length_map = dict()

all_method_success_rate_map = dict()
 
    
# [inflation_ratio][path_count][map_name]    

for single_data in all_single_data:
    
    map_name = single_data.map_name

    for line_data in single_data.data_list:
        method_name     = line_data.method
        path_count      = line_data.path_count
        inflation_ratio = line_data.inflation_ratio
                
        # initialize all_method_time_cost_map
        if all_method_time_cost_map.get(inflation_ratio) == None:
            all_method_time_cost_map[inflation_ratio] = dict()
            
            
        if all_method_time_cost_map[inflation_ratio].get(method_name) == None:
            all_method_time_cost_map[inflation_ratio][method_name] = dict()    
            
            
        if all_method_time_cost_map[inflation_ratio][method_name].get(map_name) == None:
            all_method_time_cost_map[inflation_ratio][method_name][map_name] = dict()
            
        
        if all_method_time_cost_map[inflation_ratio][method_name][map_name].get(path_count) == None:
            all_method_time_cost_map[inflation_ratio][method_name][map_name][path_count] = list()    
            
            
        # initialize all_method_mean_path_length_map
        if all_method_mean_path_length_map.get(inflation_ratio) == None:
            all_method_mean_path_length_map[inflation_ratio] = dict()
            
            
        if all_method_mean_path_length_map[inflation_ratio].get(method_name) == None:
            all_method_mean_path_length_map[inflation_ratio][method_name] = dict()    
            
            
        if all_method_mean_path_length_map[inflation_ratio][method_name].get(map_name) == None:
            all_method_mean_path_length_map[inflation_ratio][method_name][map_name] = dict()
            
        
        if all_method_mean_path_length_map[inflation_ratio][method_name][map_name].get(path_count) == None:
            all_method_mean_path_length_map[inflation_ratio][method_name][map_name][path_count] = list()      
            
            
        # initialize all_method_success_rate_map
        if all_method_success_rate_map.get(inflation_ratio) == None:
            all_method_success_rate_map[inflation_ratio] = dict()
            
            
        if all_method_success_rate_map[inflation_ratio].get(method_name) == None:
            all_method_success_rate_map[inflation_ratio][method_name] = dict()    
            
            
        if all_method_success_rate_map[inflation_ratio][method_name].get(map_name) == None:
            all_method_success_rate_map[inflation_ratio][method_name][map_name] = dict()
            
        
        if all_method_success_rate_map[inflation_ratio][method_name][map_name].get(path_count) == None:
            all_method_success_rate_map[inflation_ratio][method_name][map_name][path_count] = list()     
            
            
        all_method_time_cost_map[inflation_ratio][method_name][map_name][path_count].append(line_data.time_cost)
        
        all_method_mean_path_length_map[inflation_ratio][method_name][map_name][path_count].append(line_data.mean_path_length)
        
        all_method_success_rate_map[inflation_ratio][method_name][map_name][path_count].append(line_data.success)


# 2, draw when inflation ratio = 1
# drawMethodMaps(all_method_time_cost_map[1.0], "Number of agents", "Time cost(ms)", "time_cost")

# drawMethodMaps(all_method_mean_path_length_map[1.0], "Number of agents", "Path length", "path_length")

# drawMethodMaps(all_method_success_rate_map[1.0], "Number of agents", "Success rate", "success_rate", True)

# 3, draw comparison under different dimension
drawSummaryOfAllInflationRatio(all_method_time_cost_map, "Number of agents", "Time cost(ms)", "time_cost_infla")

drawSummaryOfAllInflationRatio(all_method_time_cost_map, "Number of agents", "Path length", "path_length_infla")

drawSummaryOfAllInflationRatio(all_method_time_cost_map, "Number of agents", "Success rate", "success_rate_infla")