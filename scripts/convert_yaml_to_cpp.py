import os
import yaml

yaml_dir = '/home/rubis/AutowareAuto/src/rubis_main/param/'

# output
out_file = '/home/rubis/AutowareAuto/src/rubis_main/include/rubis_main/rubis_main_parsed_dev.hpp'

output_header = '''
#ifndef RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
#define RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_

#include "rclcpp/rclcpp.hpp"
'''

output_body = '''
RUBIS
'''
output_footer = '''
#endif
'''

output = output_header + output_body + output_footer
with open(out_file, 'w') as f:
    f.write(output)
exit()




def dict2cpp(d, nested_k=[]):
    for k, v in d.items():
        if type(v) == type({}):
            n = nested_k[:]
            n.append(k)
            dict2cpp(d[k], n)
        else:
            k_str = '.'.join(nested_k)
            if k_str != '':
                k_str += '.' + k
            else:
                k_str = k
            if v == 1.00001:
                print('params.emplace_back("{}", 1.0);'.format(k_str))    
            elif v == 0.00001:
                print('params.emplace_back("{}", 0.0);'.format(k_str))
            elif v == 1.00002:
                print('params.emplace_back("{}", 1);'.format(k_str))    
            elif v == 0.00002:
                print('params.emplace_back("{}", 0);'.format(k_str))    
            elif v == True:
                print('params.emplace_back("{}", true);'.format(k_str))    
            elif v == False:
                print('params.emplace_back("{}", false);'.format(k_str))    
            elif issubclass(type(v), str):
                print('params.emplace_back("{}", "{}");'.format(k_str, v))    
            else:
                print('params.emplace_back("{}", {});'.format(k_str, v))
            

cnt = 0
for yf in os.listdir(yaml_dir):
    yfull = os.path.join(yaml_dir, yf)
    print("")
    print(yfull)
    print("---------------------------------------------------------")
    # print('processing {}'.format(yfull))
    if yfull.split('.')[-1] != 'yaml':
        print('skipping')
        print("---------------------------------------------------------")
        continue
    
    with open(yfull, 'r') as y:
        r_params = yaml.load(y, Loader=yaml.FullLoader)
    
    if '/**' in r_params:
        if 'ros__parameters' in r_params['/**']:
            r_data = r_params['/**']['ros__parameters']
    else:
        r_data = r_params
    
    dict2cpp(r_data)
    print("---------------------------------------------------------")