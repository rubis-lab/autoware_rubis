import os
import yaml

target = [
    'rubis_detect',
    'rubis_drive',
    'point_cloud_filter_transform',
    'point_cloud_fusion',
    'voxel_grid',
    'ray_ground_classifier',
    'euclidean_cluster',
    'ndt_localizer'
]



yaml_dir = '/home/rubis/AutowareAuto/src/rubis_main/param/'

# output
out_file = '/home/rubis/AutowareAuto/src/rubis_main/include/rubis_main/rubis_main_parsed.hpp'

output_header = '''
#ifndef RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
#define RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_

#include "rclcpp/rclcpp.hpp"
'''

output_body = '''
'''
output_footer = '''
#endif  //RUBIS_MAIN__RUBIS_MAIN_PARSED_HPP_
'''

def dict2cpp(d, nested_k=[]):
    temp_string = ''
    for k, v in d.items():
        if type(v) == type({}):
            n = nested_k[:]
            n.append(k)
            temp_string += dict2cpp(d[k], n)
        else:
            k_str = '.'.join(nested_k)
            if k_str != '':
                k_str += '.' + k
            else:
                k_str = k
            if v == 1.00001:
                temp_string += '  params.emplace_back("{}", 1.0);\n'.format(k_str)
            elif v == 0.00001:
                temp_string += '  params.emplace_back("{}", 0.0);\n'.format(k_str)
            elif v == 1.00002:
                temp_string += '  params.emplace_back("{}", 1);\n'.format(k_str)
            elif v == 0.00002:
                temp_string += '  params.emplace_back("{}", 0);\n'.format(k_str)
            elif v == True:
                temp_string += '  params.emplace_back("{}", true);\n'.format(k_str)
            elif v == False:
                temp_string += '  params.emplace_back("{}", false);\n'.format(k_str)
            elif issubclass(type(v), str):
                temp_string += '  params.emplace_back("{}", "{}");\n'.format(k_str, v)
            else:
                temp_string += '  params.emplace_back("{}", {});\n'.format(k_str, v)

    return temp_string

cnt = 0
for yf in os.listdir(yaml_dir):
    yfull = os.path.join(yaml_dir, yf)
    if yfull.split('.')[-1] != 'yaml':
        # print('skipping')
        # print("---------------------------------------------------------")
        continue

    node_name = yfull.split('param/')[-1].split('.param')[0]
    
    if node_name not in target:
        continue

    # output_footer += node_name
    if node_name == "point_cloud_filter_transform":
        output_header += '#include \"' + node_name + "_nodes" + '/' +  node_name + '_node.hpp\"\n'
        
        with open(yfull, 'r') as y:
            r_params = yaml.load(y, Loader=yaml.FullLoader)
    
        if '/**' in r_params:
            if 'ros__parameters' in r_params['/**']:
                r_data = r_params['/**']['ros__parameters']
        else:
            r_data = r_params

        for i in r_data:

            output_body += "\n"
            # output_body += yfull    #yfull = yaml file name
            output_body += "rclcpp::NodeOptions configure_"
            output_body += node_name + "_" + i + "_nodes(void) {\n"
            output_body += "  std::vector<rclcpp::Parameter> params;\n\n"
            # print('processing {}'.format(yfull))

            output_body += dict2cpp(r_data[i])
            output_body += "  rclcpp::NodeOptions node_options;\n"
            output_body += "  node_options.parameter_overrides(params);\n\n"
            output_body += "  return node_options;\n}\n"

    elif node_name == "rubis_drive" or node_name == "rubis_detect":
        output_header += '#include \"' + node_name + '/' +  node_name + '_node.hpp\"\n'

        output_body += "\n"
        # output_body += yfull    #yfull = yaml file name
        output_body += "rclcpp::NodeOptions configure_"
        output_body += node_name + "_nodes(void) {\n"
        output_body += "  std::vector<rclcpp::Parameter> params;\n\n"
        # print('processing {}'.format(yfull))
        
        with open(yfull, 'r') as y:
            r_params = yaml.load(y, Loader=yaml.FullLoader)
        
        if '/**' in r_params:
            if 'ros__parameters' in r_params['/**']:
                r_data = r_params['/**']['ros__parameters']
        else:
            r_data = r_params
        
        output_body += dict2cpp(r_data)+"\n"
        output_body += "  rclcpp::NodeOptions node_options;\n"
        output_body += "  node_options.parameter_overrides(params);\n\n"
        output_body += "  return node_options;\n}\n"

    elif node_name == "voxel_grid":
        output_header += '#include \"' + node_name + "_nodes" + '/' +  "voxel_cloud" + '_node.hpp\"\n'

        output_body += "\n"
        # output_body += yfull    #yfull = yaml file name
        output_body += "rclcpp::NodeOptions configure_"
        output_body += node_name + "_nodes(void) {\n"
        output_body += "  std::vector<rclcpp::Parameter> params;\n\n"
        # print('processing {}'.format(yfull))
        
        with open(yfull, 'r') as y:
            r_params = yaml.load(y, Loader=yaml.FullLoader)
        
        if '/**' in r_params:
            if 'ros__parameters' in r_params['/**']:
                r_data = r_params['/**']['ros__parameters']
        else:
            r_data = r_params
        
        output_body += dict2cpp(r_data)+"\n"
        output_body += "  rclcpp::NodeOptions node_options;\n"
        output_body += "  node_options.parameter_overrides(params);\n\n"
        output_body += "  return node_options;\n}\n"
        
    elif node_name == "ray_ground_classifier":
        output_header += '#include \"' + node_name + "_nodes" + '/' +  node_name + '_cloud_node.hpp\"\n'

        output_body += "\n"
        # output_body += yfull    #yfull = yaml file name
        output_body += "rclcpp::NodeOptions configure_"
        output_body += node_name + "_nodes(void) {\n"
        output_body += "  std::vector<rclcpp::Parameter> params;\n\n"
        # print('processing {}'.format(yfull))
        
        with open(yfull, 'r') as y:
            r_params = yaml.load(y, Loader=yaml.FullLoader)
        
        if '/**' in r_params:
            if 'ros__parameters' in r_params['/**']:
                r_data = r_params['/**']['ros__parameters']
        else:
            r_data = r_params
        
        output_body += dict2cpp(r_data)+"\n"
        output_body += "  rclcpp::NodeOptions node_options;\n"
        output_body += "  node_options.parameter_overrides(params);\n\n"
        output_body += "  return node_options;\n}\n"    

    elif node_name == "ndt_localizer":
        output_header += '#include \"' + "ndt" + "_nodes" + '/' +  node_name + '_nodes.hpp\"\n'

        output_body += "\n"
        # output_body += yfull    #yfull = yaml file name
        output_body += "rclcpp::NodeOptions configure_"
        output_body += node_name + "_nodes(void) {\n"
        output_body += "  std::vector<rclcpp::Parameter> params;\n\n"
        # print('processing {}'.format(yfull))
        
        with open(yfull, 'r') as y:
            r_params = yaml.load(y, Loader=yaml.FullLoader)
        
        if '/**' in r_params:
            if 'ros__parameters' in r_params['/**']:
                r_data = r_params['/**']['ros__parameters']
        else:
            r_data = r_params
        
        output_body += dict2cpp(r_data)+"\n"
        output_body += "  rclcpp::NodeOptions node_options;\n"
        output_body += "  node_options.parameter_overrides(params);\n\n"
        output_body += "  return node_options;\n}\n"   

    else:
        output_header += '#include \"' + node_name + "_nodes" + '/' +  node_name + '_node.hpp\"\n'

        output_body += "\n"
        # output_body += yfull    #yfull = yaml file name
        output_body += "rclcpp::NodeOptions configure_"
        output_body += node_name + "_nodes(void) {\n"
        output_body += "  std::vector<rclcpp::Parameter> params;\n\n"
        # print('processing {}'.format(yfull))
        
        with open(yfull, 'r') as y:
            r_params = yaml.load(y, Loader=yaml.FullLoader)
        
        if '/**' in r_params:
            if 'ros__parameters' in r_params['/**']:
                r_data = r_params['/**']['ros__parameters']
        else:
            r_data = r_params
        
        output_body += dict2cpp(r_data)+"\n"
        output_body += "  rclcpp::NodeOptions node_options;\n"
        output_body += "  node_options.parameter_overrides(params);\n\n"
        output_body += "  return node_options;\n}\n"        
    # print("---------------------------------------------------------")

output = output_header + output_body + output_footer
with open(out_file, 'w') as f:
    f.write(output)