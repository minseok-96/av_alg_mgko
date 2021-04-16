#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys


current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *


load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes=node_set.nodes
links=link_set.lines
print('# of nodes: ', len(node_set.nodes))
print('# of links: ', len(link_set.lines))


#for idx in nodes :
#	print(nodes[idx].point)
nodes['A119BS010285'].print_all_related_nodes_and_links()
#print(node_set.nodes['A119BS010285'].point)  # 노드 이름들 출력
# print(link_set.lines.keys())
# print(node_set.nodes['A119BS010318'].print_all_related_nodes_and_links())
# print(link_set.lines['A219BS010644'].from_node.junctions)
