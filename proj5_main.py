import numpy as np
import cv2
import os
import time
import csv
import matplotlib.pyplot as plt
import random

print("Project 5 RRT A* implementation")

start_t = time.time()

def rand_node():
	while True:
		rand_x = random.uniform(0, 600)
		rand_y = random.uniform(0, 250)
		if not obstacle(rand_x,rand_y):
			return rand_x, rand_y

def shortest_distance(coords, rand_point):
	distances = np.sqrt(np.sum(np.square(coords - rand_point), axis=1))
	min_distance = np.min(distances)
	min_index = np.argmin(distances)
	return min_distance, min_index

def obstacle(x,y):          # obstacle map
	if((x>=100  and x<=150 and y>=0 and y<=100)   or (x>=100 and x<=150 and y>=150 and y<=250) or      # for upper and lower rectangle
		((y >= 2*x - (895)) and (y <= -2*x + (1145)) and (x >= 460)) or                            # for triangle
		((x >= (230)) and (x <= (370)) and ((x + 2*y) >= 395) and ((x - 2*y) <= 205) and ((x - 2*y) >= -105) and ((x + 2*y) <= 705))  or  # for hexagon
		(x<=0 ) or (x>=600 ) or (y<=0) or (y>=250)):      # for boundary
		return 1
	else:
		return 0

def sample_k_pts(k):
	sample_pts = np.zeros([k,7], dtype=np.float32)
	for j in range(k):
		rand_x, rand_y = rand_node()
		sample_pts[j] = np.array([rand_x, rand_y, 0, 0, 0, 0, 0])

	return sample_pts


def find_closest_points_list(coords, pt_list):

	for pt in pt_list:
		min_dist, min_idx = shortest_distance(coords, pt[0], pt[1])

def path(center_coords, radius, rand_coords):
	if np.sqrt(np.sum(np.square(center_coords - rand_coords))) < radius:
		return -1,-1
	dr = 1
	r_curr = 0
	x_curr = center_coords[0]
	y_curr = center_coords[1]
	theta = np.arctan2(rand_coords[1] - y_curr, rand_coords[0] - x_curr)

	while radius > r_curr:
		if obstacle(x_curr,y_curr):
			return -1,-1
		x_curr += dr*np.cos(theta)
		y_curr += dr*np.sin(theta)
		r_curr += dr

	plt.plot([center_coords[0], x_curr], [center_coords[1], y_curr], color="blue")
	return x_curr, y_curr

##################################################### MAIN ################################
initial_x_config = 50
initial_y_config = 50
final_x_config = 50
final_y_config = 220
k_size = 4
start_ctc = 0.0
# Euclidean Distance
# start_ctg = np.sqrt((initial_x_config-final_x_config)**2 + (initial_y_config-final_y_config)**2)

# Manhattan Distance
start_ctg = np.absolute(initial_x_config-final_x_config) + np.absolute(initial_y_config-final_y_config)
start_tc = start_ctc + start_ctg

coords = np.empty((1, 7))   # x,y,node id, parent id
coords = coords[1:]
coords = np.vstack([coords,[initial_x_config, initial_y_config, 0, 0, start_ctc, start_ctg, start_tc]])

goal_threshold = 7
node_radius = 5
x_curr_node = initial_x_config
y_curr_node = initial_y_config

node_id = 0
############################# MAP ###############################
fig, ax = plt.subplots()
plt.xlim(0,600)
plt.ylim(0,250)
plt.plot(initial_x_config, initial_y_config, color='black', marker='o')
plt.plot(final_x_config, final_y_config, color='black', marker='o')
## obstacle space ##
x, y = np.meshgrid(np.arange(0, 600), np.arange(0, 250))
rect1 = (x>=100) & (x<=150) & (y>=0) & (y<=100)
ax.fill(x[rect1], y[rect1], color='lightblue')
rect2 = (x>=100) & (x<=150) & (y>=150) & (y<=250)
ax.fill(x[rect2], y[rect2], color='lightblue')
hex = (x >= (230)) & (x <= (370)) & ((x + 2*y) >= 395) & ((x - 2*y) <= 205) & ((x - 2*y) >= -105) & ((x + 2*y) <= 705)
ax.fill(x[hex], y[hex], color='lightblue')
tri = (y >= 2*x - (895)) & (y <= -2*x + (1145)) & (x >= 460)
ax.fill(x[tri], y[tri], color='lightblue')
boundary1 = (x<=0) 
ax.fill(x[boundary1], y[boundary1], color='lightblue')
boundary2 = (x>=600)
ax.fill(x[boundary2], y[boundary2], color='lightblue')
boundary3 = (y<=0)
ax.fill(x[boundary3], y[boundary3], color='lightblue')
boundary4 = (y>=250)
ax.fill(x[boundary4], y[boundary4], color='lightblue')
#################################################################

######################################### loop ###################################
while np.sqrt((x_curr_node-final_x_config)**2 + (y_curr_node-final_y_config)**2) > goal_threshold:
	j = 0
	k_rand_pts = sample_k_pts(k_size)
	
	for row in k_rand_pts:
		dist, index = shortest_distance(coords[:, :2], row[:2])
		ctc_new = dist + coords[index, 4]
		# ctg_new = np.sum(np.abs(coords[index, :2] - row[:2]))
		ctg_new = np.sqrt(np.sum(np.square(coords[index, :2] - row[:2])))
		# print(ctg_new)
		total_cost = ctc_new + 0.5*ctg_new
		k_rand_pts[j, 3:7] = np.array([index, ctc_new, ctg_new, total_cost])
		j += 1

	min_total_dist_idx = np.argmin(k_rand_pts[:, 6])
	best_new_node = k_rand_pts[min_total_dist_idx]
	parent_idx = best_new_node[3]	

	x_curr_node, y_curr_node = path(coords[index, :2], node_radius, best_new_node[:2])

	if x_curr_node>=0 and y_curr_node>=0:
		coords = np.vstack([coords, [x_curr_node, y_curr_node, node_id, parent_idx, best_new_node[4], best_new_node[5], best_new_node[6]]])
		node_id +=1
#################################################################################

###################################### backtrack ####################
end_t = time.time()

print("Planning time: ", end_t - start_t)

node = coords[-1][3]
# while (node):
# 	print(node)
# 	val = np.where(coords[:, 2] == node)[0]
# 	node = coords[int(val)][3]
# 	plt.plot(coords[int(val)][0], coords[int(val)][1], color='orange', marker='o')

plt.show()