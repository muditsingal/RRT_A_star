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

def rand_node_circle(r, x, y):
	while True:
		rand_r = random.uniform(0, r)
		rand_th = np.deg2rad(random.uniform(0, 360))
		rand_x = x + rand_r*np.cos(rand_th)
		rand_y = y + rand_r*np.sin(rand_th)
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

def sample_k_pts(coords, k, nb_radius):
	sample_pts = np.zeros([k,7], dtype=np.float32)
	min_idx_ctg = np.argmin(coords[:, 5])
	print(min_idx_ctg)
	x = coords[min_idx_ctg, 0]
	y = coords[min_idx_ctg, 1]
	j = 0
	while j < k:
		# rand_x, rand_y = rand_node()
		rand_x, rand_y = rand_node_circle(nb_radius, x, y)
		# dist = np.sqrt((x-rand_x)**2 + (y-rand_y)**2)
		# print(rand_x, rand_y)
		rand_pt_arr = np.array([rand_x, rand_y])
		dist, index_s = shortest_distance(coords[:, :2], rand_pt_arr)
		ctc_new = dist + coords[index_s, 4]

		x_curr_node, y_curr_node = path(coords[index_s, :2], node_radius, rand_pt_arr)

		if x_curr_node>=0 and y_curr_node>=0:
			#sample_pts[j] = np.array([rand_x, rand_y, 0, index, ctc_new, 0, 0])
			sample_pts[j] = np.array([x_curr_node, y_curr_node, 0, index_s, ctc_new, 0, 0])
			j += 1

	# time.sleep(2)
	return sample_pts


def sample_k_pts_old(coords, k, nb_radius):
	sample_pts = np.zeros([k,7], dtype=np.float32)
	x = coords[-1, 0]
	y = coords[-1, 1]
	index = int(coords[-1, 2])
	j = 0
	while j < k:
		# rand_x, rand_y = rand_node()
		rand_x, rand_y = rand_node_circle(nb_radius, x, y)
		dist = np.sqrt((x-rand_x)**2 + (y-rand_y)**2)
		# print(rand_x, rand_y)
		rand_pt_arr = np.array([rand_x, rand_y])
		# dist, index = shortest_distance(coords[:, :2], rand_pt_arr)
		ctc_new = dist + coords[index, 4]

		x_curr_node, y_curr_node = path(coords[index, :2], node_radius, rand_pt_arr)

		if x_curr_node>=0 and y_curr_node>=0:
			#sample_pts[j] = np.array([rand_x, rand_y, 0, index, ctc_new, 0, 0])
			sample_pts[j] = np.array([x_curr_node, y_curr_node, 0, index, ctc_new, 0, 0])
			j += 1

	# time.sleep(2)
	return sample_pts


def find_closest_points_list(coords, pt_list):

	for pt in pt_list:
		min_dist, min_idx = shortest_distance(coords, pt[0], pt[1])

def path(center_coords, radius, rand_coords):
	dist = np.sqrt(np.sum(np.square(center_coords - rand_coords)))
	if dist < radius:
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
final_x_config = 375
final_y_config = 220
k_size = 70
start_ctc = 0.0
weight = 1
nb_radius = 35
# Euclidean Distance
# start_ctg = np.sqrt((initial_x_config-final_x_config)**2 + (initial_y_config-final_y_config)**2)

# Manhattan Distance
start_ctg = np.absolute(initial_x_config-final_x_config) + np.absolute(initial_y_config-final_y_config)
start_tc = start_ctc + weight*start_ctg

coords = np.empty((1, 7))   # x,y,node id, parent id
coords = coords[1:]
coords = np.vstack([coords,[initial_x_config, initial_y_config, 0, -1, start_ctc, start_ctg, start_tc]])

goal_threshold = 5
node_radius = 6
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
ax.fill(x[rect1], y[rect1], color='black')
rect2 = (x>=100) & (x<=150) & (y>=150) & (y<=250)
ax.fill(x[rect2], y[rect2], color='black')
hex = (x >= (230)) & (x <= (370)) & ((x + 2*y) >= 395) & ((x - 2*y) <= 205) & ((x - 2*y) >= -105) & ((x + 2*y) <= 705)
ax.fill(x[hex], y[hex], color='black')
tri = (y >= 2*x - (895)) & (y <= -2*x + (1145)) & (x >= 460)
ax.fill(x[tri], y[tri], color='black')
boundary1 = (x<=0) 
ax.fill(x[boundary1], y[boundary1], color='black')
boundary2 = (x>=600)
ax.fill(x[boundary2], y[boundary2], color='black')
boundary3 = (y<=0)
ax.fill(x[boundary3], y[boundary3], color='black')
boundary4 = (y>=250)
ax.fill(x[boundary4], y[boundary4], color='black')
#################################################################

# print(rand_node_circle(8))

######################################### loop ###################################
while np.sqrt((x_curr_node-final_x_config)**2 + (y_curr_node-final_y_config)**2) > goal_threshold:
	k_rand_pts = sample_k_pts(coords, k_size, nb_radius)
	node_id += 1
	j=0
	for row in k_rand_pts:
		ctc_new = row[4]
		index = int(row[3])
		# Euclidean Distance
		# ctg_new = np.sqrt(np.sum(np.square(coords[index, :2] - row[:2])))
		# Manhattan distance
		ctg_new = np.sum(np.absolute(coords[index, :2] - row[:2]))
		# print(ctg_new)
		total_cost = ctc_new + weight*ctg_new
		k_rand_pts[j, 5:7] = np.array([ctg_new, total_cost])
		j += 1

	min_total_dist_idx = np.argmin(k_rand_pts[:, 6])
	best_new_node = k_rand_pts[min_total_dist_idx]
	parent_idx = best_new_node[3]	

	x_curr_node = best_new_node[0]
	y_curr_node = best_new_node[1]
	# print(coords.shape)

	coords = np.vstack([coords, [x_curr_node, y_curr_node, node_id, parent_idx, best_new_node[4], best_new_node[5], best_new_node[6]]])
	

#################################################################################

###################################### backtrack ####################
end_t = time.time()

print("Planning time: ", end_t - start_t)

curr_idx = int(coords[-1][2])
parent_idx = int(coords[-1][3])

while parent_idx >= 0:
	curr_node = coords[curr_idx]
	parent_node = coords[parent_idx]
	x_curr = curr_node[0]
	y_curr = curr_node[1]
	x_parent = parent_node[0]
	y_parent = parent_node[1]
	plt.plot([x_parent, x_curr], [y_parent, y_curr], color="red")
	curr_idx = int(parent_node[2])
	parent_idx = int(parent_node[3])

plt.show()