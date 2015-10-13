
#Header header                        	# header for time/frame information
sensor_msgs/PointCloud2 input_pc     	# defines the point cloud on which grasps should be calculated (objects)
string goal_frame_id				 	# the frame_id to which the point cloud should be transformed before grasps are calculated
geometry_msgs/Point grasp_area_center 	# center of the region where grasps are searched
float32 grasp_area_length_x				# defines the length of the rectangle (in x direction (in m)) where grasps are searched (with center grasp_area_center)
float32 grasp_area_length_y				# defines the length of the rectangle (in y direction (in m)) where grasps are searched (with center grasp_area_center)


duration max_calculation_time			# maximal calculation time before grasp result is returned
bool show_only_best_grasp				# If true, only the best grasp is shown in visualization
int32 threshold_grasp_evaluation		# defines the threshold for return_first_grasp_over_th if it is set to true (otherwise it is ignored)

geometry_msgs/Vector3 approach_vector	# defines the direction from where a grasp should be executed 
int32 gripper_opening_width				# defines the factor (actually 1/factor) that is used for pre-grasp opening gripper width 

#geometry_msgs/Vector3 scale_gripper	# Scale of the gripper in x and y direction if it deviates from a gripper of the size of a huMAN hand

