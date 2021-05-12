# remove old gazebo cache file, so gazebo is forced to use the new height map.
rm -r ~/.gazebo/paging/virtual_maize_field_heightmap

rosrun virtual_maize_field generate_world.py \
--row_length 5 \
--rows_left 0 \
--rows_right 5 \
--rows_curve_budget 0.78539816339 \
--row_segments straight,curved \
--row_segment_curved_radius_min 4.0 \
--row_segment_curved_radius_max 5.0


