# remove old gazebo cache file, so gazebo is forced to use the new height map.
rm -r ~/.gazebo/paging/virtual_maize_field_heightmap

rosrun virtual_maize_field generate_world.py \
--row_length 3.5 \
--rows_left 0 \
--rows_right 7 \
--row_segments straight \
--hole_prob 0.04 \
--max_hole_size 7 \
--litters 5 \
--weeds 5 \
--ghost_objects true
