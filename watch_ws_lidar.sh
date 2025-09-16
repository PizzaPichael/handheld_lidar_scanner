#1/bin/bash
while inotifywait -r -e modify,create,delete,move src; do
	colcon build
	source install/setup.bash
done
