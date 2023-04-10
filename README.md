# CG2111A

Install serialize.zip file to arduino before running

gcc alex-pi.cpp serial.cpp serialize.cpp -pthread -o Alex-pi

./Alex-pi

gcc alextest-pi.cpp serial.cpp serialize.cpp -pthread -o alextest-pi

./alextest-pi

roslaunch rplidar_ros rplidar.launch

Alex2_testultra can get distance from front and back from get stats. Latest edition, get distance data from doing other things?

gcc alextestultra-pi.cpp serial.cpp serialize.cpp -pthread -o alextestultra-pi

WASD to move, e to move 2 cm forward, Z to stop, 1 to get colour, 2 to get stats, 3 to clear stats, q to quit

read the RGB value from pi and determine the colour ourself?

ssh -x pi@192.168.22.115
