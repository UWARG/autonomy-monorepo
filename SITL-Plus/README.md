SITL-Plus
For Windows devices, ensure you have Windows 11 and WSL version 2. Then, if you want to use the docker container to run the sim, run :

docker compose build
docker compose up

If you dont have Windows 11, or if you are on Mac, make sure you install an X Server. Linux devices should work with their built in wayland and x11 server/display.  

python3 ./Tools/autotest/sim_vehicle.py -N -v ArduCopter -f quad --model JSON: <YOUR IPV4 ADDR>  --console --map --out tcpin:0.0.0.0:5761

On a seperate terminal, run the pybullet sim, and your airside and groundside code or to run the test code, on seperate terminals run the following:

terminal 1:
warg run sitl-plus run
terminal 2:
warg run sitl-plus airside
terminal 3:
warg run sitl-plus groundside

If you would rather run the sim locally on your own WSL terminal, follow the following guide https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux. After you are finished setting up the environment, run this command in the ardupilot directory on a WSL terminal to start the ardupilot sitl. Also make sure to replace the TCP address in camera.py and range_finder.py with the constant "HOST" to local address 127.0.0.1 


