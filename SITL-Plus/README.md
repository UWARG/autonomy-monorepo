Windows requirements for docker:
Windows 11 with WSLg
WSL 2 

open wsl ubuntu terminal:
docker compose build
docker compose up

If you want more or less than 3 tcp streams, edit sensor_ports.py and docker-compose.yml and adjust accordingly to how many ports you want.

if not using docker:
Windows:
change tcp address in camera.py and range_finder.py to local address 127.0.0.1,

sim_vehicle command on wsl:
python3 ./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --model JSON:{insert your ipv_4 id}  --console --map --out tcpin:0.0.0.0:5761

terminal 1: warg run sitl-plus run
terminal 2: warg run sitl-plus airside
terminal 3: warg run sitl-plus groundside