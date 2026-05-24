workflow:

git clone --recurse-submodules https://github.com/UWARG/autonomy-monorepo
cd autonomy-monorepo
git submodule update --init --recursive

terminal 1
cd SITL-Plus
warg setup
warg run SITL-Plus run -- --vehicle iris

terminal 2
docker compose up ardupilot