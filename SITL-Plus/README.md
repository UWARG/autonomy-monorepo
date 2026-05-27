workflow:

git clone --recurse-submodules https://github.com/UWARG/autonomy-monorepo
cd autonomy-monorepo
git submodule update --init --recursive

terminal 1
warg run SITL-Plus run

terminal 2
warg run SITL-Plus airside

terminal 3
warg run SITL-Plus groundside

