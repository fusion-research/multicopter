# Bash script for compiling the autopilot submodule of multicopter.
cd multicopter
python bois.py
cd autopilot
g++ model.cpp --std=c++11 -O3 -shared -fPIC -o libmodel.so
cd ../..
