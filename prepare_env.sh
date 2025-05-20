#!/bin/bash

cd ~/catkinws/OSSDC-SIM-ART-Linux/wise
python3 -m http.server 9090 &

cd ~/catkinws/OSSDC-SIM-ART-Linux/
./start_sim_local.sh &

cd ~/catkinws/HitchOpen-LGSVL
lgsvl_bridge --port 9091


