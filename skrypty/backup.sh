#!/bin/bash
day=$(date +"%d_%m")
hour=$(date +"%H-%M")

cd MichalDzialakiewicz/projekt-przejsciowy/pomiary_x
cp ~/aero_ws/src/bbp_controller/scripts/results.csv ./
mv results.csv results_$day\_$hour.csv

cd ..
git add *
git commit -m "X $day $hour"
git push
