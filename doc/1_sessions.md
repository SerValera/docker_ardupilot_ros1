

#### To enter the docker use from host pc:
``` bash
docker exec -it ardupilot_sim bash
``` 

### Start simulation (ALL IN DOCKER): 

In first terminal launch: (first run will take a few seconds)

```sh
roslaunch drone_sim launch_world_drone.launch 
```

In second terminal window, enter the ArduCopter directory and start the SITL simulation (It will do compilation in first launch):

```sh
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```

Then in third terminal:

```sh
roslaunch drone_sim apm.launch
```

Then in third terminal:

```sh
roslaunch drone_sim ground_station.launch
```



### Remove ros logs is case low memory

```bash
rm -rf ~/.ros
```


[Back: Go back to docker preporation](0_docker.md)