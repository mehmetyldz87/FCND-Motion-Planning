# FCND-Motion-Planning
2nd Project - 3D Motion Planning 


## Explaning the Starter Code

* 1- Read Global Home , Global Position and Local Position
```
print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
```
* 2- Read "Collider.csv" file and obtaining obstacle in the map
```
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
```    
* 3- Creat Grid with a particular altitude and safety margin around obstacles via Planning_utils.py 
```
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

```  
      * a-Find North_min , North_max , East_min and East_max
      * b-Find Nort and East Size 
      * c-Creat a zero grid array by using Nort and East Size  
      * d-Find obstacles and insert into the grid array
  
* 4- Define Start and Goal Point
```    
 grid_start = (-north_offset, -east_offset)
       
 grid_goal = (-north_offset + 10, -east_offset + 10)
```
* 5- Run A* Search Algorithm to find parth via Planning_utils.py
```    
  path, _ = a_star(grid, heuristic, grid_start, grid_goal)
```
* 6- Create Waypoint List by using Path which is found by A* Search Algorithm ( Step 5 )
```    
   waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
```
* 7- Send Waypoint List to Waypoint Array
```    
  self.waypoints = waypoints
```
* 8- Send Waypoint List to Simulator
```    
  self.send_waypoints()
```

## Implementing Path Planning Algorithm

