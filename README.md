# FCND-Motion-Planning
2nd Project - 3D Motion Planning 


# Explaning the Starter Code

* 1- Read Global Home , Global Position and Local Position
* 2- Read "Collider.csv" file and assign those values to a data array
* 3- Creat Grid via Planning_utils.py
  * a-Find North_min , North_max , East_min and East_max
  * b-Find Nort and East Size 
  * c-Creat a zero grid array by using Step 2 values 
  * d-Find obstacles and insert into the grid array 
* 4- Define Start and Goal Point
* 5- Run A* Search Algorithm to find parth via Planning_utils.py
* 6- Create Waypoint List by using Path which is found by A* Search Algorithm ( Step 5 )
* 7- Send Waypoint List to Simulator

# Implementing Path Planning Algorithm

