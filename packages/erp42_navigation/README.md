# erp42_navigation

This is a navigation package using the ros open package. 

It consists of **amcl** launch file  for estimating the current location, 

**DWA or TEB** launch file for Local Planner, and **move_base** launch file for route planning.

- amcl.lanuch
- erp42_dwa_local_planner.launch
- erp42_teb_local_planner.launch
- move_base.launch



### Usage

**If you want to use the DWA Local planner,**

`roslaunch erp42_navigation erp42_dwa_local_planner.launch `

**If you want to use the TEB Local planner**,

`roslaunch erp42_navigation erp42_teb_local_planner.launch `



### Demo

***You can test the navigation in the Gazebo environment.***

**Demo DWA Local planner**,

`roslaunch erp42_vehicle_gazebo erp42_vehicle_gazebo.launch`

`roslaunch erp42_navigation erp42_dwa_local_planner.launch `



**Demo TEB Local planner**,

`roslaunch erp42_vehicle_gazebo erp42_vehicle_gazebo.launch`

`roslaunch erp42_navigation erp42_teb_local_planner.launch `

![erp42_navigation](https://user-images.githubusercontent.com/35681273/119609890-4b3f9600-be33-11eb-8bf3-980880a858b3.gif)