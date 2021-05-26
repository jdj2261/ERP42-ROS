# erp42_driver

This Package is for calculating the robot's odometry after successful communication connection.

### Usage

`$roslaunch erp42_driver erp42_driver.launch`

~~~
<?xml version="1.0"?>
<launch>
        <arg name="namespace" default="/erp42_serial"/>
        <group ns="$(arg namespace)">
            <node pkg="erp42_driver" name="erp42_driver_node" type="erp42_driver_node" output="screen">
                <param name="wheel_radius" value="0.265"/>
                <param name="wheel_base" value="1.040"/>
                <param name="wheel_tread" value="0.985"/>
                <param name="max_vel" value="5.0"/>
                <param name="min_vel" value="-5.0"/>
                <param name="max_steer_angle" value="28.169"/>
                <param name="min_steer_angle" value="-28.169"/>
            </node>
        </group>
</launch>
~~~

The default namespace is **/erp42_serial**.

But in the case of CAN communication, the namespace is **/erp42_can**.

So, `roslaunch erp42_driver erp42_driver.launch namespace:=/erp42_can`

