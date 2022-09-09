# crazyflie-local
<table>
<tr>
<td>  File    </td> <td> Description </td>
</tr>

<tr>
<td> crazyflie_connect_testv6.py </td>
<td>
Same as v5, now with yaw pitch roll from both the stabilizer (gyro) and our own lighthouse angles graphed and orientation matrix also graphed. Notes: The yaw and pitch measurements match up pretty well, but our roll measurements seem to have some trouble when rolling too far away from the lighthouse (facing the opposite direction), which is expected.
</td>
</tr>

<td> position_cal.py </td>
<td>
Helper functions for calculating the position and orientation from the raw sweep angles.
</tr>

<td> pos_grapher.py </td>
<td>
Helper functions for graphing lines in 3d for the position graphs and the 3d orientation matrix.
</td>
</tr>

<td> kalman_testing.py </td>
<td>
Untested, some basic incomplete starter code for future Extended Kalman Filter testing.
</tr>


<td> lighthouse_testew.py </td>
<td>
Test Lighthouse logger from https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/lighthouse/multi_bs_geometry_estimation.py, runs estimate geometry for the Lighthouses.
</td>
</tr>

## Testing Setup
1. Plug in Base Stations (should light up green, the light will be blue when they are moved and go back to green when they are at rest)
2. Plug in Crazyflie radio to computer (USB-A)
3. Turn on Crazyflie (plug in on board battery)
4. Run cfclient in terminal -> cfclient will open
    1. Click Scan
    2. Click Connect
    3. Go to the Lighthouse Positioning tab
        1. Click Manage Geometry
        2. Click Estimate Geometry
        3. Click Write to Crazyflie
5. Move the Crazyflie and see if the Base Stations and the Crazyflie show up properly in the cfclient
6. Click Disconnect to disconnect from cfclient
7. Navigate to crazyflie-local
8. Run python3 [filename].py
9. Move the Crazyflie around

## Running the Arena Simulation
1. Create a room in Arena
2. Open arenasim.py and change the 'scene' and 'namespace' parameters to fit the room you created.
3. Choose the testing file you want to run [parameter 'f'].
4. Run the arenasim.py file and switch back to your Arena page.

Graphics should pop up tracing the crazyflie’s position in 3D after 10 seconds (this can be changed in simple_log_async function with 
```python
time.sleep(seconds)
```


Project Write Up: https://docs.google.com/document/d/1VSMvVgrFVtS3kB8ejUPHN-_43rjTEzWLhK_M4yw6zpg/edit#

Elaine's Github: https://github.com/elainemwang/crazyflie-local 
