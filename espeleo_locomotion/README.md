# Espeleo Locomotion

This is a ROS package containing the node and library for EspeleoRobo locomotion. It provides a Skid-steer (espeleo_locomotion_lib) and differential kinematics (espeleo_locomotion_differential_lib) and an odometry publisher.

## Espeleo wheel configuration

![image](https://user-images.githubusercontent.com/44469467/76210126-3bae4000-61e2-11ea-8136-c450aebd3c85.png)


## F.A.Q.

### Error with linalg l
When trying to run **espeleo_locomotion.py**, if you get an error similar to:
```
x = np.linalg.lstsq(self.wheeled_kinematic_A, b, rcond=None)[0]
    File "/usr/lib/python2.7/dist-packages/numpy/linalg/linalg.py", line 1915, in lstsq 0, work, -1, iwork, 0)
    TypeError: a float is required
```
It is probably due to an older **numpy** version. Try to update it using:
```
$ sudo pip install --upgrade numpy
```
Now **espeleo_locomotion.py** should work correctly.
When updating **numpy**, if you get the following error:
```
Traceback (most recent call last):
  File "/usr/bin/pip", line 9, in <module>
    from pip import main
ImportError: cannot import name main
```
It is probably because even your **pip** version is old. Update it using:
```
$ sudo -H pip install --upgrade pip
```
