^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tracking_pid
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2019-04-19)
------------------
* Add Apache 2.0 as license as per ROSIN
* Add backwards compatibility to ROS-Kinetic by overloading planner initialize function.
* Add ~loop param to make interpolator loop
* Deal with 0.0 velocity in Python path_interpolator
* Ported path_interpolator.py/PathInterpolator itself to path_interpolator.cpp and integrating it
* Re-format code according to ROS standard (https://github.com/davetcoleman/roscpp_code_format)
* Contributors: Alaa Alassi, Ferry Schoenmakers, Jasper Verhoeven, Loy van Beek, Mukunda Bharatheesha, Tim Clephas

0.6.4 (2019-01-03)
------------------
* Drive backwards when control point is behind robot (negative)
* Deal with paths that have duplicate poses
* Contributors: Loy van Beek, Tim Clephas

0.6.3 (2018-08-24)
------------------
* Added dynamic reconfigure parameter for velocity
* Add options for different start-time than current time to allow for faster resuming after paused state
* Do not re-initialise interpolator on every pause callback but only when paused
* Contributors: Tim Clephas

0.6.2 (2018-07-19)
------------------
* Publish when the path is done tracking
* The functionality of the Interpolator would also be suitable for an ActionServer, called with a path
* Add node to interpolate nav_msgs/Path and send that to the tracking_pid node
* Added interpolation between data points to allow for paths with data points further apart in time and space
* Add parameter to allow chosing controller frames
* Contributors: Loy van Beek, Tim Clephas, Yuri Steinbuch

0.1.0 (2017-10-16)
------------------
* Added backwards driving compatibility to the controller
* Fixed feedforward issue and some improvements in the perf_logger
* Added performance logger wrt to future lifetime tests
* Added general launch file in which desired trajectory can be set. Added dynamic reconfigure for the global_planner to change trajectories online easily
* Bugfixes and dynamic reconfigure of local planner
* Added feedforward actions, improved overall performance, bug fixes
* Contributors: Michiel Francke, Tim Clephas, Yuri Steinbuch
