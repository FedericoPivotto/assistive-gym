^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_parallel_gripper_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2023-05-11)
------------------

1.0.8 (2023-03-23)
------------------

1.0.7 (2023-02-15)
------------------
* Merge branch 'add_is_grasped' into 'erbium-devel'
  Add is_grasped topic working in simulation and on the real robot
  See merge request robots/pal_gripper!19
* Merge branch 'release-service' into 'add_is_grasped'
  Release service
  See merge request robots/pal_gripper!21
* Remove opening time parameter and add arguments to launch file
* Update if statemens with python syntax, remove uneccesary white lines
* Fix changes
* add service to release
* fix typo and add tolerance to launch file
* Add tolerance parameter for the pal gripper + comments
* Apply Sai suggestion for naming and syntax
* Improve grasping in simulation and on real robot + change is_grasped topic name
* Add is_grasped topic working in simulation and on the real robot
* Contributors: David ter Kuile, saikishor, thomaspeyrucain

1.0.6 (2022-09-23)
------------------

1.0.5 (2021-06-11)
------------------
* Merge branch 'grasp_service_fix' into 'erbium-devel'
  Grasp service fix
  See merge request robots/pal_gripper!11
* Reduce a bit the max position error
* Fix the command publisher topic name
* Contributors: Sai Kishor Kothakota, victor

1.0.4 (2021-03-29)
------------------
* Merge branch 'gripper_state_fix' into 'erbium-devel'
  Gripper state fix
  See merge request robots/pal_gripper!10
* Fix the node name to avoid double underscores
* Update gripper_grasping.py
* Contributors: jordanpalacios, saikishor

1.0.3 (2020-04-30)
------------------

1.0.2 (2019-06-11)
------------------
* Merge branch 'more-side-fixes' into 'erbium-devel'
  More side fixes
  See merge request robots/pal_gripper!8
* More side fixes
* Contributors: Victor Lopez

1.0.1 (2019-03-26)
------------------
* Merge branch 'add-sides' into 'erbium-devel'
  Add option to specify side of gripper
  See merge request robots/pal_gripper!7
* Add option to specify side of gripper
* Contributors: Victor Lopez

1.0.0 (2018-07-30)
------------------

0.0.13 (2018-04-13)
-------------------

0.0.12 (2018-02-20)
-------------------

0.0.11 (2018-01-24)
-------------------

0.0.10 (2018-01-24)
-------------------
* move scripts and config files from tiago_robot
* Contributors: Jordi Pages

0.0.9 (2016-10-14)
------------------
* fix maintainer
* 0.0.8
* Update changelog
* 0.0.7
* Update changelogs
* Merge branch 'master' of gitlab:robots/pal_gripper
* Added a service to grasp, so, to close the gripper up to a point where its not forcing excesively the motors so they don't warm up so much
* 0.0.6
* Update cahngelog
* 0.0.5
* Update changelog
* 0.0.4
* Update changelgo
* 0.0.3
* Update changelogs
* Fixed package.xml
* Add changelog
* Fixed open value
* Add install rules and a couple of testing scripts
* Tested version
* initial version of fake parallel gripper controller
* Contributors: Jordi Pages, Sam Pfeiffer, Victor Lopez

0.0.1 (2016-06-01)
------------------
