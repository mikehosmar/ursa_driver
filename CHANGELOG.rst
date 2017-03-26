^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ursa_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Forthcoming
-----------
* Added frame to publisher
* Minor Change
* Merge pull request `#1 <https://github.com/mars-uoit/URSAII-Driver/issues/1>`_ from mars-uoit/travis
  Added Travis CI and roslaunch check.
* Added Travis CI and roslaunch check.
* Fixed changelog for github 2
* Fixed changelog for github
* fixed README.md
* Contributors: Tony Baltovski, mikehosmar

0.1.0 (2015-09-10)
------------------
* Initial public release
* removed compiled doxygen
* Interface class documentation finished.
* More code documentation.
* The beginings of doxygen docs.
* Added services to ROS node for start/stoping acquiring and clearing spectra. Used when not in imeadiate mode. Some formatting.
* Fixed battery voltage issues incuding fixed `#5 <https://github.com/mars-uoit/URSAII-Driver/issues/5>`_.  Renamed mcs mode back to GM mode to make driver make more sense. Commented out dangerous untested function in driver.  Moved enums to ursa namespace.  Added coarse gain INFO. Formatting Fixes.
* Contributors: mikehosmar

0.0.1 (2015-08-17)
------------------
* Tested ROS Node in Imeadiate mode.  Made some minor changes to the Node.  Added launch file.  Start/stop services to come.
* Added basic ROS Node (not tested)
  Added counter to HV ramp for load prev
  Created messages for GM(MCS) mode and spectrum mode.
  Renamed main.cpp to ursa_example.cpp
* Finished GM mode requestCounts function. Added clearSpectra function.  Cleaned up main to be later used as example.  Made more logical error messages.  Small serial performance changes.
* Finished threshold/offset function. Minimum offset has been laft as a variable.  Renamed getPulses to getSpectra.
* Many of the functions are now tested and working.  Battery voltage in non acquire mode and setting threshold and offset still needs finishing.
* Merge branch 'master' of bitbucket.org:mikehosmar/ursaii-driver
* Added gain setting function.  Tested with real URSA
* README.md edited online with Bitbucket
* Initial Commit, layout almost finished.
* Contributors: mikehosmar


.. _`0.0.0`: https://github.com/mars-uoit/URSAII-Driver/commit/e1c2bf2
.. _`0.0.1`: https://github.com/mars-uoit/URSAII-Driver/compare/e1c2bf2...0.0.1
.. _`0.1.0`: https://github.com/mars-uoit/URSAII-Driver/compare/0.0.1...0.1.0