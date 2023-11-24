^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vesc_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2020-12-12)
------------------
* Merge pull request `#1 <https://github.com/f1tenth/vesc/issues/1>`_ from f1tenth/melodic-devel
  Updating for Melodic
* Exclude crc.h from roslint.
* Replacing boost::crc with CRCPP.
* Replacing boost::begin, boost::end, and boost::distance with Standard Library equivalents.
* Replacing boost::bind with Standard Library equivalent.
* Replaing boost::noncopyable with C++ equivalent.
* Replacing boost::function with Standard Library version.
* Replacing Boost smart pointers with Standard Library equivalents.
* Removing unnecessary v8stdint.h.
* Updating package.xml to format 3 and setting C++ standard to 11.
* Contributors: Joshua Whitley

1.0.0 (2020-12-02)
------------------
* Applying roslint and replacing registration macro with templated class.
* Adding roslint.
* Adding licenses.
* Updating maintainers, authors, and URLs.
* added onboard car
* Contributors: Joshua Whitley, billyzheng
