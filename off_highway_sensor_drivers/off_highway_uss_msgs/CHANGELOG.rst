^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_uss_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.3 (2024-07-09)
------------------

0.6.2 (2024-06-13)
------------------

0.6.1 (2024-06-04)
------------------

0.6.0 (2024-05-14)
------------------

0.5.1 (2024-03-27)
------------------
* Disable irrelevant warnings for Clang
  Anonymous structs are used in PCL for type punning:
  https://github.com/PointCloudLibrary/pcl/issues/2303
  Subobject braces are not needed:
  https://bugs.llvm.org/show_bug.cgi?id=21629
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25137
* Contributors: Robin Petereit
