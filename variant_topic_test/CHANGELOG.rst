^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package variant_topic_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2016-05-12)
------------------
* Fixes related to OS X clang compatibility
* Added message data type member access by name
* * Added type name resolution for member definitions neglecting package names
  * Added receipt time to variant subscriber callback
* * Code cleanups
  * Added variant publisher and subscriber
  * Added MD5 sum calculation for variant message types
  * Updated README
* Added convenience serialization methods to message class
* Added method for serializer creation to variants
* Basic serializer testing
* Added numeric value access for built-in types
* Message variant tests successful
* Message data type now separates constant and variable members
* Refined traits, code cleanups
* Removed SharedVariant and friend-relationship between Variant and CollectionVariant
* Renamed some classes, code cleanups
* * Created test package
  * Moved test binaries to test package
* Contributors: Ralf Kaestner
