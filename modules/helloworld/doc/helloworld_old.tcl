# $Id$ #

package require helloworld

#
# The low level swig functions
#

set hello [helloworld::new_HelloWorld]

helloworld::HelloWorld_printHello $hello

helloworld::HelloWorld_setHello $hello "Hello world \!"

puts ""

helloworld::HelloWorld_printHello $hello

