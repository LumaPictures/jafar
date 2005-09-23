# $Id$ #

package require kernel
package require helloworld

#
# using procedure new
#

set hw [helloworld::newHelloWorld]

$hw printHello

$hw setHello "Hello world \!"

puts ""

$hw printHello

#
# tcl procedure in package helloworld::demoHello
#

# help procedure
help helloworld::demoHello

helloworld::demoHello 4
