# $Id$ #

#
#/** demo macro for HelloWorld class. \file demoHello.tcl  \ingroup helloworld */
#

package require helloworld

namespace eval helloworld {

    proc demoHello { n } {
        # this is a demo tcl procedure
        # create n HelloWorld object and print them
        for {set i 0} {$i<$n} {incr i} {
            set h [helloworld::new_HelloWorld "Hello world ! ($i)"]
            $h printHello
	    helloworld::delete_HelloWorld $h
        }
    }

}

package provide helloworld 0.3
