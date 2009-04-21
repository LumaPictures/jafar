# $Id$ #

#
#/** GDHE macro to draw landmarks. \file gdhe.tcl  \ingroup slam */
#

namespace eval slam {

    proc drawPoint {r g b {id -1} {radius 0.10}} {
	color $r $g $b
	if {$id != -1} {
	    drawString 0 0 0.2 "$id"
	}
	sphere 0 0 0 $radius
    }

}

package provide slam 2.0