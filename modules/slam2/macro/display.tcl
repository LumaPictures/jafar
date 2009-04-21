# $Id$ #

#
#/** macro to draw on 2D display. \file display.tcl  \ingroup slam */
#

namespace eval slam {

    proc displayOmniZones {img omniImageManager} {
	# display selection zone of omniImageManager on img

	set camera [$omniImageManager cget -camera]

	# draw circles
	set radiusStep [expr ([$camera cget -imageRadius] - [$camera cget -maskRadius] - 2*[$omniImageManager cget -imageMargin]) / [$omniImageManager cget -nbZonesInRadius]]
	set radius [expr [$camera cget -maskRadius] + [$omniImageManager cget -imageMargin]]
	for {set i 0} {$i <= [$omniImageManager cget -nbZonesInRadius]} {incr i} {
	    display::drawCircle $img  [$camera cget -mirrorCenterU]  [$camera cget -mirrorCenterV] $radius 2 "blue" "zone"
	    set radius [expr $radius + $radiusStep]
	}

	# draw radius
	set 2pi [expr 2*3.14159265358979323846]
	set thetaStep [expr $2pi / [$omniImageManager cget -nbZonesInPeriphery]]
	set theta 0
	set radius1 [expr [$camera cget -maskRadius] + [$omniImageManager cget -imageMargin]]
	set radius2 [expr [$camera cget -imageRadius] - [$omniImageManager cget -imageMargin]]
	for {set i 0} {$i <= [$omniImageManager cget -nbZonesInPeriphery]} {incr i} {
	    set u1 [expr [$camera cget -mirrorCenterU] + cos($theta)*$radius1]
	    set v1 [expr [$camera cget -mirrorCenterV] + sin($theta)*$radius1]
	    set u2 [expr [$camera cget -mirrorCenterU] + cos($theta)*$radius2]
	    set v2 [expr [$camera cget -mirrorCenterV] + sin($theta)*$radius2]

	    display::drawLine $img $u1 $v1 $u2 $v2 2 "blue" "zone"
	
	    set theta [expr $theta + $thetaStep]
	}
    }

}

package provide slam 2.0