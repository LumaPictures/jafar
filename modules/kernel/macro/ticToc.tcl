# $Id$ #

#
#/** tic and toc macro to measure elapsed time in millisecond (like matlab) \file ticToc.tcl \ingroup kernel */
#

package require kernel

namespace eval kernel {

    proc tictoc {command} {
        # measure duration of command (in millisecond)
        kernel::tic
        eval $command
        return [kernel::toc]
    }

}

package provide kernel 0.2
