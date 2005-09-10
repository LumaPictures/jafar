# $Id$ #

#
#/** jafar tcl tools. \file tools.tcl \ingroup kernel */
#

namespace eval kernel {

    # proc print {obj} {
    #     regexp {_${[0-9][a-f]}*_p_${[0-9][a-z]}*__${[0-9][a-z]}*__} $obj m1 ptr n1 n2
    # }

    proc help {procname} {
	# reports a proc's args and leading comments.
	# usage: help procname
	# Multiple documentation lines are allowed.
	# documentation can be ended by a blank line

	# This comment should not appear in the docstring
	set res "{usage: $procname [args $procname]}"
	
	set procBody [split [info body $procname] \n]
	set firstLine [lindex $procBody 0]
	if {[string trim $firstLine] eq ""} {
	    set procBody [lrange $procBody 1 end]
	}

	foreach line $procBody {
	    if {[string trim $line] eq ""} break
	        if ![regexp {\s*#(.+)} $line -> line] break
		lappend res [string trim $line]
		set blankLine false
	    }
	    join $res \n
	}

    proc args {procname} {
	# Signature of a proc: arguments with defaults
	set res ""
	foreach a [info args $procname] {
	    if [info default $procname $a default] {
		lappend a $default
	    }
	    lappend res $a
	}
	set res
    }

    proc ask {procname} {
	# interactively asks for proc arguments
	# it also takes into account default arguments if any
	set command [list $procname]
	foreach arg [info args $procname] {
	    set hasDefault [info default $procname $arg defaultValue]
	    set notok 1
	    while {$notok} {
		puts -nonewline "$arg \[$defaultValue\]: "
		flush stdout
		set c [gets stdin argsValue($arg)]
		if "$c>0" {
		    set notok 0
		} elseif "$hasDefault" {
		    set argsValue($arg) $defaultValue
		    set notok 0
		}
	    }
	    lappend command "$argsValue($arg)"
	}
	puts "evaluating \{$command\}"
	uplevel 1 "eval $command"
    }

    proc rehash {} {
	    # re-source user macros in directories listed by jafar_macros
	
	global jafar_macros
	
	if "![info exist jafar_macros]" {
	    puts "jafar_macros variable is not set"
	    return
	}

	foreach dir $jafar_macros {
	    if [file exist $dir] {
		puts "$dir"

		set tclFilesList [glob -nocomplain -directory "$dir" *.tcl]
		
		foreach file $tclFilesList {
		    puts -nonewline "  [lindex [file split $file] end]..."
		    flush stdout
		    if { [catch {uplevel source $file} error] } {
			puts "Warning: $error"
		    } else {
			puts "ok"
		    }
		}
	    } else {
		puts "** skipping $dir **"
	    }
	}
    }

     namespace export help
     namespace export args
     namespace export ask
     namespace export rehash
}

package provide kernel 0.1
