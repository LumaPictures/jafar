# $Id$ #

package require kernel

set log [kernel::new_DataLog "log.dat"]
kernel::print $log

$log writeComment "Test of DataLog class"
$log writeTime

for {set i 0} {$i <= 10} {incr i} {
    puts $i
    $log write "$i [expr 2*$i]"
}

$log breakLogging

$log write "*** this should not be there ***"

puts "isLogging: [$log isLogging]"

$log resumeLogging

$log endl 

$log write "end of test"
