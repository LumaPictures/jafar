# $Id$ #

set jafar_dir $env(JAFAR_DIR)
set jafar_cfg_guess [file join $jafar_dir config config.guess]
set jafar_cfg_sub [file join $jafar_dir config config.sub]

set buildalias [ exec $jafar_cfg_guess ]
if { $buildalias == "" } {
    puts "[jfrsh] Can't guess build system"
    exit
}
set build [exec $jafar_cfg_sub $buildalias]
regsub {^([^-]*)-([^-]*)-(.*)$} $build "\\1-\\3" buildcpu_buildos

set jafar_pkg_dir [file join $jafar_dir tclpkg $buildcpu_buildos]

lappend auto_path $jafar_pkg_dir
set jafar_macros ""

puts ""
puts "Welcome to jafar !         (c) 2004-2005 LAAS-CNRS"
puts ""
puts "jafar_dir: $jafar_dir"
puts ""
puts "Available packages:"

package require kernel
namespace import ::kernel::*

set pkgList [glob -tail -directory $jafar_pkg_dir *]

foreach pkg "$pkgList" {
    set v [package versions $pkg]
    if "[string length $v]==0" {
        set v "** NOT LOADABLE **\ntry \"load [file join $jafar_pkg_dir $pkg  $pkg][info sharedlibextension]\" to guess the problem..."
    }
    puts "  $pkg...$v"
}

puts "" 

