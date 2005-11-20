# $Id$ #

##########################################
# local M4 macros for configure/autoconf #
# DO NOT EDIT THIS FILE !!               #
##########################################

AC_DEFUN(MD_SHLIB_SUFFIX,[
	case $build_os in
	     cygwin*) SHLIB_SUFFIX='.dll' ;;
	     darwin*) SHLIB_SUFFIX='.dylib' ;;
	     hpux*)   SHLIB_SUFFIX='.sl'  ;;
	     *)	      SHLIB_SUFFIX='.so'  ;;
	esac
])


#########################################################

dnl MD_EXTRACT_REGEX(list, regEx)
dnl variable $regExResult returns the first entry in 'list' that matches the
dnl regular expression 'regEx', using the 'expr' utility
dnl $regExResult = "" if nothing is found
AC_DEFUN([MD_EXTRACT_REGEX],
[
    regExResult=""
    if test "$1" != ""; then
        for i in $1; do
            regExResult=`expr "$i" : "$2"`
            if test "$regExResult" != ""; then
                break
            fi
        done
    fi
])dnl

#########################################################

dnl MD_FINDPROG
AC_DEFUN([MD_FINDPROG],
[
    if test "$findprog" = ""; then
        if test "$CYGWIN" = "yes"; then
          findprog="/usr/bin/find"
        else
          findprog="find"
        fi
    fi
])dnl

#########################################################

dnl MD_FIND_LIBRARY(packageName, packageLib)
dnl example:
dnl     MD_FIND_LIBRARY(tiff, tiff)
AC_DEFUN([MD_FIND_LIBRARY],
[
    AC_MSG_CHECKING([for lib$2 ])

    # check whether the include is in the path by default (e.g. /usr/include)
    lib_found=""
    if test "$with_[$1]lib" = "" -a "$with_[$1]" = "yes"; then
        ac_save_LIBS="$LIBS"
        LIBS="$LIBS -l$2"
        AC_TRY_LINK([], [],
            lib_found="in default path",
            lib_found="")
        LIBS=$ac_save_LIBS
    fi

    # if not found by default, try searching for it
    if test "$lib_found" = ""; then
        if test "$libext" = ""; then
            libext=".a"
        fi
	
	MD_SHLIB_SUFFIX
        if test "$solibext" = ""; then
	    solibext="$SHLIB_SUFFIX"
        fi

        if test "$libpre" = ""; then
            libpre="lib"
        fi

        MD_FINDPROG

        dirs=""
        if test "$with_[$1]lib" != ""; then
            dirs=$with_[$1]lib
        elif test "$with_[$1]" != "yes"; then
            dirs=$with_[$1]
        else
            dirs="/usr/local/lib /usr/local/gnu/lib /usr/local/[$1] /usr/local/[$1]/lib /opt/lib /opt/gnu/lib /opt/[$1] /opt/[$1]/lib /opt/local/lib /usr/lib /sw /sw/lib /sw/[$1] /sw/[$1]/lib \$HOME \$HOME/lib \$HOME/[$1] \$HOME/[$1]/lib \$HOME/usr \$HOME/usr/lib \$HOME/usr/[$1] \$HOME/opt \$HOME/opt/lib \$HOME/opt/[$1] \$HOME/local \$HOME/local/include \$HOME/local/[$1]"
        fi
        found=""
        for i in $dirs; do
            if test -d $i; then
                found="$found `$findprog $i -follow -name "${libpre}[$2]$solibext" -print 2> /dev/null; $findprog $i -follow -name "${libpre}[$2]$libext" -print 2> /dev/null`"
            fi
        done

        MD_EXTRACT_REGEX($found, \(.*lib\)/${libpre}$2$solibext)
        if test "$regExResult" = ""; then
            MD_EXTRACT_REGEX($found, \(.*lib\)/${libpre}$2$libext)
        fi
        if test "$regExResult" = ""; then
            MD_EXTRACT_REGEX($found, \(.*\)/${libpre}$2$solibext)
        fi
        if test "$regExResult" = ""; then
            MD_EXTRACT_REGEX($found, \(.*\)/${libpre}$2$libext)
        fi
        lib_found=$regExResult
    fi
    if test "$lib_found" = ""; then
        with_[$1]lib=""
        AC_MSG_RESULT([not found in $dirs])
    else
        AC_MSG_RESULT($lib_found)
        if test "$lib_found" != "/usr/lib" -a \
                "$lib_found" != "in default path"; then
            with_[$1]lib="-L$lib_found"
        else
            with_[$1]lib=""
        fi
    fi
])dnl

#########################################################

dnl MD_FIND_INCLUDE(packageName, packageInc)
dnl example:
dnl     MD_FIND_INCLUDE(tiff, tiff.h)
AC_DEFUN([MD_FIND_INCLUDE],
[
    if test "$2" != ""; then
        AC_MSG_CHECKING([for $2 ])

        # check whether the include is in the path by default (e.g. /usr/include)
        include_found=""
        if test "$with_[$1]inc" = "" -a "$with_[$1]" = "yes"; then
            AC_TRY_COMPILE(
[#include <stdio.h>  /* necessary because jpeglib.h fails to include it */
#include <$2>
], [],
                include_found="in default path",
                include_found="")
        fi

        # if not found by default, try searching for it
        if test "$include_found" = ""; then
            dirs=""
            if test "$with_[$1]inc" != ""; then
                dirs="$with_[$1]inc"
            elif test "$with_[$1]" != "yes"; then
                dirs="$with_[$1]"
            else
                dirs="/usr/local/include /usr/local/gnu/include /usr/local/[$1] /usr/local/[$1]/include /opt/include /opt/gnu/include /opt/[$1] /opt/[$1]/include /opt/local/include /opt/local/include/[$1] /usr/include /sw /sw/include /sw/[$1] /sw/[$1]/include \$HOME/ \$HOME/include \$HOME/usr \$HOME/usr/include \$HOME/usr/[$1] \$HOME/opt \$HOME/opt/include \$HOME/opt/[$1] \$HOME/[$1] \$HOME/local \$HOME/local/include \$HOME/local/[$1]"
            fi

            # use find for searching
            MD_FINDPROG

            # first, look for the given header file without directory components..
            found=""
            for i in $dirs; do
                if test -d $i; then
                    found="$found `$findprog $i -follow -name patsubst([$2], .*/, ) -print 2> /dev/null`"
                fi
            done

            # now, check each found file for relative path prefix.
            MD_EXTRACT_REGEX($found, \(.*include\)/patsubst([$2], \., \\.))
            if test "$regExResult" = ""; then
                MD_EXTRACT_REGEX($found, \(.*\)/patsubst([$2], \., \\.))
            fi
            include_found=$regExResult
        fi

        # report the search result and set result variables
        if test "$include_found" = ""; then
            with_[$1]inc=""
            AC_MSG_RESULT([not found in $dirs])
        else
            AC_MSG_RESULT($include_found)
            if test "$include_found" != "/usr/include" -a \
                    "$include_found" != "in default path"; then
                with_[$1]inc="$include_found"
                [$1]_cppflags="-I$include_found"
            else
                with_[$1]inc=""
                [$1]_cppflags=""
            fi
        fi
    else
        with_[$1]inc=""
        [$1]_cppflags=""
    fi
])dnl

#########################################################

dnl MD_FIND_PACKAGE(packageName, packageLib, packageInc, packageComment)
dnl defines with_packageName=yes/no
dnl         with_packageNamelib=<path>/empty if not found
dnl         with_packageNameinc=<path>/empty if not found
dnl example:
dnl     MD_FIND_PACKAGE(tiff, tiff, tiff.h, support import/export of tiff images)
dnl Stop configure if not found
AC_DEFUN([MD_FIND_PACKAGE],
[
    AC_ARG_WITH([$1], [
  --with-$1
  --with-$1=dir
  --without-$1
  ifelse([$2],[],[], [--with-$1lib=dir])
  ifelse([$3],[],[], [--with-$1inc=dir])
      $4. default: --with-$1
      if --with-$1 or --with-$1=yes is given: $1 package files will be
         searched for in some standard directories (the default).
      if --with-$1=dir is given, and dir is a directory: $1 package files
         will be searched for below 'dir' using 'find'.
      if --with-$1=no or --without-$1 is given: $1 package will
         not be used.
      alternatively, you can specify:], ,)
    ifelse([$2],[],[], [AC_ARG_WITH([$1lib], [        --with-$1lib=dir : the $1 package's lib directory], ,)])
    ifelse([$3],[],[], [AC_ARG_WITH([$1inc], [        --with-$1inc=dir : the $1 package's include directory], ,)])


    # default is "yes"
    if test ${with_[$1]:-""} = ""; then
        with_[$1]="yes"
    fi

    if test ${with_[$1]:-""} != "no"; then
	found_$1="yes"
        ifelse([$3],[],[], [
		MD_FIND_INCLUDE($1, $3)
		if test "x$include_found" = "x"; then
			found_$1="no"
        	fi	
	])
        ifelse([$2],[],[], [
		MD_FIND_LIBRARY($1, $2)
		if test "x$lib_found" = "x"; then
			found_$1="no"
        	fi
	])
	ifelse([$found_[$1]],[no],[AC_MSG_ERROR(Cannot find [$1] . see ./configure --help for more option, 1)])
    fi

])dnl


#########################################################

dnl MD_FIND_PACKAGE_OPT(packageName, packageLib, packageInc, packageComment, packageHeaderDefine)
dnl defines with_packageName=yes/no
dnl         with_packageNamelib=<path>/empty if not found
dnl         with_packageNameinc=<path>/empty if not found
dnl example:
dnl     MD_FIND_PACKAGE_OPT(tiff, tiff, tiff.h, support import/export of tiff images)
dnl juste warn if not found
AC_DEFUN([MD_FIND_PACKAGE_OPT],
[
    AC_ARG_WITH([$1], [
  --with-$1
  --with-$1=dir
  --without-$1
  ifelse([$2],[],[], [--with-$1lib=dir])
  ifelse([$3],[],[], [--with-$1inc=dir])
      $4. default: --with-$1
      if --with-$1 or --with-$1=yes is given: $1 package files will be
         searched for in some standard directories (the default).
      if --with-$1=dir is given, and dir is a directory: $1 package files
         will be searched for below 'dir' using 'find'.
      if --with-$1=no or --without-$1 is given: $1 package will
         not be used.
      alternatively, you can specify:], ,)
    ifelse([$2],[],[], [AC_ARG_WITH([$1lib], [        --with-$1lib=dir : the $1 package's lib directory], ,)])
    ifelse([$3],[],[], [AC_ARG_WITH([$1inc], [        --with-$1inc=dir : the $1 package's include directory], ,)])


    # default is "no"
    if test ${with_[$1]:-""} = ""; then
        with_[$1]="no"
    fi

    if test ${with_[$1]:-""} != "no"; then
	found_$1="yes"
        ifelse([$3],[],[], [
        	MD_FIND_INCLUDE($1, $3)
	 	if test "x$include_found" = "x"; then
			found_$1="no"
        	fi
	])

	ifelse([$2],[],[], [
        	MD_FIND_LIBRARY($1, $2)
		if test "x$lib_found" = "x"; then
            		found_$1="no"
        	fi
	])
	ifelse([$found_[$1]],[no],[AC_MSG_WARN(  Configuring without [$1] support )],[with_[$1]="yes"]);
    else
	with_[$1]="no"
	AC_MSG_WARN(  Configuring without [$1] support )
    fi

])dnl

#########################################################


dnl *********** extra includes ***********
AC_DEFUN(MD_EXTRA_INCLUDES,
	[AC_ARG_WITH(includes, [  --with-includes=DIR     search include DIR for optional packages below])
case "x$withval" in
x/*|x.*)
  extra_include=$withval
  AC_MSG_RESULT([adding $extra_include to include search path for following packages])
  if test ! -d $extra_include; then
    AC_MSG_RESULT([Warning: Directory $extra_include does not exist])
  fi
  ;;
*)
  extra_include=""
  ;;
esac])dnl

dnl *********** extra libs ***************
AC_DEFUN(MD_EXTRA_LIBS,
	[AC_ARG_WITH(libraries, [  --with-libraries=DIR    search library DIR for optional packages below])
case "x$withval" in
x/*|x.*)
  extra_lib=$withval
  AC_MSG_RESULT([adding $extra_lib to library search path for following packages])
  if test ! -d $extra_lib; then
    AC_MSG_RESULT([Warning: Directory $extra_lib does not exist])
  fi
  ;;
*)
  extra_lib=""
  ;;
esac])dnl
