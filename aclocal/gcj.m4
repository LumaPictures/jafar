# Check for Java compiler.                                  -*- Autoconf -*-
# For now we only handle the GNU compiler.

# Copyright (C) 1999, 2000, 2003, 2005  Free Software Foundation, Inc.
#
# This file is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

AC_DEFUN([AM_PROG_GCJ],[
AC_CHECK_PROGS(GCJ, [gcj gcj-3.2 gcj-3.1 gcj-3.0 gcj-2.95], gcj)
test -z "$GCJ" && AC_MSG_ERROR([no acceptable gcj found in \$PATH])
if test "x${GCJFLAGS-unset}" = xunset; then
   GCJFLAGS="-g -O2"
fi
AC_SUBST(GCJFLAGS)
_AM_DEPENDENCIES(GCJ)
])
