dnl----------------------------------------------------------------------
dnl
dnl Tcl/Tk Config
dnl 
dnl MD_FIND_FILE(file, path, if-found, if-not-found)
AC_DEFUN([MD_FIND_FILE], [
    AC_MSG_CHECKING([for $1])

    for ac_dir in $2; do
        if test -r "$ac_dir/$1"; then
          md_found_dir=$ac_dir
          break
        fi
    done    
    md_found_file="$md_found_dir/$1"
    AS_IF([test -r $md_found_file], [AC_MSG_RESULT([$md_found_file]); $3], [AC_MSG_RESULT([not found]); $4])
])

dnl MD_FIND_FILE(package, file, path, if-found, if-not-found)
AC_DEFUN([MD_USER_FIND_FILE], [
    AC_ARG_WITH([$1],
        [AC_HELP_STRING([--with-$1=DIR], [directory containing $2])],
        [md_find_path="$withval"], [md_find_path="$3"])
    MD_FIND_FILE([$2], [$md_find_path], [$4], [$5])
])




dnl MD_CHECK_TCLTK([if-found], [if-not_found])
AC_DEFUN([MD_CHECK_TCLTK],[
    has_tcl=yes
    
    MD_USER_FIND_FILE(tcl, [tclConfig.sh], [${exec_prefix}/lib \
            /usr/local/lib/tcl8.4 \
            /usr/local/tcl-8.4/lib \
            /usr/local/tcl-8.3/lib \
            /usr/local/tcl-8.4 \
            /usr/local/lib \
            /usr/pkg/lib \
            /usr/lib/tcl8.4 \
            /usr/lib \
            /sw/lib \ 
            /opt/local/lib], [file=$md_found_file], [has_tcl=no])

    if test "$has_tcl" = "yes"; then
        . $file

        dnl substitute variables in TCL_LIB_FILE
        eval TCL_LIB_FILE=${TCL_LIB_FILE}

        test -z "$tcl_test_include" && tcl_test_include=tcl.h
        MD_FIND_FILE([$tcl_test_include], [$TCL_PREFIX/include/tcl$TCL_VERSION \
                $TCL_PREFIX/include \
                /usr/local/include/tcl$TCL_VERSION \
                /usr/local/include \
                /usr/include \
                /sw/include \ 
                /opt/local/include \
                /Library/Frameworks/Tcl.framework/Headers \
                $extra_include], 
            [ac_tcl_includes="$md_found_dir"], [has_tcl=no])
    fi

    if test "$has_tcl" = "yes"; then
        test -z "$tcl_test_lib" && tcl_test_lib="${TCL_LIB_FILE}"
        MD_FIND_FILE([$tcl_test_lib], [$TCL_PREFIX/lib \
                    /usr/local/lib \
                    /usr/lib \
                    /sw/lib \ 
                    /opt/local/lib \
                    /Library/Frameworks/Tcl.framework \
                    $extra_lib],
                [ac_tcl_libs=$md_found_dir
                    LIB_RUNTIME_DIR=$ac_tcl_libs
                ], [has_tcl=no])
    fi

    if test "$has_tcl" = "yes"; then
        has_tk=yes
        MD_USER_FIND_FILE([tk], [tkConfig.sh], [${TCL_PREFIX}/lib \
                ${exec_prefix}/lib \
                /usr/local/lib/tk$TCL_VERSION \
                /usr/local/tk$TCL_VERSION \
                /usr/local/tk$TCL_VERSION/lib \
                /usr/local/lib \
                /usr/pkg/lib \
                /usr/lib/tk$TCL_VERSION \
                /usr/lib \
                /sw/lib/tk$TCL_VERSION \
                /sw/lib \
                /opt/local/lib/tk$TCL_VERSION \
                /opt/local/lib], 
            [file=$md_found_file], [has_tk=no])
    fi

    if test "$has_tk" = "yes"; then
        . $file
        dnl substitute variables in TK_LIB_FILE
        eval TK_LIB_FILE=${TK_LIB_FILE}

        AC_MSG_CHECKING([for tk headers])
        test -z "$tk_test_include" && tk_test_include=tk.h
        MD_FIND_FILE([$tk_test_include], [$TK_PREFIX/include/tk$TK_VERSION	\
                $TK_PREFIX/include 			\
                $ac_tcl_includes			\
                /usr/local/include/tk$TK_VERSION 	\
                /usr/local/include 			\
                /usr/include 				\
                /sw/include/tk$TK_VERSION 	\
                /sw/include 			\
                /opt/local/include/tk$TK_VERSION \
                /opt/local/include \
                /Library/Frameworks/Tk.framework/Headers \
                $extra_include],
            [ac_tk_includes=$md_found_dir], [has_tk=no])
    fi

    has_tkint=yes
    if test "$has_tk" = "yes"; then
        dnl look for tkInt.h
        MD_FIND_FILE([tkInt.h], [$TK_SRC_DIR/generic \
                $TK_PREFIX/include/generic	\
                $TK_PREFIX/include/tk$TK_VERSION/generic \
                /Library/Frameworks/Tk.framework/PrivateHeaders \
                $srcdir/tk/$TK_VERSION], 
            [ac_tk_int=$md_found_dir], [has_tkint=no])
    fi

    if test "$has_tcl" = yes; then
        if test "$ac_tcl_includes" != "/usr/include"; then
          TCL_CPPFLAGS=-I$ac_tcl_includes
        fi

        eval TCL_LDFLAGS=\"${TCL_LD_FLAGS} ${TCL_LD_SEARCH_FLAGS}\"
    fi

    if test "$has_tk" = yes; then
        if test "$ac_tk_includes" != "$ac_tcl_includes"; then
          TCL_CPPFLAGS="$TCL_CPPFLAGS -I$ac_tk_includes"
        fi
        if test "$ac_dir" = "$srcdir/tk/$TK_VERSION"; then
                TCL_CPPFLAGS="$TCL_CPPFLAGS -I\${abs_srcdir}/tk/${TK_VERSION}"
        else
                TCL_CPPFLAGS="$TCL_CPPFLAGS -I$ac_tk_int"
        fi

        eval TK_LDFLAGS=\"${TK_LD_SEARCH_FLAGS}\"
        if test "$TK_LDFLAGS" != "$TCL_LDFLAGS"; then
          TCL_LDFLAGS="$TCL_LDFLAGS $TK_LDFLAGS"
        fi

        eval TK_SHLIB_LDFLAGS=\"${TK_LD_SEARCH_FLAGS}\"
        TCL_SHLIB_LDFLAGS="$TK_SHLIB_LDFLAGS"
        TCL_LIB_FLAGS="$TCL_LIB_FLAG $TK_LIB_FLAG"
    fi

    AS_IF([test "$has_tcl" = yes && test "$has_tk" = yes], [
        HAS_TCL_SUPPORT=yes

        AC_SUBST(HAS_TCL_SUPPORT)
        AC_SUBST(TCL_CPPFLAGS)
        AC_SUBST(TCL_EXTRA_CFLAGS)
        dnl for modules
        AC_SUBST(TCL_SHLIB_SUFFIX)
        AC_SUBST(TCL_CC)
        AC_SUBST(LIB_RUNTIME_DIR)
        AC_SUBST(TK_LIB_SPEC)
        AC_SUBST(TCL_LDFLAGS)
        AC_SUBST(TCL_LIB_SPEC)
        AC_SUBST(TCL_SHLIB_LDFLAGS)
        AC_SUBST(TCL_SHLIB_CFLAGS)
        AC_SUBST(TCL_SHLIB_LD)
        AC_SUBST(TCL_LIB_FLAGS)
        $1
    ], [
        AC_MSG_WARN([Tcl/Tk not found])
        $2
    ])
])


