dnl MD_CHECK_RUBY(action-if-found, action-if-not-found)
AC_DEFUN([MD_CHECK_RUBY], [
    md_ruby_support=yes
    AC_PATH_PROG([RUBY], [ruby], [md_ruby_support=no])

    if test "$md_ruby_support" = "yes"; then
        [RUBY_EXTENSION_BASEDIR=`$RUBY -e "puts Config::MAKEFILE_CONFIG['topdir']"`]
        [RUBY_VERSION=`$RUBY -e "puts Config::CONFIG['ruby_version']"`]
        
        md_ruby_cppflags=$CPPFLAGS
        CPPFLAGS="$CPPFLAGS -I$RUBY_EXTENSION_BASEDIR"
        AC_CHECK_HEADER([ruby.h], [], [md_ruby_support=no])
    fi

    if test "$md_ruby_support" = "yes"; then
        HAS_RUBY_SUPPORT=yes
        AC_SUBST(RUBY_EXTENSION_BASEDIR)
        AC_SUBST(RUBY_VERSION)
        ifelse([$1], [], [], [$1])
    else
        AC_MSG_WARN([Ruby support disabled])
        HAS_RUBY_SUPPORT=no
        ifelse([$2], [], [], [$2])
    fi
])

