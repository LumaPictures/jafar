dnl MD_BOOST
dnl Test for the Boost C++ libraries 
dnl Defines:
dnl   BOOST_CPPFLAGS to the set of flags required to compiled Boost
AC_DEFUN([MD_BOOST], 
[
  AC_SUBST(BOOST_CPPFLAGS)
  AC_SUBST(BOOST_LDFLAGS)
  BOOST_CPPFLAGS=""
  BOOST_LDFLAGS=""

dnl Extract the path name from a --with-boost=PATH argument
  AC_ARG_WITH(boost,
    AC_HELP_STRING([--with-boost=PATH],[absolute path name where the Boost C++ libraries reside]),
    [
        BOOST_ROOT="$withval"
	CPPFLAGS_OLD="$CPPFLAGS"
	CPPFLAGS="$CPPFLAGS -I$BOOST_ROOT/include"
	AC_LANG_SAVE
	AC_LANG_CPLUSPLUS
	AC_CHECK_HEADER([boost/version.hpp], [have_boost="yes"],)
	AC_LANG_RESTORE
	if test "x$have_boost" = "x"; then
		AC_MSG_WARN(["Can't find boost/version.hpp"])
	else
		BOOST_CPPFLAGS="-I$BOOST_ROOT/include"
		BOOST_LDFLAGS="-L$BOOST_ROOT/lib"
	fi
   ]
  )

])

