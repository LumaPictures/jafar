############ 
# TP devin #
############

#
# GNU
#
setenv GNUHOME "/usr/local/gnu"
setenv PATH "${GNUHOME}/bin:${PATH}"
setenv LD_LIBRARY_PATH "${GNUHOME}/lib:${LD_LIBRARY_PATH}"
setenv MANPATH "${GNUHOME}/man:${MANPATH}"

#
# gcc 3.3
#
setenv PATH /usr/local/gcc-3.3/bin:${PATH}
setenv LD_LIBRARY_PATH  "/usr/local/gcc-3.3/lib/:${LD_LIBRARY_PATH}"

#
# TCL/TK 8.4
#
setenv TCL_TK_HOME "/usr/local/tcl-8.4"
setenv PATH "${TCL_TK_HOME}/bin:${PATH}"
setenv MANPATH "${MANPATH}:${TCL_TK_HOME}/man"
setenv LD_LIBRARY_PATH "${TCL_TK_HOME}/lib:${LD_LIBRARY_PATH}"

#
# Swig
#
setenv PATH "${PATH}:/usr/local/swig/bin"

# boost
setenv LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/home/tlemaire/usr/sparc-solaris/local/boost/lib"

# jafar
setenv JAFAR_DIR "${HOME}/tp_jafar"
setenv LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:${JAFAR_DIR}/lib/sparc-solaris2.9"

# alias
alias jafar /home/tlemaire/usr/sparc-solaris/bin/eltclsh
alias gm gnumake
alias jfr_configure ./configure --with-boost=/home/tlemaire/usr/sparc-solaris/local/boost --with-swig=/usr/local/swig/bin
