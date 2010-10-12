#! /bin/sh
#set -e

SCRIPT_PATH=$(cd ${0%/*} && echo $PWD/${0##*/})
MY_JAFAR_DIR=`dirname "$SCRIPT_PATH"`/..

. ${MY_JAFAR_DIR}/bin/git_config.sh

usage()
{
    cat <<EOF
	add_module [name]

    imports the jafar module 'name' 

The script adds C, C++, documentation, Swig and scripts 
You have to add any other files (if any) manually
EOF
    exit 1
}

MODULE=`echo $1 | sed 's,/$,,'`
test -z "$MODULE" && usage;

if ! test -d "$MODULE"; then
    echo "ERROR: $MODULE is not a directory"
    exit 1
fi

# Check if a module of same kind exists
# Otherwise, create the repository
# For this, call create_jafar_git.sh, with $1 = $MODULE
sed "s,\${1},$MODULE,"  ${MY_JAFAR_DIR}/bin/create_jafar_git.sh | ssh ${VCS_USER}@${HOST} "sh -s"

if [ $? -ne 0 ]
then
	exit 1
fi

TOPDIR=$PWD/$MODULE

cd $TOPDIR
git init 

git add COPYRIGHT README doc include macro src test_suite .gitignore

cd $TOPDIR/doc
git add *.doxy *.tcl *.hpp *.cpp

cd $TOPDIR/include
git add *.i

cd $TOPDIR/include/$MODULE
git add *.hpp *.h *.i

cd $TOPDIR/macro
git add *.tcl *.rb

cd $TOPDIR/src
git add *.cpp *.c ruby

cd $TOPDIR/src/ruby
git add extconf.rb

cd $TOPDIR/test_suite
git add *.cpp *.hpp

cd $TOPDIR

git commit -m "Initial commit of ${MODULE}"

git push ${BASE_URL}/${MODULE} master

echo "[remote \"origin\"]
	url = ${BASE_URL}/${MODULE}
	fetch = +refs/heads/*:refs/remotes/origin/*
[branch \"master\"]
	remote = origin
	merge = refs/heads/master" >> ${TOPDIR}/.git/config
