#!/bin/sh

SCRIPT_PATH=$(cd ${0%/*} && echo $PWD/${0##*/})
MY_JAFAR_DIR=`dirname "$SCRIPT_PATH"`/..

. ${MY_JAFAR_DIR}/bin/git_config.sh
cd ${MY_JAFAR_DIR}/modules

REMOTE_MODULES=`ssh ${VCS_USER}@${HOST} "sh -c 'cd ${BASE_DIRECTORY} ; ls -d */ | sed s,.git/,,'"`

for m in $REMOTE_MODULES; do
	if [ -d $m ]; then
		echo "Pulling $m"
		cd $m
		git pull --rebase
		cd -
	else
		echo "Cloning $m"
		git clone $BASE_URL$m
	fi;

done

