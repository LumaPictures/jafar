#!/bin/sh

. ${JAFAR_DIR}/bin/git_config.sh
pushd ${JAFAR_DIR}/modules > /dev/null

REMOTE_MODULES=`ssh trac "sh -c 'cd /git/robots/jafar/modules ; ls -d */ | sed s,.git/,,'"`

for m in $REMOTE_MODULES; do
	if [ -d $m ]; then
		echo "Pulling $m"
		pushd $m > /dev/null
		git pull --rebase
		popd > /dev/null
	else
		echo "Cloning $m"
		git clone $BASE_URL$m
	fi;

done

popd > /dev/null
