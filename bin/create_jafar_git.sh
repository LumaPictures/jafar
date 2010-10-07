#! /bin/sh

BASE_GIT=/git/robots
BASE_DIRECTORY=${BASE_GIT}/jafar/modules/

MODULE_DIRECTORY="${BASE_DIRECTORY}${1}.git"
if [ -d ${MODULE_DIRECTORY} ]
then
	echo "The module ${1} seems to already exists !!"
	exit 1
fi


mkdir ${MODULE_DIRECTORY}
cd ${MODULE_DIRECTORY}
git --bare init --shared

# description
echo "jafar/modules/${1}" > ${MODULE_DIRECTORY}/description

# Add hooks
rm ${MODULE_DIRECTORY}/hooks/post-receive
ln -s ${BASE_GIT}/laas-git-hooks.git/post-receive ${MODULE_DIRECTORY}/hooks/

# Add config hooks
echo "[hooks]" >> ${MODULE_DIRECTORY}/config
echo "        emailprefix = " >> ${MODULE_DIRECTORY}/config
echo "        mailinglist = jafar-source@laas.fr" >> ${MODULE_DIRECTORY}/config
