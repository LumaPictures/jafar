# XXX Don't forget to change value in create_jafar_git.sh too

HOST=softs.laas.fr
BASE_GIT=/git/robots
BASE_DIRECTORY=${BASE_GIT}/jafar/modules/test/

if [ "$GIT_LAAS_USER" != "" ]
then
	VCS_USER=$GIT_LAAS_USER
else
	VCS_USER=$USER
fi

echo "Will connect to VCS server using login : ${VCS_USER}"

BASE_URL=ssh://${VCS_USER}@${HOST}/${BASE_DIRECTORY}
