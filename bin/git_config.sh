#!/bin/sh

SCRIPT_PATH=$(cd ${0%/*} && echo $PWD/${0##*/})
MY_JAFAR_DIR=`dirname "$SCRIPT_PATH"`/..

JAFAR_URL=`grep "url = " ${MY_JAFAR_DIR}/.git/config | sed -e "s#\t*url = ##"`
GITREPO_SUFFIX=`echo ${JAFAR_URL} | sed -e "s#\(\.git\$\)##"`
if [[ $GITREPO_SUFFIX != $JAFAR_URL ]]; then GITREPO_SUFFIX=".git"; else GITREPO_SUFFIX=""; fi
BASE_URL=`echo ${JAFAR_URL} | sed -e "s#jafar/jafar\(\.git\)*#jafar#"`
PROTOCOL=`echo ${BASE_URL} | sed -r -e "s#([htpshgi]*)://.*#\1#"`
VCS_USER=`echo ${BASE_URL} | sed -e "s#.*://##" -e "s#trac.*##" -e "s#@##"`

if [ "$VCS_USER" == "" ]
then
	VCS_USER=$USER
fi

# XXX Don't forget to change BASE_GIT and BASE_DIRECTORY in create_jafar_git.sh too

HOST=trac.laas.fr
BASE_GIT=/git/robots
BASE_DIRECTORY=${BASE_GIT}/jafar

if [[ $PROTOCOL == "ssh" ]]; then
	echo "Will connect to VCS server using login : ${VCS_USER}"
fi
