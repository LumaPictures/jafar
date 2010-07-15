#! /bin/sh
set -e

#. ${JAFAR_DIR}/bin/git_config.sh

OPTIONAL_MODULES=""
MAKEFILE=CMakeLists.txt
OLDMAKEFILE=User.make


# Really checkout the module, need to be changed when switching to git
checkout_module()
{
	local module_name=$1
	echo "Checkout module : |${module_name}|"
    url="svn+ssh://${VCS_USER}@svn.laas.fr/svn/jafar/jafarModules/trunk/${module_name}"
	svn co ${url} > /dev/null
}

# Get the list of dependencies of a module, and try to get it, if necessary
get_dependency()
{
	local module_name=$1
	echo "Checking dependencies of ${module_name}"
	if [ ! -r ${module_name}/${MAKEFILE} ]
	then
		user_make=${module_name}/${OLDMAKEFILE}
		required_modules=$(awk 'BEGIN { FS = "=" } /REQUIRED_MODULES/ {print $2}' $user_make)
		optional_modules=$(awk 'BEGIN { FS = "=" } /OPTIONAL_MODULES/ {print $2}' $user_make)
	else	
		user_make="${module_name}/${MAKEFILE}"
		required_modules=$(awk '/REQUIRED_MODULES/  {for (i=2; i <= NF; i++) 
							{ printf "%s ", $i }
						  print ""
						}' $user_make)
		optional_modules=$(awk '/OPTIONAL_MODULES/  {for (i=2; i <= NF; i++) 
							{ printf "%s ", $i }
						  print ""
						}' $user_make)
	fi

	for i in ${required_modules}; do
		checkout_module_with_dependencies $i
	done

	OPTIONAL_MODULES="${optional_modules} ${OPTIONAL_MODULES}"
}

checkout_module_with_dependencies()
{
	module_name=$1
	if [ -d ${module_name} ]
	then
		return
	fi

	checkout_module $module_name

	if [ ! -d ${module_name} ]
	then
      echo "An error has occured when checkouting module : ${module_name}, please check that it exists"
	  exit 1
	fi

	if [ ! -r ${module_name}/${MAKEFILE} ]
	then
		echo "No ${MAKEFILE} file found !!!"
		echo "Trying to fallback to old ${OLDMAKEFILE} !!!"
		if [ ! -r ${module_name}/${OLDMAKEFILE} ]
		then
			echo "Can't find ${OLDMAKEFILE} too. Exiting ..."
			exit 1
		fi
	fi

	get_dependency $module_name
}

remove_duplicate_optional()
{
	OPTIONAL_MODULES=$(echo $OPTIONAL_MODULES |
		awk	'	{split($0, a, " "); 
			for (i in a) 
				{  uniq[$i]++}}
		END { 
			for (u in uniq) 
				printf "%s ",u; 
				printf "\n" 
			}' 
	)
}

main()
{
	local module_name=$1

	if [ "$SVN_LAAS_USER" != "" ]
	then
		VCS_USER=$SVN_LAAS_USER
	else
		VCS_USER=$USER
	fi
	echo "Will use ${VCS_USER} to login on svn.laas.fr"


	checkout_module_with_dependencies $module_name
	echo "Module ${module_name} and all its dependencies have been successfully"
	echo "installed, you can now run make in the directory modules/${module_name}"
	 
	remove_duplicate_optional

	if [ "$OPTIONAL_MODULES" != "" ]
	then
		echo ""
		echo "Optional dependencies that you may want to install : ${OPTIONAL_MODULES}"
	fi
}

usage()
{
  echo "jafar-co-module.sh [modulename]"
  echo "This scripts will attempt to checkout"
  echo "a specific module and its dependencies"
  echo "You need to use your LAAS password !"
  exit 1
}

if [ $# != 1 ]
then 
	usage
fi
main $1
