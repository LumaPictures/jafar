# $Id$ #
#!/bin/bash

# input: list of modules you wish to process
# output: list of all modules needed sorted by order of dependency

#echo "dep.sh running: $@" >&2

# first, a small filter

modules=""
for mod in $@; do
	modules+=" `basename $mod`"
done

#echo ""
#echo $modules
#echo ""

# now add dependencies

finished=0
olddeps=$modules

while [[ finished -eq 0 ]]; do
	mdeps=""
	odeps=""
	for m in $olddeps; do
		mdeps="`cat ${JAFAR_DIR}/modules/$m/User.make | grep REQUIRED_MODULES | cut -d"=" -f2` $mdeps"
		odeps="`cat ${JAFAR_DIR}/modules/$m/User.make | grep OPTIONAL_MODULES | cut -d"=" -f2` $odeps"
	done
	mdeps=`echo $mdeps | awk '{n=split($0,mods," "); for(i=1;i<=n;i++) { if (!x[mods[i]]++) print mods[i];}}'`
	odeps=`echo $odeps | awk '{n=split($0,mods," "); for(i=1;i<=n;i++) { if (!x[mods[i]]++) print mods[i];}}'`

	newdeps=""
	for m in $mdeps; do
		if [[ -d "${JAFAR_DIR}/modules/$m" ]]; then
			newdeps+=" $m";
		else
			echo "[ERROR] Required module $m not available. You need to checkout this module" >&2;
		    exit 1;
		fi;
	done;
	for m in $odeps; do
		if [[ -d "${JAFAR_DIR}/modules/$m" ]]; then
			newdeps+=" $m";
		else
			echo "[WARNING] Optional module $m not available. Some functionalities might be disabled" >&2;
		fi;
	done;
	modules="$newdeps $modules"
	olddeps=$newdeps

	finished=1
	for m in $newdeps; do
		if [[ $m != "kernel" ]]; then finished=0; fi;
	done
done

#echo $modules
#echo ""

# and now remove duplicate dependencies

modules=`echo $modules | awk '{n=split($0,mods," "); for(i=1;i<=n;i++) { if (!x[mods[i]]++) print mods[i];}}'`

# check if module present
cleanmodules=""
for m in $modules; do
	if [[ -d "${JAFAR_DIR}/modules/$m" ]]; then cleanmodules+=" $m"; fi
done

echo $cleanmodules


