#!/bin/bash
vercomp () {
    if [[ $1 == $2 ]]
    then
        return 0
    fi
    local IFS=.
    local i ver1=($1) ver2=($2)
    # fill empty fields in ver1 with zeros
    for ((i=${#ver1[@]}; i<${#ver2[@]}; i++))
    do
        ver1[i]=0
    done
    for ((i=0; i<${#ver1[@]}; i++))
    do
        if [[ -z ${ver2[i]} ]]
        then
            # fill empty fields in ver2 with zeros
            ver2[i]=0
        fi
        if ((10#${ver1[i]} > 10#${ver2[i]}))
        then
            return 1
        fi
        if ((10#${ver1[i]} < 10#${ver2[i]}))
        then
            return 2
        fi
    done
    return 0
}

if [ "$#" -lt 2 ]; then
    echo "USAGE:     ./publish2ppa.bash path/to/src/directory versionnumber"
else
    currentDir=$(pwd)
    datetimestring=$(date +'%d%m%Y_%H-%M')
    echo "-------"
    echo "WARNING: I will try to publish $1 with versionnumber $2 to ppa"
    echo "-------"
    read -r -p "Proceed? [y/n] " response
    response=${response,,}    # tolower
    if [[ $response =~ ^(yes|y)$ ]]; then
		mkdir tmp_publish2ppa_$datetimestring
		packagename=$1
		packagename=${packagename/*\//}
		packagename=$(echo ${packagename}|tr '_' '-') 
		echo "###############     processing ${packagename}   ################"
		cp $1 -r tmp_publish2ppa_$datetimestring
		cd tmp_publish2ppa_$datetimestring
		versionnumber=$(grep -Po '(?<=\()\d.\d.\d' ${packagename}/debian/changelog)
		echo "version number is $versionnumber"
		echo $3
		vercomp $2 $versionnumber
		if [ $? == 2 ]; then
			echo "FAILED: versionnumber should be the same or bigger for successful upload"
			echo "override by appending -f"
		else
			grep -rl "$versionnumber" $packagename/debian/changelog | xargs sed -i "s/$versionnumber/$2/g"
			tar -acf ${packagename}_$2.orig.tar.gz ${packagename}
			cd ${packagename}
			debuild -S
			cd ..
			dput ppa:letrend/${packagename} ${packagename}_$2-1_source.changes
		fi
    else
        echo "ABORT"
    fi
    cd ${currenDir}
fi

