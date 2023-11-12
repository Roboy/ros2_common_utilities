#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "USAGE:     ./publish_ros_package2ppa.bash path/to/src/directory versionnumber"
else
    currentDir=$(pwd)
    datetimestring=$(date +'%d%m%Y_%H-%M')
    cd $1
    find . ! -path . -maxdepth 1 -type d|column -c 100
    echo "-------"
    echo "WARNING: the above packages will be published to ppa with versionnumber $2"
    echo "-------"
    read -r -p "Proceed? [y/n] " response
    response=${response,,}    # tolower
    if [[ $response =~ ^(yes|y)$ ]]; then
	mkdir -p ${currentDir}/tmp
        find . ! -path . -maxdepth 1 -type d| while read FILE ; do
	    packagename=${FILE/*\//}
	    packagename=ros-indigo-$(echo ${packagename}|tr '_' '-') 
	    echo "###############     processing ${packagename} in directory: ${FILE}    ################"
	    bloom-generate rosdebian ${FILE} --os-name ubuntu --os-version trusty --ros-distro indigo          
	    mkdir -p ${currentDir}/tmp/${packagename}/src && cp ${FILE}/* ${currentDir}/tmp/${packagename}/src -r
	    mv debian ${currentDir}/tmp/${packagename}
	    rm ${currentDir}/tmp/${packagename}/debian/rules && cp ${currentDir}/rules ${currentDir}/tmp/${packagename}/debian
	    rm ${currentDir}/tmp/${packagename}/debian/changelog && cp ${currentDir}/changelog ${currentDir}/tmp/${packagename}/debian
	    grep -rl "PACKAGENAME" ${currentDir}/tmp/${packagename}/debian/changelog | xargs sed -i "s/PACKAGENAME/${packagename}/g"
	    grep -rl "PACKAGENAME" ${currentDir}/tmp/${packagename}/debian/rules | xargs sed -i "s/PACKAGENAME/${packagename}/g"
	    grep -rl "VERSIONNUMBER" ${currentDir}/tmp/${packagename}/debian/changelog | xargs sed -i "s/VERSIONNUMBER/$2/g"
	    sed -i '/Maintainer/c\Maintainer: Simon Trendel <simon.trendel@tum.de>' ${currentDir}/tmp/${packagename}/debian/control
	    sed -i '/Standards-Version/c\Standards-Version: 3.9.5' ${currentDir}/tmp/${packagename}/debian/control
	    cd ${currentDir}/tmp/
	    tar -acf ${packagename}_$2.orig.tar.gz ${packagename}
	    cd ${packagename}
	    debuild -S
	    cd ..
	    dput -f ppa:letrend/${packagename} ${packagename}_$2-1_source.changes
	    cd ..
	done
    else
        echo "ABORT"
    fi
    cd ${currenDir}
fi

