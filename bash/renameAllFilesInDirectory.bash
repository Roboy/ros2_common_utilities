#!/bin/bash
if [ "$#" -ne 3 ]; then
    echo "USAGE:     ./renameAllFilesInDirectory.bash path/to/directory wordToReplace wordToReplaceWith"
    echo "EXAMPLE:   ls ."
    echo "           fuckMe.bash"
    echo "           ./renameAllFilesInDirectory.bash . Me You"
    echo "           ls ."
    echo "           fuckYou.bash"
else
    ls -1 -R $1|grep $2|column -c 100
    echo "-------"
    echo "WARNING: the above files will be changed *$2* -> *$3*"
    echo "-------"
    read -r -p "Proceed? [y/n] " response
    response=${response,,}    # tolower
    if [[ $response =~ ^(yes|y)$ ]]; then
	datetimestring=$(date +'%d%m%Y_%H-%M')
        find $1 -type f -iname "*$2*" | while read FILE ; do
            newfile="$(echo ${FILE} |sed -e "s/$2/$3/")" ;
            mv "${FILE}" "${newfile}" ;
	    echo "${FILE} -> ${newfile}" >> renameAllFilesInDirectory_${datetimestring}.log;
        done
    else
        echo "ABORT"
    fi
fi 
