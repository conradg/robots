#!/bin/sh

if [ -f .timestampupsh ] 
then
    find . -newer .timestampupsh -regex ".*\.py" | xargs ./upload.sh
else
    echo *.py
fi
touch .timestampupsh
