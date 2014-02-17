#!/bin/bash
if [ $# != 0 ]
then 
    scp $@ pi@robots.mine.nu:~/prac-files
else
    echo "No files to upload "
    echo "USAGE: "
    echo "./upload file1 file2 file..."
fi
