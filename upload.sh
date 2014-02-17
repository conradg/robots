#!/bin/bash

for ARG in "$@"
do
    scp $ARG pi@robots.mine.nu:~/prac-files
done
