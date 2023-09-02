#!/bin/sh

git add . 
git commit -m "update commit"
git remote add  origin git@github.com:RobotEngineerWilliam/ATI_assemble.git
git push -u origin master
