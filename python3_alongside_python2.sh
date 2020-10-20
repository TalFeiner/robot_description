#!/bin/bash

###################################################################################################################################################################################################
########  An extension to make your ROS workspace compatible with python3 alongside python2. made by BGU CS department Tal Feiner. please consult your technical support before using it  #########
###################################################################################################################################################################################################

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'

echo -e "${WHITE_TXT}\nWould you like to make your ROS workspace compatible with python3 alongside python2? <y/n> ${NO_COLOR}\c"
read -n 2 choice

case $choice in
     y)
	
	echo -e "\n"
	read -p "Please, insert workspace name: " workspace
	echo -e "${WHITE_TXT}Starts making the necessary changes and installations...${NO_COLOR}"
	
	mkdir $HOME/tmp
	rm -rvf $HOME/$workspace/src/CMakeLists.txt
	cp -TR $HOME/$workspace/src $HOME/tmp
	cd $HOME
	rm -rvf $HOME/$workspace
	mkdir -p $HOME/$workspace/src
	sudo apt-get -y update
	sudo apt-get -y upgrade
	sudo apt-get -y install python3-empy  
	sudo apt-get -y install python3-catkin-pkg-modules
	sudo apt-get -y install python3-rospkg-modules
	cd $HOME/$workspace/
	catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
	cp -TR $HOME/tmp $HOME/$workspace/src
	catkin_make
	rm -rvf $HOME/tmp

	echo -e "${GREEN_TXT}\nDone, $workspace is compatible for python3 alongside python2.${NO_COLOR}\n"
	;;

     n)
	
	echo -e "${WHITE_TXT}\nYou chose not to use the extension${NO_COLOR}\n"
	;;

     *)

	echo -e "${RED_TXT}\nSorry, invalid input. Please, choose: y or n${NO_COLOR}"
	./python3_alongside_python2.sh
	;;

esac

#########################################
########  End of the extension  #########
#########################################
