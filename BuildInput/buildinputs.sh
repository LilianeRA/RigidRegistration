#!/usr/bin/bash

#bin_path="$(pwd)"/bin/Release
bin_path="$(pwd)"/build
cd $bin_path

if [ "$1" = "rebuild" ] || [ "$1" = "build" ] 
then
	echo $1 $2
	#ulimit -Sv 2000000
	if [ "$1" = "rebuild" ] 
	then
		if [ "$2" = "debug" ]
		then
			make clean_debug
		fi
		if [ "$2" = "release" ]
		then
			make clean_release
		fi
	fi
	if [ "$2" = "debug" ]
	then
		make -j4 debug
	fi
	if [ "$2" = "release" ]
	then
		make -j4 release
	fi

else
	# if you created project with CMake and 
	# you saved the project in a subdir
	input_dir="../" 
	# if you created project with CMake and 
	# you saved the project in the same dir as the main.cpp
	##input_dir="" 
	filename="bun000"
	downscale=45
	for rotation in 45 #90
	do
	for translation in 1 #2
	do
	for holes in 0 #1 2
	do
	radius=0.03
	if [ $holes = 0 ] 
	then
		radius=0.0
	fi
	for noise in 0 #1 2 3
	do
	for outlier in 0 #5 20
	do
	for seed in 1
	do
		LD_LIBRARY_PATH=:. $bin_path/BuildInput $input_dir $filename $downscale $rotation $translation $holes $radius $noise $outlier $seed
	done #seed
	done #outlier
	done #noise
	done #holes
	done #translation
	done #rotation
fi
