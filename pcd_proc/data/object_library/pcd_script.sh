#!/usr/bin/env bash
ls
for file in $(find /home/jason/Desktop/ycb_models/ycb-tools/models/ycb -name downsampled.obj); do
	echo "hey"
	echo "$file"

	# Set comma as delimiter
	IFS='/'

	#Read the split words into an array based on comma delimiter
	read -a strarr <<< "$file"

	#Print the splitted words
	echo "folder_name : ${strarr[-2]}"

	#mkdir ${strarr[-2]}
	pcl_obj2pcd "$file" ${strarr[-2]}.pcd
	#mv ${strarr[-2]}.pcd ${strarr[-2]}

done
