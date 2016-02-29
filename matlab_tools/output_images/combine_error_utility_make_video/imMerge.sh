#!/bin/bash

exp=cloth_table
exp=colab_folding
exp=rope_cylinder
base_folder=../$exp
error_file_list=$base_folder/weighted_pseudoinverse_error_over_time_video/*.png

for error_file in $error_file_list; do
    utility_file=${error_file/error/utility}
    result=${error_file:62}
    echo $error_file
    echo $utility_file 
    echo $result
    convert $error_file $utility_file +append $result
done

./imToMov.sh 50 ${exp}_utility_results.wmv png
rm *.png
