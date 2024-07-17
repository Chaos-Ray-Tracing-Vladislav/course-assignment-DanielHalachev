#usr/bin/env sh

hw=$1
lowerLimit=$2
upperLimit=$3

for i in $lowerLimit .. $upperLimit; do
    ffmpeg -i "./../Images/HW${hw}/result${i}.ppm" "./../Images/HW${hw}/result${i}.png"
done

exit 0
