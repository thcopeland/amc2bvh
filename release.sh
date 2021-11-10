#!/bin/bash

version=$1
base=amc2bvh-$version
log=release.log

if [ -z $version ]; then
    echo "Usage: ./release.sh VERSION"
    exit 1
fi

if [[ $(uname -p) != "x86_64" ]]; then
    # you'll have to cross-compile if you don't have a x86_64 processor
    echo "./release.sh should be run on a x86_64 system"
    exit 1
fi

printf "creating source archive..."
dir=${base}_source
mkdir $dir
cp README.md Makefile amc2bvh.c amc2bvh.h hashmap.c hashmap.h -t $dir/
make clean &>> /dev/null
tar cf $dir.tar $dir/
rm -rf $dir
echo "done"

printf "building linux x86_64..."
dir=${base}_x86_64_linux
mkdir $dir
make &> $log
mv amc2bvh $dir/
cp README.md $dir/
make clean &>> /dev/null
tar cf $dir.tar $dir/
rm -rf $dir
echo "done"

printf "building windows x86_64..."
dir=${base}_x86_64_windows
mkdir $dir
CC=x86_64-w64-mingw32-gcc make &>> $log
mv amc2bvh.exe $dir/
cp README.md $dir/
make clean &> /dev/null
zip -r -qq $dir.zip $dir/
rm -rf $dir
echo "done"

printf "building windows i686..."
dir=${base}_i686_windows
mkdir $dir
CC=i686-w64-mingw32-gcc make &>> $log
mv amc2bvh.exe $dir/
cp README.md $dir/
make clean &> /dev/null
zip -r -qq $dir.zip $dir/
rm -rf $dir
echo "done"
