#!/bin/bash

shdir=$(dirname $(readlink -f $0))
partitioner_dir=$shdir/src/partitioner/
builder_dir=$partitioner_dir/build/
bitwuzla_path=$shdir/linux-pre_built/binaries/bitwuzla-0.8.0-bin

# build partitioner
cd $partitioner_dir

# [ -d $builder_dir ] && rm -rf $builder_dir
[ ! -d $builder_dir ] && mkdir $builder_dir 

cd $builder_dir 

cmake -DSTATICCOMPILE=ON -DCMAKE_INSTALL_PREFIX:PATH="." ..
# cmake -DCMAKE_INSTALL_PREFIX:PATH="." ..
cmake --build . --parallel "$(nproc)"

[ ! -d $shdir/bin ] && mkdir $shdir/bin
[ ! -d $shdir/bin/binaries ] && mkdir $shdir/bin/binaries

cp $shdir/src/BV-Parti.py $shdir/bin/BV-Parti.py

cp stp $shdir/bin/binaries/partitioner-bin

cp $bitwuzla_path $shdir/bin/binaries/bitwuzla-0.8.0-bin