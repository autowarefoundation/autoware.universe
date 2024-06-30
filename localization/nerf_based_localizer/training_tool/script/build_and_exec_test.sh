#!/bin/bash
set -eux

ROOT_DIR=$(readlink -f $(dirname $0)/../)
TRAIN_RESULT_DIR=$(readlink -f $1)
DATASET_DIR=$(readlink -f $2)

cd ${ROOT_DIR}
cmake . -B build
cmake --build build --config RelWithDebInfo -j8

cd ${ROOT_DIR}/build
./main test ${TRAIN_RESULT_DIR} ${DATASET_DIR}
