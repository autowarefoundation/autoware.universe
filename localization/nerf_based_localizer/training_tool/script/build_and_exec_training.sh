#!/bin/bash
set -eux

ROOT_DIR=$(readlink -f $(dirname $0)/../)
TRAIN_RESULT_DIR=$(readlink -f $1)
DATASET_DIR=$(readlink -f $2)

cd ${ROOT_DIR}
cmake . -B build
cmake --build build --config RelWithDebInfo -j8

cd ${ROOT_DIR}/build
rm -rf ${TRAIN_RESULT_DIR}
mkdir ${TRAIN_RESULT_DIR}
cp ${ROOT_DIR}/config/train_config.yaml ${TRAIN_RESULT_DIR}/
./main train ${TRAIN_RESULT_DIR} ${DATASET_DIR}
