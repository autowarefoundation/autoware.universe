#!/bin/bash

set -eux

TRAIN_RESULT_DIR=$(readlink -f $1)
DATASET_DIR=$(readlink -f $2)

cd $(dirname $0)/../build/

make -j $(nproc)

./main test ${TRAIN_RESULT_DIR} ${DATASET_DIR}
