#!/bin/bash

set -eux

TRAIN_RESULT_DIR=$(readlink -f $1)
DATASET_PATH=$(readlink -f $2)
cd $(dirname $0)

./build_and_exec_training.sh ${TRAIN_RESULT_DIR} ${DATASET_PATH}

./build_and_exec_test.sh ${TRAIN_RESULT_DIR} ${DATASET_PATH}
