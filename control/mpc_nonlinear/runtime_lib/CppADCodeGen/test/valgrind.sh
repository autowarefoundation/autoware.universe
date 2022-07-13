#!/bin/bash

valgrind --read-var-info=yes --leak-check=full --track-origins=yes --leak-resolution=high --freelist-vol=100000000 --show-possibly-lost=no $1 2> valgrind.txt
