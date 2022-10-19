#!/bin/bash

mkdir -p /tmp/evaluator_summarizer/
evaluator_summarizer -p autoware_dev -j "$1" -o /tmp/evaluator_summarizer/
python3 plot_cutin_boundary_condition_from_csv.py -f /tmp/evaluator_summarizer/*.csv
