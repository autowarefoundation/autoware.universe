#!/usr/bin/env python3

import argparse
import yaml
import json
import sys
from pathlib import Path


def repos2workspace(repos_file, output_file):
    with open(repos_file, 'r') as f:
        repos = yaml.load(f, Loader=yaml.SafeLoader)

    paths = [f'src/{path}' for path in repos['repositories']]
    folders = [{'path': '.'}]

    workspace = {
        'folders': folders,
        'settings': {'git.ignoredRepositories': ['.'], 'git.scanRepositories': paths},
    }

    with open(output_file, 'w') as f:
        json.dump(workspace, f, indent=2, sort_keys=False)


def main(args):
    parser = argparse.ArgumentParser()
    parser.add_argument('repos_file', type=Path)
    parser.add_argument('-o', '--output', dest='output_file',
                        type=Path, default=None)
    ns = parser.parse_args(args)

    if not ns.output_file:
        ns.output_file = f'{ns.repos_file.absolute().parent.name}.code-workspace'

    repos2workspace(ns.repos_file, ns.output_file)


if __name__ == '__main__':
    main(sys.argv[1:])
