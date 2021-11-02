#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import sys

import yaml


def repos2workspace(repos_file: Path, output_file: Path):
    with open(repos_file, "r") as f:
        repos = yaml.load(f, Loader=yaml.SafeLoader)

    paths = [f"src/{path}" for path in repos["repositories"]]
    folders = [{"path": path} for path in paths]
    folders += [{"path": "."}]

    workspace = {
        "folders": folders,
    }

    with open(output_file, "w") as f:
        json.dump(workspace, f, indent=2, sort_keys=False)


def main(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("repos_file", type=Path)
    parser.add_argument("-o", "--output", dest="output_file", type=Path, default=None)
    ns = parser.parse_args(args)

    if not ns.output_file:
        parent_dir = ns.repos_file.absolute().parent
        ns.output_file = parent_dir / f"{parent_dir.name}.code-workspace"

    repos2workspace(ns.repos_file, ns.output_file)


if __name__ == "__main__":
    main(sys.argv[1:])
