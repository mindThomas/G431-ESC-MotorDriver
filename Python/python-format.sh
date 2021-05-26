#!/usr/bin/env bash
# This script should be run from the project root.

python3 -m isort .
python3 -m black .
#flake8 # for checking only