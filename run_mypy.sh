#!/bin/bash

set -eou pipefail  # bash strict mode

export MYPYPATH=python
rm -rf .mypy_cache
files=$(find . -iname '*.py')
mypy $files --ignore-missing-imports
