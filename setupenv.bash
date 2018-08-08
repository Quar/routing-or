#!/bin/bash

PY=python3

echo "==> Setting up Python Environment ..."
echo

if ! [ -x "$(command -v python3)" ]; then
  echo "=== Requires Python3 to run, assuming 'python' is python3 ==="
  PY=python
fi

$PY -m pip install -r ./requirements.txt

echo
echo "==> Done."
