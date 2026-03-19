#! /bin/bash

set -e

pip install twine
twine upload wheels/wheelhouse/* --verbose