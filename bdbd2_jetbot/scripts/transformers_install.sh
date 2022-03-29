#!/bin/bash

set -x

# Transformers

#git clone https://github.com/google/sentencepiece.

#git checkout tags/v0.1.84
#cd sentencepiece/python
#python3 setup.py install

#curl https://sh.rustup.rs -sSf | sh
#rustc --version

python3 -m pip install setuptools_rust
python3 -m pip install tokenizers
python3 -m pip install transformers
