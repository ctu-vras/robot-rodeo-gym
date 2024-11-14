#!/bin/bash

setup_folder="./setup"

sudo apt-get install $(cat "$setup_folder/packages.apt")

rosinstall . dependencies.rosinstall
