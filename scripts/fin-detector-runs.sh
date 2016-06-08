#!/bin/bash

# Sweep for covariance threshold
./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/covariance-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s covar_threshold -h 1
