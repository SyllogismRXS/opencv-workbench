#!/bin/bash

# Sweep for minimum velocity
#./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/velocity-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s min_velocity_threshold -h 1

# Sweep for maximum velocity
#./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/velocity-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s max_velocity_threshold -h 1

# Sweep age confirmed
./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/velocity-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s class_age_confirmed -h 1
