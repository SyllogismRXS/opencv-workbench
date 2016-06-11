#!/bin/bash

# Sweep for covariance threshold
#./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/covariance-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s covar_threshold -h 1

#./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/covariance-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s min_velocity_threshold_2 -h 1

#./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/covariance-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s class_age_confirmed -h 1

#./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/covariance-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s class_age_confirmed -h 1

./scripts/learn-test-velocity.sh -y ./data/yaml-range-params/covariance-threshold.yaml -f ./data/scenarios/velocity-train-test.yaml -s covar_norm_threshold -h 1
