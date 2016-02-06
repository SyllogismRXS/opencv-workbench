#!/bin/bash

./scripts/learn-test-threshold.sh -y ./data/yaml-range-params/static-threshold.yaml -f ./data/scenarios/threshold-train-test.yaml -s static_threshold

./scripts/learn-test-threshold.sh -y ./data/yaml-range-params/ratio-threshold.yaml -f ./data/scenarios/threshold-train-test.yaml -s ratio_threshold

./scripts/learn-test-threshold.sh -y ./data/yaml-range-params/gradient-threshold.yaml -f ./data/scenarios/threshold-train-test.yaml -s gradient_threshold
