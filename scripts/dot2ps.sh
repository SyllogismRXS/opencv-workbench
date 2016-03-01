#!/bin/bash

dot -Tps ~/test.dot -o ~/test.ps
evince ~/test.ps
