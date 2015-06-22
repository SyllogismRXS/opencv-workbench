#!/bin/bash

# Rename all *.txt to *.text
for f in *.xml; do 
    #echo "mv -- \"$f\" \"${f%.txt}.text\" "
    #git mv "$f" "$(basename "$f" .xml).truth.xml"
done

