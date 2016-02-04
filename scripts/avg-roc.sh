#!/bin/bash

# This script generates both a standalone PDF of the plot as well as
# A latex/pdf combination that can be included in a larget latex file.
# Most of the info about epslatex taken from:
# http://www.gnuplotting.org/tag/epslatex/

#FILENAMES=("larks-roc" "method0" "method1" "method2" "method3" "method4" "method5")
#TITLES=("LARKS" "Squared Difference" "Normalized Squared Difference" "Cross Correlation" "Normalized Cross Correlation" "Cross Corr Coefficient" "Normalized Cross Corr Coefficient")

if [ $# -lt 1 ]; then
    echo "Missing csv file"
    exit;
fi

FILENAMES=($1)
TITLES=("Baseline")

count=0
for i in "${FILENAMES[@]}"
do
    :
    echo "Next: ${i}"    

    # Strip path and extension
    IN_STEM=${i%.*}

    DEC=""
    if [ $# -eq 2 ]; then
        DEC="every $2"
        OUT_STEM="$IN_STEM-dec-$2"
    else
        OUT_STEM="${IN_STEM}"
    fi
    
    DIR=$(dirname "${i}")

    pushd $DIR >& /dev/null
    
    PLOT_CMD="plot '${IN_STEM}.csv' ${DEC} using 1:2:3:4:5:6 with xyerrorbars ls 5 title '${TITLE}'"

    # Start gnuplot
    gnuplot -persist <<PLOT

set terminal epslatex size 3.5,2.62 color colortext
set output "${OUT_STEM}.tex"

set title "${TITLES[count]} ROC"
set xlabel "False Positive Rate"
set ylabel "True Positive Rate"
set xrange [0:1]
set yrange [0:1]

set style line 5 lt rgb "blue" lw 1 pt 7 ps 0.5

set arrow from 0,0 to 1,1 nohead lc rgb 'black'

set datafile separator ","
${PLOT_CMD}

set terminal epslatex size 3.5,2.62 standalone color colortext 10
set output "${OUT_STEM}-sa.tex"
${PLOT_CMD}

save "${OUT_STEM}.gnu"

quit
PLOT

    # Since most IEEE papers can't include eps file types, we have to convert the
    # generated eps file into a PDF
    epstopdf ${OUT_STEM}.eps

    # Generate standalone postscript plot
    latex ${OUT_STEM}-sa.tex
    dvips -o ${OUT_STEM}-sa.ps ${OUT_STEM}-sa.dvi

    # Convert ps file to pdf
    ps2pdf ${OUT_STEM}-sa.ps

    # Clean up intermediate files
    rm -rf ${OUT_STEM}-sa.tex
    rm -rf ${OUT_STEM}-sa.dvi
    rm -rf ${OUT_STEM}-sa.ps
    rm -rf *.eps
    rm -rf *.aux
    rm -rf *.dvi
    rm -rf *.log
    
    popd >& /dev/null

count=${count}+1

done

# Display the "standalone" pdf file
#okular ${FILENAME}-sa.pdf
