#!/bin/bash

# This script generates both a standalone PDF of the plot as well as
# A latex/pdf combination that can be included in a larget latex file.
# Most of the info about epslatex taken from:
# http://www.gnuplotting.org/tag/epslatex/

FILENAME="all"

# Start gnuplot
gnuplot -persist <<PLOT

set terminal epslatex size 3.5,2.62 color colortext
set output "${FILENAME}.tex"

set title "ROC Plots"
set xlabel "False Positive Rate"
set ylabel "True Positive Rate"
set xrange [0:1]
set yrange [0:1]

set style line 5 lt rgb "blue" lw 3 pt 6
set style line 6 lt rgb "yellow" lw 3 pt 6
set style line 7 lt rgb "green" lw 3 pt 6
set style line 8 lt rgb "black" lw 3 pt 6
set style line 9 lt rgb "cyan" lw 3 pt 6
set style line 10 lt rgb "red" lw 3 pt 6
set style line 11 lt rgb "brown" lw 3 pt 6

#set arrow from 0,0 to 1,1 nohead lc rgb 'black'

set datafile separator ","
plot "larks-roc.csv" using 9:8 title 'LARKS' with linespoints ls 5,\
     "method0.csv" using 9:8 title 'SQ Diff' with linespoints ls 6,\
     "method1.csv" using 9:8 title 'SQ Diff Norm' with linespoints ls 7,\
     "method2.csv" using 9:8 title 'CCorr' with linespoints ls 8,\
     "method3.csv" using 9:8 title 'CCorr Norm' with linespoints ls 9,\
     "method4.csv" using 9:8 title 'CCorr Coeff' with linespoints ls 10,\
     "method5.csv" using 9:8 title 'CCorr Coeff Norm' with linespoints ls 11

set terminal epslatex size 3.5,2.62 standalone color colortext 10
set output "${FILENAME}-sa.tex"

set datafile separator ","

plot "larks-roc.csv" using 9:8 title 'LARKS' with linespoints ls 5,\
     "method0.csv" using 9:8 title 'SQ Diff' with linespoints ls 6,\
     "method1.csv" using 9:8 title 'SQ Diff Norm' with linespoints ls 7,\
     "method2.csv" using 9:8 title 'CCorr' with linespoints ls 8,\
     "method3.csv" using 9:8 title 'CCorr Norm' with linespoints ls 9,\
     "method4.csv" using 9:8 title 'CCorr Coeff' with linespoints ls 10,\
     "method5.csv" using 9:8 title 'CCorr Coeff Norm' with linespoints ls 11

quit
PLOT

# Since most IEEE papers can't include eps file types, we have to convert the
# generated eps file into a PDF
epstopdf ${FILENAME}.eps

# Generate standalone postscript plot
latex ${FILENAME}-sa.tex
dvips -o ${FILENAME}-sa.ps ${FILENAME}-sa.dvi

# Convert ps file to pdf
ps2pdf ${FILENAME}-sa.ps

# Clean up intermediate files
rm -rf ${FILENAME}-sa.tex
rm -rf ${FILENAME}-sa.dvi
rm -rf ${FILENAME}-sa.ps
rm -rf *.eps
rm -rf *.aux
rm -rf *.dvi
rm -rf *.log

# Display the "standalone" pdf file
okular ${FILENAME}-sa.pdf
