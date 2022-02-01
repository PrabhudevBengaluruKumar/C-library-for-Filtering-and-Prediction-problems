## Installation steps

sudo apt update

sudo apt install gsl-bin

sudo apt-get install libgsl-dev

sudo apt-get install libgsl-dbg

sudo apt-get install libgsl2

sudo apt-get install check

### command to compile

gcc kalmanLibrary.h -o kalmanLibrary

gcc Test_file.c linear.c kalmanLibrary.c `pkg-config --cflags --libs gsl` -o Test_file

gcc Test_file.c differential_drive.c kalmanLibrary.c `pkg-config --cflags --libs gsl` -o Test_file

gcc unit_tests.c linear.c kalmanLibrary.c `pkg-config --cflags --libs check` -lgsl -lgslcblas -lm -o unit_tests

gcc unit_tests.c differential_drive.c kalmanLibrary.c `pkg-config --cflags --libs check` -lgsl -lgslcblas -lm -o unit_tests

### command to run the code

./Test_file
