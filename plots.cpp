
#include <iostream>
#include <sstream>
#include <string>
#include <gnuplot_ci.h>
#include <stdlib.h>

/*--------plot-------*/

using namespace gnuplot_ci;
using namespace std;
double xb[3] = {0,0,0};
double dx[3] = {1, 1, 1};

void plots(){

    //plot tree1
    GP_handle G4("/usr/bin/", "x-coordinate (pixels)", "y-coordinate (pixels)");
    G4.gnuplot_cmd("set terminal wxt");
    G4.gnuplot_cmd("set xrange [-320:320]");
    G4.gnuplot_cmd("set yrange [-320:320]");
    G4.gnuplot_cmd("set xtics 320");
    G4.gnuplot_cmd("set ytics 240");
    G4.gnuplot_cmd("plot '/home/mithun/build-rrt_planning_2d/start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', '/home/mithun/build-rrt_planning_2d/goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
    G4.gnuplot_cmd("replot '/home/mithun/build-rrt_planning_2d/tree1.txt'  u 1:2 w p pt 0 t 'Tree_1'");
    G4.gnuplot_cmd("replot '/home/mithun/build-rrt_planning_2d/tree1back.txt'  u 1:2 w l lw 1 t 's^*'");
    G4.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1.5 font 'Arial,22'");
    G4.gnuplot_cmd("set output '/home/mithun/build-rrt_planning_2d/tree1.eps'");
    G4.gnuplot_cmd("replot");

    cout << "Press enter to save plots" << endl;
    getchar();
}




