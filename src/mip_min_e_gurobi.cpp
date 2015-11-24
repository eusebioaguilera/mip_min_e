/*
 * mipgurobi.cpp
 *
 * Copyright 2015 Eusebio Aguilera <eusebio.aguilera@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

#include <iostream>                    /// cout & endl
#include <unistd.h>                    /// getopt
#include <cstdlib>                     /// atoi
#include <sys/time.h>                  /// gettimeofday
#include "contextalgorithm.hpp"        /// ContextAlgorithm
#include "gurobistrategyalgorithm.hpp" /// GUROBIStrategyAlgorithm
#include <contour.hpp>                 /// Contour

using namespace std;

void help(void)
{
    cout << "Usage:" << endl << endl;
    cout << "\t-h:\t\t\tPrints this help." << endl;
    cout << "\t-v:\t\t\tPrints version of the program." << endl;
    cout << "\t-l <log_level>:\t\tPrints log of the program. Valid values are 0 (no log), 1 (minimum log) and 2 (full "
            "log)." << endl;
    cout << "\t-f <contour_file>:\tContour file used for calculating the optimum polygonal approximation." << endl;
    cout << "\t-m <number_of_points>:\tNumber of points (M) of the polygonal approximation." << endl;
    cout << "\t-t:\t\t\tReturn the computation time needed to get the solution." << endl;
}

int main(int argc, char** argv)
{
    /// Version
    int mayor = 1, minor = 0, rev = 0;

    /// loglevel
    int loglevel = 0;

    /// file to use
    char* contourFile = NULL;

    /// Polygonal approximation M
    int M = 0;

    int c;

    /// Contour
    libcontour::ContourStats* cnt;

    /// Time check structs
    struct timeval startTime, finishTime;
    long mtime, seconds, useconds, total = 0.0;

    /// Computation time flag
    bool time = false;

    /// CR
    double cr;

    /// Checks for options
    while ((c = getopt(argc, argv, "hvl:f:m:t")) != -1)
        switch (c) {
        case 'h':
            help();
            exit(0);
            break;
        case 'v':
            cout << "Version " << mayor << "." << minor << "." << rev << endl;
            break;
        case 'l':
            loglevel = atoi(optarg);
            break;
        case 'f':
            contourFile = optarg;
            break;
        case 'm':
            M = atoi(optarg);
            break;
        case 't':
            time = true;
            break;
        default:
            abort();
        }

    if (contourFile) {
        cnt = new libcontour::ContourStats(contourFile);
    } else {
        help();
        cerr << "Contour file was not specified" << endl;
        abort();
    }

    if (M < 3) {
        help();
        cerr << "Incorrect number of point for the polygonal approximation" << endl;
        abort();
    }

    if (time)
        gettimeofday(&startTime, NULL);

    GUROBIStrategyAlgorithm gurobi(loglevel);
    gurobi.LoadModel(*cnt, M);
    ContextAlgorithm context(gurobi);

    context.ExecuteAlgorithm();

    if (time) {
        gettimeofday(&finishTime, NULL);

        seconds = finishTime.tv_sec - startTime.tv_sec;
        useconds = finishTime.tv_usec - startTime.tv_usec;

        mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
    }

    cr = (double)cnt->getContour().size() / M;

    std::cout << std::endl << "Solution" << std::endl;

    if (time) {
        std::cout << std::setw(10) << "M" << std::setw(20) << "ISE" << std::setw(10) << "CR" << std::setw(10)
                  << "ISE/CR" << std::setw(10) << "Time (s)" << std::endl;
        std::cout << std::setw(10) << M << std::setw(20) << context.GetISE() << std::setw(10) << cr << std::setw(10)
                  << context.GetISE() / cr << std::setw(10) << mtime / 1000.0 << std::endl;
    } else {
        std::cout << std::setw(10) << "M" << std::setw(20) << "ISE" << std::setw(10) << "CR" << std::setw(10)
                  << "ISE/CR" << std::endl;
        std::cout << std::setw(10) << M << std::setw(20) << context.GetISE() << std::setw(10) << cr << std::setw(10)
                  << context.GetISE() / cr << std::endl;
    }

    return 0;
}
