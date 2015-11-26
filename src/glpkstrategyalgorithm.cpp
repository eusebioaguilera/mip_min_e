/*
 * glpkstrategyalgorithm.cpp
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

#include "glpkstrategyalgorithm.hpp"

GLPKStrategyAlgorithm::GLPKStrategyAlgorithm(int log) : StrategyAlgorithm()
{
    this->logLevel = log;
}

void GLPKStrategyAlgorithm::LoadModel(const libcontour::ContourStats& cnt, int npoints)
{

    int N = cnt.getContour().size(); /// Contour size
    int rows = (2 * N) + 2;
    int cols = (int)pow(N, 2);
    int ccounter = 1; /// Constraint counter
    int mcounter = 1; /// Matrix item counter
    std::stringstream ss;

    /// Create the problem
    this->mip = glp_create_prob();
    /// Set problem's name
    glp_set_prob_name(this->mip, "Polygonal approximation problem");
    /// Set function to minimize
    glp_set_obj_dir(this->mip, GLP_MIN);

    /// Add the number of constraints we need
    glp_add_rows(mip, rows);
    glp_add_cols(mip, cols);

    /// Create objective function
    this->ObjectiveFunction(cnt);

    /// Allocate memory for constraint matrix
    int* ia, *ja;
    double* ar;

    /// Number of items
    int items;
    int m = 0;

    for (int i = 1; i < N; i++) {
        m += i;
    }

    items = N + (N * N) + (N * (2 * (N - 1))) + m;

    ia = (int*)malloc(sizeof(int) * (1 + items));
    ja = (int*)malloc(sizeof(int) * (1 + items));
    ar = (double*)malloc(sizeof(double) * (1 + items));

    if (this->logLevel > 0)
        std::cout << "Setting constraint 1 for " << npoints << " points ... ";
    /// Constraint 1 (mcounter += N)
    /// _CAN : D_N_N = 0
    for (int i = 0; i < N; i++) {
        ss << "_CA" << i;
        glp_set_row_name(this->mip, ccounter, ss.str().c_str());
        glp_set_row_bnds(this->mip, ccounter, GLP_FX, 0, 0);
        ia[mcounter] = ccounter, ja[mcounter] = ((i * N) + i + 1), ar[mcounter] = 1;
        mcounter++;
        ccounter++;
        /// Clear ss
        ss.str(std::string());
    }

    if (this->logLevel > 0) {
        std::cout << "OK" << std::endl;
        std::cout << "Setting constraint 2 for " << npoints << " points ... ";
    }

    /// Constraint 2 (mcounter += N^2)
    /// _CBN : D_0_0 + D_0_1 + ... + D_59_59 = npoints
    ss.str(std::string());
    ss << "_CB" << 0;
    glp_set_row_name(this->mip, ccounter, ss.str().c_str());
    glp_set_row_bnds(this->mip, ccounter, GLP_FX, npoints, npoints);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            ia[mcounter] = ccounter, ja[mcounter] = ((N * i) + j + 1), ar[mcounter] = 1;
            mcounter++;
        }
    }
    ccounter++;

    if (this->logLevel > 0) {
        std::cout << "OK" << std::endl;
        std::cout << "Setting constraint 3 for " << npoints << " points ... ";
    }

    /// Constraint 3 (mcounter += N*(2*(N-1))
    /// _CCN : D_0_1 - D_1_0 + D_0_2 - D_2_0 + ... + D_0_59 - D_59_0 = 0
    for (int i = 0; i < N; i++) {

        ss.str(std::string());
        ss << "_CC" << i;
        glp_set_row_name(this->mip, ccounter, ss.str().c_str());
        glp_set_row_bnds(this->mip, ccounter, GLP_FX, 0, 0);

        for (int j = 0; j < N; j++) {
            if (i != j) {
                ia[mcounter] = ccounter, ja[mcounter] = ((N * i) + j + 1), ar[mcounter] = 1; /// +1 X_i_j
                mcounter++;
                ia[mcounter] = ccounter, ja[mcounter] = ((N * j) + i + 1), ar[mcounter] = -1; /// -1 X_j_i
                mcounter++;
            }
        }
        ccounter++;
    }

    if (this->logLevel > 0) {
        std::cout << "OK" << std::endl;
        std::cout << "Setting constraint 4 for " << npoints << " points ... ";
    }

    /// Constraint 4
    /// _CDN: D_1_0 + D_2_0 + D_2_1 + ... + D_N-1_N-2 = 1
    ss.str(std::string());
    ss << "_CD" << 0;
    glp_set_row_name(this->mip, ccounter, ss.str().c_str());
    glp_set_row_bnds(this->mip, ccounter, GLP_FX, 1, 1);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < i; j++) {
            ia[mcounter] = ccounter, ja[mcounter] = ((N * i) + j + 1), ar[mcounter] = 1;
            mcounter++;
        }
    }

    if (this->logLevel > 0) {
        std::cout << "OK" << std::endl;
        std::cout << "Loading constraint matrix for " << npoints << " points ... ";
    }

    /// Load constraint matrix
    glp_load_matrix(this->mip, items, ia, ja, ar);

    /// Free ia, ja, ar
    free(ia);
    free(ja);
    free(ar);

    if (this->logLevel > 0) {
        std::cout << "OK" << std::endl;
    }
}

/**
    This method calculates the objetctive function and set to the MIP problem
    @param cnt This contour object represents the contour which the method uses to solve the problem
*/

void GLPKStrategyAlgorithm::ObjectiveFunction(const libcontour::ContourStats& cnt)
{
    double coef = 0.0;

    int npoints = cnt.getContour().size();
    int idx = 0;
    char buffer[256];

    // Create the objective function
    for (int i = 0; i < npoints; i++) {
        for (int j = 0; j < npoints; j++) {
            // cout << "i " << i << " j " << j << endl;

            idx = (i * npoints) + j + 1;
            sprintf(buffer, "X_%d_%d", i, j);
            glp_set_col_name(this->mip, idx, buffer); /// Set name to X_i_j
            glp_set_col_kind(this->mip, idx, GLP_BV); /// Set var to binary type

            // if ( i != j ) {
            ////cout << "Rect(" << i << ", " << j << ") to points: ";
            ////Rect r = Rect( cnt[i],cnt[j] );
            // libcontour::Point pi = cnt[i];
            // libcontour::Point pj = cnt[j];

            ////int k = (i + 1) % npoints;
            ////int k_end = i+( (j-i) % npoints );
            ////while ( k != k_end ) {
            //////cout << k << ",";
            ////libcontour::Point pk = cnt[k];
            ////if ( !(pi == pj) ) {
            ////coef += pow( ( (pk.x() - pi.x()) * (pj.y() - pi.y()) - (pk.y() - pi.y()) * (pj.x() - pi.x()) ) ,2 ) / (
            /// pow( pi.x() - pj.x() ,2 ) + pow( pi.y() - pj.y() , 2) );
            ////} else {
            ////// Distance from pi(pj) to pk
            ////coef += sqrt( pow( pi.x() - pk.x(), 2 ) + pow( pi.y() - pk.y(), 2 ) );
            ////}
            ////k = (k + 1) % npoints;
            ////}

            // glp_set_obj_coef(this->mip, idx, coef);

            ////cout << coef << " dominants_" << i << "_" << j << " + ";
            // coef = 0.0;
            //}
            glp_set_obj_coef(this->mip, idx, cnt.computeISEForSegment(i, j));
        }
    }
}

void GLPKStrategyAlgorithm::Execute(void)
{
    glp_iocp parm; /// Parm for MIP

    /// std::cout << "Executing from GLPK" << std::endl;

    glp_init_iocp(&parm);
    parm.presolve = GLP_ON;
    parm.gmi_cuts = GLP_ON; /// Gomory's Cuts

    if (this->logLevel > 1)
        glp_term_out(1);
    else
        glp_term_out(0);

    // glp_write_lp( this->mip, NULL, "temp.lp" );

    int err = glp_intopt(this->mip, &parm);

    double value = glp_mip_obj_val(this->mip);

    this->SetISE(value);
}
