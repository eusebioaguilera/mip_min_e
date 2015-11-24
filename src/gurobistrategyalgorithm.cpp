/*
 * gurobistrategyalgorithm.cpp
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

#include "gurobistrategyalgorithm.hpp"

GUROBIStrategyAlgorithm::GUROBIStrategyAlgorithm(int log) : StrategyAlgorithm()
{
    this->logLevel = log;
}

void GUROBIStrategyAlgorithm::LoadModel(const libcontour::ContourStats& cnt, int npoints)
{
    try
    {
        int N = cnt.getContour().size();
        if (this->logLevel > 0)
            std::cout << "Creating objective function for " << npoints << " points ... ";
        /// Create objective function
        this->ObjectiveFunction(cnt);

        /// Add constraints to the model

        if (this->logLevel > 0) {
            std::cout << "OK" << std::endl;
            std::cout << "Setting constraint 1 for " << npoints << " points ... ";
        }

        /// Constraint 1 (mcounter += N)
        /// _CAN : D_N_N = 0
        // GRBLinExpr expr;
        for (int i = 0; i < N; i++) {
            std::stringstream ss;
            ss << "CA" << i;
            this->mip->addConstr(this->x[i][i] == 0, ss.str());
        }

        if (this->logLevel > 0) {
            std::cout << "OK" << std::endl;
            std::cout << "Setting constraint 2 for " << npoints << " points ... ";
        }

        /// Constraint 2 (mcounter += N^2)
        /// _CBN : D_0_0 + D_0_1 + ... + D_59_59 = npoints
        GRBLinExpr expr2;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                expr2 += this->x[i][j];
            }
        }

        this->mip->addConstr(expr2 == npoints, "CB0");

        if (this->logLevel > 0) {
            std::cout << "OK" << std::endl;
            std::cout << "Setting constraint 3 for " << npoints << " points ... ";
        }

        /// Constraint 3 (mcounter += N*(2*(N-1))
        /// _CCN : D_0_1 - D_1_0 + D_0_2 - D_2_0 + ... + D_0_59 - D_59_0 = 0
        for (int i = 0; i < N; i++) {
            GRBLinExpr expr3;
            std::stringstream ss;
            for (int j = 0; j < N; j++) {
                if (i != j) {
                    expr3 += this->x[i][j] - this->x[j][i];
                }
            }
            ss << "CC" << i;
            this->mip->addConstr(expr3 == 0, ss.str());
        }

        if (this->logLevel > 0) {
            std::cout << "OK" << std::endl;
            std::cout << "Setting constraint 4 for " << npoints << " points ... ";
        }

        /// Constraint 4
        /// _CDN: D_1_0 + D_2_0 + D_2_1 + ... + D_N-1_N-2 = 1
        GRBLinExpr expr4;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < i; j++) {
                expr4 += this->x[i][j];
            }
        }

        this->mip->addConstr(expr4 == 1, "CD0");

        if (this->logLevel > 0) {
            std::cout << "OK" << std::endl;
            std::cout << "Loading constraint matrix for " << npoints << " points ... ";
        }
    }
    catch (GRBException e)
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
}

void GUROBIStrategyAlgorithm::ObjectiveFunction(const libcontour::ContourStats& cnt)
{
    double** delta = NULL;
    double coef = 0.0;
    // libcontour::ContourStats stats;

    // stats.initialize(cnt);

    int npoints = cnt.getContour().size();
    int idx = 0;
    char buffer[256];

    // this->env = GRBEnv();		/// Enviroment

    this->mip = new GRBModel(this->env); /// Create the model

    if (this->logLevel > 0)
        std::cout << "Create the env" << std::endl;

    /// Create decision variables
    this->x = new GRBVar* [npoints];
    delta = new double* [npoints];

    for (int i = 0; i < npoints; i++) {
        this->x[i] = new GRBVar[npoints];
        delta[i] = new double[npoints];
    }

    if (this->logLevel > 0)
        std::cout << "Create the delta & x matrices" << std::endl;

    // Create the objective function
    for (int i = 0; i < npoints; i++) {
        for (int j = 0; j < npoints; j++) {
            // cout << "i " << i << " j " << j << endl;
            std::stringstream ss;
            ss << "x_" << i << j;
            this->x[i][j] = this->mip->addVar(0.0, 1.0, 0.0, GRB_BINARY, ss.str());

            delta[i][j] = cnt.computeISEForSegment(i, j);
        }
    }

    // Integrate new variables

    this->mip->update();

    if (this->logLevel > 0)
        std::cout << "Create the coef values" << std::endl;

    GRBLinExpr expr;

    for (int i = 0; i < npoints; i++) {
        for (int j = 0; j < npoints; j++) {
            if (i != j) {
                expr += delta[i][j] * this->x[i][j];
            }
        }
    }

    if (this->logLevel > 0) {
        std::cout << "Create the objective function" << std::endl;
        std::cout << "Size " << expr.size() << std::endl;
    }

    /// Add objective function
    this->mip->setObjective(expr, GRB_MINIMIZE);

    if (this->logLevel > 0)
        std::cout << "Set the objective function" << std::endl;
}

void GUROBIStrategyAlgorithm::Execute(void)
{
    /// Set log level
    int _log = 0;
    if (this->logLevel > 0)
        _log = 1;

    this->mip->getEnv().set(GRB_IntParam_OutputFlag, _log);

    /// Optimize the model
    this->mip->optimize();

    /// Obtain the function value
    double value = this->mip->get(GRB_DoubleAttr_ObjVal);

    /// Set value
    this->SetISE(value);

    /// Compute the mask
    int numvars = this->mip->get(GRB_IntAttr_NumVars);
    int N = std::sqrt(numvars);
    // std::cout << "Size " << numvars << std::endl;
    std::string mask;

    for (int i = 0; i < N; i++) {
        bool dominant = false;
        for (int j = 0; j < N; j++) {
            // std::cout << this->x[i][j].get(GRB_DoubleAttr_X) << ", ";
            if (this->x[i][j].get(GRB_DoubleAttr_X) > 0.5) {
                dominant = true;
                // std::cout << "(" << i << ", " << j << ") ";
            }
            // std::cout <<  << ", ";
        }
        if (dominant) {
            mask += '1';
        } else {
            mask += '0';
        }
        // std::cout << std::endl;
    }

    this->SetMask(mask);
}
