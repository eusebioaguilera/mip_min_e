/*
 * gurobistrategyalgorithm.hpp
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

/**
 * @class GUROBIStrategyAlgorithm
 * @author Eusebio Aguilera
 * @date 24/11/15
 * @file gurobistrategyalgorithm.hpp
 * @brief This class implements the Gurobi strategy of the strategy pattern. Please see
 * https://en.wikipedia.org/wiki/Strategy_pattern to obtain a definition of the pattern
 * behavior.
 */

#ifndef GUROBISTRATEGYALGORITHM_HPP
#define GUROBISTRATEGYALGORITHM_HPP

#include "strategyalgorithm.hpp"
#include <gurobi_c++.h> /// GUROBI
#include <iostream>
#include <sstream> /// StringStream
#include <contour.hpp>
//#include <point.hpp>

class GUROBIStrategyAlgorithm : public StrategyAlgorithm
{
public:
    GUROBIStrategyAlgorithm(int log = 0);
    void LoadModel(const libcontour::ContourStats& c, int npoints);
    void Execute(void);

private:
    GRBModel* mip; /// Model
    GRBVar** x;    /// Decision variables matrix
    GRBEnv env;    /// Gurobi enviroment
    void ObjectiveFunction(const libcontour::ContourStats& cnt);
    int logLevel; /// Log level used
};

#endif /* GUROBISTRATEGYALGORITHM_HPP */
