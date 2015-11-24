/*
 * glpkstrategyalgorithm.hpp
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
 * @class GLPKStrategyAlgorithm
 * @author Eusebio Aguilera
 * @date 24/11/15
 * @file glpkstrategyalgorithm.hpp
 * @brief This class implements the GLPK strategy of the strategy pattern. Please see
 * https://en.wikipedia.org/wiki/Strategy_pattern to obtain a definition of the pattern
 * behavior.
 */

#ifndef GLPKSTRATEGYALGORITHM_HPP
#define GLPKSTRATEGYALGORITHM_HPP

#include "strategyalgorithm.hpp"
#include <glpk.h>  /// GLPK
#include <cstdlib> /// malloc
#include <iostream>
#include <sstream> /// String Stream
#include <contour.hpp>
//#include <point.hpp>

class GLPKStrategyAlgorithm : public StrategyAlgorithm
{
public:
    GLPKStrategyAlgorithm(int log = 0);
    void LoadModel(const libcontour::ContourStats& c, int npoints);
    void Execute(void);

private:
    glp_prob* mip; /// MIP problem
    void ObjectiveFunction(const libcontour::ContourStats& cnt);
    int logLevel; /// Log level used
};

#endif /* GLPKSTRATEGYALGORITHM_HPP */
