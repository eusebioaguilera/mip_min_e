/*
 * strategyalgorithm.h
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
 * @class StrategyAlgorithm
 * @author Eusebio Aguilera
 * @date 24/11/15
 * @file strategyalgorithm.hpp
 * @brief This class implements the strategy item of the strategy pattern. Please see
 * https://en.wikipedia.org/wiki/Strategy_pattern to obtain a definition of the pattern
 * behavior.
 */

#ifndef STRATEGYALGORITHM_HPP
#define STRATEGYALGORITHM_HPP

#include <contour.hpp>
#include <iostream>
#include <string>

class StrategyAlgorithm
{
public:
    /// virtual StrategyAlgorithm();
    virtual void LoadModel(const libcontour::ContourStats& c, int npoints) = 0;
    virtual void Execute(void) = 0;
    double GetISE(void)
    {
        return this->ise;
    }
    void SetISE(double value)
    {
        this->ise = value;
    }
    std::string GetMask(void)
    {
        return this->mask;
    }
    void SetMask(const std::string& value)
    {
        this->mask = value;
    }

private:
    double ise;
    std::string mask;
};

#endif /* STRATEGYALGORITHM_H */
