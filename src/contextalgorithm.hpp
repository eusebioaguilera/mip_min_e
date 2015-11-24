/*
 * contextalgorithm.hpp
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
 * @class ContextAlgorithm
 * @author Eusebio Aguilera
 * @date 24/11/15
 * @file contextalgorithm.hpp
 * @brief This class implements the context item of the strategy pattern. Please see
 * https://en.wikipedia.org/wiki/Strategy_pattern to obtain a definition of the pattern
 * behavior.
 */

#ifndef CONTEXTALGORITHM_HPP
#define CONTEXTALGORITHM_HPP

#include "strategyalgorithm.hpp"
#include <iostream>
#include <string>

class ContextAlgorithm
{
public:
    ContextAlgorithm(StrategyAlgorithm& s) : strategy(&s) {};
    void ExecuteAlgorithm(void);
    double GetISE(void)
    {
        return this->strategy->GetISE();
    }
    std::string GetMask(void)
    {
        return this->strategy->GetMask();
    }

private:
    StrategyAlgorithm* strategy;
};

#endif /* CONTEXTALGORITHM_HPP */
