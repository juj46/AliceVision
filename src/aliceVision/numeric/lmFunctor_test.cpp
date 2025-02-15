// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>
#include <fstream>
#include <string>
#include <aliceVision/system/Logger.hpp>
#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/numeric/LMFunctor.hpp"

#include "dependencies/vectorGraphics/svgDrawer.hpp"

#define BOOST_TEST_MODULE LMFunctor

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace svg;

// Implementation of the prolem found here :
// digilander.libero.it/foxes/optimiz/Optimiz1.htm

// Eigen LM functor to compute a mono-dimensional exponential regression
// We are looking for the exponential function that best fit the given point x,y
// f(x, c1, c2) = exp(c1*x) + c2
struct lm_Refine_functor : LMFunctor<double>
{
    lm_Refine_functor(int inputs, int values, const Vec& x, const Vec& y)
      : LMFunctor<double>(inputs, values),
        _x(x),
        _y(y)
    {}

    // The residual operator compute errors for the given x parameter
    int operator()(const Vec& x, Vec& fvec) const
    {
        // x contain the putative optimizer values
        double c1 = x[0];
        double c2 = x[1];

        // Evaluate the function cost for each input value
        for (Mat::Index i = 0; i < _x.rows(); ++i)
        {
            fvec[i] = _y[i] - (exp(c1 * _x[i]) + c2);
        }
        return 0;
    }

    const Vec &_x, &_y;  // Store data reference for cost evaluation
};

BOOST_AUTO_TEST_CASE(LM_MimimaSearchViaLM)
{
    Vec x(10);
    x << .5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5;
    Vec y(10);
    y << 1.7788, 1.6065, 1.4723, 1.3678, 1.2865, 1.2231, 1.1737, 1.1353, 1.1053, 1.0820;
    // We are looking for the exponential function that best fit the given point x,y
    // f(x, c1, c2) = exp(c1*x) + c2
    lm_Refine_functor functor(2,         // cardinal of the parameters: 2 (c1,c2)
                              x.rows(),  // cardinal of computed residual
                              x,
                              y  // Data that allow to compute the residual
    );
    typedef Eigen::NumericalDiff<lm_Refine_functor> NumDiffT;
    NumDiffT numDiff(functor);

    Eigen::LevenbergMarquardt<NumDiffT> lm(numDiff);
    lm.parameters.maxfev = 100;

    // The starting point to optimize (a sufficiently approximate value)
    Vec xlm(2);
    xlm << 0.0, 0.0;

    // Optimization by using LevenbergMarquardt routine
    int info = lm.minimize(xlm);
    // Get back optimized value
    ALICEVISION_LOG_DEBUG("info: " << info);

    Vec minima = xlm;
    Vec2 GT(-.4999, .9999);
    EXPECT_MATRIX_NEAR(GT, xlm, 1e-3);

    // Evaluation of the residual of the found solution
    Vec fvec(10);
    functor(xlm, fvec);
    BOOST_CHECK_SMALL(fvec.norm(), 1e-3);
    // We cannot expect more precision since input data are limited in precision

    // Export computed result to a SVG file
    {
        // Draw input data and found curve (magnification by 10 for visualization):
        double dFactor = 10.0;
        svgDrawer svgSurface(6 * dFactor, 4 * dFactor);
        // Draw found curve
        Vec xFound(30), yFound(30);
        int cpt = 0;
        for (double i = 0.0; i < 6.0; i += .2, ++cpt)
        {
            xFound[cpt] = i * dFactor;
            yFound[cpt] = (4 - (exp(xlm[0] * i) + xlm[1])) * dFactor;
        }

        svgSurface.drawPolyline(xFound.data(), xFound.data() + 30, yFound.data(), yFound.data() + 30, svgStyle().stroke("blue", 1.f));

        // Draw point in a second time to put them in the top layer
        for (Vec::Index i = 0; i < x.rows(); ++i)
            svgSurface.drawCircle(x[i] * dFactor, (4 - y[i]) * dFactor, 1, svgStyle().fill("red").noStroke());

        std::ostringstream osSvg;
        osSvg << "exponentialRegression_unit_test.svg";
        std::ofstream svgFile(osSvg.str());
        svgFile << svgSurface.closeSvgFile().str();
    }
}
