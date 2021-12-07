/*
 * Copyright (c) 2021, Jianming Han
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <limits>
#include <assert.h>
#include "dubins.h"

using namespace std;


//----------------------------------------------------------------
//          DEFINE A STRING ARRAY FOR PRINTING
//----------------------------------------------------------------
string dubinsWords[6] =
{
    "LSL",
    "RSR",
    "LSR",
    "RSL",
    "RLR",
    "LRL"
};


//----------------------------------------------------------------
//          DEFINE AUXILIARY UTILITY FUNCTIONS
//----------------------------------------------------------------
// Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise
float sinc(float t)
{
    float s;

    if (abs(t) < 0.002) // For small values of t use Taylor series approximation
    {
        s = 1 - pow(t,2) / 6.0 * (1 - pow(t,2) / 20.0);
    }
    else
    {
        s = sin(t) / t;
    }
    return s;
}

// Normalize an angle in range [0,2*pi)
float mod2Pi(float angle)
{
    return angle - 2 * M_PI * floor(angle / (2 * M_PI));
}

// Normalize an angular difference in range (-pi, pi]
float rangeSymm(float angle)
{
    float new_angle;

    new_angle = angle - 2 * M_PI * floor(angle / (2 * M_PI));
    if (new_angle > M_PI)
    {
        new_angle -= 2 * M_PI;
    }

    return new_angle;
}

// Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc-length s
robotPos circLine(robotPos pos0, float L, float k)
{
    robotPos pos1;

    pos1.x = pos0.x + L * sinc(k * L / 2.0) * cos(pos0.th + k * L / 2);
    pos1.y = pos0.y + L * sinc(k * L / 2.0) * sin(pos0.th + k * L / 2);
    pos1.th = mod2Pi(pos0.th + k *  L);
    
    return pos1;
}

// Check validity of a solution by evaluating explicitly the 3 equations
// defining a Dubins problem (in standard form)
bool check(standardLength sl, float *ks, scParams sc)
{
    float x0 = -1;
    float y0 = 0;
    float xf = 1;
    float yf = 0;
    double eq1, eq2, eq3;
    bool Lpos;

    eq1 = x0 + sl.sc_s1 * sinc((1/2.) * ks[0] * sl.sc_s1)
                        * cos(sc.sc_th0 + (1/2.) * ks[0] * sl.sc_s1)
             + sl.sc_s2 * sinc((1/2.) * ks[1] * sl.sc_s2)
                        * cos(sc.sc_th0 + ks[0] * sl.sc_s1 + (1/2.) * ks[1] * sl.sc_s2)
             + sl.sc_s3 * sinc((1/2.) * ks[2] * sl.sc_s3)
                        * cos(sc.sc_th0 + ks[0] * sl.sc_s1 + ks[1] * sl.sc_s2 + (1/2.) * ks[2] * sl.sc_s3)
             - xf;
    eq2 = y0 + sl.sc_s1 * sinc((1/2.) * ks[0] * sl.sc_s1)
                        * sin(sc.sc_th0 + (1/2.) * ks[0] * sl.sc_s1)
             + sl.sc_s2 * sinc((1/2.) * ks[1] * sl.sc_s2)
                        * sin(sc.sc_th0 + ks[0] * sl.sc_s1 + (1/2.) * ks[1] * sl.sc_s2)
             + sl.sc_s3 * sinc((1/2.) * ks[2] * sl.sc_s3)
                        * sin(sc.sc_th0 + ks[0] * sl.sc_s1 + ks[1] * sl.sc_s2 + (1/2.) * ks[2] * sl.sc_s3)
             - yf;
    eq3 = rangeSymm(ks[0] * sl.sc_s1 + ks[1] * sl.sc_s2 + ks[2] * sl.sc_s3 + sc.sc_th0 - sc.sc_thf);

    Lpos = (sl.sc_s1 > 0) || (sl.sc_s2 > 0) || (sl.sc_s3 > 0);

    return (sqrt(pow(eq1,2) + pow(eq2,2) + pow(eq3,2)) < 1.e-6) && Lpos;
}


//----------------------------------------------------------------
//          DEFINE SCALE FUNCTIONS
//----------------------------------------------------------------
// Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
scParamsWithLambda scaleToStandard(robotPos pos0, robotPos posf, float Kmax)
{
    scParamsWithLambda sc_lambda;
    float dx, dy, phi;

    dx = posf.x - pos0.x;
    dy = posf.y - pos0.y;
    phi = atan2(dy, dx);
    sc_lambda.lambda = hypot(dx, dy) / 2;
    sc_lambda.sc.sc_Kmax = Kmax * sc_lambda.lambda;
    sc_lambda.sc.sc_th0 = mod2Pi(pos0.th - phi);
    sc_lambda.sc.sc_thf = mod2Pi(posf.th - phi);

    return sc_lambda;
}

// Scale the solution to the standard problem back to the original problem
originalLength scaleFromStandard(float lambda, standardLength sl)
{
    originalLength ol;

    ol.s1 = sl.sc_s1 * lambda;
    ol.s2 = sl.sc_s2 * lambda;
    ol.s3 = sl.sc_s3 * lambda;

    return ol;
}
