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
#include <cmath>
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


//----------------------------------------------------------------
//          DEFINE SIX DUBINS FUNCTIONS
//----------------------------------------------------------------
// LSL function
primitiveResult LSL(scParams sc)
{
    primitiveResult primitive;

    float invK = 1 / sc.sc_Kmax;
    float C = cos(sc.sc_thf) - cos(sc.sc_th0);
    float S = 2 * sc.sc_Kmax + sin(sc.sc_th0) - sin(sc.sc_thf);
    float temp1 = atan2(C, S);
    primitive.sl.sc_s1 = invK * mod2Pi(temp1 - sc.sc_th0);
    float temp2 = 2 + 4 * pow(sc.sc_Kmax,2) - 2 * cos(sc.sc_th0 - sc.sc_thf)
                    + 4 * sc.sc_Kmax * (sin(sc.sc_th0) - sin(sc.sc_thf));
    
    if (temp2 < 0)
    {
        primitive.ok = false;
        primitive.sl.sc_s1 = 0.0;
        primitive.sl.sc_s2 = 0.0;
        primitive.sl.sc_s3 = 0.0;
    }
    else
    {
        primitive.ok = true;
        primitive.sl.sc_s2 = invK * sqrt(temp2);
        primitive.sl.sc_s3 = invK * mod2Pi(sc.sc_thf - temp1);
    }

    return primitive;
}

// RSR function
primitiveResult RSR(scParams sc)
{
    primitiveResult primitive;

    float invK = 1 / sc.sc_Kmax;
    float C = cos(sc.sc_th0) - cos(sc.sc_thf);
    float S = 2 * sc.sc_Kmax - sin(sc.sc_th0) + sin(sc.sc_thf);
    float temp1 = atan2(C, S);
    primitive.sl.sc_s1 = invK * mod2Pi(sc.sc_th0 - temp1);
    float temp2 = 2 + 4 * pow(sc.sc_Kmax,2) - 2 * cos(sc.sc_th0 - sc.sc_thf)
                    - 4 * sc.sc_Kmax * (sin(sc.sc_th0) - sin(sc.sc_thf));
    
    if (temp2 < 0)
    {
        primitive.ok = false;
        primitive.sl.sc_s1 = 0.0;
        primitive.sl.sc_s2 = 0.0;
        primitive.sl.sc_s3 = 0.0;
    }
    else
    {
        primitive.ok = true;
        primitive.sl.sc_s2 = invK * sqrt(temp2);
        primitive.sl.sc_s3 = invK * mod2Pi(temp1 - sc.sc_thf);
    }

    return primitive;
}

// LSR function
primitiveResult LSR(scParams sc)
{
    primitiveResult primitive;

    float invK = 1 / sc.sc_Kmax;
    float C = cos(sc.sc_thf) + cos(sc.sc_th0);
    float S = 2 * sc.sc_Kmax + sin(sc.sc_th0) + sin(sc.sc_thf);
    float temp1 = atan2(-C, S);
    float temp3 = 4 * pow(sc.sc_Kmax,2) - 2 + 2 * cos(sc.sc_th0 - sc.sc_thf)
                  + 4 * sc.sc_Kmax * (sin(sc.sc_th0) + sin(sc.sc_thf));
    
    if (temp3 < 0)
    {
        primitive.ok = false;
        primitive.sl.sc_s1 = 0.0;
        primitive.sl.sc_s2 = 0.0;
        primitive.sl.sc_s3 = 0.0;
    }
    else
    {
        primitive.ok = true;
        primitive.sl.sc_s2 = invK * sqrt(temp3);
        float temp2 = -atan2(-2, primitive.sl.sc_s2 * sc.sc_Kmax);
        primitive.sl.sc_s1 = invK * mod2Pi(temp1 + temp2 - sc.sc_th0);
        primitive.sl.sc_s3 = invK * mod2Pi(temp1 + temp2 - sc.sc_thf);
    }

    return primitive;
}

// RSL function
primitiveResult RSL(scParams sc)
{
    primitiveResult primitive;

    float invK = 1 / sc.sc_Kmax;
    float C = cos(sc.sc_thf) + cos(sc.sc_th0);
    float S = 2 * sc.sc_Kmax - sin(sc.sc_th0) - sin(sc.sc_thf);
    float temp1 = atan2(C, S);
    float temp3 = 4 * pow(sc.sc_Kmax,2) - 2 + 2 * cos(sc.sc_th0 - sc.sc_thf)
                  - 4 * sc.sc_Kmax * (sin(sc.sc_th0) + sin(sc.sc_thf));
    
    if (temp3 < 0)
    {
        primitive.ok = false;
        primitive.sl.sc_s1 = 0.0;
        primitive.sl.sc_s2 = 0.0;
        primitive.sl.sc_s3 = 0.0;
    }
    else
    {
        primitive.ok = true;
        primitive.sl.sc_s2 = invK * sqrt(temp3);
        float temp2 = atan2(2, primitive.sl.sc_s2 * sc.sc_Kmax);
        primitive.sl.sc_s1 = invK * mod2Pi(sc.sc_th0 - temp1 + temp2);
        primitive.sl.sc_s3 = invK * mod2Pi(sc.sc_thf - temp1 + temp2);
    }

    return primitive;
}

// RLR function
primitiveResult RLR(scParams sc)
{
    primitiveResult primitive;

    float invK = 1 / sc.sc_Kmax;
    float C = cos(sc.sc_th0) - cos(sc.sc_thf);
    float S = 2 * sc.sc_Kmax - sin(sc.sc_th0) + sin(sc.sc_thf);
    float temp1 = atan2(C, S);
    float temp2 = 0.125 * (6 - 4 * pow(sc.sc_Kmax,2) + 2 * cos(sc.sc_th0 - sc.sc_thf)
                  + 4 * sc.sc_Kmax * (sin(sc.sc_th0) - sin(sc.sc_thf)));
    
    if (abs(temp2) > 1)
    {
        primitive.ok = false;
        primitive.sl.sc_s1 = 0.0;
        primitive.sl.sc_s2 = 0.0;
        primitive.sl.sc_s3 = 0.0;
    }
    else
    {
        primitive.ok = true;
        primitive.sl.sc_s2 = invK * mod2Pi(2 * M_PI - acos(temp2));
        primitive.sl.sc_s1 = invK * mod2Pi(sc.sc_th0 - temp1 +
                             0.5 * primitive.sl.sc_s2 * sc.sc_Kmax);
        primitive.sl.sc_s3 = invK * mod2Pi(sc.sc_th0 - sc.sc_thf +
                             sc.sc_Kmax * (primitive.sl.sc_s2 - primitive.sl.sc_s1));
    }

    return primitive;
}

// LRL function
primitiveResult LRL(scParams sc)
{
    primitiveResult primitive;
    
    float invK = 1 / sc.sc_Kmax;
    float C = cos(sc.sc_thf) - cos(sc.sc_th0);
    float S = 2 * sc.sc_Kmax + sin(sc.sc_th0) - sin(sc.sc_thf);
    float temp1 = atan2(C, S);
    float temp2 = 0.125 * (6 - 4 * pow(sc.sc_Kmax,2) + 2 * cos(sc.sc_th0 - sc.sc_thf)
                  - 4 * sc.sc_Kmax * (sin(sc.sc_th0) - sin(sc.sc_thf)));
    
    if (abs(temp2) > 1)
    {
        primitive.ok = false;
        primitive.sl.sc_s1 = 0.0;
        primitive.sl.sc_s2 = 0.0;
        primitive.sl.sc_s3 = 0.0;
    }
    else
    {
        primitive.ok = true;
        primitive.sl.sc_s2 = invK * mod2Pi(2 * M_PI - acos(temp2));
        primitive.sl.sc_s1 = invK * mod2Pi(temp1 - sc.sc_th0 +
                             0.5 * primitive.sl.sc_s2 * sc.sc_Kmax);
        primitive.sl.sc_s3 = invK * mod2Pi(sc.sc_thf - sc.sc_th0 +
                             sc.sc_Kmax * (primitive.sl.sc_s2 - primitive.sl.sc_s1));
    }

    return primitive;
}


//----------------------------------------------------------------
//          CREATE DUBINS CURVE
//----------------------------------------------------------------
// Create the arc of a dubins curve
dubinsArc createArc(robotPos pos0, float k, float L)
{
    dubinsArc arc;

    arc.pos0 = pos0;
    arc.k = k;
    arc.L = L;
    arc.posf = circLine(pos0, L, k);

    return arc;
}

// Create the dubins curve
dubinsCurve createCurve(robotPos pos0, originalLength ol, float *ks)
{
    dubinsCurve curve;

    curve.a1 = createArc(pos0, ks[0], ol.s1);
    curve.a2 = createArc(curve.a1.posf, ks[1], ol.s2);
    curve.a3 = createArc(curve.a2.posf, ks[2], ol.s3);
    curve.L = ol.s1 + ol.s2 + ol.s3;

    return curve;
}


//----------------------------------------------------------------
//          FIND SHORTEST DUBINS CURVE
//----------------------------------------------------------------
shortestDubinsResult dubinsShortestPath(robotPos pos0, robotPos posf, float Kmax, bool print)
{
    shortestDubinsResult result;

    scParamsWithLambda sc_lambda = scaleToStandard(pos0, posf, Kmax);

    if (print)
    {
        printf("\n****** Find the shortest dubins curve! ******\n");
        printf("\nStart pos: (%.2f, %.2f, %.2f)\n", pos0.x, pos0.y, pos0.th);
        printf("End pos: (%.2f, %.2f, %.2f)\n", posf.x, posf.y, posf.th);
        printf("Kmax = %.2f\n", Kmax);
        printf("\nScaled to standard:\n\n\tsc_th0: %.2f\n\tsc_thf: %.2f\n\tsc_Kmax: %.2f\n\tlambda: %.2f\n",
                sc_lambda.sc.sc_th0,
                sc_lambda.sc.sc_thf,
                sc_lambda.sc.sc_Kmax,
                sc_lambda.lambda);
    }

    DubinsTypes primitives[] =
    {
        LSL,
        RSR,
        LSR,
        RSL,
        RLR,
        LRL
    };

    const int ksigns[6][3] =
    {
         1,  0,  1,
        -1,  0, -1,
         1,  0, -1,
        -1,  0,  1,
        -1,  1, -1,
         1, -1,  1
    };

    float L = numeric_limits<float>::infinity();
    float Lcur;
    primitiveResult pr_cur;
    standardLength sl_best;

    for (int i=0; i<6; i++)
    {
        pr_cur = primitives[i](sc_lambda.sc);
        Lcur = pr_cur.sl.sc_s1 + pr_cur.sl.sc_s2 + pr_cur.sl.sc_s3;

        if (pr_cur.ok && Lcur < L)
        {
            L = Lcur;
            sl_best = pr_cur.sl;
            result.pidx = i;
        }

        if (print)
        {
            printf("\n--> %s:\n", dubinsWords[i].c_str());
            printf("\tsc_s1 = %.2f\n\tsc_s2 = %.2f\n\tsc_s3 = %.2f\n\tsum = %.2f\n",
                    pr_cur.sl.sc_s1,
                    pr_cur.sl.sc_s2,
                    pr_cur.sl.sc_s3,
                    Lcur);
            printf("\nCurrent best: %s, std length: %.2f\n", dubinsWords[result.pidx].c_str(), L);
        }
    }

    if (result.pidx > -1)
    {
        originalLength ol = scaleFromStandard(sc_lambda.lambda, sl_best);
        float ks[3] =
        {
            ksigns[result.pidx][0] * Kmax,
            ksigns[result.pidx][1] * Kmax,
            ksigns[result.pidx][2] * Kmax
        };
        float sc_ks[3] = 
        {
            ksigns[result.pidx][0] * sc_lambda.sc.sc_Kmax,
            ksigns[result.pidx][1] * sc_lambda.sc.sc_Kmax,
            ksigns[result.pidx][2] * sc_lambda.sc.sc_Kmax
        };

        result.curve = createCurve(pos0, ol, ks);

        result.dubinsWPList = getDubinsWaypoints(result.curve);

        // assert(check(sl_best, sc_ks, sc_lambda.sc));

        if (print)
        {
            printf("\n****** Final result ******\n");
            printf("\nBest solution: %s, total lenght: %.2f\n",
                    dubinsWords[result.pidx].c_str(),
                    result.curve.L);
            
            printf("\n--> Curve 1/3:\n");
            printf("\tStart pos: (%.2f, %.2f, %.2f)\n",
                    result.curve.a1.pos0.x,
                    result.curve.a1.pos0.y,
                    result.curve.a1.pos0.th);
            printf("\tL = %.2f\n", result.curve.a1.L);
            printf("\tk = %.2f\n", result.curve.a1.k);

            printf("\n--> Curve 2/3:\n");
            printf("\tStart pos: (%.2f, %.2f, %.2f)\n",
                    result.curve.a2.pos0.x,
                    result.curve.a2.pos0.y,
                    result.curve.a2.pos0.th);
            printf("\tL = %.2f\n", result.curve.a2.L);
            printf("\tk = %.2f\n", result.curve.a2.k);

            printf("\n--> Curve 3/3:\n");
            printf("\tStart pos: (%.2f, %.2f, %.2f)\n",
                    result.curve.a3.pos0.x,
                    result.curve.a3.pos0.y,
                    result.curve.a3.pos0.th);
            printf("\tL = %.2f\n", result.curve.a3.L);
            printf("\tk = %.2f\n", result.curve.a3.k);

            printf("\nWaypoint list:\n");
            for (auto it = result.dubinsWPList.begin(); it != result.dubinsWPList.end(); ++it)
            {
                printf("x = %.2f\ty = %.2f\ttheta = %.2f\ts = %.2f\tk = %.2f\n",
                        (*it).pos.x, (*it).pos.y, (*it).pos.th, (*it).s, (*it).k);
            }
        }
    }

    return result;
}


//----------------------------------------------------------------
//          GET DUBINS PATH WAYPOINTS
//----------------------------------------------------------------
// Get waypoints of arc
void getArcWaypoints(dubinsArc arc, vector<dubinsWaypoint>& dubinsWPList)
{
    float ds = 0.05;
    float s = dubinsWPList.empty() ? 0.0 : dubinsWPList.back().s;
    for (float l=0; l<arc.L; l+=ds)
    {
        dubinsWaypoint wpt;
        wpt.pos = circLine(arc.pos0, l, arc.k);
        wpt.s = s + l;
        wpt.k = arc.k;
        dubinsWPList.push_back(wpt);
    }
}

// Get waypoints of curve
vector<dubinsWaypoint> getDubinsWaypoints(dubinsCurve curve)
{
    vector<dubinsWaypoint> dubinsWPList;
    getArcWaypoints(curve.a1, dubinsWPList);
    getArcWaypoints(curve.a2, dubinsWPList);
    getArcWaypoints(curve.a3, dubinsWPList);

    return dubinsWPList;
}