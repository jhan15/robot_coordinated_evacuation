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
#include "collision_detect.hpp"

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
    primitive.sum_sl = primitive.sl.sc_s1 + primitive.sl.sc_s2 + primitive.sl.sc_s3;

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
    primitive.sum_sl = primitive.sl.sc_s1 + primitive.sl.sc_s2 + primitive.sl.sc_s3;

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
    primitive.sum_sl = primitive.sl.sc_s1 + primitive.sl.sc_s2 + primitive.sl.sc_s3;

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
    primitive.sum_sl = primitive.sl.sc_s1 + primitive.sl.sc_s2 + primitive.sl.sc_s3;

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
    primitive.sum_sl = primitive.sl.sc_s1 + primitive.sl.sc_s2 + primitive.sl.sc_s3;

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
    primitive.sum_sl = primitive.sl.sc_s1 + primitive.sl.sc_s2 + primitive.sl.sc_s3;

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
shortestDubinsResult dubinsPath(robotPos pos0, robotPos posf, float Kmax, vector<vector<pt>> obstacles, bool print)
{
    shortestDubinsResult sd;

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

    primitiveResult pr_cur;
    standardLength sl_best;
    vector<dubinsCurve> result;

    for (int i=0; i<6; i++)
    {
        pr_cur = primitives[i](sc_lambda.sc);
        pr_cur.id = i;
        if (print)
        {
            printf("\n--> %s:\n", dubinsWords[i].c_str());
            printf("\tsc_s1 = %.2f\n\tsc_s2 = %.2f\n\tsc_s3 = %.2f\n\tsum = %.2f\n",
                    pr_cur.sl.sc_s1,
                    pr_cur.sl.sc_s2,
                    pr_cur.sl.sc_s3,
                    pr_cur.sum_sl);
        }

        if (!pr_cur.ok) continue;

        originalLength ol = scaleFromStandard(sc_lambda.lambda, pr_cur.sl);
        float ks[3] =
        {
            ksigns[pr_cur.id][0] * Kmax,
            ksigns[pr_cur.id][1] * Kmax,
            ksigns[pr_cur.id][2] * Kmax
        };
        dubinsCurve dc = createCurve(pos0, ol, ks);

        if (result.size()==0) result.push_back(dc);
        else if (result.size()==1)
        {
            if (dc.L > result.back().L) result.push_back(dc);
            else result.insert(result.begin(), dc);
        }
        else
        {
            if (dc.L < result[0].L) result.insert(result.begin(), dc);
            else if (dc.L > result.back().L) result.push_back(dc);
            else {
                for (int j=0; j<(result.size()-1); j++)
                {
                    if (dc.L >= result[j].L && dc.L <= result[j+1].L)
                    {
                        result.insert(result.begin()+j+1, dc);
                        break;
                    }
                }
            }
        }
    }

    if (result.size() > 0){
        for (int i = 0; i < result.size(); i++){
            bool coll = false;
            if (result[i].a1.L > 0){
                coll = checkCollision(result[i].a1, obstacles, Kmax);
            }
            if (result[i].a2.L > 0 && !coll){
                coll = checkCollision(result[i].a2, obstacles, Kmax);
            }
            if (result[i].a3.L > 0 && !coll){
                coll = checkCollision(result[i].a3, obstacles, Kmax);
            }
            if (!coll){
                sd.find_dubins = true;
                sd.curve = result[i];
                sd.dubinsWPList = getDubinsWaypoints(result[i]);
                break;
            }
        }
    }

    return sd;
}

// Check if two arcs are intersected
bool checkCollision(dubinsArc a, vector<vector<pt>> obs, float Kmax, bool print)
{
    vector<cv::Point2d> line2;
    double x, y, s, e;

    if (a.k == 0){
        line2 = {cv::Point2d(a.pos0.x, a.pos0.y), cv::Point2d(a.posf.x, a.posf.y)};
    }
    else {
        if (a.k > 0){
            x = a.pos0.x - sin(a.pos0.th)/Kmax;
            y = a.pos0.y + cos(a.pos0.th)/Kmax;
            s = mod2Pi(a.pos0.th - M_PI/2);
            e = mod2Pi(a.posf.th - M_PI/2);
        }
        if (a.k < 0){
            x = a.pos0.x + sin(a.pos0.th)/Kmax;
            y = a.pos0.y - cos(a.pos0.th)/Kmax;
            s = mod2Pi(a.posf.th + M_PI/2);
            e = mod2Pi(a.pos0.th + M_PI/2);
        }
    }
    for (int i = 0; i < obs.size(); i++){
        for (int j = 0; j < (obs[i].size()-1); j++){
            vector<cv::Point2d> line1 = {cv::Point2d(obs[i][j].x, obs[i][j].y),
                                     cv::Point2d(obs[i][j+1].x, obs[i][j+1].y)};
            if (a.k == 0){
                vector<cv::Point2d> pts = line_line_coll(line1, line2);
                if (pts.size() > 0) return true;
            }
            else {
                bool re = arc_line_coll(x, y, 1/Kmax, s, e, line1);
                if (re) return true;
            }
        }
    }

    return false;
}

// Check if two dubins curves are intersected (not used)
bool checkTwoDubins(dubinsCurve c1, dubinsCurve c2, float Kmax)
{
    vector<dubinsArc> v1 = {c1.a1, c1.a2, c1.a3};
    vector<dubinsArc> v2 = {c2.a1, c2.a2, c2.a3};
    vector<cv::Point2d> line1, line2;
    double x1, y1, s1, e1;
    double x2, y2, s2, e2;

    for (auto it0 = v1.begin(); it0 != v1.end(); ++it0){
        for (auto it1 = v2.begin(); it1 != v2.end(); ++it1){
            if ((*it0).k == 0 && (*it1).k == 0){
                line1 = {cv::Point2d((*it0).pos0.x, (*it0).pos0.y), cv::Point2d((*it0).posf.x, (*it0).posf.y)};
                line2 = {cv::Point2d((*it1).pos0.x, (*it1).pos0.y), cv::Point2d((*it1).posf.x, (*it1).posf.y)};
                vector<cv::Point2d> pts = line_line_coll(line1, line2);
                if (pts.size() > 0) return true;
            }
            if ((*it0).k == 0 && (*it1).k != 0){
                line1 = {cv::Point2d((*it0).pos0.x, (*it0).pos0.y), cv::Point2d((*it0).posf.x, (*it0).posf.y)};
                if ((*it1).k > 0){
                    x2 = (*it1).pos0.x - sin((*it1).pos0.th)/Kmax;
                    y2 = (*it1).pos0.y + cos((*it1).pos0.th)/Kmax;
                    s2 = mod2Pi((*it1).pos0.th - M_PI/2);
                    e2 = mod2Pi((*it1).posf.th - M_PI/2);
                }
                if ((*it1).k < 0){
                    x2 = (*it1).pos0.x + sin((*it1).pos0.th)/Kmax;
                    y2 = (*it1).pos0.y - cos((*it1).pos0.th)/Kmax;
                    s2 = mod2Pi((*it1).posf.th + M_PI/2);
                    e2 = mod2Pi((*it1).pos0.th + M_PI/2);
                }
                bool re = arc_line_coll(x2, y2, 1/Kmax, s2, e2, line1);
                if (re) return true;
            }
            if ((*it0).k != 0 && (*it1).k == 0){
                line2 = {cv::Point2d((*it1).pos0.x, (*it1).pos0.y), cv::Point2d((*it1).posf.x, (*it1).posf.y)};
                if ((*it0).k > 0){
                    x1 = (*it0).pos0.x - sin((*it0).pos0.th)/Kmax;
                    y1 = (*it0).pos0.y + cos((*it0).pos0.th)/Kmax;
                    s1 = mod2Pi((*it0).pos0.th - M_PI/2);
                    e1 = mod2Pi((*it0).posf.th - M_PI/2);
                }
                if ((*it0).k < 0){
                    x1 = (*it0).pos0.x + sin((*it0).pos0.th)/Kmax;
                    y1 = (*it0).pos0.y - cos((*it0).pos0.th)/Kmax;
                    s1 = mod2Pi((*it0).posf.th + M_PI/2);
                    e1 = mod2Pi((*it0).pos0.th + M_PI/2);
                }
                bool re = arc_line_coll(x1, y1, 1/Kmax, s1, e1, line2);
                if (re) return true;
            }
            if ((*it0).k != 0 && (*it1).k != 0){
                if ((*it0).k > 0){
                    x1 = (*it0).pos0.x - sin((*it0).pos0.th)/Kmax;
                    y1 = (*it0).pos0.y + cos((*it0).pos0.th)/Kmax;
                    s1 = mod2Pi((*it0).pos0.th - M_PI/2);
                    e1 = mod2Pi((*it0).posf.th - M_PI/2);
                }
                if ((*it0).k < 0){
                    x1 = (*it0).pos0.x + sin((*it0).pos0.th)/Kmax;
                    y1 = (*it0).pos0.y - cos((*it0).pos0.th)/Kmax;
                    s1 = mod2Pi((*it0).posf.th + M_PI/2);
                    e1 = mod2Pi((*it0).pos0.th + M_PI/2);
                }
                if ((*it1).k > 0){
                    x2 = (*it1).pos0.x - sin((*it1).pos0.th)/Kmax;
                    y2 = (*it1).pos0.y + cos((*it1).pos0.th)/Kmax;
                    s2 = mod2Pi((*it1).pos0.th - M_PI/2);
                    e2 = mod2Pi((*it1).posf.th - M_PI/2);
                }
                if ((*it1).k < 0){
                    x2 = (*it1).pos0.x + sin((*it1).pos0.th)/Kmax;
                    y2 = (*it1).pos0.y - cos((*it1).pos0.th)/Kmax;
                    s2 = mod2Pi((*it1).posf.th + M_PI/2);
                    e2 = mod2Pi((*it1).pos0.th + M_PI/2);
                }
                bool re = arc_arc_coll(x1, y1, 1/Kmax, s1, e1, x2, y2, 1/Kmax, s2, e2);
                if (re) return true;
            }
        }
    }
    return false;
}


//----------------------------------------------------------------
//          GET DUBINS PATH WAYPOINTS
//----------------------------------------------------------------
// Get waypoints of arc
void getArcWaypoints(dubinsArc arc, vector<dubinsWaypoint>& dubinsWPList)
{
    float ds = 0.02;
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


//----------------------------------------------------------------
//          MULTIPOINTS DUBINS
//----------------------------------------------------------------
vector<shortestDubinsResult> dubinsIDP(vector<robotPos> path_points, vector<vector<pt>> obs, float Kmax)
{
    vector<shortestDubinsResult> mdubins;

    int ntheta = 72;
    int npoint = path_points.size();
    float dp[npoint-1][ntheta];
    int dp_index[npoint-1][ntheta];

    float dtheta = 2*M_PI/ntheta;

    if (npoint == 2){
        dp[0][0] = numeric_limits<float>::infinity();
        for (int i = 0; i < ntheta; i++){
            path_points[1].th = i*dtheta;
            shortestDubinsResult sd = dubinsPath(path_points[0], path_points[1], Kmax, obs);
            
            if (!sd.find_dubins) continue;
            
            if (sd.curve.L < dp[0][0]){
                dp[0][0] = sd.curve.L;
                dp_index[0][0] = i;
            }
        }
    }
    else{
        for (int i = npoint-2; i > 0; i--){
            for (int j = 0; j < ntheta; j++){
                dp[i][j] = numeric_limits<float>::infinity();
                path_points[i].th = j*dtheta;
                for (int k = 0; k < ntheta; k++){
                    path_points[i+1].th = k*dtheta;
                    shortestDubinsResult sd = dubinsPath(path_points[i], path_points[i+1], Kmax, obs);
                    
                    if (!sd.find_dubins) continue;

                    if (i == npoint-2){
                        if (sd.curve.L < dp[i][j]){
                            dp[i][j] = sd.curve.L;
                            dp_index[i][j] = k;
                        }
                    }
                    else{
                        if ((sd.curve.L + dp[i+1][k]) < dp[i][j]){
                            dp[i][j] = sd.curve.L + dp[i+1][k];
                            dp_index[i][j] = k;
                        }
                    }
                }
            }
        }
        dp[0][0] = numeric_limits<float>::infinity();
        for (int i = 0; i < ntheta; i++){
            path_points[1].th = i*dtheta;
            shortestDubinsResult sd = dubinsPath(path_points[0], path_points[1], Kmax, obs);

            if (!sd.find_dubins) continue;
            
            if ((sd.curve.L + dp[1][i]) < dp[0][0]){
                dp[0][0] = sd.curve.L + dp[1][i];
                dp_index[0][0] = i;
            }
        }
    }

    if (dp[0][0] < numeric_limits<float>::infinity()){
        int id;
        cout<<"---- points "<<0<<" ("<<path_points[0].x<<", "<<path_points[0].y<<", "<<path_points[0].th<<")"<<endl;
        for (int i = 0; i < npoint-1; i++){
            if (i == 0){
                id = dp_index[0][0];
            }
            else{
                id = dp_index[i][id];
            }
            path_points[i+1].th = id*dtheta;
            cout<<"---- points "<<i+1<<" ("<<path_points[i+1].x<<", "<<path_points[i+1].y<<", "<<path_points[i+1].th<<")"<<endl;
            shortestDubinsResult sd = dubinsPath(path_points[i], path_points[i+1], Kmax, obs);
            mdubins.emplace_back(sd);
        }
        cout<<"     feasible dubins found with length: "<<dp[0][0]<<endl<<endl;
    }

    return mdubins;
}
