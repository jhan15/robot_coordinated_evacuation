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

#include <vector>
using namespace std;

#ifndef DUBINS_H
#define DUBINS_H


//----------------------------------------------------------------
//          DEFINE SOME STRUCTS THAT WILL BE USED
//----------------------------------------------------------------


struct pt
{
    float x;
    float y;
};

// Define the structure of robot pos
struct robotPos
{
    float x;
    float y;
    float th;
};

// Define the structure of dubins arc: line segment or arc
struct dubinsArc
{
    robotPos pos0;
    robotPos posf;
    float k;
    float L;
};

// Define the structure of a complete dubins curve
// Three parts: a1, a2, a3
// Total length: L
struct dubinsCurve
{
    dubinsArc a1;
    dubinsArc a2;
    dubinsArc a3;
    float L;
};

// Define the structure of parameters scaled to standard
struct scParams
{
    float sc_th0;
    float sc_thf;
    float sc_Kmax;
};

// Define the structure of parameters scaled to standard with lambda
struct scParamsWithLambda
{
    scParams sc;
    float lambda;
};

// Define the structure of standard lengths of a dubins curve
struct standardLength
{
    float sc_s1;
    float sc_s2;
    float sc_s3;
};

// Define the structure of original lengths of a dubins curve
struct originalLength
{
    float s1;
    float s2;
    float s3;
};

// Define the structure of result by one of the six dubins curve types
// LSL, RSR, LSR, RSL, RLR, LRL
struct primitiveResult
{
    bool ok;
    standardLength sl;
    float sum_sl;
    int id;
};

// Define dubins waypoint structure
struct dubinsWaypoint
{
    robotPos pos;
    float s;
    float k;
};

// Define the result structure of finding the shortest dubins path
struct shortestDubinsResult
{
    bool find_dubins = false;
    dubinsCurve curve;
    vector<dubinsWaypoint> dubinsWPList;
};


//----------------------------------------------------------------
//          DEFINE POINTER TO ALL DUBINS TYPES
//----------------------------------------------------------------
typedef primitiveResult (*DubinsTypes) (scParams sc);


//----------------------------------------------------------------
//          FUNCTIONS FOR DUBINS CURVE
//----------------------------------------------------------------

//----------------------------------------------------------------
//          DEFINE AUXILIARY UTILITY FUNCTIONS
//----------------------------------------------------------------
// Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise
float sinc(float t);

// Normalize an angle in range [0,2*pi)
float mod2Pi(float angle);

// Normalize an angular difference in range (-pi, pi]
float rangeSymm(float angle);

// Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc-length s
robotPos circLine(robotPos pos0, float L, float k);

// Check validity of a solution by evaluating explicitly the 3 equations
// defining a Dubins problem (in standard form)
bool check(standardLength sl, float *ks, scParams sc);

//----------------------------------------------------------------
//          DEFINE SCALE FUNCTIONS
//----------------------------------------------------------------
// Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
scParamsWithLambda scaleToStandard(robotPos pos0, robotPos posf, float Kmax);

// Scale the solution to the standard problem back to the original problem
originalLength scaleFromStandard(float lambda, standardLength sl);

//----------------------------------------------------------------
//          DEFINE SIX DUBINS FUNCTIONS
//----------------------------------------------------------------
// LSL function
primitiveResult LSL(scParams sc);

// RSR function
primitiveResult RSR(scParams sc);

// LSR function
primitiveResult LSR(scParams sc);

// RSL function
primitiveResult RSL(scParams sc);

// RLR function
primitiveResult RLR(scParams sc);

// LRL function
primitiveResult LRL(scParams sc);

//----------------------------------------------------------------
//          CREATE DUBINS CURVE
//----------------------------------------------------------------
// Create the arc of a dubins curve
dubinsArc createArc(robotPos pos0, float k, float L);

// Create the dubins curve
dubinsCurve createCurve(robotPos pos0, originalLength ol, float *ks);

//----------------------------------------------------------------
//          FIND SHORTEST DUBINS CURVE
//----------------------------------------------------------------
shortestDubinsResult dubinsPath(robotPos pos0, robotPos posf, float Kmax, vector<vector<pt>> obstacles, bool print=false);
bool checkCollision(dubinsArc a, vector<vector<pt>> obs, float Kmax);
bool checkTwoDubins(dubinsCurve c1, dubinsCurve c2, float Kmax);

//----------------------------------------------------------------
//          GET DUBINS PATH WAYPOINTS
//----------------------------------------------------------------
// Get waypoints of arc
void getArcWaypoints(dubinsArc arc, vector<dubinsWaypoint>& dubinsWPList);

// Get waypoints of curve
vector<dubinsWaypoint> getDubinsWaypoints(dubinsCurve curve);

//----------------------------------------------------------------
//          MULTIPOINTS DUBINS
//----------------------------------------------------------------
vector<shortestDubinsResult> dubinsIDP(vector<robotPos> path_points, vector<vector<pt>> obs, float Kmax);

#endif /* DUBINS_H */
