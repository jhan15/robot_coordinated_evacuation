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

#ifndef DUBINS_H
#define DUBINS_H


//----------------------------------------------------------------
//          DEFINE SOME STRUCTS THAT WILL BE USED
//----------------------------------------------------------------

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
};

struct shortestDubinsResult
{
    int pidx = -1;
    dubinsCurve curve;
};

#endif /* DUBINS_H */
