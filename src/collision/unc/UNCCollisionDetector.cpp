/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/01/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dynamics/Shape.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"

#include "collision/unc/UNCCollisionNode.h"
#include "collision/unc/UNCCollisionDetector.h"

using namespace std;

namespace dart {
namespace collision {

UNCCollisionDetector::UNCCollisionDetector()
    : CollisionDetector(),
      mNumMaxContacts(100) {
}

UNCCollisionDetector::~UNCCollisionDetector() {
}

CollisionNode* UNCCollisionDetector::createCollisionNode(dynamics::BodyNode* _bodyNode) {
    CollisionNode* collisionNode = NULL;

    collisionNode = new UNCCollisionNode(_bodyNode);

    return collisionNode;
}

bool UNCCollisionDetector::checkCollision(bool _checkAllCollisions,
                                          bool _calculateContactPoints) {
    // TODO: _checkAllCollisions

    clearAllContacts();

    fcl::CollisionResult result;

    // only evaluate contact points if data structure for returning the contact
    // points was provided
    fcl::CollisionRequest request;
    request.enable_contact = _calculateContactPoints;
    request.num_max_contacts = mNumMaxContacts;
//    request.enable_cost;
//    request.num_max_cost_sources;
//    request.use_approximate_cost;

    unsigned int numCollisionNodePairs = mCollisionNodePairs.size();
    UNCCollisionNode* collNode1 = NULL;
    UNCCollisionNode* collNode2 = NULL;

    for (unsigned int i = 0; i < numCollisionNodePairs; ++i) {
//        const CollisionNodePair& collisionNodePair = mCollisionNodePairs[i];

//        if (collisionNodePair.collidable == false)
//            continue;

//        collNode1 = dynamic_cast<UNCCollisionNode*>(collisionNodePair.collisionNode1);
//        collNode2 = dynamic_cast<UNCCollisionNode*>(collisionNodePair.collisionNode2);

//        fcl::collide(collNode1->getCollisionGeometry(),
//                     collNode1->getFCLTransform(),
//                     collNode2->getCollisionGeometry(),
//                     collNode2->getFCLTransform(),
//                     request, result);

//        unsigned int numContacts = result.numContacts();
//        for (unsigned int j = 0; j < numContacts; ++j) {
//            const fcl::Contact& contact = result.getContact(j);

//            Contact contactPair;
//            contactPair.point(0) = contact.pos[0];
//            contactPair.point(1) = contact.pos[1];
//            contactPair.point(2) = contact.pos[2];
//            contactPair.normal(0) = contact.normal[0];
//            contactPair.normal(1) = contact.normal[1];
//            contactPair.normal(2) = contact.normal[2];
//            contactPair.collisionNode1 = collisionNodePair.collisionNode1;
//            contactPair.collisionNode2 = collisionNodePair.collisionNode2;
//            //contactPair.bdID1 = collisionNodePair.collisionNode1->getBodyNodeID();
//            //contactPair.bdID2 = collisionNodePair.collisionNode2->getBodyNodeID();
//            contactPair.penetrationDepth = contact.penetration_depth;

//            mContacts.push_back(contactPair);
//        }
    }

    return !mContacts.empty();
}


#define COMPUTECOLLISION_EPS  1E-6

typedef double dVector3[4];
typedef double dMatrix3[12];

struct dContactGeom
{
    dVector3 pos;
    double depth;
};

inline double Inner (const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2]); }

inline double Inner14(const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[4] + (a)[2]*(b)[8]); }

inline double Inner41(const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[4]*(b)[1] + (a)[8]*(b)[2]); }

inline double Inner44(const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[4]*(b)[4] + (a)[8]*(b)[8]); }

#define dMULTIPLYOP0_331(A,op,B,C) \
    (A)[0] op Inner((B),(C)); \
    (A)[1] op Inner((B+4),(C)); \
    (A)[2] op Inner((B+8),(C));

#define dMULTIPLYOP1_331(A,op,B,C) \
    (A)[0] op Inner41((B),(C)); \
    (A)[1] op Inner41((B+1),(C)); \
    (A)[2] op Inner41((B+2),(C));

inline void dMULTIPLY0_331(double *A, const double *B, const double *C)
{ dMULTIPLYOP0_331(A,=,B,C) }

inline void dMULTIPLY1_331(double *A, const double *B, const double *C)
{ dMULTIPLYOP1_331(A,=,B,C) }

#define dRecip(x) (1.0/(x))

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].

void cullPoints (int n, double p[], int m, int i0, int iret[])
{
    // compute the centroid of the polygon in cx,cy
    int i,j;
    double a,cx,cy,q;
    if (n==1) {
        cx = p[0];
        cy = p[1];
    }
    else if (n==2) {
        cx = 0.5*(p[0] + p[2]);
        cy = 0.5*(p[1] + p[3]);
    }
    else {
        a = 0;
        cx = 0;
        cy = 0;
        for (i=0; i<(n-1); i++) {
            q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
            a += q;
            cx += q*(p[i*2]+p[i*2+2]);
            cy += q*(p[i*2+1]+p[i*2+3]);
        }
        q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
        a = dRecip(3.0*(a+q));
        cx = a*(cx + q*(p[n*2-2]+p[0]));
        cy = a*(cy + q*(p[n*2-1]+p[1]));
    }

    // compute the angle of each point w.r.t. the centroid
    double A[8];
    for (i=0; i<n; i++) A[i] = atan2(p[i*2+1]-cy,p[i*2]-cx);

    // search for points that have angles closest to A[i0] + i*(2*pi/m).
    int avail[8];
    for (i=0; i<n; i++) avail[i] = 1;
    avail[i0] = 0;
    iret[0] = i0;
    iret++;
    for (j=1; j<m; j++) {
        a = double(j)*(2*M_PI/m) + A[i0];
        if (a > M_PI) a -= 2*M_PI;
        double maxdiff=1e9,diff;
        for (i=0; i<n; i++) {
            if (avail[i]) {
                diff = fabs (A[i]-a);
                if (diff > M_PI) diff = 2*M_PI - diff;
                if (diff < maxdiff) {
                    maxdiff = diff;
                    *iret = i;
                }
            }
        }
        avail[*iret] = 0;
        iret++;
    }
}

void dLineClosestApproach (const dVector3 pa, const dVector3 ua,
                           const dVector3 pb, const dVector3 ub,
                           double *alpha, double *beta)
{
    dVector3 p;
    p[0] = pb[0] - pa[0];
    p[1] = pb[1] - pa[1];
    p[2] = pb[2] - pa[2];
    double uaub = Inner(ua,ub);
    double q1 =  Inner(ua,p);
    double q2 = -Inner(ub,p);
    double d = 1-uaub*uaub;
    if (d <= 0) {
        // @@@ this needs to be made more robust
        *alpha = 0;
        *beta  = 0;
    }
    else {
        d = dRecip(d);
        *alpha = (q1 + uaub*q2)*d;
        *beta  = (uaub*q1 + q2)*d;
    }
}

int intersectRectQuad (double h[2], double p[8], double ret[16])
{
    // q (and r) contain nq (and nr) coordinate points for the current (and
    // chopped) polygons
    int nq=4,nr=0;
    double buffer[16];
    double *q = p;
    double *r = ret;
    for (int dir=0; dir <= 1; dir++) {
        // direction notation: xy[0] = x axis, xy[1] = y axis
        for (int sign=-1; sign <= 1; sign += 2) {
            // chop q along the line xy[dir] = sign*h[dir]
            double *pq = q;
            double *pr = r;
            nr = 0;
            for (int i=nq; i > 0; i--) {
                // go through all points in q and all lines between adjacent points
                if (sign*pq[dir] < h[dir]) {
                    // this point is inside the chopping line
                    pr[0] = pq[0];
                    pr[1] = pq[1];
                    pr += 2;
                    nr++;
                    if (nr & 8) {
                        q = r;
                        goto done;
                    }
                }
                double *nextq = (i > 1) ? pq+2 : q;
                if ((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir])) {
                    // this line crosses the chopping line
                    pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
                        (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
                    pr[dir] = sign*h[dir];
                    pr += 2;
                    nr++;
                    if (nr & 8) {
                        q = r;
                        goto done;
                    }
                }
                pq += 2;
            }
            q = r;
            r = (q==ret) ? buffer : ret;
            nq = nr;
        }
    }
done:
    if (q != ret) memcpy (ret,q,nr*2*sizeof(double));
    return nr;
}

// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.
int dBoxBox__MARK7(const dVector3 p1, const dMatrix3 R1, const dVector3 side1,
                   const dVector3 p2, const dMatrix3 R2, const dVector3 side2,
                   CollisionInfoArray& result)
{
    const double fudge_factor = 1.05;
    dVector3 p,pp,normalC;
    const double *normalR = 0;
    double A[3],B[3],R11,R12,R13,R21,R22,R23,R31,R32,R33,Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33,s,s2,l;
    int i,j,invert_normal,code;

    // get vector from centers of box 1 to box 2, relative to box 1
    p[0] = p2[0] - p1[0];
    p[1] = p2[1] - p1[1];
    p[2] = p2[2] - p1[2];
    dMULTIPLY1_331 (pp,R1,p);		// get pp = p relative to body 1

    // get side lengths / 2
    A[0] = side1[0];
    A[1] = side1[1];
    A[2] = side1[2];
    B[0] = side2[0];
    B[1] = side2[1];
    B[2] = side2[2];

    // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
    R11 = Inner44(R1+0,R2+0); R12 = Inner44(R1+0,R2+1); R13 = Inner44(R1+0,R2+2);
    R21 = Inner44(R1+1,R2+0); R22 = Inner44(R1+1,R2+1); R23 = Inner44(R1+1,R2+2);
    R31 = Inner44(R1+2,R2+0); R32 = Inner44(R1+2,R2+1); R33 = Inner44(R1+2,R2+2);

    Q11 = fabs(R11); Q12 = fabs(R12); Q13 = fabs(R13);
    Q21 = fabs(R21); Q22 = fabs(R22); Q23 = fabs(R23);
    Q31 = fabs(R31); Q32 = fabs(R32); Q33 = fabs(R33);

    // for all 15 possible separating axes:
    //   * see if the axis separates the boxes. if so, return 0.
    //   * find the depth of the penetration along the separating axis (s2)
    //   * if this is the largest depth so far, record it.
    // the normal vector will be set to the separating axis with the smallest
    // depth. note: normalR is set to point to a column of R1 or R2 if that is
    // the smallest depth normal so far. otherwise normalR is 0 and normalC is
    // set to a vector relative to body 1. invert_normal is 1 if the sign of
    // the normal should be flipped.

#define TST(expr1,expr2,norm,cc) \
    s2 = fabs(expr1) - (expr2); \
    if (s2 > s) { \
    s = s2; \
    normalR = norm; \
    invert_normal = ((expr1) < 0); \
    code = (cc); \
    }

    s = -1E12;
    invert_normal = 0;
    code = 0;

    // separating axis = u1,u2,u3
    TST (pp[0],(A[0] + B[0]*Q11 + B[1]*Q12 + B[2]*Q13),R1+0,1);
    TST (pp[1],(A[1] + B[0]*Q21 + B[1]*Q22 + B[2]*Q23),R1+1,2);
    TST (pp[2],(A[2] + B[0]*Q31 + B[1]*Q32 + B[2]*Q33),R1+2,3);

    // separating axis = v1,v2,v3
    TST (Inner41(R2+0,p),(A[0]*Q11 + A[1]*Q21 + A[2]*Q31 + B[0]),R2+0,4);
    TST (Inner41(R2+1,p),(A[0]*Q12 + A[1]*Q22 + A[2]*Q32 + B[1]),R2+1,5);
    TST (Inner41(R2+2,p),(A[0]*Q13 + A[1]*Q23 + A[2]*Q33 + B[2]),R2+2,6);

    // note: cross product axes need to be scaled when s is computed.
    // normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1,expr2,n1,n2,n3,cc) \
    s2 = fabs(expr1) - (expr2); \
    l = sqrt ((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); \
    if (l > 0) { \
    s2 /= l; \
    if (s2*fudge_factor > s) { \
    s = s2; \
    normalR = 0; \
    normalC[0] = (n1)/l; normalC[1] = (n2)/l; normalC[2] = (n3)/l; \
    invert_normal = ((expr1) < 0); \
    code = (cc); \
    } \
    }

    // separating axis = u1 x (v1,v2,v3)
    TST(pp[2]*R21-pp[1]*R31,(A[1]*Q31+A[2]*Q21+B[1]*Q13+B[2]*Q12),0,-R31,R21,7);
    TST(pp[2]*R22-pp[1]*R32,(A[1]*Q32+A[2]*Q22+B[0]*Q13+B[2]*Q11),0,-R32,R22,8);
    TST(pp[2]*R23-pp[1]*R33,(A[1]*Q33+A[2]*Q23+B[0]*Q12+B[1]*Q11),0,-R33,R23,9);

    // separating axis = u2 x (v1,v2,v3)
    TST(pp[0]*R31-pp[2]*R11,(A[0]*Q31+A[2]*Q11+B[1]*Q23+B[2]*Q22),R31,0,-R11,10);
    TST(pp[0]*R32-pp[2]*R12,(A[0]*Q32+A[2]*Q12+B[0]*Q23+B[2]*Q21),R32,0,-R12,11);
    TST(pp[0]*R33-pp[2]*R13,(A[0]*Q33+A[2]*Q13+B[0]*Q22+B[1]*Q21),R33,0,-R13,12);

    // separating axis = u3 x (v1,v2,v3)
    TST(pp[1]*R11-pp[0]*R21,(A[0]*Q21+A[1]*Q11+B[1]*Q33+B[2]*Q32),-R21,R11,0,13);
    TST(pp[1]*R12-pp[0]*R22,(A[0]*Q22+A[1]*Q12+B[0]*Q33+B[2]*Q31),-R22,R12,0,14);
    TST(pp[1]*R13-pp[0]*R23,(A[0]*Q23+A[1]*Q13+B[0]*Q32+B[1]*Q31),-R23,R13,0,15);

#undef TST

    if (!code) return 0;
    if (s > 0.0) return 0;

    // if we get to this point, the boxes interpenetrate. compute the normal
    // in global coordinates.

    Vec3 normal;
    Vec3 point_vec;
    double penetration;

    if (normalR) {
        normal.setValues(normalR[0],normalR[4],normalR[8]);
    }
    else {
        normal.setValues(Inner((R1),(normalC)), Inner((R1+4),(normalC)), Inner((R1+8),(normalC)));
        //dMULTIPLY0_331 (normal,R1,normalC);
    }
    if (invert_normal) {
        normal *= -1.0;
    }



    // compute contact point(s)

    // single point
    if (code > 6) {
        // an edge from box 1 touches an edge from box 2.
        // find a point pa on the intersecting edge of box 1
        dVector3 pa;
        double sign;
        for (i=0; i<3; i++) pa[i] = p1[i];
        for (j=0; j<3; j++)
        {
#define TEMP_INNER14(a,b) (a[0]*(b)[0] + a[1]*(b)[4] + a[2]*(b)[8])
            sign = (TEMP_INNER14(normal,R1+j) > 0) ? 1.0 : -1.0;

            //sign = (Inner14(normal,R1+j) > 0) ? 1.0 : -1.0;

            for (i=0; i<3; i++) pa[i] += sign * A[j] * R1[i*4+j];
        }

        // find a point pb on the intersecting edge of box 2
        dVector3 pb;
        for (i=0; i<3; i++) pb[i] = p2[i];
        for (j=0; j<3; j++) {
            sign = (TEMP_INNER14(normal,R2+j) > 0) ? -1.0 : 1.0;
#undef TEMP_INNER14
            for (i=0; i<3; i++) pb[i] += sign * B[j] * R2[i*4+j];
        }

        double alpha,beta;
        dVector3 ua,ub;
        for (i=0; i<3; i++) ua[i] = R1[((code)-7)/3 + i*4];
        for (i=0; i<3; i++) ub[i] = R2[((code)-7)%3 + i*4];

        dLineClosestApproach (pa,ua,pb,ub,&alpha,&beta);
        for (i=0; i<3; i++) pa[i] += ua[i]*alpha;
        for (i=0; i<3; i++) pb[i] += ub[i]*beta;


        {
            point_vec.setValues(0.5*(pa[0]+pb[0]), 0.5*(pa[1]+pb[1]), 0.5*(pa[2]+pb[2]));
            penetration = -s;

            CollisionInfo newContact;
            newContact.point = point_vec;
            newContact.normal = normal;
            newContact.penetration = penetration;
            result.push_back(newContact);
        }
        return 1;
    }

    // okay, we have a face-something intersection (because the separating
    // axis is perpendicular to a face). define face 'a' to be the reference
    // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
    // the incident face (the closest face of the other box).

    const double *Ra,*Rb,*pa,*pb,*Sa,*Sb;
    if (code <= 3) {
        Ra = R1;
        Rb = R2;
        pa = p1;
        pb = p2;
        Sa = A;
        Sb = B;
    }
    else {
        Ra = R2;
        Rb = R1;
        pa = p2;
        pb = p1;
        Sa = B;
        Sb = A;
    }

    // nr = normal vector of reference face dotted with axes of incident box.
    // anr = absolute values of nr.
    dVector3 normal2,nr,anr;
    if (code <= 3) {
        normal2[0] = normal[0];
        normal2[1] = normal[1];
        normal2[2] = normal[2];
    }
    else {
        normal2[0] = -normal[0];
        normal2[1] = -normal[1];
        normal2[2] = -normal[2];
    }
    dMULTIPLY1_331 (nr,Rb,normal2);
    anr[0] = fabs (nr[0]);
    anr[1] = fabs (nr[1]);
    anr[2] = fabs (nr[2]);

    // find the largest compontent of anr: this corresponds to the normal
    // for the indident face. the other axis numbers of the indicent face
    // are stored in a1,a2.
    int lanr,a1,a2;
    if (anr[1] > anr[0]) {
        if (anr[1] > anr[2]) {
            a1 = 0;
            lanr = 1;
            a2 = 2;
        }
        else {
            a1 = 0;
            a2 = 1;
            lanr = 2;
        }
    }
    else {
        if (anr[0] > anr[2]) {
            lanr = 0;
            a1 = 1;
            a2 = 2;
        }
        else {
            a1 = 0;
            a2 = 1;
            lanr = 2;
        }
    }

    // compute center point of incident face, in reference-face coordinates
    dVector3 center;
    if (nr[lanr] < 0) {
        for (i=0; i<3; i++) center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i*4+lanr];
    }
    else {
        for (i=0; i<3; i++) center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i*4+lanr];
    }

    // find the normal and non-normal axis numbers of the reference box
    int codeN,code1,code2;
    if (code <= 3) codeN = code-1; else codeN = code-4;
    if (codeN==0) {
        code1 = 1;
        code2 = 2;
    }
    else if (codeN==1) {
        code1 = 0;
        code2 = 2;
    }
    else {
        code1 = 0;
        code2 = 1;
    }

    // find the four corners of the incident face, in reference-face coordinates
    double quad[8];	// 2D coordinate of incident face (x,y pairs)
    double c1,c2,m11,m12,m21,m22;
    c1 = Inner14 (center,Ra+code1);
    c2 = Inner14 (center,Ra+code2);
    // optimize this? - we have already computed this data above, but it is not
    // stored in an easy-to-index format. for now it's quicker just to recompute
    // the four dot products.
    m11 = Inner44 (Ra+code1,Rb+a1);
    m12 = Inner44 (Ra+code1,Rb+a2);
    m21 = Inner44 (Ra+code2,Rb+a1);
    m22 = Inner44 (Ra+code2,Rb+a2);
    {
        double k1 = m11*Sb[a1];
        double k2 = m21*Sb[a1];
        double k3 = m12*Sb[a2];
        double k4 = m22*Sb[a2];
        quad[0] = c1 - k1 - k3;
        quad[1] = c2 - k2 - k4;
        quad[2] = c1 - k1 + k3;
        quad[3] = c2 - k2 + k4;
        quad[4] = c1 + k1 + k3;
        quad[5] = c2 + k2 + k4;
        quad[6] = c1 + k1 - k3;
        quad[7] = c2 + k2 - k4;
    }

    // find the size of the reference face
    double rect[2];
    rect[0] = Sa[code1];
    rect[1] = Sa[code2];

    // intersect the incident and reference faces
    double ret[16];
    int n = intersectRectQuad (rect,quad,ret);
    if (n < 1) return 0;		// this should never happen

    // convert the intersection points into reference-face coordinates,
    // and compute the contact position and depth for each point. only keep
    // those points that have a positive (penetrating) depth. delete points in
    // the 'ret' array as necessary so that 'point' and 'ret' correspond.
    //real point[3*8];		// penetrating contact points
    double point[24];		// penetrating contact points
    double dep[8];			// depths for those points
    double det1 = dRecip(m11*m22 - m12*m21);
    m11 *= det1;
    m12 *= det1;
    m21 *= det1;
    m22 *= det1;
    int cnum = 0;			// number of penetrating contact points found
    for (j=0; j < n; j++) {
        double k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
        double k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
        for (i=0; i<3; i++)
        {
            point[cnum*3+i] = center[i] + k1*Rb[i*4+a1] + k2*Rb[i*4+a2];
        }
        dep[cnum] = Sa[codeN] - Inner(normal2,point+cnum*3);
        if (dep[cnum] >= 0) {
            ret[cnum*2] = ret[j*2];
            ret[cnum*2+1] = ret[j*2+1];
            cnum++;
        }
    }
    if (cnum < 1) return 0;	// this should never happen

    // we can't generate more contacts than we actually have
    int maxc = 4;
    if (maxc > cnum) maxc = cnum;
    //if (maxc < 1) maxc = 1;		//bk~

    if (cnum <= maxc) {
        // we have less contacts than we need, so we use them all
        for (j=0; j < cnum; j++)
        {
            point_vec.setValues(point[j*3+0] + pa[0], point[j*3+1] + pa[1], point[j*3+2] + pa[2]);
            CollisionInfo newContact;
            newContact.point = point_vec;
            newContact.normal = normal;
            newContact.penetration = dep[j];
            result.push_back(newContact);
        }
    }
    else {
        // we have more contacts than are wanted, some of them must be culled.
        // find the deepest point, it is always the first contact.
        int i1 = 0;
        double maxdepth = dep[0];
        for (i=1; i<cnum; i++) {
            if (dep[i] > maxdepth) {
                maxdepth = dep[i];
                i1 = i;
            }
        }

        int iret[8];
        cullPoints (cnum,ret,maxc,i1,iret);

        cnum = maxc;
        for (j=0; j < cnum; j++)
        {
            point_vec.setValues(point[iret[j]*3+0] + pa[0], point[iret[j]*3+1] + pa[1], point[iret[j]*3+2] + pa[2]);
            CollisionInfo newContact;
            newContact.point = point_vec;
            newContact.normal = normal;
            newContact.penetration = dep[iret[j]];
            result.push_back(newContact);
        }
    }
    return cnum;
}

int _BoxBox_____________MARK8(const Vec3& size0, const SE3& T0,
                              const Vec3& size1, const SE3& T1,
                              CollisionInfoArray& result)
{
    Vec3 halfSize0 = 0.5*size0;
    Vec3 halfSize1 = 0.5*size1;

    dMatrix3 R0, R1;

    R0[0] = T0[0];	R0[1] = T0[3];	R0[2] = T0[6];		R0[3] = T0[9];
    R0[4] = T0[1];	R0[5] = T0[4];	R0[6] = T0[7];		R0[7] = T0[10];
    R0[8] = T0[2];	R0[9] = T0[5];	R0[10] = T0[8];		R0[11] = T0[11];

    R1[0] = T1[0];	R1[1] = T1[3];	R1[2] = T1[6];		R1[3] = T1[9];
    R1[4] = T1[1];	R1[5] = T1[4];	R1[6] = T1[7];		R1[7] = T1[10];
    R1[8] = T1[2];	R1[9] = T1[5];	R1[10] = T1[8];		R1[11] = T1[11];


    return dBoxBox__MARK7(&T1[9], R1, &halfSize1[0],
                          &T0[9], R0, &halfSize0[0],
                          result);
}

int	_BoxSphere__________MARK8(const Vec3& size0, const SE3& T0,
                              double r1,    const SE3& T1,
                              CollisionInfoArray& result)
{
    Vec3 halfSize0 = 0.5*size0;

    bool inside_box = true;

    // clipping a center of the sphere to a boundary of the box
    //Vec3 c0(&T0[9]);
    Vec3 c0 = T0.getPosition();

    Vec3 p = T1 % c0;

    if (p[0] < -halfSize0[0]) { p[0] = -halfSize0[0]; inside_box = false; }
    if (p[0] >  halfSize0[0]) { p[0] =  halfSize0[0]; inside_box = false; }

    if (p[1] < -halfSize0[1]) { p[1] = -halfSize0[1]; inside_box = false; }
    if (p[1] >  halfSize0[1]) { p[1] =  halfSize0[1]; inside_box = false; }

    if (p[2] < -halfSize0[2]) { p[2] = -halfSize0[2]; inside_box = false; }
    if (p[2] >  halfSize0[2]) { p[2] =  halfSize0[2]; inside_box = false; }


    Vec3 normal(0.0);
    double penetration;

    if ( inside_box )
    {
        // find nearest side from the sphere center
        double min = halfSize0[0] - fabs(p[0]);
        double tmin = halfSize0[1] - fabs(p[1]);
        int idx = 0;

        if ( tmin < min )
        {
            min = tmin;
            idx = 1;
        }
        tmin = halfSize0[2] - fabs(p[2]);
        if ( tmin < min )
        {
            min = tmin;
            idx = 2;
        }

        //normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
        normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
        normal = Rotate(T1, normal);
        penetration = min + r1;

        CollisionInfo newContact;
        newContact.point = c0;
        newContact.normal = normal;
        newContact.penetration = penetration;
        result.push_back(newContact);

        return 1;
    }


    Vec3 contactpt = T1 * p;
    //normal = c0 - contactpt;
    normal = contactpt - c0;
    double mag = sqrt(SquareSum(normal));
    penetration = r1 - mag;

    if (penetration < 0.0)
    {
        return 0;
    }

    if (mag > COMPUTECOLLISION_EPS)
    {
        normal *= (1.0/mag);

        CollisionInfo newContact;
        newContact.point = contactpt;
        newContact.normal = normal;
        newContact.penetration = penetration;
        result.push_back(newContact);
    }
    else
    {
        double min = halfSize0[0] - fabs(p[0]);
        double tmin = halfSize0[1] - fabs(p[1]);
        int idx = 0;

        if ( tmin < min )
        {
            min = tmin;
            idx = 1;
        }
        tmin = halfSize0[2] - fabs(p[2]);
        if ( tmin < min )
        {
            min = tmin;
            idx = 2;
        }
        normal.setValues(0.0, 0.0, 0.0);
        //normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
        normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
        normal = Rotate(T1, normal);

        CollisionInfo newContact;
        newContact.point = contactpt;
        newContact.normal = normal;
        newContact.penetration = penetration;
        result.push_back(newContact);
    }
    return 1;
}

// a simple root finding algorithm is used to find the value of 't' that
// satisfies:
//		d|D(t)|^2/dt = 0
// where:
//		|D(t)| = |p(t)-b(t)|
// where p(t) is a point on the line parameterized by t:
//		p(t) = p1 + t*(p2-p1)
// and b(t) is that same point clipped to the boundary of the box. in box-
// relative coordinates d|D(t)|^2/dt is the sum of three x,y,z components
// each of which looks like this:
//
//	    t_lo     /
//	      ______/    -->t
//	     /     t_hi
//	    /
//
// t_lo and t_hi are the t values where the line passes through the planes
// corresponding to the sides of the box. the algorithm computes d|D(t)|^2/dt
// in a piecewise fashion from t=0 to t=1, stopping at the point where
// d|D(t)|^2/dt crosses from negative to positive.

void dClosestLineBoxPoints (const dVector3 p1, const dVector3 p2,
                            const dVector3 c, const dMatrix3 R,
                            const dVector3 side,
                            dVector3 lret, dVector3 bret)
{
    int i;

    // compute the start and delta of the line p1-p2 relative to the box.
    // we will do all subsequent computations in this box-relative coordinate
    // system. we have to do a translation and rotation for each point.
    dVector3 tmp,s,v;
    tmp[0] = p1[0] - c[0];
    tmp[1] = p1[1] - c[1];
    tmp[2] = p1[2] - c[2];
    dMULTIPLY1_331 (s,R,tmp);
    tmp[0] = p2[0] - p1[0];
    tmp[1] = p2[1] - p1[1];
    tmp[2] = p2[2] - p1[2];
    dMULTIPLY1_331 (v,R,tmp);

    // mirror the line so that v has all components >= 0
    dVector3 sign;
    for (i=0; i<3; i++) {
        if (v[i] < 0) {
            s[i] = -s[i];
            v[i] = -v[i];
            sign[i] = -1;
        }
        else sign[i] = 1;
    }

    // compute v^2
    dVector3 v2;
    v2[0] = v[0]*v[0];
    v2[1] = v[1]*v[1];
    v2[2] = v[2]*v[2];

    // compute the half-sides of the box
    double h[3];
    h[0] = side[0];
    h[1] = side[1];
    h[2] = side[2];

    // region is -1,0,+1 depending on which side of the box planes each
    // coordinate is on. tanchor is the next t value at which there is a
    // transition, or the last one if there are no more.
    int region[3];
    double tanchor[3];

    // find the region and tanchor values for p1
    for (i=0; i<3; i++) {
        if (v[i] > 0) {
            if (s[i] < -h[i]) {
                region[i] = -1;
                tanchor[i] = (-h[i]-s[i])/v[i];
            }
            else {
                region[i] = (s[i] > h[i]);
                tanchor[i] = (h[i]-s[i])/v[i];
            }
        }
        else {
            region[i] = 0;
            tanchor[i] = 2;		// this will never be a valid tanchor
        }
    }

    // compute d|d|^2/dt for t=0. if it's >= 0 then p1 is the closest point
    double t=0;
    double dd2dt = 0;
    for (i=0; i<3; i++) dd2dt -= (region[i] ? v2[i] : 0) * tanchor[i];
    if (dd2dt >= 0) goto got_answer;

    do {
        // find the point on the line that is at the next clip plane boundary
        double next_t = 1;
        for (i=0; i<3; i++) {
            if (tanchor[i] > t && tanchor[i] < 1 && tanchor[i] < next_t)
                next_t = tanchor[i];
        }

        // compute d|d|^2/dt for the next t
        double next_dd2dt = 0;
        for (i=0; i<3; i++) {
            next_dd2dt += (region[i] ? v2[i] : 0) * (next_t - tanchor[i]);
        }

        // if the sign of d|d|^2/dt has changed, solution = the crossover point
        if (next_dd2dt >= 0) {
            double m = (next_dd2dt-dd2dt)/(next_t - t);
            t -= dd2dt/m;
            goto got_answer;
        }

        // advance to the next anchor point / region
        for (i=0; i<3; i++) {
            if (tanchor[i] == next_t) {
                tanchor[i] = (h[i]-s[i])/v[i];
                region[i]++;
            }
        }
        t = next_t;
        dd2dt = next_dd2dt;
    }
    while (t < 1);
    t = 1;

got_answer:

    // compute closest point on the line
    for (i=0; i<3; i++) lret[i] = p1[i] + t*tmp[i];	// note: tmp=p2-p1

    // compute closest point on the box
    for (i=0; i<3; i++) {
        tmp[i] = sign[i] * (s[i] + t*v[i]);
        if (tmp[i] < -h[i]) tmp[i] = -h[i];
        else if (tmp[i] > h[i]) tmp[i] = h[i];
    }
    dMULTIPLY0_331 (s,R,tmp);
    for (i=0; i<3; i++) bret[i] = s[i] + c[i];
}

//int dColDetBoxSphere__MARK7(double r0, const SE3 &T0,
//                            const Vec3 &size, const SE3 &T1,
//                            ContactConstraint * pConstraint)
//{
//    bool inside_box = true;

//    // clipping a center of the sphere to a boundary of the box
//    //Vec3 c0(&T0[9]);
//    Vec3 c0 = T0.getPosition();
//    Vec3 p = T1 % c0;

//    if (p[0] < -size[0]) { p[0] = -size[0]; inside_box = false; }
//    if (p[0] >  size[0]) { p[0] =  size[0]; inside_box = false; }

//    if (p[1] < -size[1]) { p[1] = -size[1]; inside_box = false; }
//    if (p[1] >  size[1]) { p[1] =  size[1]; inside_box = false; }

//    if (p[2] < -size[2]) { p[2] = -size[2]; inside_box = false; }
//    if (p[2] >  size[2]) { p[2] =  size[2]; inside_box = false; }


//    Vec3 normal(0.0);
//    double penetration;

//    if ( inside_box )
//    {
//        // find nearest side from the sphere center
//        double min = size[0] - fabs(p[0]);
//        double tmin = size[1] - fabs(p[1]);
//        int idx = 0;

//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 1;
//        }
//        tmin = size[2] - fabs(p[2]);
//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 2;
//        }

//        normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
//        normal = Rotate(T1, normal);
//        penetration = min + r0;

//        pConstraint->HowAboutThisPoint(c0, normal, penetration);
//        return 1;
//    }


//    Vec3 contactpt = T1 * p;
//    normal = contactpt - c0;
//    double mag = sqrt(SquareSum(normal));
//    penetration = r0 - mag;

//    if (penetration < 0.0)
//    {
//        return 0;
//    }

//    if (mag > COMPUTECOLLISION_EPS)
//    {
//        normal *= (1.0/mag);
//        pConstraint->HowAboutThisPoint(contactpt,normal,penetration);
//    }
//    else
//    {
//        double min = size[0] - fabs(p[0]);
//        double tmin = size[1] - fabs(p[1]);
//        int idx = 0;

//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 1;
//        }
//        tmin = size[2] - fabs(p[2]);
//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 2;
//        }
//        normal.setValues(0.0, 0.0, 0.0);
//        normal[idx] = (p[idx] > 0.0 ? -1.0 : 1.0);
//        normal = Rotate(T1, normal);

//        pConstraint->HowAboutThisPoint(contactpt,normal,penetration);
//    }
//    return 1;
//}


//int	CollisionPair::_BoxCapsule_________MARK8()
//{
//    // CapsuleBox normal
//    // Capsule
//    double r0 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];	// radius
//    double h0 = 0.5 * pRightCol->getDimensionOfCollisionShape()[1];	// half of height
//    const SE3 & T0 = pRightCol->getFrame();

//    // Box
//    Vec3 size = 0.5 * pLeftCol->getDimensionOfCollisionShape();
//    const SE3 & T1 = pLeftCol->getFrame();



//    // capsule info
//    //Vec3 CapAxis(&T0[6]),Center(&T0[9]);
//    Vec3 CapAxis = T0.getRz();
//    Vec3 Center = T0.getPosition();

//    Vec3 CapBot=Center-h0*CapAxis, CapTop=Center+h0*CapAxis;


//    // copy out box center, rotation matrix, and side array
//    dMatrix3 R;
//    R[0] = T1[0];	R[1] = T1[3];	R[2] = T1[6];	R[3] = T1[9];
//    R[4] = T1[1];	R[5] = T1[4];	R[6] = T1[7];	R[7] = T1[10];
//    R[8] = T1[2];	R[9] = T1[5];	R[10] = T1[8];	R[11] = T1[11];

//    // get the closest point between the cylinder axis and the box
//    Vec3 pl1;
//    Vec3 pb1;
//    dClosestLineBoxPoints(&CapTop[0], &CapBot[0], &T1[9], R, &size[0], &pl1[0], &pb1[0]);

//    Vec3 pl2;
//    Vec3 pb2;
//    dClosestLineBoxPoints(&CapBot[0], &CapTop[0], &T1[9], R, &size[0], &pl2[0], &pb2[0]);


//    //////////////////////////////////////////////////////////////////////////

//    //Vec3 normal(pl1[0] - pb1[0], pl1[1] - pb1[1], pl1[2] - pb1[2]);
//    Vec3 normal(pb1[0] - pl1[0], pb1[1] - pl1[1], pb1[2] - pl1[2]);
//    double normal_sqr = SquareSum(normal);

//    if (normal_sqr > r0 * r0)
//    {
//        return 0;
//    }

//    if (SquareSum(pl1-pl2) > COMPUTECOLLISION_EPS ) //  line contact 2 return
//    {
//        if (normal_sqr < COMPUTECOLLISION_EPS)
//        {
//            SE3 T_Line(pl1);
//            dColDetBoxSphere__MARK7(r0,T_Line,size,T1,pContactConstraint);
//            T_Line.setPosition(pl2);
//            dColDetBoxSphere__MARK7(r0,T_Line,size,T1,pContactConstraint);
//            return 2;
//        }

//        normal_sqr = sqrt(normal_sqr);
//        normal *= (1.0/normal_sqr);
//        double penetration = r0 - normal_sqr;

//        pContactConstraint->HowAboutThisPoint(pb1,normal,penetration);
//        pContactConstraint->HowAboutThisPoint(pb2,normal,penetration);

//        return 2;
//    }
//    else	// 1 return
//    {
//        if (normal_sqr < COMPUTECOLLISION_EPS)
//        {
//            SE3 T_Line(pl1);
//            // ø..
//            return 1;
//        }

//        normal_sqr = sqrt(normal_sqr);
//        normal *= (1.0/normal_sqr);
//        double penetration = r0 - normal_sqr;

//        pContactConstraint->HowAboutThisPoint(pb1,normal,penetration);
//        return 1;
//    }
//}

int _SphereBox__________MARK8(double r0, const SE3& T0,
                              const Vec3& size, const SE3& T1,
                              CollisionInfoArray& result)
{
    Vec3 halfSize = 0.5*size;

    bool inside_box = true;

    // clipping a center of the sphere to a boundary of the box
    Vec3 c0 = T0.getPosition();
    Vec3 p = T1 % c0;

    if (p[0] < -halfSize[0]) { p[0] = -halfSize[0]; inside_box = false; }
    if (p[0] >  halfSize[0]) { p[0] =  halfSize[0]; inside_box = false; }

    if (p[1] < -halfSize[1]) { p[1] = -halfSize[1]; inside_box = false; }
    if (p[1] >  halfSize[1]) { p[1] =  halfSize[1]; inside_box = false; }

    if (p[2] < -halfSize[2]) { p[2] = -halfSize[2]; inside_box = false; }
    if (p[2] >  halfSize[2]) { p[2] =  halfSize[2]; inside_box = false; }


    Vec3 normal(0.0);
    double penetration;

    if ( inside_box )
    {
        // find nearest side from the sphere center
        double min = halfSize[0] - fabs(p[0]);
        double tmin = halfSize[1] - fabs(p[1]);
        int idx = 0;

        if ( tmin < min )
        {
            min = tmin;
            idx = 1;
        }
        tmin = halfSize[2] - fabs(p[2]);
        if ( tmin < min )
        {
            min = tmin;
            idx = 2;
        }

        normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
        normal = Rotate(T1, normal);
        penetration = min + r0;

        CollisionInfo newContact;
        newContact.point = c0;
        newContact.normal = normal;
        newContact.penetration = penetration;
        result.push_back(newContact);

        return 1;
    }


    Vec3 contactpt = T1 * p;
    normal = c0 - contactpt;
    double mag = sqrt(SquareSum(normal));
    penetration = r0 - mag;

    if (penetration < 0.0)
    {
        return 0;
    }

    if (mag > COMPUTECOLLISION_EPS)
    {
        normal *= (1.0/mag);

        CollisionInfo newContact;
        newContact.point = contactpt;
        newContact.normal = normal;
        newContact.penetration = penetration;
        result.push_back(newContact);
    }
    else
    {
        double min = halfSize[0] - fabs(p[0]);
        double tmin = halfSize[1] - fabs(p[1]);
        int idx = 0;

        if ( tmin < min )
        {
            min = tmin;
            idx = 1;
        }
        tmin = halfSize[2] - fabs(p[2]);
        if ( tmin < min )
        {
            min = tmin;
            idx = 2;
        }
        normal.setValues(0.0, 0.0, 0.0);
        normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
        normal = Rotate(T1, normal);

        CollisionInfo newContact;
        newContact.point = contactpt;
        newContact.normal = normal;
        newContact.penetration = penetration;
        result.push_back(newContact);
    }
    return 1;
}

int _SphereSphere_______MARK8(double r0, const SE3& T0,
                              double r1, const SE3& T1,
                              CollisionInfoArray& result)
{
    double rsum = r0 + r1;
    Vec3 normal(T0[9] - T1[9], T0[10] - T1[10], T0[11] - T1[11]);
    double normal_sqr = SquareSum(normal);

    if ( normal_sqr > rsum * rsum )
    {
        return 0;
    }

    r0 /= rsum;
    r1 /= rsum;

    Vec3 point(r1 * T0[9] + r0 * T1[9], r1 * T0[10] + r0 * T1[10], r1 * T0[11] + r0 * T1[11]);
    double penetration;

    if (normal_sqr < COMPUTECOLLISION_EPS)
    {
        normal.setValues(0.0,0.0,1.0);
        penetration = rsum;

        CollisionInfo newContact;
        newContact.point = point;
        newContact.normal = normal;
        newContact.penetration = penetration;
        result.push_back(newContact);

        return 1;
    }

    normal_sqr = sqrt(normal_sqr);
    normal *= (1.0/normal_sqr);
    penetration = rsum - normal_sqr;

    CollisionInfo newContact;
    newContact.point = point;
    newContact.normal = normal;
    newContact.penetration = penetration;
    result.push_back(newContact);

    return 1;

}

//int dSphereSphere__MARK7(double r0, const Vec3 &c0,
//                         double r1, const Vec3 &c1,
//                         ContactConstraint * pContactConstraint)
//{
//    double rsum = r0 + r1;
//    Vec3 normal(c0[0] - c1[0], c0[1] - c1[1], c0[2] - c1[2]);
//    double normal_sqr = SquareSum(normal);

//    if ( normal_sqr > rsum * rsum)
//    {
//        return 0;
//    }

//    r0 /= rsum;
//    r1 /= rsum;


//    Vec3 point(r1 * c0[0] + r0 * c1[0], r1 * c0[1] + r0 * c1[1], r1 * c0[2] + r0 * c1[2]);
//    double penetration;

//    if (normal_sqr < COMPUTECOLLISION_EPS)
//    {
//        normal.setValues(0.0,0.0,1.0);
//        penetration = rsum;
//        pContactConstraint->HowAboutThisPoint(point, normal, penetration);
//        return 1;
//    }

//    normal_sqr = sqrt(normal_sqr);
//    normal *= (1.0/normal_sqr);
//    penetration = rsum - normal_sqr;
//    pContactConstraint->HowAboutThisPoint(point, normal, penetration);
//    return 1;
//}


//int CollisionPair::_SphereCapsule______MARK8()
//{
//    // CapsuleSphere normal

//    // Sphere
//    const SE3 & T1 = pLeftCol->getFrame();
//    double r1 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];	// radius

//    // Capsule
//    const SE3 & T0 = pRightCol->getFrame();
//    double r0 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];	// radius
//    double h0 = 0.5 * pRightCol->getDimensionOfCollisionShape()[1];


//    //Vec3 dir(&T0[6]);
//    //Vec3 c0(&T0[9]);
//    //Vec3 c1(&T1[9]);

//    Vec3 dir = T0.getRz();
//    Vec3 c0 = T0.getPosition();
//    Vec3 c1 = T1.getPosition();

//    double t = Inner(dir, c1 - c0);

//    if ( t > h0 ) t = h0;
//    if ( t < -h0 ) t = -h0;

//    dir *= t;
//    c0 += dir;

//    return dSphereSphere__MARK7(r1, c1, r0, c0, pContactConstraint);
//}

//int dColDetSphereBox__MARK7(double r0, const SE3 &T0,
//                           const Vec3 &size, const SE3 &T1,
//                           ContactConstraint * pConstraint)
//{
//    bool inside_box = true;

//    // clipping a center of the sphere to a boundary of the box
//    //Vec3 c0(&T0[9]);
//    Vec3 c0 = T0.getPosition();
//    Vec3 p = T1 % c0;

//    if (p[0] < -size[0]) { p[0] = -size[0]; inside_box = false; }
//    if (p[0] >  size[0]) { p[0] =  size[0]; inside_box = false; }

//    if (p[1] < -size[1]) { p[1] = -size[1]; inside_box = false; }
//    if (p[1] >  size[1]) { p[1] =  size[1]; inside_box = false; }

//    if (p[2] < -size[2]) { p[2] = -size[2]; inside_box = false; }
//    if (p[2] >  size[2]) { p[2] =  size[2]; inside_box = false; }


//    Vec3 normal(0.0);
//    double penetration;

//    if ( inside_box )
//    {
//        // find nearest side from the sphere center
//        double min = size[0] - fabs(p[0]);
//        double tmin = size[1] - fabs(p[1]);
//        int idx = 0;

//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 1;
//        }
//        tmin = size[2] - fabs(p[2]);
//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 2;
//        }

//        normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
//        normal = Rotate(T1, normal);
//        penetration = min + r0;

//        pConstraint->HowAboutThisPoint(c0, normal, penetration);
//        return 1;
//    }


//    Vec3 contactpt = T1 * p;
//    normal = c0 - contactpt;
//    double mag = sqrt(SquareSum(normal));
//    penetration = r0 - mag;

//    if (penetration < 0.0)
//    {
//        return 0;
//    }

//    if (mag > COMPUTECOLLISION_EPS)
//    {
//        normal *= (1.0/mag);
//        pConstraint->HowAboutThisPoint(contactpt,normal,penetration);
//    }
//    else
//    {
//        double min = size[0] - fabs(p[0]);
//        double tmin = size[1] - fabs(p[1]);
//        int idx = 0;

//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 1;
//        }
//        tmin = size[2] - fabs(p[2]);
//        if ( tmin < min )
//        {
//            min = tmin;
//            idx = 2;
//        }
//        normal.setValues(0.0, 0.0, 0.0);
//        normal[idx] = (p[idx] > 0.0 ? 1.0 : -1.0);
//        normal = Rotate(T1, normal);

//        pConstraint->HowAboutThisPoint(contactpt,normal,penetration);
//    }
//    return 1;
//}

//int CollisionPair::_CapsuleBox_________MARK8()
//{
//    // Capsule
//    double r0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];
//    double h0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[1];
//    const SE3 & T0 = pLeftCol->getFrame();

//    // Box
//    Vec3 size = 0.5 * pRightCol->getDimensionOfCollisionShape();
//    const SE3 & T1 = pRightCol->getFrame();



//    // capsule info
//    //Vec3 CapAxis(&T0[6]),Center(&T0[9]);
//    Vec3 CapAxis = T0.getRz();
//    Vec3 Center = T0.getPosition();
//    Vec3 CapBot=Center-h0*CapAxis, CapTop=Center+h0*CapAxis;


//    // copy out box center, rotation matrix, and side array
//    dMatrix3 R;
//    R[0] = T1[0];	R[1] = T1[3];	R[2] = T1[6];	R[3] = T1[9];
//    R[4] = T1[1];	R[5] = T1[4];	R[6] = T1[7];	R[7] = T1[10];
//    R[8] = T1[2];	R[9] = T1[5];	R[10] = T1[8];	R[11] = T1[11];

//    // get the closest point between the cylinder axis and the box
//    Vec3 pl1;
//    Vec3 pb1;
//    dClosestLineBoxPoints(&CapTop[0], &CapBot[0], &T1[9], R, &size[0], &pl1[0], &pb1[0]);

//    Vec3 pl2;
//    Vec3 pb2;
//    dClosestLineBoxPoints(&CapBot[0], &CapTop[0], &T1[9], R, &size[0], &pl2[0], &pb2[0]);


//    //////////////////////////////////////////////////////////////////////////

//    Vec3 normal(pl1[0] - pb1[0], pl1[1] - pb1[1], pl1[2] - pb1[2]);
//    double normal_sqr = SquareSum(normal);

//    if (normal_sqr > r0 * r0)
//    {
//        return 0;
//    }

//    if (SquareSum(pl1-pl2) > COMPUTECOLLISION_EPS ) //  line contact 2 return
//    {
//        if (normal_sqr < COMPUTECOLLISION_EPS)
//        {
//            SE3 T_Line(pl1);
//            dColDetSphereBox__MARK7(r0,T_Line,size,T1,pContactConstraint);
//            T_Line.setPosition(pl2);
//            dColDetSphereBox__MARK7(r0,T_Line,size,T1,pContactConstraint);
//            return 2;
//        }

//        normal_sqr = sqrt(normal_sqr);
//        normal *= (1.0/normal_sqr);
//        double penetration = r0 - normal_sqr;

//        pContactConstraint->HowAboutThisPoint(pb1,normal,penetration);
//        pContactConstraint->HowAboutThisPoint(pb2,normal,penetration);

//        return 2;
//    }
//    else	// 1 return
//    {
//        if (normal_sqr < COMPUTECOLLISION_EPS)
//        {
//            SE3 T_Line(pl1);
//            // ø..
//            return 1;
//        }

//        normal_sqr = sqrt(normal_sqr);
//        normal *= (1.0/normal_sqr);
//        double penetration = r0 - normal_sqr;

//        pContactConstraint->HowAboutThisPoint(pb1,normal,penetration);
//        return 1;
//    }
//}

//int CollisionPair::_CapsuleSphere______MARK8()
//{
//    // Sphere
//    const SE3 & T1 = pRightCol->getFrame();
//    double r1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];

//    // Capsule
//    const SE3 & T0 = pLeftCol->getFrame();
//    double r0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];
//    double h0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[1];


//    //Vec3 dir(&T0[6]);
//    //Vec3 c0(&T0[9]);
//    //Vec3 c1(&T1[9]);

//    Vec3 dir = T0.getRz();
//    Vec3 c0 = T0.getPosition();
//    Vec3 c1 = T1.getPosition();


//    double t = Inner(dir, c1 - c0);

//    if ( t > h0 ) t = h0;
//    if ( t < -h0 ) t = -h0;

//    dir *= t;
//    c0 += dir;

//    return dSphereSphere__MARK7(r0, c0, r1, c1, pContactConstraint);
//}

//int CollisionPair::_CapsuleCapsule_____MARK8()
//{
//    double r0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];
//    double h0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[1];
//    const SE3	& T0 = pLeftCol->getFrame();

//    double r1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];
//    double h1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[1];
//    const SE3	& T1 = pRightCol->getFrame();




//    //Vec3 d0(&T0[6]);					// direction of line segment 0
//    //Vec3 d1(&T1[6]);					// direction of line segment 1
//    //Vec3 c0(&T0[9]);					// center of line segment 0
//    //Vec3 c1(&T1[9]);					// center of line segment 1

//    Vec3 d0 = T0.getRz();					// direction of line segment 0
//    Vec3 d1 = T1.getRz();					// direction of line segment 1
//    Vec3 c0 = T0.getPosition();					// center of line segment 0
//    Vec3 c1 = T1.getPosition();					// center of line segment 1

//    Vec3 c = c1 - c0;
//    double d0d1 = Inner(d0, d1);
//    double cd0 = Inner(c, d0);
//    double cd1 = Inner(c, d1);
//    double D = 1.0 - d0d1 * d0d1;
//    double t0, t1;

//    // true : point  false : line
//    bool mode = true;
//    Vec3 ptA, ptB;						// contact line segment : from ptA to ptB

////	h0 *= 0.5;							// line segment 0 is c0 + t0 * d0, where t0 \in (-h0, h0).
////	h1 *= 0.5;							// line segment 1 is c1 + t1 * d1, where t1 \in (-h1, h1).

//    if ( D < COMPUTECOLLISION_EPS )				// line segments 0 and 1 are parallel.
//    {
//        double tu = h0 * d0d1 - cd1;	// project c0 + h0 * d0 to line (c1, d1)
//        double tl = -h0 * d0d1 - cd1;	// project c0 - h0 * d0 to line (c1, d1)

//        if ( tu < -h1 )
//        {
//            t0 = h0;
//            t1 = -h1;
//        } else if ( tu < h1 )
//        {
//            mode = false;
//            if ( tl < -h1 )
//            {
//                t1 = 0.5 * (tu - h1);
//                t0 = t1 * d0d1 + cd0;

//                ptA = c1 + tu * d1;
//                ptB = c1 - h1 * d1;
//            } else // line segment (c1, d1, h1) includes (c0, d0, h0)
//            {
//                t0 = 0.0;
//                t1 = -cd1;

//                ptA = c1 + tu * d1;
//                ptB = c1 + tl * d1;
//            }
//        } else // tu > h1
//        {
//            if ( tl > h1 )
//            {
//                t0 = -h0;
//                t1 = h1;
//            } else if ( tl > -h1 )
//            {
//                mode = false;
//                t1 = 0.5 * (h1 + tl);
//                t0 = t1 * d0d1 + cd0;

//                ptA = c1 + h1 * d1;
//                ptB = c1 + tl * d1;
//            } else // line segment (c0, d0, h0) includes (c1, d1, h1)
//            {
//                mode = false;
//                t0 = cd0;
//                t1 = 0.0;

//                ptA = c1 - h1 * d1;
//                ptB = c1 + h1 * d1;
//            }
//        }
//    } else
//    {
//        t0 = (cd0 - d0d1 * cd1) / D;
//        t1 = (d0d1 * cd0 - cd1) / D;

//        if ( t0 > h0 )
//        {
//            if ( t1 > h1 )
//            {
//                t0 = h1 * d0d1 + cd0;
//                if ( h0 > t0 )
//                {
//                    t1 = h1;
//                    if ( t0 < -h0 ) t0 = -h0;
//                } else
//                {
//                    t0 = h0;
//                    t1 = h0 * d0d1 - cd1;
//                    if ( t1 < -h1 ) t1 = -h1;
//                    if ( t1 > h1 ) t1 = h1;
//                }
//            } else if ( t1 < -h1 )
//            {
//                t0 = cd0 - h1 * d0d1;
//                if ( h0 > t0 )
//                {
//                    t1 = -h1;
//                    if ( t0 < -h0 ) t0 = -h0;
//                } else
//                {
//                    t0 = h0;
//                    t1 = h0 * d0d1 - cd1;
//                    if ( t1 < -h1 ) t1 = -h1;
//                    if ( t1 > h1 ) t1 = h1;
//                }
//            } else
//            {
//                t0 = h0;
//                t1 = h0 * d0d1 - cd1;
//                if ( t1 < -h1 ) t1 = -h1;
//                if ( t1 > h1 ) t1 = h1;
//            }
//        } else if ( t0 < -h0 )
//        {
//            if ( t1 > h1 )
//            {
//                t0 = cd0 + h1 * d0d1;
//                if ( -h0 < t0 )
//                {
//                    t1 = h1;
//                    if ( t0 > h0 ) t0 = h0;
//                } else
//                {
//                    t0 = -h0;
//                    t1 = -h0 * d0d1 - cd1;
//                    if ( t1 < -h1 ) t1 = -h1;
//                    if ( t1 > h1 ) t1 = h1;
//                }
//            } else if ( t1 < -h1 )
//            {
//                t0 = cd0 - h1 * d0d1;
//                if ( -h0 < t0 )
//                {
//                    t1 = -h1;
//                    if ( t0 > h0 ) t0 = h0;
//                } else
//                {
//                    t0 = -h0;
//                    t1 = -h0 * d0d1 - cd1;
//                    if ( t1 < -h1 ) t1 = -h1;
//                    if ( t1 > h1 ) t1 = h1;
//                }
//            } else
//            {
//                t0 = -h0;
//                t1 = -h0 * d0d1 - cd1;
//                if ( t1 < -h1 ) t1 = -h1;
//                if ( t1 > h1 ) t1 = h1;
//            }
//        } else
//        {
//            if ( t1 > h1 )
//            {
//                t0 = cd0 + h1 * d0d1;
//                t1 = h1;
//                if ( t0 < -h0 ) t0 = -h0;
//                if ( t0 > h0 ) t0 = h0;
//            } else if ( t1 < -h1 )
//            {
//                t0 = cd0 - h1 * d0d1;
//                t1 = -h1;
//                if ( t0 < -h0 ) t0 = -h0;
//                if ( t0 > h0 ) t0 = h0;
//            }
//        }
//    }

//    d0 *= t0;
//    d0 += c0;
//    d1 *= t1;
//    d1 += c1;

//    if (!mode) // line
//    {
//        double rsum = r0 + r1;
//        Vec3 normal = c0 - c1;
//        double normal_sqr = SquareSum(normal);
//        double penetration;

//        if ( normal_sqr > rsum * rsum)
//        {
//            return 0;
//        }

//        if (normal_sqr < COMPUTECOLLISION_EPS)
//        {
//            normal.setValues(0.0,0.0,1.0);
//            penetration = rsum;
//            c = ptA + r1 * normal;
//            pContactConstraint->HowAboutThisPoint(c, normal, penetration);
//            c = ptB + r1 * normal;
//            pContactConstraint->HowAboutThisPoint(c, normal, penetration);
//            return 2;
//        }

//        normal_sqr = sqrt(normal_sqr);
//        normal *= (1.0/normal_sqr);
//        penetration = rsum - normal_sqr;

//        c = ptA + r1 * normal;
//        pContactConstraint->HowAboutThisPoint(c, normal, penetration);
//        c = ptB + r1 * normal;
//        pContactConstraint->HowAboutThisPoint(c, normal, penetration);
//        return 2;
//    }
//    else // mode = true : point
//    {
//        return dSphereSphere__MARK7(r0, d0, r1, d1, pContactConstraint);
//    }

//}

//int CollisionPair::_PlaneSphere________MARK8()
//{
//    // Plane
//    const SE3 & T0 = pLeftCol->getFrame();

//    // Sphere
//    const SE3 & T1 = pRightCol->getFrame();
//    double r1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];


//    //Vec3 c0(&T0[9]);
//    //Vec3 c1(&T1[9]);


//    Vec3 c0 = T0.getPosition();
//    Vec3 c1 = T1.getPosition();

//    Vec3 pt = c0 - c1;
//    //Vec3 normal(&T0[6]);
//    Vec3 normal = T0.getRz();
//    double penetration = Inner(normal,pt) + r1;
//    if ( penetration < 0.0 )
//    {
//        return 0;
//    }
//    Vec3 point = (c1 - r1 * normal);
//    normal = -normal;
//    pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//    return 1;

//}

//int CollisionPair::_PlaneCapsule_______MARK8()
//{
//    // Plane
//    const SE3 & T0 = pLeftCol->getFrame();

//    // Capsule
//    const SE3 & T1 = pRightCol->getFrame();
//    double r1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];
//    double h1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[1];


//    //Vec3 c0(&T0[9]);
//    //Vec3 c1(&T1[9]);
//    //Vec3 d1(&T1[6]);
//    //Vec3 normal(&T0[6]);

//    Vec3 c0 = T0.getPosition();
//    Vec3 c1 = T1.getPosition();
//    Vec3 d1 = T1.getRz();
//    Vec3 normal = T0.getRz();


//    double sign = (Inner(normal,d1) > 0.0) ? -1.0 : 1.0;

//    Vec3 c = c1 + h1*sign*d1;
//    Vec3 pt = c0 - c;

//    double penetration = Inner(normal, pt) + r1;
//    if ( penetration < 0.0 )
//    {
//        return 0;
//    }

//    Vec3 point = (c - r1 * normal);
//    Vec3 rnormal = -normal;
//    pContactConstraint->HowAboutThisPoint(point,rnormal,penetration);

//    c = c1 - h1*sign*d1;
//    pt = c0 -c;

//    penetration = Inner(normal, pt) + r1;
//    if ( penetration < 0.0 )
//    {
//        return 1;
//    }
//    point = (c - r1 * normal);
//    pContactConstraint->HowAboutThisPoint(point,rnormal,penetration);
//    return 2;
//}

//int CollisionPair::_SpherePlane________MARK8()
//{
//    // Plane
//    const SE3 & T0 = pRightCol->getFrame();

//    // Sphere
//    const SE3 & T1 = pLeftCol->getFrame();
//    double r1 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];


//    //Vec3 c0(&T0[9]);
//    //Vec3 c1(&T1[9]);

//    Vec3 c0 = T0.getPosition();
//    Vec3 c1 = T1.getPosition();

//    Vec3 pt = c0 - c1;
//    //Vec3 normal(&T0[6]);
//    Vec3 normal = T0.getRz();

//    double penetration = Inner(normal,pt) + r1;
//    if ( penetration < 0.0 )
//    {
//        return 0;
//    }
//    Vec3 point = (c1 - r1 * normal);
//    pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//    return 1;

//}

//int CollisionPair::_CapsulePlane_______MARK8()
//{
//    // Plane
//    const SE3 & T0 = pRightCol->getFrame();

//    // Capsule
//    const SE3 & T1 = pLeftCol->getFrame();
//    double r1 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];
//    double h1 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[1];


//    //Vec3 c0(&T0[9]);
//    //Vec3 c1(&T1[9]);
//    //Vec3 d1(&T1[6]);
//    //Vec3 normal(&T0[6]);

//    Vec3 c0 = T0.getPosition();
//    Vec3 c1 = T1.getPosition();
//    Vec3 d1 = T1.getRz();
//    Vec3 normal = T0.getRz();

//    double sign = (Inner(normal,d1) > 0.0) ? -1.0 : 1.0;

//    Vec3 c = c1 + h1*sign*d1;
//    Vec3 pt = c0 - c;

//    double penetration = Inner(normal, pt) + r1;
//    if ( penetration < 0.0 )
//    {
//        return 0;
//    }

//    Vec3 point = (c - r1 * normal);
//    pContactConstraint->HowAboutThisPoint(point,normal,penetration);

//    c = c1 - h1*sign*d1;
//    pt = c0 -c;

//    penetration = Inner(normal, pt) + r1;
//    if ( penetration < 0.0 )
//    {
//        return 1;
//    }
//    point = (c - r1 * normal);
//    pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//    return 2;

//}


////////////////////////////////////////////////////////////////////////////
////   ʿ䰡
//int CollisionPair::_PlaneBox___________MARK8()
//{
//    // Plane
//    const SE3 & T0 = pLeftCol->getFrame();

//    // Box
//    const SE3 & T1 = pRightCol->getFrame();
//    Vec3 size = 0.5 * pRightCol->getDimensionOfCollisionShape();

//    //Vec3 c0(&T0[9]);
//    //Vec3 normal(&T0[6]);

//    //Vec3 c1(&T1[9]);						//box center
//    //Vec3 vX(&T1[0]),vY(&T1[3]),vZ(&T1[6]);	//3-dir


//    Vec3 c0 = T0.getPosition();
//    Vec3 normal = T0.getRz();

//    Vec3 c1 = T1.getPosition();
//    Vec3 vX = T1.getRx();
//    Vec3 vY = T1.getRy();
//    Vec3 vZ = T1.getRz();



//    Vec3 vVertex[8];					//8 pts
//    vVertex[0]=c1+size[0]*vX+size[1]*vY+size[2]*vZ;
//    vVertex[1]=c1-size[0]*vX+size[1]*vY+size[2]*vZ;
//    vVertex[2]=c1+size[0]*vX-size[1]*vY+size[2]*vZ;
//    vVertex[3]=c1-size[0]*vX-size[1]*vY+size[2]*vZ;
//    vVertex[4]=c1+size[0]*vX+size[1]*vY-size[2]*vZ;
//    vVertex[5]=c1-size[0]*vX+size[1]*vY-size[2]*vZ;
//    vVertex[6]=c1+size[0]*vX-size[1]*vY-size[2]*vZ;
//    vVertex[7]=c1-size[0]*vX-size[1]*vY-size[2]*vZ;

//    int i,j,tmp;
//    double dist[8];
//    int idx_dist[8];
//    int nCp = 0;
//    for (i = 0 ; i < 8 ; i++) {
//        dist[i]=Inner(c0-vVertex[i], normal);
//        if ( dist[i] > 0.0 ) {
//            idx_dist[nCp] = i;
//            nCp++;
//        }
//    }
//    if (nCp < 1) return 0;

//    for (i = 0 ; i < nCp-1 ; i++) {
//        for (j = i+1 ; j < nCp ; j++) {
//            if (dist[idx_dist[i]] < dist[idx_dist[j]]) {

//                tmp = idx_dist[i];
//                idx_dist[i] = idx_dist[j];
//                idx_dist[j] = tmp;
//            }
//        }
//    }
//    if (nCp > 4) nCp = 4;
//    normal = -normal;
//    for (i = 0 ; i < nCp ; i++)
//    {
//        pContactConstraint->HowAboutThisPoint(vVertex[idx_dist[i]],normal,dist[idx_dist[i]]);
//    }
//    return nCp;

//}

int _BoxPlane___________MARK8(const SE3& T0,
                              const Vec3& size, const SE3& T1,
                              CollisionInfoArray& result)
{
    // Plane
    //const SE3 & T0 = pRightCol->getFrame();

    // Box
    //const SE3 & T1 = pLeftCol->getFrame();
    Vec3 halfSize = 0.5 * size;

    //Vec3 c0(&T0[9]);
    //Vec3 normal(&T0[6]);

    //Vec3 c1(&T1[9]);						//box center
    //Vec3 vX(&T1[0]),vY(&T1[3]),vZ(&T1[6]);	//3-dir
    //Vec3 vVertex[8];					//8 pts

    Vec3 c0 = T0.getPosition();
    Vec3 normal = T0.getRz();

    Vec3 c1 = T1.getPosition();
    Vec3 vX = T1.getRx();
    Vec3 vY = T1.getRy();
    Vec3 vZ = T1.getRz();
    Vec3 vVertex[8];

    vVertex[0]=c1+halfSize[0]*vX+halfSize[1]*vY+halfSize[2]*vZ;
    vVertex[1]=c1-halfSize[0]*vX+halfSize[1]*vY+halfSize[2]*vZ;
    vVertex[2]=c1+halfSize[0]*vX-halfSize[1]*vY+halfSize[2]*vZ;
    vVertex[3]=c1-halfSize[0]*vX-halfSize[1]*vY+halfSize[2]*vZ;
    vVertex[4]=c1+halfSize[0]*vX+halfSize[1]*vY-halfSize[2]*vZ;
    vVertex[5]=c1-halfSize[0]*vX+halfSize[1]*vY-halfSize[2]*vZ;
    vVertex[6]=c1+halfSize[0]*vX-halfSize[1]*vY-halfSize[2]*vZ;
    vVertex[7]=c1-halfSize[0]*vX-halfSize[1]*vY-halfSize[2]*vZ;

    int i,j,tmp;
    double dist[8];
    int idx_dist[8];
    int nCp = 0;
    for (i = 0 ; i < 8 ; i++) {
        dist[i]=Inner(c0-vVertex[i], normal);
        if ( dist[i] > 0.0 ) {
            idx_dist[nCp] = i;
            nCp++;
        }
    }
    if (nCp < 1) return 0;

    for (i = 0 ; i < nCp-1 ; i++) {
        for (j = i+1 ; j < nCp ; j++) {
            if (dist[idx_dist[i]] < dist[idx_dist[j]]) {

                tmp = idx_dist[i];
                idx_dist[i] = idx_dist[j];
                idx_dist[j] = tmp;
            }
        }
    }
    if (nCp > 4) nCp = 4;
    for (i = 0 ; i < nCp ; i++)
    {
        CollisionInfo newContact;
        newContact.point = vVertex[idx_dist[i]];
        newContact.normal = normal;
        newContact.penetration = dist[idx_dist[i]];
        result.push_back(newContact);
    }
    return nCp;

}

////////////////////////////////////////////////////////////////////////////
////  ̱

//// Cylinder series
//int CollisionPair::_BoxCylinder________MARK8()
//{
//    return 0;
//}

//int CollisionPair::_SphereCylinder_____MARK8()
//{
//    // Cylinder
//    const SE3 & T0 = pRightCol->getFrame();
//    double r0 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];	//radius
//    double h0 = 0.5 * pRightCol->getDimensionOfCollisionShape()[1];	// half of height

//    // Sphere
//    const SE3 & T1 = pLeftCol->getFrame();
//    double r1 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];	// radius

//    //
//    //Vec3 c0(&T0[9]);
//    //Vec3 d0(&T0[6]);
//    //Vec3 c1(&T1[9]);

//    Vec3 c0 = T0.getPosition();
//    Vec3 d0 = T0.getRz();
//    Vec3 c1 = T1.getPosition();


//    double t= Inner(c1-c0,d0);
//    if ( t < 0 )
//    {
//        t = -t;
//        d0 = -d0;
//    }

//    Vec3 p0 = c0 + t*d0;		//closest pt on the cylinder's axis from the sphere
//    Vec3 vec = c1 - p0;
//    //double dist = vec.Normalize();
//    double dist = vec.getNorm();
//    bool bnorm = vec.normalize();

//    Vec3 point;
//    Vec3 normal;
//    double penetration;

//    /// (1)
//    if ( t < h0 )
//    {
//        penetration = (r0+r1)-dist;
//        //if ( penetration > 0 )
//        if ( penetration > 0 && bnorm )
//        {
//            point = p0 + r0*vec;
//            normal = vec;

//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            return 1;
//        }
//        else
//        {
//            return 0;
//        }
//    }
//    else // ( t >= h0 )
//    {
//        /// (2)
//        if ( dist < r0 )
//        {
//            penetration = (h0+r1)-t;
//            //if ( penetration > 0 )
//            if ( penetration > 0 && bnorm )
//            {
//                point = c0 + h0*d0 + dist*vec;
//                normal = d0;

//                pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//                return 1;
//            }
//            else
//            {
//                return 0;
//            }
//        }
//        /// (3)
//        else // ( dist >= r0 )
//        {
//            point = c0 + h0*d0 + r0*vec;
//            normal = c1 - point;

//            //dist = normal.Normalize();
//            dist = normal.getNorm();
//            bnorm = normal.normalize();

//            penetration = r1-dist;
//            //if ( penetration > 0 )
//            if ( penetration > 0 && bnorm )
//            {
//                pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//                return 1;
//            }
//            else
//            {
//                return 0;
//            }
//        }
//    }
//}

//int CollisionPair::_CylinderBox________MARK8()
//{
//    return 0;
//}

//int CollisionPair::_CylinderSphere_____MARK8()
//{
//    // Cylinder
//    const SE3 & T0 = pLeftCol->getFrame();
//    double r0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[0];
//    double h0 = 0.5 * pLeftCol->getDimensionOfCollisionShape()[1];

//    // Sphere
//    const SE3 & T1 = pRightCol->getFrame();
//    double r1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];

//    //
//    //Vec3 c0(&T0[9]);
//    //Vec3 d0(&T0[6]);
//    //Vec3 c1(&T1[9]);

//    Vec3 c0 = T0.getPosition();
//    Vec3 d0 = T0.getRz();
//    Vec3 c1 = T1.getPosition();

//    double t = Inner(c1-c0,d0);
//    if ( t < 0 )
//    {
//        t = -t;
//        d0 = -d0;
//    }

//    Vec3 p0 = c0 + t*d0;		//closest pt on the cylinder's axis from the sphere
//    Vec3 vec = c1 - p0;
//    //double dist = vec.Normalize();
//    double dist = vec.getNorm();
//    bool bnorm = vec.normalize();




//    Vec3 point;
//    Vec3 normal;
//    double penetration;

//    /// (1)
//    if ( t < h0 )
//    {
//        penetration = (r0+r1)-dist;
//        //if ( penetration > 0 )
//        if ( penetration > 0 && bnorm )
//        {
//            point = p0 + r0*vec;
//            normal = -vec;

//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            return 1;
//        }
//        else
//        {
//            return 0;
//        }
//    }
//    else // ( t >= h0 )
//    {
//        /// (2)
//        if ( dist < r0 )
//        {
//            penetration = (h0+r1)-t;
//            //if ( penetration > 0 )
//            if ( penetration > 0 && bnorm )
//            {
//                point = c0 + h0*d0 + dist*vec;
//                normal = -d0;

//                pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//                return 1;
//            }
//            else
//            {
//                return 0;
//            }
//        }
//        /// (3)
//        else // ( dist >= r0 )
//        {
//            point = c0 + h0*d0 + r0*vec;
//            normal = point - c1;

//            //dist = normal.Normalize();
//            dist = normal.getNorm();
//            bnorm = normal.normalize();

//            penetration = r1-dist;
//            //if ( penetration > 0 )
//            if ( penetration > 0 && bnorm )
//            {
//                pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//                return 1;
//            }
//            else
//            {
//                return 0;
//            }
//        }
//    }
//}

//int CollisionPair::_CylinderCapsule____MARK8()
//{
//    return 0;
//}

//int CollisionPair::_CylinderCylinder___MARK8()
//{
//    return 0;
//}

//int CollisionPair::_CapsuleCylinder____MARK8()
//{
//    return 0;
//}

//// XContactConstraint.cpp  ִ Լ
//#define SQRT2_OVER_2	0.7071067811865475244008443621048490
//inline void __TangentPlane(const Vec3& n, Vec3& p, Vec3& q)
//{
//    if (fabs(n[2]) > SQRT2_OVER_2) {
//        // choose p in y-z plane
//        double a = n[1]*n[1] + n[2]*n[2];
//        double k = 1.0/sqrt(a);
//        p.setValues(0,-n[2]*k,n[1]*k);
//        // set q = n x p
//        q.setValues(a*k,-n[0]*p[2],n[0]*p[1]);
//    }
//    else {
//        // choose p in x-y plane
//        double a = n[0]*n[0] + n[1]*n[1];
//        double k = 1.0/sqrt(a);
//        p.setValues(-n[1]*k,n[0]*k,0);
//        // set q = n x p
//        q.setValues(-n[2]*p[1],n[2]*p[0],a*k);
//    }
//}

//int CollisionPair::_PlaneCylinder______MARK8()
//{
//    // Plane
//    const SE3 & T0 = pLeftCol->getFrame();

//    // Cylinder
//    const SE3 & T1 = pRightCol->getFrame();
//    double r1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];
//    double h1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[1];

//    //
//    //Vec3 c0(&T0[9]);
//    //Vec3 d0(&T0[6]);
//    //Vec3 c1(&T1[9]);
//    //Vec3 d1(&T1[6]);

//    Vec3 c0 = T0.getPosition();
//    Vec3 d0 = T0.getRz();
//    Vec3 c1 = T1.getPosition();
//    Vec3 d1 = T1.getRz();


//    const double toleranz = 0.0000001;

//    double s = Inner(d0,d1);
//    double t = s;
//    double sign;

//    Vec3 vec1,vec2,vec3;
//    Vec3 point;
//    Vec3 normal = -d0;
//    double penetration;

//    if(s < 0)
//    {
//        sign = 1.0;
//        s += 1.0; // is ca. 0, if vDir1 and PlaneNormal are parallel
//    }
//    else
//    {
//        sign = -1.0;
//        s -= 1.0; // is ca. 0, if vDir1 and PlaneNormal are parallel
//    }

//    if(s < toleranz && s > (-toleranz))
//    {
//        vec1 = c1 + sign*h1*d1; // bottom

//        penetration = Inner((c0 - vec1), d0);
//        if (penetration < 0.0)
//        {
//            return 0;
//        }

//        __TangentPlane(d1,vec2,vec3);
//        vec2 *= r1;
//        vec3 *= r1;
//        int ret = 0;

//        // 1
//        point = vec1 + vec2;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        // 2
//        point = vec1 + vec3;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        // 3
//        point = vec1 - vec2;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        // 4
//        point = vec1 - vec3;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        return ret;
//    }
//    else
//    {
//        vec1 = t*d1 - d0;
//        t = Norm(vec1);
//        t = r1/t;
//        vec1 *= t;


//        point = c1 + sign*h1*d1 + vec1;
//        penetration = Inner((c0 - point), d0);
//        if ( penetration < 0.0 )
//        {
//            return 0;
//        }

//        pContactConstraint->HowAboutThisPoint(point,normal,penetration);

//        point = c1 - sign*h1*d1 + vec1;
//        penetration = Inner((c0 - point), d0);
//        if ( penetration < 0.0 )
//        {
//            return 1;
//        }

//        pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//        return 2;
//    }
//}

//int CollisionPair::_CylinderPlane______MARK8()
//{
//    // Plane
//    const SE3 & T0 = pLeftCol->getFrame();

//    // Cylinder
//    const SE3 & T1 = pRightCol->getFrame();
//    double r1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[0];
//    double h1 = 0.5 * pRightCol->getDimensionOfCollisionShape()[1];

//    //
//    //Vec3 c0(&T0[9]);
//    //Vec3 d0(&T0[6]);
//    //Vec3 c1(&T1[9]);
//    //Vec3 d1(&T1[6]);

//    Vec3 c0 = T0.getPosition();
//    Vec3 d0 = T0.getRz();
//    Vec3 c1 = T1.getPosition();
//    Vec3 d1 = T1.getRz();

//    const double toleranz = 0.0000001;

//    double s = Inner(d0,d1);
//    double t = s;
//    double sign;

//    Vec3 vec1,vec2,vec3;
//    Vec3 point;
//    Vec3 normal = d0;
//    double penetration;

//    if(s < 0)
//    {
//        sign = 1.0;
//        s += 1.0; // is ca. 0, if vDir1 and PlaneNormal are parallel
//    }
//    else
//    {
//        sign = -1.0;
//        s -= 1.0; // is ca. 0, if vDir1 and PlaneNormal are parallel
//    }

//    if(s < toleranz && s > (-toleranz))
//    {
//        vec1 = c1 + sign*h1*d1; // bottom

//        penetration = Inner((c0 - vec1), d0);
//        if (penetration < 0.0)
//        {
//            return 0;
//        }

//        __TangentPlane(d1,vec2,vec3);
//        vec2 *= r1;
//        vec3 *= r1;
//        int ret = 0;

//        // 1
//        point = vec1 + vec2;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        // 2
//        point = vec1 + vec3;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        // 3
//        point = vec1 - vec2;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        // 4
//        point = vec1 - vec3;
//        penetration = Inner((c0 - point), d0);
//        if (penetration > 0.0)
//        {
//            pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//            ret++;
//        }

//        return ret;
//    }
//    else
//    {
//        vec1 = t*d1 - d0;
//        t = Norm(vec1);
//        t = r1/t;
//        vec1 *= t;


//        point = c1 + sign*h1*d1 + vec1;
//        penetration = Inner((c0 - point), d0);
//        if ( penetration < 0.0 )
//        {
//            return 0;
//        }

//        pContactConstraint->HowAboutThisPoint(point,normal,penetration);

//        point = c1 - sign*h1*d1 + vec1;
//        penetration = Inner((c0 - point), d0);
//        if ( penetration < 0.0 )
//        {
//            return 1;
//        }

//        pContactConstraint->HowAboutThisPoint(point,normal,penetration);
//        return 2;
//    }
//}

////////////////////////////////////////////////////////////////////////////
//// Rough Check
//bool	CollisionPair::_PlaneOthers________RoughCheck_TEMP()
//{
//    SE3 tmp_left_to_right = pLeftCol->getFrame() % pRightCol->getFrame();
//    if (pRightCol->getBoundingRadius() >= tmp_left_to_right[11])
//    {
//        return true;
//    }
//    return false;
//}

//bool	CollisionPair::_OthersPlane________RoughCheck_TEMP()
//{
//    SE3 tmp_right_to_left = pRightCol->getFrame() % pLeftCol->getFrame();
//    if (pLeftCol->getBoundingRadius() >= tmp_right_to_left[11])
//    {
//        return true;
//    }
//    return false;
//}

//bool	CollisionPair::_OthersOthers_______RoughCheck_TEMP()
//{
//    double rsum = pLeftCol->getBoundingRadius() + pRightCol->getBoundingRadius();
//    if ( fabs((pLeftCol->getFrame())[11] - (pRightCol->getFrame())[11]) > rsum ) return false;	// z-axis
//    if ( fabs((pLeftCol->getFrame())[9] - (pRightCol->getFrame())[9]) > rsum )	 return false;	// x-axis
//    if ( fabs((pLeftCol->getFrame())[10] - (pRightCol->getFrame())[10]) > rsum ) return false;	// y-axis
//    return true;
//}

int collide(dynamics::Shape* shape0, const SE3& T0,
            dynamics::Shape* shape1, const SE3& T1,
            CollisionInfoArray& result, int n)
{
//    srGeometryInfo::SHAPETYPE LeftType = pLeftCol->getCollisionShape();
//    srGeometryInfo::SHAPETYPE RightType = pRightCol->getCollisionShape();

//    switch(LeftType)
//    {
//    case srGeometryInfo::BOX:

//        switch(RightType)
//        {
//        case srGeometryInfo::BOX:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxBox_____________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::SPHERE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxSphere__________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CYLINDER:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxCylinder________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CAPSULE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxCapsule_________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::PLANE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_BoxPlane___________MARK8;
//            return true;

//            break;
//        default:
//            return false;

//            break;
//        }

//        break;
//    case srGeometryInfo::SPHERE:

//        switch(RightType)
//        {
//        case srGeometryInfo::BOX:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereBox__________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::SPHERE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereSphere_______MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CYLINDER:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereCylinder_____MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CAPSULE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SphereCapsule______MARK8;
//            return true;

//            break;
//        case srGeometryInfo::PLANE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_SpherePlane________MARK8;
//            return true;

//            break;
//        default:
//            return false;

//            break;
//        }

//        break;
//    case srGeometryInfo::CYLINDER:

//        switch(RightType)
//        {
//        case srGeometryInfo::BOX:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderBox________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::SPHERE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderSphere_____MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CYLINDER:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderCylinder___MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CAPSULE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderCapsule____MARK8;
//            return true;

//            break;
//        case srGeometryInfo::PLANE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CylinderPlane______MARK8;
//            return true;

//            break;
//        default:
//            return false;

//            break;
//        }

//        break;
//    case srGeometryInfo::CAPSULE:

//        switch(RightType)
//        {
//        case srGeometryInfo::BOX:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleBox_________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::SPHERE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleSphere______MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CYLINDER:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleCylinder____MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CAPSULE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsuleCapsule_____MARK8;
//            return true;

//            break;
//        case srGeometryInfo::PLANE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_CapsulePlane_______MARK8;
//            return true;

//            break;
//        default:
//            return false;

//            break;
//        }

//        break;

//    case srGeometryInfo::PLANE:

//        switch(RightType)
//        {
//        case srGeometryInfo::BOX:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneBox___________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::SPHERE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneSphere________MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CYLINDER:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneCylinder______MARK8;
//            return true;

//            break;
//        case srGeometryInfo::CAPSULE:
//            m_pfn_NarrowPhase_Algoritm = &CollisionPair::_PlaneCapsule_______MARK8;
//            return true;

//            break;
//        case srGeometryInfo::PLANE:
//            return false;


//            break;
//        default:
//            return false;

//            break;
//        }

//        break;

//    default:
//        return false;

//        break;
//    }
//	return false;
}


} // namespace collision
} // namespace dart
