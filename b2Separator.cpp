//
//  b2Separator.cpp
//  Thermite
//
//  Created by Jarad Delorenzo on 1/7/13.
//
//

#include "b2Separator.h"

#include <Box2D/Box2D.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <exception>

#define EPSILON 0.00001

void err()
{
    throw std::runtime_error(
                             "Unable to seperate into convex parts. Use Validate() to find the "
                             "purpose.");
}

void b2Separator::Separate(b2Body* pBody, b2FixtureDef* pFixtureDef,
                           const std::vector<b2Vec2>& pVerticesVec)
{
    std::vector<std::vector<b2Vec2> > figsVec;
    b2PolygonShape polyShape;
    
    calcShapes(pVerticesVec, figsVec);
    
    for (int i = 0; i < figsVec.size(); i++) {
        std::vector<b2Vec2>& vec = figsVec[i];
        polyShape.Set(&vec[0], (int)vec.size());
        pFixtureDef->shape = &polyShape;
        pBody->CreateFixture(pFixtureDef);
    }
}

/**
 * Checks whether the vertices in can be properly distributed into the new
 * fixtures (more specifically, it makes sure there are no overlapping segments
 * and the vertices are in clockwise order).
 * It is recommended that you use this method for debugging only, because it may
 * cost more CPU usage.
 * @param verticesVec The vertices to be validated.
 * @return An integer which can have the following values:
 * 0 if the vertices can be properly processed.
 * 1 If there are overlapping lines.
 * 2 if the points are <b>not</b> in clockwise order.
 * 3 if there are overlapping lines and the points are not in clockwise order.
 * */
int b2Separator::Validate(const std::vector<b2Vec2>& verticesVec)
{
    int i, n = (int)verticesVec.size(), ret = 0;
    float j, j2, i2, i3, d;
    bool fl, fl2 = false;
    
    for (i = 0; i < n; i++) {
        i2 = (i < n - 1) ? i + 1 : 0;
        i3 = (i > 0) ? i - 1 : n - 1;
        
        fl = false;
        for (j = 0; j < n; j++) {
            if ((j != i) && (j != i2)) {
                if (!fl) {
                    d = det(verticesVec[i].x, verticesVec[i].y, verticesVec[i2].x,
                            verticesVec[i2].y, verticesVec[j].x, verticesVec[j].y);
                    if ((d > 0)) {
                        fl = true;
                    }
                }
                
                if ((j != i3)) {
                    j2 = (j < n - 1) ? j + 1 : 0;
                    b2Vec2 tmp;
                    if (hitSegment(tmp, verticesVec[i].x, verticesVec[i].y,
                                   verticesVec[i2].x, verticesVec[i2].y, verticesVec[j].x,
                                   verticesVec[j].y, verticesVec[j2].x,
                                   verticesVec[j2].y)) {
                        ret = 1; // TODO: This may be wrong!!!
                    }
                }
            }
        }
        
        if (!fl) {
            fl2 = true;
        }
    }
    
    if (fl2) {
        if (ret == 1) {
            ret = 3;
        }
        else {
            ret = 2;
        }
    }
    return ret;
}

void b2Separator::calcShapes(const std::vector<b2Vec2>& pVerticesVec,
                             std::vector<std::vector<b2Vec2> >& result)
{
    int i1, i2, i3;
    int j1, j2;
    int k = 0, h = 0;
    b2Vec2 hitV(0, 0);
    
    std::vector<b2Vec2> vec1, vec2;
    std::queue<std::vector<b2Vec2> > queue;
    
    queue.push(pVerticesVec);
    
    while (!queue.empty()) {
        std::vector<b2Vec2>& vec = queue.front();
        bool isConvex = true;
        
        int n = (int)vec.size();
        for (int i = 0; i < n; i++) {
            i1 = i;
            i2 = (i < n - 1) ? i + 1 : i + 1 - n;
            i3 = (i < n - 2) ? i + 2 : i + 2 - n;
            
            b2Vec2 &p1 = vec[i1];
            b2Vec2 &p2 = vec[i2];
            b2Vec2 &p3 = vec[i3];
            
            if (det(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y) < 0) {
                isConvex = false;
                float minLen = HUGE_VALF;
                
                for (int j = 0; j < n; j++) {
                    if ((j != i1) && (j != i2)) {
                        j1 = j;
                        j2 = (j < n - 1) ? j + 1 : 0;
                        
                        b2Vec2 v1 = vec[j1];
                        b2Vec2 v2 = vec[j2];
                        
                        b2Vec2 v;
                        bool success = hitRay(v, p1.x, p1.y, p2.x, p2.y, v1.x, v1.y, v2.x, v2.y);
                        
                        if (success) {
                            float t = (p2 - v).LengthSquared();
                            
                            if (t < minLen) {
                                h = j1;
                                k = j2;
                                hitV = v;
                                minLen = t;
                            }
                        }
                    }
                }
                
                if (minLen == HUGE_VALF) {
                    err();
                }
                
                vec1.clear();
                vec2.clear();
                
                j1 = h;
                j2 = k;
                const b2Vec2& v1 = vec[j1];
                const b2Vec2& v2 = vec[j2];
                
                if (!pointsMatch(hitV.x, hitV.y, v2.x, v2.y)) {
                    vec1.push_back(hitV);
                }
                if (!pointsMatch(hitV.x, hitV.y, v1.x, v1.y)) {
                    vec2.push_back(hitV);
                }
                
                h = -1;
                k = i1;
                while (true) {
                    if ((k != j2)) {
                        vec1.push_back(vec[k]);
                    }
                    else {
                        if (((h < 0) || h >= n)) {
                            err();
                        }
                        if (!isOnSegment(v2.x, v2.y, vec[h].x, vec[h].y, p1.x, p1.y)) {
                            vec1.push_back(vec[k]);
                        }
                        break;
                    }
                    
                    h = k;
                    if (((k - 1) < 0)) {
                        k = n - 1;
                    }
                    else {
                        k--;
                    }
                }
                
                std::reverse(vec1.begin(), vec1.end());
                
                h = -1;
                k = i2;
                while (true) {
                    if ((k != j1)) {
                        vec2.push_back(vec[k]);
                    }
                    else {
                        if (((h < 0) || h >= n)) {
                            // TODO: Throw Error !!!
                            err();
                        }
                        if (((k == j1) && !isOnSegment(v1.x, v1.y, vec[h].x, vec[h].y, p2.x, p2.y))) {
                            vec2.push_back(vec[k]);
                        }
                        break;
                    }
                    
                    h = k;
                    if (((k + 1) > n - 1)) {
                        k = 0;
                    }
                    else {
                        k++;
                    }
                }
                
                queue.push(vec1);
                queue.push(vec2);
                queue.pop();
                
                break;
            }
        }
        
        if (isConvex) {
            result.push_back(queue.front());
            queue.pop();
        }
    }
}

bool b2Separator::hitRay(b2Vec2& out, float x1, float y1, float x2, float y2,
                         float x3, float y3, float x4, float y4)
{
    float t1 = x3 - x1;
    float t2 = y3 - y1;
    float t3 = x2 - x1;
    float t4 = y2 - y1;
    float t5 = x4 - x3;
    float t6 = y4 - y3;
    float t7 = t4 * t5 - t3 * t6;
    
    // DBZ Error. Undefined hit segment.
    if (t7 == 0)
        return false;
    
    float a = (((t5 * t2) - t6 * t1) / t7);
    float px = x1 + a * t3;
    float py = y1 + a * t4;
    bool b1 = isOnSegment(x2, y2, x1, y1, px, py);
    bool b2 = isOnSegment(px, py, x3, y3, x4, y4);
    
    if (b1 && b2) {
        out.Set(px, py);
        return true;
    }
    return false;
}

bool b2Separator::hitSegment(b2Vec2& out, float x1, float y1, float x2,
                             float y2, float x3, float y3, float x4, float y4)
{
    float t1 = x3 - x1;
    float t2 = y3 - y1;
    float t3 = x2 - x1;
    float t4 = y2 - y1;
    float t5 = x4 - x3;
    float t6 = y4 - y3;
    float t7 = t4 * t5 - t3 * t6;
    
    // DBZ Error. Undefined hit segment.
    if (t7 == 0)
        return false;
    
    float a = (((t5 * t2) - t6 * t1) / t7);
    float px = x1 + a * t3;
    float py = y1 + a * t4;
    bool b1 = isOnSegment(px, py, x1, y1, x2, y2);
    bool b2 = isOnSegment(px, py, x3, y3, x4, y4);
    
    if (b1 && b2) {
        out.Set(px, py);
        return true;
    }
    return false;
}

bool b2Separator::isOnSegment(float px, float py, float x1, float y1, float x2,
                              float y2)
{
    bool b1 = ((x1 + EPSILON >= px && px >= x2 - EPSILON) || (x1 - EPSILON <= px && px <= x2 + EPSILON));
    bool b2 = ((y1 + EPSILON >= py && py >= y2 - EPSILON) || (y1 - EPSILON <= py && py <= y2 + EPSILON));
    return (b1 && b2 && isOnLine(px, py, x1, y1, x2, y2));
}

bool b2Separator::pointsMatch(float x1, float y1, float x2, float y2)
{
    float dx = (x2 >= x1) ? x2 - x1 : x1 - x2;
    float dy = (y2 >= y1) ? y2 - y1 : y1 - y2;
    return ((dx < EPSILON) && dy < EPSILON);
}

bool b2Separator::isOnLine(float px, float py, float x1, float y1, float x2,
                           float y2)
{
    if (x2 - x1 > EPSILON || x1 - x2 > EPSILON) {
        float a = (y2 - y1) / (x2 - x1);
        float possibleY = a * (px - x1) + y1;
        float diff = (possibleY > py ? possibleY - py : py - possibleY);
        return (diff < EPSILON);
    }
    return (px - x1 < EPSILON || x1 - px < EPSILON);
}

float b2Separator::det(float x1, float y1, float x2, float y2, float x3,
                       float y3)
{
    return x1 * y2 + x2 * y3 + x3 * y1 - y1 * x2 - y2 * x3 - y3 * x1;
}