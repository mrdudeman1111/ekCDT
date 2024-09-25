
/*
MIT License

Copyright (c) 2024 C. E. Wilbur

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <vector>
#include <cstdint>
#include <algorithm>
#include <cmath>

namespace ekCdt
{
  //! Two component vector containing x/s and y/t.
  struct vec2
  {
  public:
    vec2() {}
    vec2(float x, float y) : x{x}, y{y} {}

    bool operator ==(vec2& O)
    {
      if(O.x == x && O.y == y) return true;
      else return false;
    }

    vec2 operator -(vec2& O)
    {
      vec2 Ret;
      Ret.x = x - O.x;
      Ret.y = y - O.y;
      return Ret;
    }
    vec2 operator +(vec2& O)
    {
      vec2 Ret;
      Ret.x = x + O.x;
      Ret.y = y + O.y;
      return Ret;
    }
    vec2 operator *(float O)
    {
      vec2 Ret;
      Ret.x = x * O;
      Ret.y = y * O;
      return Ret;
    }
    vec2 operator *(double O)
    {
      vec2 Ret;
      Ret.x = x * O;
      Ret.y = y * O;
      return Ret;
    }

    union { float x; float s; };
    union { float y; float t; };
  };

  //! A structure that stores a vector with an index.
  struct Point
  {
    Point() {}
    Point(vec2 P, uint32_t I) : Pos{P}, Index{I} {}

    vec2 Pos;
    uint32_t Index;

    bool operator ==(const Point& O)
    {
      if(O.Index == Index)
      {
        return true;
      }

      return false;
    }
  };

  //! A structure storing two points that form an edge.
  class Edge
  {
    public:
      Edge(Point A, Point B) : Origin(A), EndPoint(B)
      {}

      Edge(vec2 A, vec2 B)
      {
        Origin.Pos = A;
        Origin.Index = 0;
        EndPoint.Pos = B;
        EndPoint.Index = 1;
      }
      
      bool operator ==(const Edge& O)
      {
        if(Origin == O.Origin && EndPoint == O.EndPoint)
        {
          return true;
        }
        else if(Origin == O.EndPoint && EndPoint == O.Origin)
        {
          return true;
        }

        return false;
      }

      /*! \brief Function used for reversing the Edge, (Swapping the origin and endpoint). */
      void Reverse()
      {
        Point tmp = Origin;
        Origin = EndPoint;
        EndPoint = tmp;
      }

      Point Origin; //!< Point One/Edge Origin
      Point EndPoint; //!< Point Two/Edge Endpoint
  };

  class Triangle
  {
    private:
      /*! \brief Finds this triangle's CircumCircle (circumcircle center, circumcircle radius, and the centroid of the triangle) */
      void CalcCircumCircle();

      //!< the center of this triangle's circumcircle
      vec2 CircumCenter;

      //!< the radius of this triangle's circumcircle
      float Radius;

    public:
      /*! \brief Forms a triangle with Points A, B, and C and finds it's circumcircle */ 
      Triangle(Point A, Point B, Point C);

      /*! \brief Determines whether or not Point P is inside this triangle's circumcircle. */
      bool ContainsPoint(Point P);

      /*! \brief Determines whether or not Edge E is part of this triangle. */
      bool ContainsEdge(Edge E);

      inline const bool operator ==(const Triangle& O)
      {
        /* check if triangle O has Points[0], if so then remove the matching point, then repeat for Points[1] and Points[2] */

        std::vector<Point> op = {O.Points[0], O.Points[1], O.Points[2]};

        if(Points[0] == op[0]) op.erase(op.begin());
        else if(Points[0] == op[1]) op.erase(op.begin()+1);
        else if(Points[0] == op[2]) op.erase(op.begin()+2);
        else return false;

        if(Points[1] == op[0]) op.erase(op.begin());
        else if(Points[1] == op[1]) op.erase(op.begin()+1);
        else return false;

        if(Points[2] == op[0]) return true;
        else return false;
      }

      Point Points[3]; //!< Point array with size of 3. Stores the Points A, B and C provided through the constructor.

      vec2 Centroid; //!< The centroid of the triangle, used for determining whether or not the triangle is inside the bounds of the polygon constraints.
  };

  /*! \brief Forms a triangulation from a set of points within the bounds of the given polygon.
  *
  * @param PositionCount The number of positions (Vertices) in the positions array.
  * @param pPositions A pointer to the array of vertices.
  * @param Polygon A vector of edges (in the form of Index pairs) representing the boundaries and holes of the triangulation.
  * @return List of indices that make up the triangles generated (in a triangle list format).
  *
  *  Takes a set of points along with a set of index pairs that form one or more polygons that represent boundaries or holes, and then triangulates the points with respect to that polygon.
  */
  std::vector<uint32_t> Triangulate(uint32_t PositionCount, vec2* pPositions, std::vector<std::pair<uint32_t, uint32_t>>& Polygon);
}

#ifdef CDT_IMPL

namespace ekCdt
{
  namespace Tools
  {
    inline float Sq(float x)
    {
      return x*x;
    }

    bool PointInsideTriangle(Triangle& Tri, vec2 Point)
    {
      double asX = Point.x - Tri.Points[0].Pos.x;
      double asY = Point.y - Tri.Points[0].Pos.y;

      bool saB = (Tri.Points[1].Pos.x - Tri.Points[0].Pos.x) * asY - (Tri.Points[1].Pos.y - Tri.Points[0].Pos.y) * asX > 0.0;

      if((Tri.Points[2].Pos.x - Tri.Points[0].Pos.x) * asY - (Tri.Points[2].Pos.y - Tri.Points[0].Pos.y) * asX > 0.0 == saB)
      {
        return false;
      }
      if((Tri.Points[2].Pos.x - Tri.Points[1].Pos.x) * (Point.y - Tri.Points[1].Pos.y) - (Tri.Points[2].Pos.y - Tri.Points[1].Pos.y) * (Point.x - Tri.Points[1].Pos.x) > 0.0 != saB)
      {
        return false;
      }

      return true;
    }

    int8_t pipCheck(std::vector<Edge>& Polygon, vec2 Point)
    {
      uint32_t HitCount = 0;

      for(uint32_t i = 0; i < Polygon.size(); i++)
      {
        if(Point == Polygon[i].Origin.Pos || Point == Polygon[i].EndPoint.Pos)
        {
          return 0; // on polygon
        }

        if(Point.y <= std::max(Polygon[i].Origin.Pos.y, Polygon[i].EndPoint.Pos.y) && Point.y >= std::min(Polygon[i].Origin.Pos.y, Polygon[i].EndPoint.Pos.y))
        {
          float C = (Polygon[i].Origin.Pos.x - Point.x) * (Polygon[i].EndPoint.Pos.y - Point.y) - (Polygon[i].EndPoint.Pos.x - Point.x) * (Polygon[i].Origin.Pos.y - Point.y);

          if(C == 0)
          {
            return 0; // on polygon
          }

          if((Polygon[i].Origin.Pos.y < Polygon[i].EndPoint.Pos.y) == (C > 0))
          {
            HitCount++;
          }
        }
      }

      if((HitCount%2) == 0)
      {
        return -1; // outside
      }
      else
      {
        return 1; // inside
      }
    }

    int8_t pipCheck(std::vector<vec2>& Polygon, vec2 Point)
    {
      uint32_t HitCount = 0;

      for(uint32_t i = 0; i < Polygon.size(); i++)
      {
        Edge E(Polygon[i], Polygon[(i+1)%Polygon.size()]);

        if(Point == E.Origin.Pos || Point == E.EndPoint.Pos)
        {
          return 0; // on polygon
        }

        if(Point.y <= std::max(E.Origin.Pos.y, E.EndPoint.Pos.y) && Point.y >= std::min(E.Origin.Pos.y, E.EndPoint.Pos.y))
        {
          float C = (E.Origin.Pos.x - Point.x) * (E.EndPoint.Pos.y - Point.y) - (E.EndPoint.Pos.x - Point.x) * (E.Origin.Pos.y - Point.y);

          if(C == 0)
          {
            return 0; // on polygon
          }

          if((E.Origin.Pos.y < E.EndPoint.Pos.y) == (C > 0))
          {
            HitCount++;
          }
        }
      }

      if((HitCount%2) == 0)
      {
        return -1; // outside
      }
      else
      {
        return 1; // inside
      }
    }

    std::vector<Edge> CreatePolygonHull(std::vector<Triangle> Triangles)
    {
      std::vector<Edge> Ret;

      uint32_t pIter = 0;

      for(uint32_t i = 0; i < Triangles.size(); i++)
      {
        for(uint32_t x = 0; x < 3; x++)
        {
          Edge MainEdge(Triangles[i].Points[x], Triangles[i].Points[(x+1)%3]);

          bool bShared = false;

          for(uint32_t t = 0; t < Triangles.size(); t++)
          {
            if(t == i)
            {
              continue;
            }

            if(Triangles[t].ContainsEdge(MainEdge))
            {
              bShared = true;
              break;
            }
          }

          if(!bShared)
          {
            Ret.push_back(MainEdge);
          }
        }
      }

      return Ret;
    }

    void SortPolygon(std::vector<Edge>& Polygon)
    {
      for(uint32_t i = 0; i < Polygon.size(); i++)
      {
        if(Polygon[i].EndPoint == Polygon[(i+1)%Polygon.size()].Origin)
        {
          continue;
        }
        else
        {
          for(uint32_t x = 0; x < Polygon.size(); x++)
          {
            if(x == i)
            {
              continue;
            }

            if(Polygon[x].Origin == Polygon[i].EndPoint)
            {
              Edge tmp = Polygon[x];
              Polygon.erase(Polygon.begin()+x);

              if(x > i)
              {
                Polygon.insert(Polygon.begin()+(i+1), tmp);
              }
              else
              {
                Polygon.insert(Polygon.begin()+i, tmp);
              }
            }
          }
        }
      }

      return;
    }

    int8_t bPolyCCW(std::vector<Edge>& Polygon)
    {
      float Area = 0.f;

      for(uint32_t i = 0; i < Polygon.size(); i++)
      {
        Area += (Polygon[i].EndPoint.Pos.x - Polygon[i].Origin.Pos.x) * (Polygon[i].EndPoint.Pos.y + Polygon[i].Origin.Pos.y);
      }

      if(Area == 0.f)
      {
        return 2; // colinear
      }

      if(Area < 0.f) return 1; // counter clockwise
      else return 0; // clockwise
    }

    void EnsureCCW(std::vector<Edge>& Polygon)
    {
      if(bPolyCCW(Polygon) == 0)
      {
        std::reverse(Polygon.begin(), Polygon.end());

        // if we're going to reverse the sorted edge array, we need to reverse each edge as well.
        for(Edge& E : Polygon)
        {
          E.Reverse();
        }

        return;
      }

      return;
    }

    bool OnSeg(vec2 A, vec2 B, vec2 C)
    {
      if(((B.x <= std::max(A.x, C.x) &&  (B.x >= std::min(A.x, C.x)) && (B.y <= std::max(A.y, C.y)) && (B.y >= std::min(A.y, C.y)))))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    uint8_t Orientation(vec2 A, vec2 B, vec2 C)
    {
      float Val = (B.y-A.y);
      Val *= (C.x-B.x);
      Val -= ((B.x-A.x) * (C.y-B.y));

      if(Val > 0.f)
      {
        return 1; // clockwise
      }
      else if(Val < 0.f)
      {
        return 2; // counter clockwise
      }
      else
      {
        return 0; // colinear
      }
    }
    
    bool bCCW(Triangle& Tri)
    {
      return Orientation(Tri.Points[0].Pos, Tri.Points[1].Pos, Tri.Points[2].Pos) == 2;
    }

    bool EdgesIntersect(Edge FirstEdge, Edge SecondEdge)
    {
      if(FirstEdge.Origin == SecondEdge.Origin || FirstEdge.EndPoint == SecondEdge.EndPoint || FirstEdge.EndPoint == SecondEdge.Origin || FirstEdge.Origin == SecondEdge.EndPoint)
      {
        return false; // the edges have a common origin/endpoint and therefore meet, but don't intersect. This could be considered an intersection, but for our purposes, we will not consider this an intersection.
      }

      uint8_t O1 = Orientation(FirstEdge.Origin.Pos, FirstEdge.EndPoint.Pos, SecondEdge.Origin.Pos);
      uint8_t O2 = Orientation(FirstEdge.Origin.Pos, FirstEdge.EndPoint.Pos, SecondEdge.EndPoint.Pos);
      uint8_t O3 = Orientation(SecondEdge.Origin.Pos, SecondEdge.EndPoint.Pos, FirstEdge.Origin.Pos);
      uint8_t O4 = Orientation(SecondEdge.Origin.Pos, SecondEdge.EndPoint.Pos, FirstEdge.EndPoint.Pos);

      if((O1 != O2) && (O3 != O4)) return true;

      // check if point is on line

      if((O1 == 0) && OnSeg(FirstEdge.Origin.Pos, SecondEdge.Origin.Pos, FirstEdge.EndPoint.Pos)) return false;
      if((O2 == 0) && OnSeg(FirstEdge.Origin.Pos, SecondEdge.EndPoint.Pos, SecondEdge.Origin.Pos)) return false;
      if((O3 == 0) && OnSeg(SecondEdge.Origin.Pos, FirstEdge.Origin.Pos, SecondEdge.EndPoint.Pos)) return false;
      if((O4 == 0) && OnSeg(SecondEdge.Origin.Pos, FirstEdge.Origin.Pos, SecondEdge.EndPoint.Pos)) return false;

      return false;
    }

    bool EdgeIntersectsTriangle(Triangle& Tri, Edge E)
    {
      if(EdgesIntersect(Edge(Tri.Points[0], Tri.Points[1]), E)) return true;
      if(EdgesIntersect(Edge(Tri.Points[1], Tri.Points[2]), E)) return true;
      if(EdgesIntersect(Edge(Tri.Points[2], Tri.Points[0]), E)) return true;

      return false;
    }

    bool TriangleIntersectsPolygon(std::vector<Edge>& Poly, Triangle& Tri)
    {
      for(uint32_t i = 0; i < Poly.size(); i++)
      {
        if(EdgeIntersectsTriangle(Tri, Poly[i]))
        {
          return true;
        }
      }

      return false;
    }

    bool bConvex(Point A, Point B, Point C)
    {
      float Res = (B.Pos.x - A.Pos.x) * (C.Pos.y - A.Pos.y) - (B.Pos.y - A.Pos.y) * (C.Pos.x - A.Pos.x);
      return Res > 0.f;
    }

    bool bEar(std::vector<Edge>& Polygon, uint32_t i)
    {
      Point Prev = Polygon[i].Origin;
      Point Curr = Polygon[i].EndPoint;
      Point Next = Polygon[(i+1)%Polygon.size()].EndPoint;

      if(!bConvex(Prev, Curr, Next))
      {
        return false;
      }

      Triangle Tri(Prev, Curr, Next);

      for(uint32_t i = 0; i < Polygon.size(); i++)
      {
        if(Polygon[i].Origin == Prev || Polygon[i].Origin == Curr || Polygon[i].Origin == Next || Polygon[i].EndPoint == Prev || Polygon[i].EndPoint == Curr || Polygon[i].EndPoint == Next)
        {
          continue;
        }
        if(PointInsideTriangle(Tri, Polygon[i].Origin.Pos) || PointInsideTriangle(Tri, Polygon[i].EndPoint.Pos))
        {
          return false;
        }
      }

      return true;
    }
  }

  Triangle::Triangle(Point inA, Point inB, Point inC)
  {
    Points[0] = inA;
    Points[1] = inB;
    Points[2] = inC;

    CalcCircumCircle();

    return;
  }

  void Triangle::CalcCircumCircle()
  {
    vec2 A(0.f, 0.f);
    vec2 B = Points[1].Pos - Points[0].Pos;
    vec2 C = Points[2].Pos - Points[0].Pos;

    float D = 2.f;
    D *= ((B.x*C.y) - (B.y*C.x));

    float Ux = 1.f/D;
    Ux *= (C.y*(Tools::Sq(B.x)+Tools::Sq(B.y)) - B.y*(Tools::Sq(C.x) + Tools::Sq(C.y)));

    float Uy = 1.f/D;
    Uy *= (B.x*(Tools::Sq(C.x) + Tools::Sq(C.y)) - C.x*(Tools::Sq(B.x) + Tools::Sq(B.y)));

    CircumCenter = vec2(Ux+Points[0].Pos.x, Uy+Points[0].Pos.y);
    Radius = sqrt((Ux*Ux)+(Uy*Uy));

    float Gx = (Points[0].Pos.x + Points[1].Pos.x + Points[2].Pos.x)/3.f;
    float Gy = (Points[0].Pos.y + Points[1].Pos.y + Points[2].Pos.y)/3.f;

    Centroid = vec2(Gx, Gy);

    return;
  }

  bool Triangle::ContainsPoint(Point Vert)
  {
    float Distance = sqrt(pow(Vert.Pos.x-CircumCenter.x, 2.f) + pow(Vert.Pos.y-CircumCenter.y, 2.f));
    return Distance < Radius;
  }

  bool Triangle::ContainsEdge(Edge e)
  {
    if(e == Edge(Points[0], Points[1]))
    {
      return true;
    }
    else if(e == Edge(Points[1], Points[2]))
    {
      return true;
    }
    else if(e == Edge(Points[2], Points[0]))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  std::vector<Triangle> EarTriangulate(std::vector<Edge> Polygon)
  {
    std::vector<Triangle> Triangles;

    while(Polygon.size() > 3)
    {
      bool bBad = true;

      for(uint32_t i = 0; i < Polygon.size(); i++)
      {
        if(Tools::bEar(Polygon, i))
        {
          Point Prev = Polygon[i].Origin;
          Point Curr = Polygon[i].EndPoint;
          Point Next = Polygon[(i+1)%Polygon.size()].EndPoint;
          Triangles.push_back(Triangle(Next, Curr, Prev));

          Edge newEdge(Prev, Next);

          // erase the two edges of this new triangle that exist in the polygon array.
          Polygon.erase(Polygon.begin()+i);
          Polygon.erase(Polygon.begin()+i);

          Polygon.insert(Polygon.begin()+i, newEdge);

          bBad = false;

          break;
        }
      }

      if(bBad) throw std::runtime_error("ekCdt : Failed to ear triangulate\n");
    }

    Point Prev = Polygon[0].Origin;
    Point Curr = Polygon[0].EndPoint;
    Point Next = Polygon[1].EndPoint;
    Triangles.push_back(Triangle(Next, Curr, Prev));

    return Triangles;
  }

  void InsertPoint(std::vector<Triangle>& Triangles, Point Vert)
  {
    std::vector<Triangle> BadTriangles;

    for(uint32_t i = 0; i < Triangles.size(); i++)
    {
      if(Triangles[i].ContainsPoint(Vert))
      {
        BadTriangles.push_back(Triangles[i]);
      }
    }

    std::vector<Edge> Polygon = Tools::CreatePolygonHull(BadTriangles);

    uint32_t TriIndex = 0;

    while(TriIndex < Triangles.size())
    {
      bool bKeep = true;

      for(uint32_t i = 0; i < BadTriangles.size(); i++)
      {
        if(Triangles[TriIndex] == BadTriangles[i])
        {
          Triangles.erase(Triangles.begin()+TriIndex);
          bKeep = false;
          break;
        }
      }

      if(bKeep) TriIndex++;
    }

    for(uint32_t i = 0; i < Polygon.size(); i++)
    {
      Triangle tmp(Vert, Polygon[i].Origin, Polygon[i].EndPoint);
      Triangles.push_back(tmp);
    }

    return;
  }

  bool InsertEdge(std::vector<Triangle>& Triangles, Edge E)
  {
    std::vector<Triangle> BadTriangles;

    for(uint32_t i = 0; i < Triangles.size(); i++)
    {
      if(Triangles[i].ContainsEdge(E)) return true;

      if(Tools::EdgeIntersectsTriangle(Triangles[i], E)) BadTriangles.push_back(Triangles[i]);
    }

    std::vector<Edge> Polygon = Tools::CreatePolygonHull(BadTriangles);
    Tools::SortPolygon(Polygon);

    uint32_t TriIndex = 0;

    while(TriIndex < Triangles.size())
    {
      bool bKeep = true;

      for(uint32_t i = 0; i < BadTriangles.size(); i++)
      {
        if(Triangles[TriIndex] == BadTriangles[i])
        {
          Triangles.erase(Triangles.begin()+TriIndex);
          bKeep = false;
          break;
        }
      }

      if(bKeep) TriIndex++;
    }

    std::vector<Triangle> NewTriangles;

    std::vector<Edge> PolyP;
    std::vector<Edge> PolyQ;

    for(uint32_t i = 0; i < Polygon.size(); i++)
    {
      if(Polygon[i].EndPoint == E.Origin)
      {
        uint32_t pIter = i;
        uint32_t qIter = (i+1)%Polygon.size();

        while(true)
        {
          if(Polygon[pIter].EndPoint == E.EndPoint)
          {
            break;
          }
          else
          {
            PolyP.push_back(Polygon[pIter]);
            pIter = (pIter-1 == -1) ? Polygon.size()-1 : pIter-1;
          }

          if(pIter > Polygon.size())
          {
            return false;
          }
        }

        while(true)
        {
          if(Polygon[qIter].Origin == E.EndPoint)
          {
            break;
          }
          else
          {
            PolyQ.push_back(Polygon[qIter]);
            qIter = (qIter+1)%Polygon.size();
          }
        }

        break;
      }
    }

    // polyQ winds in the opposite direction as polyP

    PolyP.push_back(E);
    PolyQ.push_back(Edge(E.EndPoint, E.Origin));

    std::reverse(PolyP.begin(), PolyP.end());

    // ensure that both the polygons are counterclockwise for ear clipping.
    Tools::EnsureCCW(PolyP);
    Tools::EnsureCCW(PolyQ);

    if(PolyP.size() > 3)
    {
      std::vector<Triangle> TriP = EarTriangulate(PolyP);

      NewTriangles.insert(NewTriangles.end(), TriP.begin(), TriP.end());
    }
    else if(PolyP.size() >= 2)
    {
      Triangle tmp(PolyP[1].EndPoint, PolyP[0].EndPoint, PolyP[0].Origin);
      NewTriangles.push_back(tmp);
    }
    else
    {
      return false;
    }

    if(PolyQ.size() > 3)
    {
      std::vector<Triangle> TriQ = EarTriangulate(PolyQ);
      NewTriangles.insert(NewTriangles.end(), TriQ.begin(), TriQ.end());
    }
    else if(PolyQ.size() >= 2)
    {
      Triangle tmp(PolyQ[1].EndPoint, PolyQ[0].EndPoint, PolyQ[0].Origin); // changed
      NewTriangles.push_back(tmp);
    }
    else
    {
      return false;
    }

    Triangles.insert(Triangles.end(), NewTriangles.begin(), NewTriangles.end());

    return true;
  }

  std::vector<Triangle> BowyerWattson(std::vector<Point>* Points)
  {
    float minX = std::numeric_limits<float>::max();
    float maxX = -minX;
    float minY = minX;
    float maxY = maxX;

    for(uint32_t i = 0; i < Points->size(); i++)
    {
      if(Points->at(i).Pos.x < minX) minX = Points->at(i).Pos.x;
      if(Points->at(i).Pos.x > maxX) maxX = Points->at(i).Pos.x;
      if(Points->at(i).Pos.y < minY) minY = Points->at(i).Pos.y;
      if(Points->at(i).Pos.y > maxY) maxY = Points->at(i).Pos.y;
    }

    vec2 bbDimension((maxX - minX), (maxY - minY));
    float DeltaMax = std::max(bbDimension.x, bbDimension.y);
    vec2 bbMidPoint((minX+maxX)/2.0, (minY+maxY)/2.0);

    vec2 A, B, C;

    A = vec2(bbMidPoint.x-10.f*DeltaMax, bbMidPoint.y-DeltaMax);
    B = vec2(bbMidPoint.x, bbMidPoint.y+10.f*DeltaMax);
    C = vec2(bbMidPoint.x+10.f*DeltaMax, bbMidPoint.y-DeltaMax);

    Triangle SuperTri(Point(A, 997), Point(B, 998), Point(C, 999));

    std::vector<Triangle> Triangles = {SuperTri};

    for(uint32_t i = 0; i < Points->size(); i++)
    {
      InsertPoint(Triangles, Points->at(i));
    }

    uint32_t TriIndex = 0;

    while(TriIndex < Triangles.size())
    {
      bool bKeep = true;

      for(uint32_t i = 0; i < 3; i++)
      {
        Point Vert = Triangles[TriIndex].Points[i];

        if(Vert.Index == 997 || Vert.Index == 998 || Vert.Index == 999)
        {
          bKeep = false;
          Triangles.erase(Triangles.begin()+TriIndex);
          break;
        }
      }

      if(bKeep) TriIndex++;
    }

    return Triangles;
  }

  std::vector<uint32_t> Triangulate(uint32_t PositionCount, vec2* pPositions, std::vector<std::pair<uint32_t, uint32_t>>& Constraints)
  {
    std::vector<Point> Vertices;
    std::vector<Triangle> Triangles;
    std::vector<Edge> ConstraintEdges;

    for(uint32_t i = 0; i < PositionCount; i++)
    {
      Vertices.push_back(Point(pPositions[i], i));
    }

    Triangles = BowyerWattson(&Vertices);

    for(uint32_t i = 0; i < Constraints.size(); i++)
    {
      ConstraintEdges.push_back(Edge(Vertices[Constraints[i].first], Vertices[Constraints[i].second]));
    }

    for(uint32_t i = 0; i < ConstraintEdges.size(); i++)
    {
      if(!InsertEdge(Triangles, ConstraintEdges[i]))
      {
        throw std::runtime_error("ekCdt : Failed to insert a constraint edge into the triangulation (are your constraint polygons enclosed? are there any flipped edges in the constraints?)\n");
      }
    }

    std::vector<Triangle> BadTriangles;

    for(uint32_t x = 0; x < Triangles.size(); x++)
    {
      if(Tools::TriangleIntersectsPolygon(ConstraintEdges, Triangles[x]))
      {
        BadTriangles.push_back(Triangles[x]);
      }
      else if(Tools::pipCheck(ConstraintEdges, Triangles[x].Centroid) == -1)
      {
        BadTriangles.push_back(Triangles[x]);
      }
    }
    
    uint32_t TriIndex = 0;

    while(TriIndex < Triangles.size())
    {
      bool bKeepTri = true;

      uint32_t i = 0;

      while(i < BadTriangles.size())
      {
        if(Triangles[TriIndex] == BadTriangles[i])
        {
          bKeepTri = false;
          Triangles.erase(Triangles.begin()+TriIndex);
          break;
        }

        i++;
      }

      if(bKeepTri) TriIndex++;
    }

    std::vector<uint32_t> Indices;

    for(uint32_t i = 0; i < Triangles.size(); i++)
    {
      Indices.push_back(Triangles[i].Points[0].Index);
      Indices.push_back(Triangles[i].Points[1].Index);
      Indices.push_back(Triangles[i].Points[2].Index);
    }

    return Indices;
  }
}

#endif
