# Header Only Constrained Delaunay Triangulation.

 This header is part of a seperate project I am currently working on, but I figured it would be a good Idea to put this header up in a seperate repository just in case someone else would like to use it.
 My goal was to create something small and portable, like the stb headers, for 2D and 3D Triangulation.

 This header does use C++ features and functions and is not compatible with C.

 To use this header, you must define CDT_IMPL and include <b>ekCdt.hpp</b> in a source file (e.g. .CXX .CP .CPP).

 <code> #define CDT_IMPL <br> #include "ekCdt.hpp" </code>

 Once the file has been included, Triangulation can be performed through the <code>ekCdt::Triangulate()</code> function. The function requires a set of two component vectors (or a casted array of 32-bit floats) to be passed along with a constraint polygon (a polygon containing all borders and holes). Here is some example usage can be seen here where we triangulate a square with square hole inside it.
```
#include <iostream>
#include <vector>

struct Mesh2D
{
    public:
        std::vector<ekCdt::vec2> Vertices;
        std::vector<uint32_t> Indices;
};

std::vector<ekCdt::vec2> Points = {
    /* Outer */
    {-2.f, 2.f},  // top left
    {2.f, 2.f},   // top right
    {2.f, -2.f},  // bottom right
    {-2.f, -2.f}, // bottom left

    /* Inner/hole */
    {-1.f, 1.f},  // top left
    {1.f, 1.f},   // top right
    {1.f, -1.f},  // bottom right
    {-1.f, -1.f}  // bottom left
};

std::vector<std::pair<uint32_t, uint32_t>> Border = {
    /* Outer */
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0},

    /* Inner/hole */
    {4, 5},
    {5, 6},
    {6, 7},
    {7, 4}
};

int main()
{
    std::vector<uint32_t> Triangles = ekCdt::Triangulate(Points.size(), Points.data(), Border);

    Mesh2D Mesh;
    Mesh.Vertices = Points;
    Mesh.Indices = Triangles;

    return 0;
}
```

<br>

Which will output a mesh that looks like this

![alt text](./Output.png "(A picture of a square with a square hole in it)")
