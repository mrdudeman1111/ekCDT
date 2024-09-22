#pragma once

#include <vector>
#include <fstream>

#include "obj_io.h"

#include "ekCdt.hpp"

// Uses obj_io to export triangulated mesh.
void Export(std::vector<ekCdt::vec2> Points, std::vector<uint32_t> Indices, const char* SaveLoc)
{
    auto vtxIter = Points.begin();
    auto vtxEnd = Points.end();
    auto vtxMapper = [&vtxIter, vtxEnd]() {
        if(vtxIter == vtxEnd)
        {
            return thinks::ObjEnd<thinks::ObjPosition<float, 3>>();
        }

        const auto Ret = thinks::ObjMap(thinks::ObjPosition<float, 3>(vtxIter->x, vtxIter->y, 0.f));
        (*vtxIter++);
        return Ret;
    };

    auto idxIter = Indices.begin();
    auto idxEnd = Indices.end();
    auto idxMapper= [&idxIter, idxEnd]() {
        if (std::distance(idxIter, idxEnd) < 3) {
            return thinks::ObjEnd<thinks::ObjTriangleFace<thinks::ObjIndex<uint32_t>>>();
        }

        const auto idxOne = thinks::ObjIndex<uint32_t>(*idxIter++);
        const auto idxTwo = thinks::ObjIndex<uint32_t>(*idxIter++);
        const auto idxThree = thinks::ObjIndex<uint32_t>(*idxIter++);
        return thinks::ObjMap(thinks::ObjTriangleFace<thinks::ObjIndex<uint32_t>>(idxOne, idxTwo, idxThree));
    };

    std::ofstream MeshFile(SaveLoc);
    if(!MeshFile.is_open()) throw std::runtime_error("Failed to open mesh for Export()");

    thinks::WriteObj(MeshFile, vtxMapper, idxMapper, nullptr, nullptr);

    MeshFile.close();
}