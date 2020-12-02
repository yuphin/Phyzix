/**********************************************************************
Copyright (c) 2016 Advanced Micro Devices, Inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
********************************************************************/

/* 
	Modified version of code from https://github.com/GPUOpen-LibrariesAndSDKs/RadeonRays_SDK 
*/

#pragma once

#ifndef BVH_TRANSLATOR_H
#define BVH_TRANSLATOR_H

#include <map>

#include "bvh.h"
#include "../../Mesh.h"

namespace RadeonRays
{
    /// This class translates pointer based BVH representation into
    /// index based one suitable for feeding to GPU or any other accelerator
    //
    class BvhTranslator
    {
    public:
        // Constructor
        BvhTranslator() = default;

		struct Node
		{
			int left_index;
			int right_index;
			int leaf;
		};

		void ProcessBLAS();
		void ProcessTLAS();
		void UpdateTLAS(const Bvh *top_level_bvh, const std::vector<MeshInstance> &instances);
		void Process(const Bvh *top_level_bvh, const std::vector<Mesh*> &meshes, const std::vector<MeshInstance> &instances);
		int top_level_idx_packed_xy = 0;
		int top_level_idx = 0;
		std::vector<Vec3f> bboxmin;
		std::vector<Vec3f> bboxmax;
		std::vector<Node> nodes;
		int node_tex_width;

    private:
		int cur_node = 0;
		int cur_tri_idx = 0;
		std::vector<int> bvh_root_start_indices;
		int ProcessBLASNodes(const Bvh::Node *root);
		int ProcessTLASNodes(const Bvh::Node *root);
		std::vector<MeshInstance> mesh_instances;
		std::vector<Mesh*> meshes;
		const Bvh *TLBvh;
    };
}

#endif // BVH_TRANSLATOR_H
