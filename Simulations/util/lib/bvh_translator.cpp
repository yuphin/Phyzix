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

#include "bvh_translator.h"

#include <cassert>
#include <stack>
#include <iostream>

namespace RadeonRays
{
	int BvhTranslator::ProcessBLASNodes(const Bvh::Node *node)
	{
		RadeonRays::bbox bbox = node->bounds;

		bboxmin[cur_node] = bbox.pmin;
		bboxmax[cur_node] = bbox.pmax;
		nodes[cur_node].leaf = 0;

		int index = cur_node;

		if (node->type == RadeonRays::Bvh::NodeType::kLeaf)
		{
			nodes[cur_node].left_index = cur_tri_idx + node->startidx;
			nodes[cur_node].right_index = node->numprims;
			nodes[cur_node].leaf = 1;
		}
		else
		{
			cur_node++;
			nodes[index].left_index = ProcessBLASNodes(node->lc);
			nodes[index].left_index = ((nodes[index].left_index % node_tex_width) << 12) | (nodes[index].left_index / node_tex_width);
			cur_node++;
			nodes[index].right_index = ProcessBLASNodes(node->rc);
			nodes[index].right_index = ((nodes[index].right_index % node_tex_width) << 12) | (nodes[index].right_index / node_tex_width);
		}
		return index;
	}

	int BvhTranslator::ProcessTLASNodes(const Bvh::Node *node)
	{
		RadeonRays::bbox bbox = node->bounds;

		bboxmin[cur_node] = bbox.pmin;
		bboxmax[cur_node] = bbox.pmax;
		nodes[cur_node].leaf = 0;

		int index = cur_node;

		if (node->type == RadeonRays::Bvh::NodeType::kLeaf)
		{
			int instanceIndex = TLBvh->m_packed_indices[node->startidx];
			int meshIndex = mesh_instances[instanceIndex].mesh_id;
			int materialID = mesh_instances[instanceIndex].material_id;

			nodes[cur_node].left_index = (bvh_root_start_indices[meshIndex] % node_tex_width) << 12 | (bvh_root_start_indices[meshIndex] / node_tex_width);
			nodes[cur_node].right_index = materialID;
			nodes[cur_node].leaf = -instanceIndex - 1;
		}
		else
		{
			cur_node++;
			nodes[index].left_index = ProcessTLASNodes(node->lc);
			nodes[index].left_index = ((nodes[index].left_index % node_tex_width) << 12) | (nodes[index].left_index / node_tex_width);
			cur_node++;
			nodes[index].right_index = ProcessTLASNodes(node->rc);
			nodes[index].right_index = ((nodes[index].right_index % node_tex_width) << 12) | (nodes[index].right_index / node_tex_width);
		}
		return index;
	}
	
	void BvhTranslator::ProcessBLAS()
	{
		int nodeCnt = 0;

		for (int i = 0; i < meshes.size(); i++)
			nodeCnt += meshes[i]->bvh->m_nodecnt;
		top_level_idx = nodeCnt;

		// reserve space for top level nodes
		nodeCnt += 2 * mesh_instances.size();
		node_tex_width = (int)(sqrt(nodeCnt) + 1);

		// Resize to power of 2
		bboxmin.resize(node_tex_width * node_tex_width);
		bboxmax.resize(node_tex_width * node_tex_width);
		nodes.resize(node_tex_width * node_tex_width);

		int bvhRootIndex = 0;
		cur_tri_idx = 0;

		for (int i = 0; i < meshes.size(); i++)
		{
			Mesh *mesh = meshes[i];
			cur_node = bvhRootIndex;

			bvh_root_start_indices.push_back(bvhRootIndex);
			bvhRootIndex += mesh->bvh->m_nodecnt;
			
			ProcessBLASNodes(mesh->bvh->m_root);
			cur_tri_idx += mesh->bvh->GetNumIndices();
		}
	}

	void BvhTranslator::ProcessTLAS()
	{
		cur_node = top_level_idx;
		top_level_idx_packed_xy = ((top_level_idx % node_tex_width) << 12) | (top_level_idx / node_tex_width);
		ProcessTLASNodes(TLBvh->m_root);
	}

	void BvhTranslator::UpdateTLAS(const Bvh *topLevelBvh, const std::vector<MeshInstance> &sceneInstances)
	{
		TLBvh = topLevelBvh;
		mesh_instances = sceneInstances;
		cur_node = top_level_idx;
		ProcessTLASNodes(TLBvh->m_root);
	}

	void BvhTranslator::Process(const Bvh *topLevelBvh, const std::vector<Mesh*> &sceneMeshes,const std::vector<MeshInstance> &sceneInstances)
	{
		TLBvh = topLevelBvh;
		meshes = sceneMeshes;
		mesh_instances = sceneInstances;
		ProcessBLAS();
		ProcessTLAS();
	}
}
