#include "NeighborhoodSearcher.h"
#include "util/vectorbase.h"
// Reference:
// https://cg.informatik.uni-freiburg.de/intern/seminar/dataStructures_2011_CGF_dataStructuresSPH.pdf


static Real norm_Sqr(const Vec3& v) {
	return v.x * v.x + v.y * v.y + v.z * v.z;
}
void NeighborhoodSearcher::register_set(const std::vector<Particle>& particle_vec) {
	particle_sets.push_back({ &particle_vec });
	num_particles += particle_vec.size();
}

void NeighborhoodSearcher::find_neighborhoods() {
	if (!initialized) {
		init();
	}
	update();
	search();
}
void NeighborhoodSearcher::init() {
	map.clear();
	hash_entries.clear();
	for (int j = 0; j < particle_sets.size(); j++) {
		auto& keys = particle_sets[j].keys;
		keys.resize(particle_sets[j].particles->size());
		for (int i = 0; i < particle_sets[j].particles->size(); i++) {
			HashVec hash_key = hash_from_pos(particle_sets[j].particles->at(i));
			auto it = map.find(hash_key);
			const auto& p = particle_sets[j].particles->at(i);
			if (it == map.end()) {
				std::vector<std::pair<int, int>> entries;
				entries.push_back({ p.set_idx, i });
				hash_entries.push_back(entries);
				map[hash_key] = hash_entries.size() - 1;
			} else {
				hash_entries[it->second].push_back({ p.set_idx, i });
			}
			keys[i] = hash_key;
		}
		particle_sets[j].old_keys = keys;
	}
	initialized = true;
}

void NeighborhoodSearcher::update() {
	std::vector<std::tuple<int, int, const HashVec& ,const HashVec&>> updated;
	updated.reserve(num_particles * 0.05);
	for (int i = 0; i < particle_sets.size(); i++) {
		if (i == 1) {
			// Assume the set idx 1 is static bnd
			continue;
		}
		auto& keys = particle_sets[i].keys;
		auto& old_keys = particle_sets[i].old_keys;
		std::swap(keys, old_keys);
		for (int j = 0; j < particle_sets[i].particles->size(); j++) {
			keys[j] = hash_from_pos(particle_sets[i].particles->at(j));
			if (keys[j] != old_keys[j]) {
				updated.push_back({ i, j, keys[j], old_keys[j]});
			}
		}
	}
	for (const auto& [set_idx, idx, key, old_key] : updated) { 
		auto it = map.find(key);
		if (it == map.end()) {
			std::vector<std::pair<int, int>> entries;
			entries.push_back({ set_idx, idx });
			hash_entries.push_back(entries);
			map[key] = hash_entries.size() - 1;
		} else {
			hash_entries[it->second].push_back({ set_idx, idx});
		}

		uint32_t old_idx = map[old_key];
		auto &old_entry = hash_entries[old_idx];
		auto it_entry = std::find(old_entry.begin(), old_entry.end(), std::make_pair(set_idx,idx));
		if (it_entry != old_entry.end()) {
			old_entry.erase(it_entry);
		}
		if (old_entry.empty()) {
			map.erase(old_key);
		}
	}
}

void NeighborhoodSearcher::search() {
	// Search for the items thats in the same cell
	for (int i = 0; i < particle_sets.size(); i++) {
		particle_sets[i].neighbor_idxs.resize(particle_sets.size());
		for (int j = 0; j < particle_sets.size(); j++) {
			particle_sets[i].neighbor_idxs[j].resize(particle_sets[i].particles->size());
			for (auto& el : particle_sets[i].neighbor_idxs[j]) {
				el.clear();
			}
		}
	}
	for (const auto& kv : map) {
		const auto& list = hash_entries[kv.second];
		for (int i = 0; i < list.size(); i++) {
			auto [set_idx, idx] = list[i];
			const Particle& pi = particle_sets[set_idx].particles->at(idx);
			for (int j = i + 1; j < list.size(); j++) {
				auto [set_idx_adj, idx_adj] = list[j];
				const auto& pj = particle_sets[set_idx_adj].particles->at(idx_adj);
				Real dist2 = norm_Sqr(pi.pos - pj.pos);
				if (dist2 < r2) {
					particle_sets[set_idx].neighbor_idxs[set_idx_adj][idx].push_back(idx_adj);
					particle_sets[set_idx_adj].neighbor_idxs[set_idx][idx_adj].push_back(idx);
				}
			}
		}
	}
	// Search for adjacent cells
	std::vector<std::vector<bool>> visited(hash_entries.size(), std::vector<bool>(27,false));
	for (const auto& kv : map) {
		const auto& list = hash_entries[kv.second];
		bool allocate = true;
		for (int dci = -1; dci <= 1; dci++) {
			for (int dcj = -1; dcj <= 1; dcj++) {
				for (int dck = -1; dck <= 1; dck++) {
					int offset = 9 * (1 + dci) + 3 * (1 + dcj) + 1 + dck;
					if (offset == 13) {
						continue;
					}
					if (visited[kv.second][offset]) {
						continue;
					}
					const HashVec adj_entry(kv.first.x + dci,
						kv.first.y + dcj,
						kv.first.z + dck);
					auto adj_it = map.find(adj_entry);
					if (adj_it == map.end()) {
						continue;
					}
					
					const auto& adj_list = hash_entries[adj_it->second];
					using template_arg = std::vector<std::pair<int, int>>;
					const std::vector<template_arg> entry_idxs{list, adj_list };
					bool outer = 0;
					if (kv.second > adj_it->second) {
						outer ^=1;
					} 
					visited[kv.second][offset] = true;
					visited[adj_it->second][26 - offset] = true;
					for (int i = 0; i < entry_idxs[outer].size(); i++) {
						auto [set_idx, idx] = entry_idxs[outer][i];
						const Particle& pi = particle_sets[set_idx].particles->at(idx);
						for (int j = 0; j < entry_idxs[!outer].size(); j++) {
							auto [set_idx_adj, idx_adj] = entry_idxs[!outer][j];
							const Particle& pj = particle_sets[set_idx_adj].particles->at(idx_adj);
							Real dist2 = norm_Sqr(pi.pos - pj.pos);
							if (dist2 < r2) {
								particle_sets[set_idx].neighbor_idxs[set_idx_adj][idx].push_back(idx_adj);
								particle_sets[set_idx_adj].neighbor_idxs[set_idx][idx_adj].push_back(idx);
							}
						}
					}
				}
			}
		}
	}

}
