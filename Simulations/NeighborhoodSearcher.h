#pragma once
#include <algorithm>
#include <unordered_map>
#include "util/vectorbase.h"
#include "util/matrixbase.h"
#include "util/quaternion.h"
#include "util/util.h"
#include "Particle.h"

struct HashVec {
	HashVec() = default;
	HashVec(int x, int y, int z) :
		x(x), y(y), z(z) { }
	int x, y, z;
	HashVec& operator=(HashVec const& rhs) {
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		return *this;
	}

	bool operator==(HashVec const& rhs) const {
		return
			x == rhs.x &&
			y == rhs.y &&
			z == rhs.z;
	}

	bool operator!=(HashVec const& other) const {
		return !(*this == other);
	}

};
struct Hash {
	size_t operator()(const HashVec& key) const {
		return static_cast<size_t>(
			static_cast<int64_t>(73856093) * static_cast<int64_t>(key.x) ^
			static_cast<int64_t>(19349663) * static_cast<int64_t>(key.y) ^
			static_cast<int64_t>(83492791) * static_cast<int64_t>(key.z)
			);
	}
};

struct ParticleSet {
	ParticleSet(std::vector<Particle> const * particles) {
		this->particles = particles;
	}
	const std::vector<Particle>* particles;
	std::vector<std::vector<std::vector<int>>> neighbor_idxs;
	std::vector<HashVec> keys, old_keys;
};

class NeighborhoodSearcher {
public:
	NeighborhoodSearcher(Real r) {
		this->r = r;
		inv_r = 1.0 / r;
		r2 = r * r;
	}

	std::vector<ParticleSet> particle_sets;
	void register_set(const std::vector<Particle>& particle_set);
	void find_neighborhoods();
private:
	void init();
	void update();
	void search();
	HashVec hash_from_pos(const Particle& p) {
		HashVec res;
		res.x = (int)p.pos.x * inv_r;
		if (p.pos.x < 0) {
			res.x--;
		}
		res.y = (int)p.pos.y * inv_r;
		if (p.pos.y < 0) {
			res.y--;
		}
		res.z = (int)p.pos.z * inv_r;
		if (p.pos.z < 0) {
			res.z--;
		}
		return res;
	}
	// Vec3i == 3D integer coordinate of a cell 
	// uint32_t == index to the hash_entry
	std::unordered_map<HashVec, uint32_t, Hash> map;
	std::vector<std::vector<std::pair<int, int>>> hash_entries;
	//std::vector<std::vector<Particle> const*> particle_sets;
	//std::vector<HashVec> keys, old_keys;
	bool initialized = false;
	Real r;
	Real inv_r;
	Real r2;
	uint32_t num_particles = 0;
};

