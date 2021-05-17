#ifndef _PHOTON_MAPPING_H_
#define _PHOTON_MAPPING_H_

#include "KDTree.h"
#include "yocto_cli.h"
#include "yocto_color.h"
#include "yocto_geometry.h"
#include "yocto_parallel.h"
#include "yocto_sampling.h"
#include "yocto_scene.h"
#include "yocto_shading.h"
#include "yocto_shape.h"
#include "yocto_trace.h"

namespace yocto {
struct Photon;
void sample_photons(const scene_model& scene, const bvh_scene& bvh,
    const trace_lights& lights, rng_state& rng,
    KDTree<Photon, 3>* m_caustics_map);

// Structure defining a photon (a directionally-resolved packet of
// energy), that will be used later for radiance estimation.
struct Photon {
  vec3f position;
  vec3f direction;
  vec3f flux;
  vec3f color;
  Photon() {
    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;

    direction.x = 0.0;
    direction.y = 0.0;
    direction.z = 0.0;

    flux.x = 0.0;
    flux.y = 0.0;
    flux.z = 0.0;
    color  = vec3f{0.0, 0.0, 0.0};
  }
  Photon(const vec3f& p, const vec3f& d, const vec3f& f)
      : position(p), direction(d), flux(f) {}
};

}  // namespace yocto
#endif