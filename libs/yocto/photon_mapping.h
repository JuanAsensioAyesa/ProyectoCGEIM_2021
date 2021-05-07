#ifndef _PHOTON_MAPPING_H_
#define _PHOTON_MAPPING_H_

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
void sample_photons(const scene_model& scene, const bvh_scene& bvh,
    const trace_lights& lights, rng_state& rng);
}

#endif