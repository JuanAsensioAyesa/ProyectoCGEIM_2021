#include "photon_mapping.h"

#include <chrono>
#include <iostream>

#include "yocto_geometry.h"
namespace yocto {

ray3f sample_random_ray(
    yocto::trace_light light, const scene_model& scene, rng_state& rng) {
  auto  light_instance = scene.instances[light.instance];
  int   light_shape_id = light_instance.shape;
  auto  light_shape    = scene.shapes[light_shape_id];
  vec3f random_position;
  vec3f random_position_normal;  // Normal del punto aleatorio
  vec2f random_uv;
  bool  valid = false;
  if (!light_shape.quads.empty()) {
    auto light_quad_index = sample_uniform(
        (int)light_shape.quads.size(), rand1f(rng));
    auto quad = light_shape.quads[light_quad_index];
    random_uv = rand2f(rng);
    auto p1   = light_shape.positions[quad.x];
    auto p2   = light_shape.positions[quad.y];
    auto p3   = light_shape.positions[quad.z];
    auto p4   = light_shape.positions[quad.w];
    p1        = transform_point(light_instance.frame, p1);
    p2        = transform_point(light_instance.frame, p2);
    p3        = transform_point(light_instance.frame, p3);
    p4        = transform_point(light_instance.frame, p4);

    auto n1 = light_shape.normals[quad.x];
    auto n2 = light_shape.normals[quad.y];
    auto n3 = light_shape.normals[quad.z];
    auto n4 = light_shape.normals[quad.w];

    n1 = transform_direction(light_instance.frame, n1);
    n2 = transform_direction(light_instance.frame, n2);
    n3 = transform_direction(light_instance.frame, n3);
    n4 = transform_direction(light_instance.frame, n4);

    random_position        = interpolate_quad(p1, p2, p3, p4, random_uv);
    random_position_normal = interpolate_quad(n1, n2, n3, n4, random_uv);
    valid                  = true;
  }
  if (!light_shape.triangles.empty()) {
    // Hacer lo mismo de arriba pero con triangulos
  }
  ray3f random_ray;
  if (valid) {
    vec3f random_direction = sample_hemisphere_cos(
        random_position_normal, random_uv);
    random_ray = ray3f{random_position, random_direction};
  }
  return random_ray;
}


bool trace_photon(const scene_model& scene, const bvh_scene& bvh, Photon &p, KDTree<Photon, 3>& m_caustics_map, rng_state& rng)
{
#ifndef MAX_PHOTON_ITERATIONS
#define MAX_PHOTON_ITERATIONS 7
#endif

	// Compute irradiance photon's energy
	vec3f energy = p.flux;

	ray3f photon_ray;
  photon_ray.o = p.position;
  photon_ray.d = p.direction;

	bool is_caustic_particle = false;
  int i= 0;
	//Iterate the path
	while (1)
	{
		// Throw ray and update current_it
		auto intersection = intersect_bvh(bvh, scene, photon_ray);
      if (intersection.hit) {
        auto instance_id           = intersection.instance;
        auto instance_intersection = scene.instances[instance_id];
        auto intersection_material = instance_intersection.material;
        if (scene.materials[intersection_material].type ==
          scene_material_type::refractive) {
             is_caustic_particle = true;
             //calcular siguiente rayo y esas cosas
             auto  outgoing = -photon_ray.d;
             auto& instance = scene.instances[intersection.instance];
             auto  element  = intersection.element;
             auto  uv       = intersection.uv;
             auto  position = eval_position(scene, instance, element, uv);
             auto normal = eval_shading_normal(scene, instance, element, uv, outgoing);
            auto material = eval_material(scene, instance, element, uv);
            auto incoming = sample_refractive(material.color, material.ior, material.roughness,
                              normal, outgoing, rand1f(rng), rand2f(rng));
            photon_ray.o = position;
            photon_ray.d = incoming;
        }
        else {
          if (is_caustic_particle){
              auto& instance         = scene.instances[intersection.instance];
              auto  element          = intersection.element;
              auto  uv               = intersection.uv;
              auto  position         = eval_position(scene, instance, element, uv);
              std::vector<float> pos = std::vector<float>();
              pos.push_back(position.x);
              pos.push_back(position.y);
              pos.push_back(position.z);
              p.position = position;
              p.direction = photon_ray.d;
              m_caustics_map.store(pos, p);
          }
          is_caustic_particle = false;
          break;
        }
      }
		  else
  			break;
		}

	return true;
}


void sample_photons(const scene_model& scene, const bvh_scene& bvh,
    const trace_lights& lights_, rng_state& rng,
    KDTree<Photon, 3>& m_caustics_map) {
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();

  auto lights = lights_;

  int photons_per_light = 1000000;  // Fotones lanzados por fuente de luz
  int intersectados     = 0;  // Mide el numero de fotones que chocan en un
  for (auto light : lights.lights) {
    if (light.instance == invalidid) {
      continue;  // Si no se hace esto hay segmentation fault;
    }

    for (int i = 0; i < photons_per_light; i++) {

      ray3f random_ray = sample_random_ray(light, scene, rng);
      // Aqui random position tiene que ser la posicion desde que se va a
      // lanzar el foton, y random_position_normal la normal en ese punto
      Photon p = Photon();
      vec3f light_power      = scene.materials[scene.instances[light.instance].shape].emission;
      float light_area       = 0;
      auto light_shape       = scene.shapes[scene.instances[light.instance].shape];
      for (auto quad : light_shape.quads){
        vec3f p0   = light_shape.positions[quad.x];
        vec3f p1   = light_shape.positions[quad.y];
        vec3f p2   = light_shape.positions[quad.z];
        vec3f p3   = light_shape.positions[quad.w];
        light_area += quad_area(p0, p1, p2, p3);
      }
      p.flux                 = light_power / (photons_per_light * (1 / (2 * pif))* (1 / light_area));
      p.position             = random_ray.o;
      p.direction            = random_ray.d;
      trace_photon(scene, bvh, p, m_caustics_map, rng);
    }
  }
  if (m_caustics_map.size() > 0)
    m_caustics_map.balance();
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  std::cout << "Lanzados " << photons_per_light * lights.lights.size()
            << " intersectan: " << intersectados << "    " << std::endl;

  std::cout << "Porcentaje "
            << float(intersectados) /
                   float(photons_per_light * lights.lights.size())
            << " Tiempo: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   end - begin)
                   .count()
            << "ms" << std::endl;
}

}  // namespace yocto