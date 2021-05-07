#include "photon_mapping.h"

#include <chrono>
#include <iostream>

#include "yocto_geometry.h"
namespace yocto {
void sample_photons(const scene_model& scene_, const bvh_scene& bvh,
    const trace_lights& lights_, rng_state& rng) {
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();

  auto scene  = scene_;
  auto lights = lights_;

  int photons_per_light = 10000000;  // Fotones lanzados por fuente de luz
  int intersectados     = 0;  // Mide el numero de fotones que chocan en un
  for (auto light : lights.lights) {
    if (light.instance == invalidid) {
      continue;  // Si no se hace esto hay segmentation fault;
    }
    auto  light_instance = scene.instances[light.instance];
    int   light_shape_id = light_instance.shape;
    auto  light_shape    = scene.shapes[light_shape_id];
    vec3f random_position;
    vec3f random_position_normal;  // Normal del punto aleatorio
    vec2f random_uv;
    bool  valid = false;
    for (int i = 0; i < photons_per_light; i++) {
      // std::cout << "Foton " << i << std::endl;
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
      if (valid) {
        // Aqui random position tiene que ser la posicion desde que se va a
        // lanzar el foton, y random_position_normal la normal en ese punto
        vec3f random_direction = sample_hemisphere_cos(
            random_position_normal, random_uv);
        ray3f random_ray   = ray3f{random_position, random_direction};
        auto  intersection = intersect_bvh(bvh, scene, random_ray);
        // std::cout << "Hola" << std::endl;
        if (intersection.hit) {
          auto instance_id           = intersection.instance;
          auto instance_intersection = scene.instances[instance_id];
          auto intersection_material = instance_intersection.material;
          if (scene.materials[intersection_material].type ==
              scene_material_type::refractive) {
            intersectados++;
          }
        }
        // std::cout << "Hola 2" << std::endl;
      }
    }
  }
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