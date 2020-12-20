#pragma once
#include <cgv\media\mesh\simple_mesh.h>

#include "lib_begin.h"
namespace cgv {
	namespace media {
		namespace mesh {

			/// the simple_mesh class is templated over the coordinate type that defaults to float
			template <typename T = float>
			class simple_mesh_ext : public simple_mesh<T>
			{
				std::vector<cgv::math::fvec<float, 4>> bone_index;
				std::vector<cgv::math::fvec<float, 4>> bone_weights;
			public:
				typedef typename simple_mesh_ext<T> mesh_type;
				/// construct from string corresponding to Conway notation (defaults to empty mesh)
				simple_mesh_ext(const std::string& conway_notation = "") {
					if (!conway_notation.empty())
						construct_conway_polyhedron(conway_notation);
				}

				box_type compute_box() const;

				unsigned extract_vertex_attribute_buffer(
					const std::vector<idx_type>& vertex_indices,
					const std::vector<vec3i>& unique_triples,
					bool include_tex_coords, bool include_normals,
					std::vector<T>& attrib_buffer, bool* include_colors_ptr = 0) const;

				/*bool read(const std::string& file_name) {
					return true;
				}*/

			};

			/// overwrite the compute_box function
			template <typename T>
			typename simple_mesh_ext<T>::box_type simple_mesh_ext<T>::compute_box() const
			{
				box_type box;
				for (const auto& p : positions)
					box.add_point(p);
				std::cout << "call compute_box in simple_mesh_ext!" << std::endl;
				return box;
			}

			/// extract vertex attribute array and element array buffers for triangulation and edges in wireframe
			template <typename T>
			unsigned simple_mesh_ext<T>::extract_vertex_attribute_buffer(
				const std::vector<idx_type>& vertex_indices,
				const std::vector<vec3i>& unique_triples,
				bool include_tex_coords, bool include_normals,
				std::vector<T>& attrib_buffer, bool* include_colors_ptr) const
			{
				// correct inquiry in case data is missing
				include_tex_coords = include_tex_coords && !tex_coord_indices.empty() && !tex_coords.empty();
				include_normals = include_normals && !normal_indices.empty() && !normals.empty();
				bool include_colors = false;
				if (include_colors_ptr)
					*include_colors_ptr = include_colors = has_colors() && *include_colors_ptr;

				// determine number floats per vertex
				unsigned nr_floats = 3;
				nr_floats += include_tex_coords ? 2 : 0;
				nr_floats += include_normals ? 3 : 0;
				unsigned color_increment = 0;
				if (include_colors) {
					color_increment = (int)ceil((float)get_color_size() / sizeof(T));
					nr_floats += color_increment;
				}

				attrib_buffer.resize(nr_floats * unique_triples.size());
				T* data_ptr = &attrib_buffer.front();
				for (auto t : unique_triples) {
					*reinterpret_cast<vec3*>(data_ptr) = positions[t[0]];
					data_ptr += 3;
					if (include_tex_coords) {
						*reinterpret_cast<vec2*>(data_ptr) = tex_coords[t[1]];
						data_ptr += 2;
					}
					if (include_normals) {
						*reinterpret_cast<vec3*>(data_ptr) = normals[t[2]];
						data_ptr += 3;
					}
					if (include_colors) {
						put_color(t[0], data_ptr);
						data_ptr += color_increment;
					}
				}
				std::cout << "call extract_vertex_attribute_buffer in simple_mesh_ext!" << std::endl;
				return color_increment;
			}

		}
	}
}
#include <cgv/config/lib_end.h>