#include "vr_meshing.h"
#include <point_cloud_interactable.h>

vr_rgbd::vr_rgbd()
{
	set_name("vr_meshing");
	rgbd_controller_index = 0;
	controller_orientation.identity();
	controller_position = vec3(0, 1.5f, 0);
	in_calibration = false;
	zoom_in = false;
	zoom_out = false;
	save_pointcloud = true;
	registration_started = false;
	rgbd_2_controller_orientation.identity();
	/*rgbd_2_controller_orientation.set_col(0, vec3(-1, 0, 0));
	rgbd_2_controller_orientation.set_col(1, vec3(0, -0.7071f, 0.7071f));
	rgbd_2_controller_orientation.set_col(2, vec3(0, 0.7071f, 0.7071f));*/
	rgbd_2_controller_position.zeros();

	rgbd_2_controller_orientation_start_calib.identity();
	rgbd_2_controller_position_start_calib.zeros();

	build_scene(12, 12, 3, 0.2f, 1.6f, 0.8f, 0.9f, 0.03f);
	vr_view_ptr = 0;
	ray_length = 2;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_rgbd::on_device_change);

	srs.radius = 0.005f;
	state[0] = state[1] = state[2] = state[3] = IS_NONE;
	rgbd_started = false;
	record_frame = false;
	record_all_frames = false;
	clear_all_frames = false;
	trigger_is_pressed = false;
	update_voxel = false;
	recording_fps = 5;
	show_points = true;
	point_style.point_size = 2;
	point_style.blend_points = false;
	point_style.blend_width_in_pixel = 0;
	max_nr_shown_recorded_pcs = 20;
	counter_pc = 0;

	connect(cgv::gui::get_animation_trigger().shoot, this, &vr_rgbd::timer_event);

	// yzy, set font style
	cgv::media::font::enumerate_font_names(font_names);
	font_enum_decl = "enums='";
	for (unsigned i = 0; i < font_names.size(); ++i) {
		if (i > 0)
			font_enum_decl += ";";
		std::string fn(font_names[i]);
		if (cgv::utils::to_lower(fn) == "calibri") {
			label_font_face = cgv::media::font::find_font(fn)->get_font_face(label_face_type);
			//label_font_idx = i;
			for (auto& btn : boxguibtns) {
				if (btn.use_label)
					btn.labeltex->label_font_idx = i; // why i? pp
			}
		}
		font_enum_decl += std::string(fn);
	}
	font_enum_decl += "'";
	label_face_type = cgv::media::font::FFA_BOLD;
	label_font_face = cgv::media::font::find_font(font_names[0])->get_font_face(label_face_type);

	//// openmesh integreation test 
	//{
	//	typedef OpenMesh::PolyMesh_ArrayKernelT<>  MyMesh;
	//	MyMesh mesh;
	//	// generate vertices
	//	MyMesh::VertexHandle vhandle[8];
	//	vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1, 1));
	//	vhandle[1] = mesh.add_vertex(MyMesh::Point(1, -1, 1));
	//	vhandle[2] = mesh.add_vertex(MyMesh::Point(1, 1, 1));
	//	vhandle[3] = mesh.add_vertex(MyMesh::Point(-1, 1, 1));
	//	vhandle[4] = mesh.add_vertex(MyMesh::Point(-1, -1, -1));
	//	vhandle[5] = mesh.add_vertex(MyMesh::Point(1, -1, -1));
	//	vhandle[6] = mesh.add_vertex(MyMesh::Point(1, 1, -1));
	//	vhandle[7] = mesh.add_vertex(MyMesh::Point(-1, 1, -1));
	//	// generate (quadrilateral) faces
	//	std::vector<MyMesh::VertexHandle>  face_vhandles;
	//	face_vhandles.clear();
	//	face_vhandles.push_back(vhandle[0]);
	//	face_vhandles.push_back(vhandle[1]);
	//	face_vhandles.push_back(vhandle[2]);
	//	face_vhandles.push_back(vhandle[3]);
	//	mesh.add_face(face_vhandles);

	//	face_vhandles.clear();
	//	face_vhandles.push_back(vhandle[7]);
	//	face_vhandles.push_back(vhandle[6]);
	//	face_vhandles.push_back(vhandle[5]);
	//	face_vhandles.push_back(vhandle[4]);
	//	mesh.add_face(face_vhandles);
	//	face_vhandles.clear();
	//	face_vhandles.push_back(vhandle[1]);
	//	face_vhandles.push_back(vhandle[0]);
	//	face_vhandles.push_back(vhandle[4]);
	//	face_vhandles.push_back(vhandle[5]);
	//	mesh.add_face(face_vhandles);
	//	face_vhandles.clear();
	//	face_vhandles.push_back(vhandle[2]);
	//	face_vhandles.push_back(vhandle[1]);
	//	face_vhandles.push_back(vhandle[5]);
	//	face_vhandles.push_back(vhandle[6]);
	//	mesh.add_face(face_vhandles);
	//	face_vhandles.clear();
	//	face_vhandles.push_back(vhandle[3]);
	//	face_vhandles.push_back(vhandle[2]);
	//	face_vhandles.push_back(vhandle[6]);
	//	face_vhandles.push_back(vhandle[7]);
	//	mesh.add_face(face_vhandles);
	//	face_vhandles.clear();
	//	face_vhandles.push_back(vhandle[0]);
	//	face_vhandles.push_back(vhandle[3]);
	//	face_vhandles.push_back(vhandle[7]);
	//	face_vhandles.push_back(vhandle[4]);
	//	mesh.add_face(face_vhandles);

	//	OpenMesh::IO::write_mesh(mesh, "output.off");
	//}

	// define grid information, initialize cell info 
	resx = 200;
	for (int i = 0; i < resx; i++) {
		for (int j = 0; j < resx; j++) {
			for (int k = 0; k < resx; k++) {
				cellpoint p;
				p.accu_tsdf = 0;
				p.num_points_used_to_updated = 0;
				cell_data.push_back(p);
			}
		}
	}
	minp = dvec3(-5.0f / 2, 0, -7.0f / 2);
	cell_scaling = dvec3(5.0f, 3.0f, 7.0f);
	cell_scaling(0) /= (resx - 1); 
	cell_scaling(1) /= (resx - 1); 
	cell_scaling(2) /= (resx - 1);

	max_negative = -cell_scaling(2);
	max_positive = cell_scaling(2);

	//point_cloud_interactable* pc_inter = new point_cloud_interactable();
	//register_object(base_ptr(pc_inter), "");
}
///
void vr_rgbd::update_voxel_by_current_pc() {
	const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
	vec3 ray_origin, ray_direction;
	state_ptr->controller[0].put_ray(&ray_origin(0), &ray_direction(0));
	vec3 rgbd_posi = ray_origin + rgbd_2_controller_position;

	#define Epslion 1e-8

	for (auto v : current_pc) {
		int i = floor((v.point.x() - minp.x()) / cell_scaling.x());
		int j = floor((v.point.y() - minp.y()) / cell_scaling.y());
		int k = floor((v.point.z() - minp.z()) / cell_scaling.z());

		// compute 8 neighbors for a cell point 
		vec3 p0 = vec3(minp.x() + (i + 0) * cell_scaling.x(),
			minp.y() + (j + 0) * cell_scaling.y(),
			minp.z() + (k + 0) * cell_scaling.z());
		vec3 p1 = vec3(minp.x() + (i + 0) * cell_scaling.x(),
			minp.y() + (j + 0) * cell_scaling.y(),
			minp.z() + (k + 1) * cell_scaling.z());
		vec3 p2 = vec3(minp.x() + (i + 0) * cell_scaling.x(),
			minp.y() + (j + 1) * cell_scaling.y(),
			minp.z() + (k + 0) * cell_scaling.z());
		vec3 p3 = vec3(minp.x() + (i + 0) * cell_scaling.x(),
			minp.y() + (j + 1) * cell_scaling.y(),
			minp.z() + (k + 1) * cell_scaling.z());

		vec3 p4 = vec3(minp.x() + (i + 1) * cell_scaling.x(),
			minp.y() + (j + 0) * cell_scaling.y(),
			minp.z() + (k + 0) * cell_scaling.z());
		vec3 p5 = vec3(minp.x() + (i + 1) * cell_scaling.x(),
			minp.y() + (j + 0) * cell_scaling.y(),
			minp.z() + (k + 1) * cell_scaling.z());
		vec3 p6 = vec3(minp.x() + (i + 1) * cell_scaling.x(),
			minp.y() + (j + 1) * cell_scaling.y(),
			minp.z() + (k + 0) * cell_scaling.z());
		vec3 p7 = vec3(minp.x() + (i + 1) * cell_scaling.x(),
			minp.y() + (j + 1) * cell_scaling.y(),
			minp.z() + (k + 1) * cell_scaling.z());

		// compute distance 
		double dist0 = (v.point - p0).length();
		double dist1 = (v.point - p1).length();
		double dist2 = (v.point - p2).length();
		double dist3 = (v.point - p3).length();

		double dist4 = (v.point - p4).length();
		double dist5 = (v.point - p5).length();
		double dist6 = (v.point - p6).length();
		double dist7 = (v.point - p7).length();

		// compute sign 
		double dist_pc2cam = (v.point - rgbd_posi).length();
		double cell2cam_0 = (p0 - rgbd_posi).length();
		double cell2cam_1 = (p1 - rgbd_posi).length();
		double cell2cam_2 = (p2 - rgbd_posi).length();
		double cell2cam_3 = (p3 - rgbd_posi).length();
		double cell2cam_4 = (p4 - rgbd_posi).length();
		double cell2cam_5 = (p5 - rgbd_posi).length();
		double cell2cam_6 = (p6 - rgbd_posi).length();
		double cell2cam_7 = (p7 - rgbd_posi).length();

		int sign0 = -1, sign1 = -1, sign2 = -1, sign3 = -1,
			sign4 = -1, sign5 = -1, sign6 = -1, sign7 = -1;
		if (cell2cam_0 < dist_pc2cam) { sign0 = 1; }
		if (cell2cam_1 < dist_pc2cam) { sign1 = 1; }
		if (cell2cam_2 < dist_pc2cam) { sign2 = 1; }
		if (cell2cam_3 < dist_pc2cam) { sign3 = 1; }
		if (cell2cam_4 < dist_pc2cam) { sign4 = 1; }
		if (cell2cam_5 < dist_pc2cam) { sign5 = 1; }
		if (cell2cam_6 < dist_pc2cam) { sign6 = 1; }
		if (cell2cam_7 < dist_pc2cam) { sign7 = 1; }

		// make sure that at least one cell point is positive
		if (sign0 < 0 && sign1 < 0 && sign2 < 0 && sign3 < 0
			&& sign4 < 0 && sign5 < 0 && sign6 < 0 && sign7 < 0)
		{
			double minimal_dist = cell2cam_0;
			if (cell2cam_1 < minimal_dist) minimal_dist = cell2cam_1;
			if (cell2cam_2 < minimal_dist) minimal_dist = cell2cam_2;
			if (cell2cam_3 < minimal_dist) minimal_dist = cell2cam_3;
			if (cell2cam_4 < minimal_dist) minimal_dist = cell2cam_4;
			if (cell2cam_5 < minimal_dist) minimal_dist = cell2cam_5;
			if (cell2cam_6 < minimal_dist) minimal_dist = cell2cam_6;
			if (cell2cam_7 < minimal_dist) minimal_dist = cell2cam_7;

			if (abs(cell2cam_0 - minimal_dist) < Epslion) sign0 = 1;
			if (abs(cell2cam_1 - minimal_dist) < Epslion) sign1 = 1;
			if (abs(cell2cam_2 - minimal_dist) < Epslion) sign2 = 1;
			if (abs(cell2cam_3 - minimal_dist) < Epslion) sign3 = 1;
			if (abs(cell2cam_4 - minimal_dist) < Epslion) sign4 = 1;
			if (abs(cell2cam_5 - minimal_dist) < Epslion) sign5 = 1;
			if (abs(cell2cam_6 - minimal_dist) < Epslion) sign6 = 1;
			if (abs(cell2cam_7 - minimal_dist) < Epslion) sign7 = 1;
		}

		double signeddist[8];
		signeddist[0] = sign0 * dist0;
		signeddist[1] = sign1 * dist1;
		signeddist[2] = sign2 * dist2;
		signeddist[3] = sign3 * dist3;

		signeddist[4] = sign4 * dist4;
		signeddist[5] = sign5 * dist5;
		signeddist[6] = sign6 * dist6;
		signeddist[7] = sign7 * dist7;

		// iterate all 8 cell points 
		for (int l = 0; l < 2; l++) {
			for (int m = 0; m < 2; m++) {
				for (int n = 0; n < 2; n++) {
					if ((i + l) < resx && (j + m) < resx && (k + n) < resx) {
						// p is the current cell point to be updated, rw operation
						cellpoint p = cell_data.at((i + l) + resx * (j + m) + resx * resx * (k + n));
						p.num_points_used_to_updated++;
						p.accu_tsdf += signeddist[n + 2 * m + 2 * 2 * l];
						//if(p.num_points_used_to_updated>1)
						//	if(p.accu_tsdf * computedval>0)// the same sign 
						//		p.accu_tsdf += computedval; // overwrite or accumulate
						//	else 
						//		p.num_points_used_to_updated--;
						cell_data.at((i + l) + resx * (j + m) + resx * resx * (k + n)) = p;
					}
				}
			}
		}
				
	}
}
///
void vr_rgbd::save_voxel_cell_data() {
	std::string fn = "Z:/rgbddatastream/output.vox";
	std::string hd_fn = cgv::utils::file::drop_extension(fn) + ".hd";
	std::ofstream os(hd_fn.c_str());
	if (os.fail())
		return;
	os << resx << " " << cell_scaling(0) << " " 
		<< cell_scaling(1) << " " << cell_scaling(2) << " " << max_negative << " " << max_positive;
	os.close();
	// calculate avg value for each cellpoint, post processing, simple avg for each shot
	cell_data_export.reserve(resx * resx * resx);
	for (int k = 0; k < resx; k++) {
		for (int j = 0; j < resx; j++) {
			for (int i = 0; i < resx; i++) {
				cellpoint p = cell_data.at(i + resx * j + resx * resx * k);
				if (p.num_points_used_to_updated > 1) {
					p.accu_tsdf = p.accu_tsdf / p.num_points_used_to_updated;
					//cell_data.at(i + resx * j + resx * resx * k) = p;
				}
				//signeddist[n+2*m+2*2*l] cast to uchar for data export 
				unsigned char tsdf_uchar;
				if (p.num_points_used_to_updated > 0)
					tsdf_uchar = (p.accu_tsdf - max_negative) / (max_positive - max_negative) * 254 + 1;
				else
					tsdf_uchar = 0;// stop value
				cell_data_export.push_back(tsdf_uchar);
			}
		}
	}
	cgv::utils::file::write(fn, (const char*)&cell_data_export.front(), cell_data_export.size());
}
///
void vr_rgbd::gen_voxel_from_pc() {
	update_voxel_by_current_pc();
	save_voxel_cell_data();
	//dvec3 d = dvec3(5.0f, 3.0f, 7.0f);
	//d(0) /= (resx - 1); d(1) /= (resx - 1); d(2) /= (resx - 1);
	//dvec3 p = dvec3(-5.0f / 2,  0, -7.0f / 2);
	//dvec3 minp = p;
	//std::string fn = "Z:/rgbddatastream/output.vox";
	//std::string hd_fn = cgv::utils::file::drop_extension(fn) + ".hd";
	//std::ofstream os(hd_fn.c_str());
	//if (os.fail())
	//	return;
	//dvec3 scaling = d; // dvec3(res, res, res);
	//os << resx << " " << scaling(0) << " " << scaling(1) << " " << scaling(2) << " " << 0 << " " << 1;
	//os.close();
	// iterate all cells 
	//int i = 0, j = 0, k = 0;
	//for (k = 0; k < res; ++k, p(2) += d(2)) {
	//	for (j = 0, p(1) = minp(1); j < res; ++j, p(1) += d(1)) {
	//		for (i = 0, p(0) = minp(0); i < res; ++i, p(0) += d(0)) {
	//			//unsigned char value;
	//			float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	//			if (r > 0.8f)
	//				data.at(i + res * j + res * res * k) = 255;
	//			else
	//				data.at(i + res * j + res * res * k) = 0;
	//			//data.push_back(value);
	//			//data.at(i + res * j + res * res * k) = 0;
	//		}
	//	}
	//}
	// iterate all points in pc
	/*for (auto v : current_pc) {
		int i = round((v.point.x() - minp.x()) / d.x());
		int j = round((v.point.y() - minp.y()) / d.y());
		int k = round((v.point.z() - minp.z()) / d.z());
		if (i < resx && j < resx && k < resx)
			cell_data.at(i + resx * j + resx * resx * k) = 255;
	}
	cgv::utils::file::write(fn, (const char*)&cell_data.front(), cell_data.size());*/
}
///
size_t vr_rgbd::construct_point_cloud()
{
	intermediate_pc.clear();
	// color_frame_2 is the frame after calibration or just a copy of original data
	const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2.frame_data.front());
	const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frame_2.frame_data.front());
	
	rgbd_inp.map_color_to_depth(depth_frame_2, color_frame_2, warped_color_frame_2);
	colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2.frame_data.front());

	int i = 0;
	for (int y = 0; y < depth_frame_2.height; ++y)
		for (int x = 0; x < depth_frame_2.width; ++x) {
			vec3 p;
			if (rgbd_inp.map_depth_to_point(x, y, depths[i], &p[0])) {
				// flipping y to make it the same direction as in pixel y coordinate
				p = -p;
				p = rgbd_2_controller_orientation * p + rgbd_2_controller_position;
				p = controller_orientation_pc * p + controller_position_pc;
				rgba8 c(colors[4 * i + 2], colors[4 * i + 1], colors[4 * i], 255);
				vertex v;
				v.point = p; 
				v.color = c;
				intermediate_pc.push_back(v);
			}
			++i;
		}
	return intermediate_pc.size();
}
/// 
rgbd::frame_type vr_rgbd::read_rgb_frame()    //should be a thread
{
	return color_frame;
}
/// 
rgbd::frame_type vr_rgbd::read_depth_frame()
{
	return depth_frame;
}
/// cast vertex to point_cloud
void vr_rgbd::copy_pointcloud(const std::vector<vertex> input, point_cloud &output){
	for (unsigned int i = 0; i < input.size(); i++){
		point_cloud_types::Pnt temp;
		temp[0] = input.at(i).point[0];
		temp[1] = input.at(i).point[1];
		temp[2] = input.at(i).point[2];
		point_cloud_types::Clr tempcolor;
		tempcolor[0] = input.at(i).color[0];
		tempcolor[1] = input.at(i).color[1];
		tempcolor[2] = input.at(i).color[2];
		output.P.push_back(temp);
		output.C.push_back(tempcolor);
	}
}
/// cast point_cloud to vertex
void vr_rgbd::pc2vertex(const point_cloud &input, std::vector<vertex> &output) {
	for (unsigned int i = 0; i < input.get_nr_points(); i++) {
		vertex temp;
		temp.point[0] = input.pnt(i).x();
		temp.point[1] = input.pnt(i).y();
		temp.point[2] = input.pnt(i).z();
		temp.color[0] = input.clr(i)[0];
		temp.color[1] = input.clr(i)[1];
		temp.color[2] = input.clr(i)[2];
		/*temp.color[0] = 0.5;
		temp.color[1] = 0.0;
		temp.color[2] = 0.0;*/
		output.push_back(temp);
	}
}
/// here should be const point cloud
void vr_rgbd::write_pcs_to_disk(int i)
{
	if (!intermediate_pc.empty())
	{
		//define point cloud type, wirte to disk
		point_cloud *pc_save = new point_cloud();
		pc_save->has_clrs = true;
		copy_pointcloud(intermediate_pc, *pc_save);
		//std::cout << std::to_string(i) << std::endl;
		///pathname
		auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();

		std::string filename = data_dir + "\\object_scanns\\pointcloud_" + std::to_string(microsecondsUTC) + ".obj";
		std::cout << filename << std::endl;
		pc_save->write(filename);
	}
}
///
void vr_rgbd::read_next_pc()
{
	if (!pointcloud_reading_path.empty())
	{
		cur_index_of_pc++;

		int num_pcs = 0;
		std::ifstream fin;
		fin.open(pointcloud_reading_path + "/config.txt");
		if (!fin.good())
			return;
		try {
			while (!fin.eof())
			{
				char buf[CHARS_PER_LINE];
				fin.getline(buf, CHARS_PER_LINE);
				std::string str(buf);
				str = trim(str); //remove whitespaces

				if (str.empty())
					continue;
				if (str.find('//') == 0)
					continue; //don't handle comments
				if (str.find(':') == 0)
					continue; //don't handle control sequences

				if (str._Equal("#num_of_pcs")) {
					fin.getline(buf, CHARS_PER_LINE);
					std::string content(buf);
					num_pcs = stoi(content);
				}

				fin.close();
			}
		}
		catch (...)
		{
			fin.close();
			return;
		}
		if (cur_index_of_pc <= num_pcs && cur_index_of_pc > 0) {
			point_cloud* pc_read = new point_cloud();
			std::string pc_file_name = pointcloud_reading_path + "/pointcloud_" + std::to_string(cur_index_of_pc) + ".obj";
			pc_read->read(pc_file_name);
			intermediate_pc.clear();
			pc2vertex(*pc_read, intermediate_pc);
			loaded_pcs.push_back(intermediate_pc);
			intermediate_pc.clear();
		}
		else if (cur_index_of_pc > num_pcs) {
			cur_index_of_pc = 0; // reset cur_index_of_pc 
			log_error("cur_index_of_pc out of range!");
		}
		log_cur_num_pc();
	}
}
///
void vr_rgbd::del_last_pc()
{
	if (loaded_pcs.size()) {
		loaded_pcs.pop_back();
		cur_index_of_pc--;
	}
}
///
void vr_rgbd::log_cur_num_pc()
{
	// log 
	if (boxguibtns.size()) {
		boxguibtns.at(0).front_append_info_to_label("@cur_index_of_pc = " + std::to_string(cur_index_of_pc));
		boxguibtns.at(0).front_append_info_to_label("@loaded_pcs = " + std::to_string(loaded_pcs.size()));
		boxguibtns.at(0).labeltex->label_outofdate = true;
		post_redraw();
	}
}
///
void vr_rgbd::log_error(std::string e) {
	if (boxguibtns.size()) {
		boxguibtns.at(0).front_append_info_to_label(e);
		boxguibtns.at(0).labeltex->label_outofdate = true;
		post_redraw();
	}
}
///
void vr_rgbd::read_pc() {
	/*point_cloud* pc_read = new point_cloud(); 
	pc_read->read(pc_file_name);
	pc2vertex(*pc_read, current_pc);*/

	std::string pc_file_name = cgv::gui::file_open_dialog("Open", "Point Cloud:*");
	if(pc_drawable)
		pc_drawable->read(pc_file_name);
}
///
void vr_rgbd::upper_pc() {
	for (auto& pp : current_pc){
		pp.point += vec3(0, 0.1, 0);
	}
	post_redraw();
}
///
void vr_rgbd::lower_pc() {
	for (auto& pp : current_pc) {
		pp.point -= vec3(0, 0.1, 0);
	}
	post_redraw();
}
///
void vr_rgbd::read_properties() {
	if (!pointcloud_reading_path.empty())
	{
		int num_pcs = 0;
		std::ifstream fin;
		fin.open(pointcloud_reading_path + "/config.txt");
		if (!fin.good())
			return;
		try {
			while (!fin.eof())
			{
				char buf[CHARS_PER_LINE];
				fin.getline(buf, CHARS_PER_LINE);
				std::string str(buf);
				str = trim(str); //remove whitespaces

				if (str.empty())
					continue;
				if (str.find('//') == 0)
					continue; //don't handle comments
				if (str.find(':') == 0)
					continue; //don't handle control sequences

				if (str._Equal("#num_of_pcs")) {
					fin.getline(buf, CHARS_PER_LINE);
					std::string content(buf);
					num_pcs = stoi(content);
				}

				fin.close();
			}
		}
		catch (...)
		{
			fin.close();
			return;
		}
		// log 
		if (boxguibtns.size()) {
			boxguibtns.at(0).front_append_info_to_label("#num_of_pcs = " + std::to_string(num_pcs));
			boxguibtns.at(0).labeltex->label_outofdate = true;
			post_redraw();
		}
	}
}
/// 
void vr_rgbd::read_pc_queue()
{
	//std::string filename = cgv::gui::file_open_dialog("Open", ".");
	if (!pointcloud_reading_path.empty())
	{
		int num_pcs = 0;
		std::ifstream fin;
		fin.open(pointcloud_reading_path + "/config.txt");
		if (!fin.good())
			return;
		try {
			while (!fin.eof())
			{
				char buf[CHARS_PER_LINE];
				fin.getline(buf, CHARS_PER_LINE);
				std::string str(buf);
				str = trim(str); //remove whitespaces

				if (str.empty())
					continue;
				if (str.find('//') == 0)
					continue; //don't handle comments
				if (str.find(':') == 0)
					continue; //don't handle control sequences

				if (str._Equal("#num_of_pcs")) {
					fin.getline(buf, CHARS_PER_LINE);
					std::string content(buf);
					num_pcs = stoi(content);
				}

				fin.close();
			}
		}
		catch (...)
		{
			fin.close();
			return;
		}
		// read all pcs from current dir
		point_cloud* pc_read = new point_cloud();
		std::string pc_file_name = "";
		for (int i = 1; i < num_pcs + 1; i++) {
			pc_file_name = pointcloud_reading_path + "/pointcloud_" + std::to_string(i) + ".obj";
			pc_read->read(pc_file_name);
			pc2vertex(*pc_read, current_pc);
		}
		post_redraw();
	}
}
/// 
void vr_rgbd::registrationPointCloud() {
	cgv::pointcloud::ICP* icp = new cgv::pointcloud::ICP();
	if (recorded_pcs.size() >= 1) {
		cgv::math::fmat<float, 3, 3> r;
		cgv::math::fvec<float, 3> t;
		r.identity();
		t.zeros();
		point_cloud *sourcePC = new point_cloud();
		point_cloud* sourcecopy = new point_cloud();
		point_cloud *targetPC = new point_cloud();
		sourcePC->resize(intermediate_pc.size());
		targetPC->resize(recorded_pcs.front().size());
		sourcecopy->resize(intermediate_pc.size());
		copy_pointcloud(recorded_pcs.front(), *targetPC);
		copy_pointcloud(intermediate_pc, *sourcePC);
		icp->set_source_cloud(*sourcePC);
		icp->set_target_cloud(*targetPC);
		icp->set_iterations(5);
		icp->set_eps(1e-10);
		icp->reg_icp(r, t);
		for (int i = 0; i < sourcePC->get_nr_points(); i++)
		{
			sourcePC->pnt(i) = r * sourcePC->pnt(i) + t;
		}
		intermediate_pc.clear();
		pc2vertex(*sourcePC, intermediate_pc);
		std::cout << "size: " << recorded_pcs.size() << " "<< intermediate_pc.size()<<std::endl;
	}		
}
///
void vr_rgbd::generate_rdm_pc(point_cloud &pc1, point_cloud& pc2) {
	mat3 rotate_m;
	rotate_m.identity();
	double theta = M_PI / 8;  // The angle of rotation in radians
	rotate_m.set_col(0, vec3(std::cos(theta), -sin(theta), 0));
	rotate_m.set_col(1, vec3(sin(theta), std::cos(theta), 0));
	rotate_m.set_col(2, vec3(0, 0, 1));
	for (int i = 0; i < 10000; i++) {
		point_cloud_types::Pnt origin;
		origin.zeros();
		origin.x() = 1024 * rand() / (RAND_MAX + 1.0f);
		origin.y() = 1024 * rand() / (RAND_MAX + 1.0f);
		origin.z() = 1024 * rand() / (RAND_MAX + 1.0f);
		pc1.pnt(i) = origin;
		origin = rotate_m * origin;
		pc2.pnt(i) = origin + vec3(0.0, 0.4, 0.4);
	}
}
///
void vr_rgbd::test_icp_gen() {
	sourcePC = new point_cloud();
	targetPC = new point_cloud();
	sourcePC->resize(10000);
	targetPC->resize(10000);
	generate_rdm_pc(*sourcePC, *targetPC);
}
///
void vr_rgbd::test_icp() {
	cgv::pointcloud::ICP* icp = new cgv::pointcloud::ICP();
	cgv::math::fmat<float, 3, 3> r;
	cgv::math::fvec<float, 3> t;
	r.identity();
	t.zeros();
	icp->set_source_cloud(*sourcePC);
	icp->set_target_cloud(*targetPC);
	//icp->set_source_cloud(*targetPC);
	//icp->set_target_cloud(*sourcePC);
	icp->set_iterations(5);
	icp->set_num_random(3);
	icp->set_eps(1e-10);
	icp->reg_icp(r, t);
}
///
void vr_rgbd::construct_TSDtree()
{
	//using pc queue to construct the TSDtree
}
///
bool vr_rgbd::record_this_frame(double t)
{
	if (!(record_frame || record_all_frames || trigger_is_pressed))
		return false;
	static double last_recording_time = -1;
	if (t - last_recording_time < 1.0 / recording_fps)
		return false;
	last_recording_time = t;
	trigger_is_pressed = false; // reset the boolean var. 
	return true;
}
///
void vr_rgbd::timer_event(double t, double dt)
{
	// in case a point cloud is being constructed
	if (future_handle.valid()) {
		// check for termination of thread
		if (future_handle.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
			size_t N = future_handle.get();
			// copy computed point cloud
			if (record_this_frame(t)) {
				if (registration_started) {
					//registrationPointCloud();
					test_icp();
				}
				recorded_pcs.push_back(intermediate_pc);
				if (save_pointcloud){ // check to save
					counter_pc++;
					write_pcs_to_disk(counter_pc);
				}
				current_pc.clear();
				record_frame = false;
				update_member(&record_frame);
			}
			else if(clear_all_frames){
				recorded_pcs.clear();
				clear_all_frames = false;
				update_member(&clear_all_frames);
			}
			else
				current_pc = intermediate_pc;
			if (update_voxel) {//update_voxel
				update_voxel_by_current_pc();
			}
			post_redraw();
		}
	}
	if (rgbd_inp.is_started()) {
		if (rgbd_inp.is_started()) {
			bool new_frame;
			bool found_frame = false;
			bool depth_frame_changed = false;
			bool color_frame_changed = false;
			do {
				new_frame = false;
				bool new_color_frame_changed = rgbd_inp.get_frame(rgbd::IS_COLOR, color_frame, 0);
				if (new_color_frame_changed) {
					++nr_color_frames;
					color_frame_changed = new_color_frame_changed;
					new_frame = true;
					update_member(&nr_color_frames);
				}
				bool new_depth_frame_changed = rgbd_inp.get_frame(rgbd::IS_DEPTH, depth_frame, 0);
				if (new_depth_frame_changed) {
					++nr_depth_frames;
					depth_frame_changed = new_depth_frame_changed;
					new_frame = true;
					update_member(&nr_depth_frames);
				}
				if (new_frame)
					found_frame = true;
			} while (new_frame);
			if (found_frame)
				post_redraw();
			if (color_frame.is_allocated() && depth_frame.is_allocated() &&
				(color_frame_changed || depth_frame_changed)) {
					
				if (!future_handle.valid()) { 
					if (!in_calibration) {
						color_frame_2 = color_frame;
						depth_frame_2 = depth_frame;
					}
					if (zoom_out && !zoom_in)
					{
						controller_orientation_pc = controller_orientation * 2;
						controller_position_pc = controller_position;
					}
					else if(zoom_in && !zoom_out)
					{
						controller_orientation_pc = controller_orientation * 0.5;
						controller_position_pc = controller_position;
					}
					else {
						// moved to above 
						controller_orientation_pc = controller_orientation;
						controller_position_pc = controller_position;
					}
					future_handle = std::async(&vr_rgbd::construct_point_cloud, this);
				}
			}
		}
	}
}
///
std::string vr_rgbd::get_type_name() const
{
	return "vr_rgbd";
}
///
void vr_rgbd::load_mesh() {
	//mmesh->set_orientation_translation(cgv::math::rotate3<double>(0.0f, vec3(0, 1, 0)), vec3(0, 1, 0));
	//mmesh->set_mesh_scale(0.02);
	std::string filename = cgv::gui::file_open_dialog("Open", "Mesh Files (*.obj):*.obj");
	//std::string mesh_dir = data_dir + "/gen_dataset/speider_simple0/spiderman.obj";
	mmesh->read_obj(filename.c_str());

	post_redraw();
}
///
void vr_rgbd::zr_calibration_init() {
	rgbd_2_controller_orientation_start_calib = controller_orientation; // V^0 = V
	rgbd_2_controller_position_start_calib = controller_position;       // r^0 = r
	in_calibration = true;
	update_member(&in_calibration);
}
///
void vr_rgbd::zr_calibration_do() {
	rgbd_2_controller_orientation = cgv::math::inv(controller_orientation) * 
		rgbd_2_controller_orientation_start_calib * rgbd_2_controller_orientation; 
	rgbd_2_controller_position = cgv::math::inv(controller_orientation) * 
		(rgbd_2_controller_orientation_start_calib * rgbd_2_controller_position + 
			rgbd_2_controller_position_start_calib - controller_position);
	in_calibration = false;
	update_member(&in_calibration);
}
///
void vr_rgbd::calibration_realworld_init() {
	rgbd_2_controller_orientation_start_calib = controller_orientation; // V^0 = V
	rgbd_2_controller_position_start_calib = controller_position;       // r^0 = r
	in_calibration = true;
	update_member(&in_calibration);
}
///
void vr_rgbd::calibration_realworld_do() {
	rgbd_2_controller_orientation = transpose(rgbd_2_controller_orientation_start_calib) * controller_orientation * rgbd_2_controller_orientation;
	rgbd_2_controller_position = transpose(rgbd_2_controller_orientation_start_calib) * ((controller_orientation * rgbd_2_controller_position + controller_position) - rgbd_2_controller_position_start_calib);
	in_calibration = false;
	update_member(&in_calibration);
}
///
void vr_rgbd::stream_help(std::ostream& os)
{
	os << "vr_rgbd: no shortcuts defined" << std::endl;
}
cgv::math::fvec<float, 3> vr_rgbd::compute_ray_plane_intersection_point(const vec3& origin, const vec3& direction)
{
	float t_result;
	vec3  p_result = vec3(0);
	vec3  n_result;
	box3 floor = box3(vec3(-100, -1, -100), vec3(100, 0, 100));
	if (cgv::media::ray_axis_aligned_box_intersection(
		origin, direction,
		floor,
		t_result, p_result, n_result, 0.000001f)) {
		return p_result;
	}

	return p_result;
}
///
void vr_rgbd::construct_handheld_gui() {
	// clear all related vectors 
	objpicklist.clear();
	objpicklist_colors.clear();
	objpicklist_translations.clear();
	objpicklist_rotations.clear();

	boxguibtns.clear();

	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);

	float tmpboxsize = 0.02f * obj_scale_factor;
	vec3 extent(tmpboxsize);
	vec3 demoposi = vec3(0, 0, -0.2f);

	std::vector<std::string> label_list;
	std::vector<vec3> local_coordi_list;

	// info panel
	boxgui_button info_button =
		boxgui_button(
			vec3(1.2, 1.2, 0.1),
			vec3(0, -1, -0.4) + vec3(0, 0.2, 0), // offset 
			quat(vec3(1, 0, 0), -45),
			rgb(0.2, 0.4, 0.2),
			1
		);
	info_button.set_static();
	info_button.set_label("info label", 15);
	boxguibtns.push_back(info_button);


	// button group 0
	label_list.push_back("pc_tools");
	label_list.push_back("mesh_\ntools");
	label_list.push_back("semantic\ntools");
	label_list.push_back("interaction\n_tools");
	label_list.push_back("gui_tools"); 
	local_coordi_list.push_back(vec3(-0.3, 0, -0.2));
	local_coordi_list.push_back(vec3(-0.3 * sqrt(2) / 2, 0.3 * sqrt(2) / 2, -0.2));
	local_coordi_list.push_back(vec3(0, 0.3, -0.2));
	local_coordi_list.push_back(vec3(0.3 * sqrt(2) / 2, 0.3 * sqrt(2) / 2, -0.2));
	local_coordi_list.push_back(vec3(0.3, 0, -0.2));
	for (int i = 0; i < label_list.size(); i++) {
		rgb tmpcol = rgb(
			0.4f * distribution(generator) + 0.1f,
			0.4f * distribution(generator) + 0.3f,
			0.4f * distribution(generator) + 0.1f
		);
		// store local coordi frames in button structure 
		boxgui_button tmpbtn = boxgui_button(
			vec3(0.2, 0.2, 0.1),
			local_coordi_list.at(i),
			quat(vec3(0, 1, 0), 0),
			tmpcol,
			0 // they blongs to group 0
		);
		tmpbtn.set_label(label_list.at(i), 50);// 8 char per line 
		boxguibtns.push_back(tmpbtn);
	}
	// clean up 
	label_list.clear();
	local_coordi_list.clear();

	{
		// button group 1 , first pop-up buttons 
		float btn_offset_y = 0.2;
		label_list.push_back("back");
		label_list.push_back("read_properties");
		label_list.push_back("load_next_pc");
		label_list.push_back("load_all_pcs");
		label_list.push_back("del_last_pc");
		label_list.push_back("align_pc_\nwith_last\n_frame");
		/*for (int i = 0; i < 12; i++) {
			label_list.push_back("test");
		}*/
		vec3 start_point = vec3(-0.45, 0.45, -1);
		local_coordi_list.push_back(vec3(start_point.x(), start_point.y() + 0.3, start_point.z()));
		int num_per_line = 4;
		float len = num_per_line * 0.3;
		for (int i = 0; i < num_per_line * num_per_line; i++) {
			float tmp_x = start_point.x() + i * 0.3 - (i / num_per_line) * len;
			float tmp_y = start_point.y() - i / num_per_line * 0.3;
			local_coordi_list.push_back(vec3(tmp_x, tmp_y, start_point.z()));
		}
		for (int i = 0; i < label_list.size(); i++) {
			rgb tmpcol = rgb(
				0.4f * distribution(generator) + 0.1f,
				0.4f * distribution(generator) + 0.3f,
				0.4f * distribution(generator) + 0.1f
			);
			// store local coordi frames in button structure 
			boxgui_button tmpbtn = boxgui_button(
				vec3(0.2, 0.2, 0.1),
				local_coordi_list.at(i) + vec3(0, btn_offset_y, 0),
				quat(vec3(0, 1, 0), 0),
				tmpcol,
				1 // belongs to group 1
			);
			tmpbtn.set_label(label_list.at(i), 50);// 8 char per line 
			boxguibtns.push_back(tmpbtn);
		}
		boxgui_button tmpbtn =
			boxgui_button(
				vec3(1.2, 1.2, 0.1),
				vec3(0, 0, -1.2) + vec3(0, btn_offset_y, 0),
				quat(vec3(0, 1, 0), 0),
				rgb(0.2, 0.4, 0.2),
				1
			);
		tmpbtn.set_static();
		boxguibtns.push_back(tmpbtn);
		// clean up 
		label_list.clear();
		local_coordi_list.clear();
	}

	{
		// button group 2 , first pop-up buttons 
		label_list.push_back("back");
		label_list.push_back("l_up_group2");
		label_list.push_back("r_up");
		label_list.push_back("r_low");
		label_list.push_back("l_low");
		for (int i = 0; i < 12; i++) {
			label_list.push_back("test");
		}
		vec3 start_point = vec3(-0.45, 0.45, -1);
		local_coordi_list.push_back(vec3(start_point.x(), start_point.y() + 0.3, start_point.z()));
		int num_per_line = 4;
		float len = num_per_line * 0.3;
		for (int i = 0; i < num_per_line * num_per_line; i++) {
			float tmp_x = start_point.x() + i * 0.3 - (i / num_per_line) * len;
			float tmp_y = start_point.y() - i / num_per_line * 0.3;
			local_coordi_list.push_back(vec3(tmp_x, tmp_y, start_point.z()));
		}
		for (int i = 0; i < label_list.size(); i++) {
			rgb tmpcol = rgb(
				0.4f * distribution(generator) + 0.1f,
				0.4f * distribution(generator) + 0.3f,
				0.4f * distribution(generator) + 0.1f
			);
			// store local coordi frames in button structure 
			boxgui_button tmpbtn = boxgui_button(
				vec3(0.2, 0.2, 0.1),
				local_coordi_list.at(i),
				quat(vec3(0, 1, 0), 0),
				tmpcol,
				2 // belongs to group 2
			);
			tmpbtn.set_label(label_list.at(i), 50);// 8 char per line 
			boxguibtns.push_back(tmpbtn);
		}
		boxgui_button tmpbtn =
			boxgui_button(
				vec3(1.2, 1.2, 0.1),
				vec3(0, 0, -1.2),
				quat(vec3(0, 1, 0), 0),
				rgb(0.2, 0.4, 0.2),
				2
			);
		tmpbtn.set_static();
		boxguibtns.push_back(tmpbtn);
		// clean up 
		label_list.clear();
		local_coordi_list.clear();
	}

	// colorful box  
	//objpicklist.push_back(box3(-0.5f * extent, 0.5f * extent));
	//objpicklist_colors.push_back(rgb(distribution(generator),
	//	distribution(generator),
	//	distribution(generator)));
	//quat rot(cur_right_hand_rot);
	//rot.normalize();
	//mat3 rot_mat;
	//rot.put_matrix(rot_mat);
	//vec3 addi_posi = rot_mat * demoposi;
	//vec3 modi_posi = cur_right_hand_posi + addi_posi; // addi direction vector should be rotated 
	//objpicklist_translations.push_back(modi_posi);
	//objpicklist_rotations.push_back(rot);
}
///
void vr_rgbd::shuffle_button_group()
{
	if (which_boxgui_group_is_going_to_be_rendered < 2)
		which_boxgui_group_is_going_to_be_rendered++;
	else
		which_boxgui_group_is_going_to_be_rendered = 0;
}
// compute intersection points of controller ray with movable boxes
void vr_rgbd::compute_intersections(const vec3& origin, const vec3& direction, bool moveboxes, bool boxgui)
{
	if (moveboxes)
		for (size_t i = 0; i < movable_boxes.size(); ++i) {
			vec3 origin_box_i = origin - movable_box_translations[i];
			movable_box_rotations[i].inverse_rotate(origin_box_i);
			vec3 direction_box_i = direction;
			movable_box_rotations[i].inverse_rotate(direction_box_i);
			float t_result;
			vec3  p_result;
			vec3  n_result;
			if (cgv::media::ray_axis_aligned_box_intersection(
				origin_box_i, direction_box_i,
				movable_boxes[i],
				t_result, p_result, n_result, 0.000001f)) {

				// transform result back to world coordinates
				movable_box_rotations[i].rotate(p_result);
				p_result += movable_box_translations[i];
				movable_box_rotations[i].rotate(n_result);

				// store intersection information
				intersection_points.push_back(p_result);
				//intersection_colors.push_back(color);
				intersection_box_indices.push_back((int)i);
				//intersection_controller_indices.push_back(ci);
			}
		}

	if (boxgui)
		// compute intersec. with boxgui boxes 
		for (size_t i = 0; i < boxguibtns.size(); ++i) {
			if (boxguibtns.at(i).group == which_boxgui_group_is_going_to_be_rendered) {
				vec3 origin_box_i = origin - boxguibtns.at(i).trans;
				boxguibtns.at(i).rot.inverse_rotate(origin_box_i);
				vec3 direction_box_i = direction;
				boxguibtns.at(i).rot.inverse_rotate(direction_box_i);
				float t_result;
				vec3  p_result;
				vec3  n_result;
				if (cgv::media::ray_axis_aligned_box_intersection(
					origin_box_i, direction_box_i,
					box3(-0.5f * boxguibtns.at(i).ext, 0.5f * boxguibtns.at(i).ext),
					t_result, p_result, n_result, 0.000001f)) {

					// transform result back to world coordinates
					boxguibtns.at(i).rot.rotate(p_result);
					p_result += boxguibtns.at(i).trans;
					boxguibtns.at(i).rot.rotate(n_result);

					intersection_boxgui_indices.push_back((int)i);
					//intersection_boxgui_controller_indices.push_back(ci);
				}
			}
		}
}
/// construct a scene with a table
void vr_rgbd::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	construct_room(w, d, h, W, false, false);
	//construct_table(0.8, 0.02, 0.4, 0.5);
	construct_environment(0.2f, 1.3 * w, 1.3 * d, h, w, d, h);
	//construct_movable_boxes(tw, td, th, tW, 20);
}
/// generate a random point cloud
void vr_rgbd::generate_point_cloud(std::vector<vertex>& pc)
{
	std::default_random_engine r;
	std::uniform_real_distribution<float> d(0.0f, 1.0f);
	vec3 S(0.0f, 2.0f, 0.0f);
	vec3 V(1.0f, 0, 0);
	vec3 U(0.0f, 1.0f, 0);
	vec3 X = cross(V, U);
	float aspect = 1.333f;
	float tan_2 = 0.3f;
	for (int i = 0; i < 10000; ++i) {
		float x = 2 * d(r) - 1;
		float y = 2 * d(r) - 1;
		float z = d(r) + 1;
		vec3  p = x * aspect * tan_2 * z * X + y * tan_2 * z * U + z * V;
		rgba8 c((cgv::type::uint8_type)(255 * d(r)), 0, 0);
		vertex v;
		v.point = S + p;
		v.color = c;
		pc.push_back(v);
	}
}
/// start the rgbd device
void vr_rgbd::start_rgbd()
{
	if (!rgbd_inp.is_attached()) {
		if (rgbd::rgbd_input::get_nr_devices() == 0)
			return;
		if (!rgbd_inp.attach(rgbd::rgbd_input::get_serial(0)))
			return;
	}
	//rgbd_inp.set_near_mode(true);
	std::vector<rgbd::stream_format> stream_formats;
	rgbd_started = rgbd_inp.start(rgbd::IS_COLOR_AND_DEPTH, stream_formats);
	update_member(&rgbd_started);
}
/// stop rgbd device
void vr_rgbd::stop_rgbd()
{
	if (!rgbd_inp.is_started())
		return;
	rgbd_started = rgbd_inp.stop();
	update_member(&rgbd_started);
}
/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_rgbd::construct_table(float w, float m, float ht, float hb)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	rgb table_clr(0.1f, 0.4f, 0.1f);
	// construct table
	/*rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(
		vec3(-0.5f*tw - 2*tW, th, -0.5f*td - 2*tW), 
		vec3( 0.5f*tw + 2*tW, th + tW, 0.5f*td + 2*tW)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f*tw, 0, -0.5f*td), vec3(-0.5f*tw - tW, th, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(-0.5f*tw, 0, 0.5f*td), vec3(-0.5f*tw - tW, th, 0.5f*td + tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, -0.5f*td), vec3(0.5f*tw + tW, th, -0.5f*td - tW)));
	boxes.push_back(box3(vec3(0.5f*tw, 0, 0.5f*td), vec3(0.5f*tw + tW, th, 0.5f*td + tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);*/

	vec3 c_base = vec3(0, hb / 2, 0);
	vec3 ext_base = vec3((w + 3 * m) / 2, hb / 2, (w + 3 * m) / 2);
	vec3 c_t1 = vec3( (m + w / 2), hb + ht / 2, -(m + w / 2));
	vec3 c_t2 = vec3(-(m + w / 2), hb + ht / 2, -(m + w / 2));
	vec3 c_t3 = vec3( (m + w / 2), hb + ht / 2,  (m + w / 2));
	vec3 c_t4 = vec3(-(m + w / 2), hb + ht / 2,  (m + w / 2));
	vec3 ext_t = vec3(w / 2, ht / 2, w / 2);
	boxes.push_back(box3(vec3(c_base - ext_base), vec3(c_base + ext_base)));
	boxes.push_back(box3(vec3(c_t1 - ext_t), vec3(c_t1 + ext_t)));
	boxes.push_back(box3(vec3(c_t2 - ext_t), vec3(c_t2 + ext_t)));
	boxes.push_back(box3(vec3(c_t3 - ext_t), vec3(c_t3 + ext_t)));
	boxes.push_back(box3(vec3(c_t4 - ext_t), vec3(c_t4 + ext_t)));
	box_colors.push_back(rgb(6 / 255.0f, 69 / 255.0f, 6 / 255.0f));
	box_colors.push_back(rgb(6 / 255.0f, 69 / 255.0f, 6 / 255.0f));
	box_colors.push_back(rgb(6 / 255.0f, 69 / 255.0f, 6 / 255.0f));
	box_colors.push_back(rgb(6 / 255.0f, 69 / 255.0f, 6 / 255.0f));
	box_colors.push_back(rgb(6 / 255.0f, 69 / 255.0f, 6 / 255.0f));
	 

}
/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_rgbd::construct_room(float w, float d, float h, float W, bool walls, bool ceiling)
{
	// construct floor
	boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d), vec3(0.5f*w, 0, 0.5f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if (walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f *d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if (ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}
/// construct boxes for environment
void vr_rgbd::construct_environment(float s, float ew, float ed, float eh, float w, float d, float h)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f*ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f*ed;
			if ( (x + s > -0.5f*w && x < 0.5f*w) && (z + s > -0.5f*d && z < 0.5f*d) )
				continue;
			float h = 0.2f*(std::max(abs(x)-0.5f*w,0.0f)+std::max(abs(z)-0.5f*d,0.0f))*distribution(generator)+0.1f;
			boxes.push_back(box3(vec3(x, 0, z), vec3(x+s, h, z+s)));
			box_colors.push_back(
				rgb(
					0.4f * distribution(generator) + 0.1f,
					0.4f * distribution(generator) + 0.3f,
					0.4f * distribution(generator) + 0.1f
				)
			);
		}
	}
}
/// construct boxes that can be moved around
void vr_rgbd::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for (size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.1f;
		extent *= std::min(tw, td)*0.2f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}
/// write posi. list to file 
void vr_rgbd::write_btnposi_to_file() {
	std::ofstream o;
#ifdef _WIN32
	std::wstring wfilename = cgv::utils::str2wstr("posi_btn.txt");
	o.open(wfilename, std::ios::out);
#else
	o.open(filename, std::ios::out);
#endif	
	if (o) {
		for (auto b : boxguibtns) {
			o << b.trans << std::endl;;
		}
	}
	o.close();
}
/// construct boxgui given a list of btn names 
void vr_rgbd::construct_movable_boxgui(std::vector<std::string> llist) {
	// todo- read posi. from file 
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	bool read_from_file = false;

	std::ifstream fin;
#ifdef _WIN32
	std::wstring wfilename = cgv::utils::str2wstr("posi_btn.txt");
	fin.open(wfilename);
#else
	fin.open(filename);
#endif

	if (fin.is_open()) {
		const int CHARS_PER_LINE = 512;
		while (!fin.eof())
		{
			char buf[CHARS_PER_LINE];
			fin.getline(buf, CHARS_PER_LINE);
			std::string str(buf);
			std::stringstream ss(str);
			float tmp_x, tmp_y, tmp_z;
			ss >> tmp_x >> tmp_y >> tmp_z;
			btnposilist.push_back(vec3(tmp_x, tmp_y, tmp_z));
		}
		fin.close();
		read_from_file = true;
	}

	for (int i = 0; i < llist.size(); i++) {
		rgb tmpcol = rgb(
			0.4f * distribution(generator) + 0.1f,
			0.4f * distribution(generator) + 0.3f,
			0.4f * distribution(generator) + 0.1f
		);
		int num_per_line = 13;
		float startposiz = 0;
		float height = 2;
		float len = num_per_line * 0.3;
		vec3 tmpposi;
		if (read_from_file) {
			tmpposi = btnposilist.at(i);
		}
		else {
			tmpposi = vec3(2.5, height - (i / num_per_line) * 0.3, startposiz + i * 0.3 - (i / num_per_line) * len);
			btnposilist.push_back(tmpposi);
		}
		boxgui_button tmpbtn = boxgui_button(
			vec3(0.1, 0.2, 0.2),
			tmpposi,
			quat(vec3(0, 1, 0), 0),
			tmpcol,
			-1
		);
		tmpbtn.set_label(llist.at(i), 50);// 8 char per line 
		boxguibtns.push_back(tmpbtn);
	}
	// write to file 
	if (!read_from_file) {
		write_btnposi_to_file();
	}
}

///
bool vr_rgbd::init(cgv::render::context& ctx)
{
	cgv::render::ref_point_renderer(ctx, 1);

	if (!cgv::utils::has_option("NO_OPENVR"))
		ctx.set_gamma(1.0f);
	cgv::gui::connect_vr_server(true);

	auto view_ptr = find_view_as_node();
	if (view_ptr) {
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);

		}
	}
	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);

	skyprog.build_program(ctx, "skycube.glpr");
	img_tex.create_from_images(ctx, data_dir + "/skybox/cm_{xp,xn,yp,yn,zp,zn}.jpg");
	pointcloud_reading_path = data_dir + "/12.11.2020_tianfangs_office/";

	bool succ = cube_prog.build_program(ctx, "color_cube.glpr", true);

	cgv::render::gl::ensure_glew_initialized();
	mmesh = std::make_shared<SkinningMesh>();
	mmesh->init_shaders(ctx);

	construct_handheld_gui();

	if (pc_drawable)
		pc_drawable->init(ctx);

	return true;
}
///
void vr_rgbd::init_frame(cgv::render::context& ctx)
{
	if (construct_boxgui)
		for (auto& btn : boxguibtns) {
			if (btn.use_label) {
				if (btn.labeltex->label_fbo.get_width() != btn.labeltex->label_resolution) {
					btn.labeltex->label_tex.destruct(ctx);
					btn.labeltex->label_fbo.destruct(ctx);
				}
				if (!btn.labeltex->label_fbo.is_created()) {
					btn.labeltex->label_tex.create(ctx, cgv::render::TT_2D, btn.labeltex->label_resolution, btn.labeltex->label_resolution);
					btn.labeltex->label_fbo.create(ctx, btn.labeltex->label_resolution, btn.labeltex->label_resolution);
					btn.labeltex->label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
					btn.labeltex->label_tex.set_mag_filter(cgv::render::TF_LINEAR);
					btn.labeltex->label_fbo.attach(ctx, btn.labeltex->label_tex);
					btn.labeltex->label_outofdate = true;
				}
				if (btn.labeltex->label_outofdate && btn.labeltex->label_fbo.is_complete(ctx)) {
					glPushAttrib(GL_COLOR_BUFFER_BIT);
					btn.labeltex->label_fbo.enable(ctx);
					btn.labeltex->label_fbo.push_viewport(ctx);
					ctx.push_pixel_coords();
					glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
					glClear(GL_COLOR_BUFFER_BIT);

					glColor4f(btn.labeltex->label_color[0], btn.labeltex->label_color[1], btn.labeltex->label_color[2], 1);
					ctx.set_cursor(20, (int)ceil(btn.labeltex->label_size) + 20);
					ctx.enable_font_face(label_font_face, btn.labeltex->label_size);
					ctx.output_stream() << btn.labeltex->label_text << "\n";
					ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

					ctx.enable_font_face(label_font_face, 0.7f * btn.labeltex->label_size);
					for (size_t i = 0; i < intersection_points.size(); ++i) {
						ctx.output_stream()
							<< "box " << intersection_box_indices[i]
							<< " at (" << intersection_points[i]
							<< ") with controller " << intersection_controller_indices[i] << "\n";
					}
					ctx.output_stream().flush();

					ctx.pop_pixel_coords();
					btn.labeltex->label_fbo.pop_viewport(ctx);
					btn.labeltex->label_fbo.disable(ctx);
					glPopAttrib();
					btn.labeltex->label_outofdate = false;

					btn.labeltex->label_tex.generate_mipmaps(ctx);
				}
			}
		}

	if (pc_drawable)
		pc_drawable->init_frame(ctx);
}
///
void vr_rgbd::clear(cgv::render::context& ctx)
{
	cgv::render::ref_point_renderer(ctx, -1);
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
}
///
void vr_rgbd::draw_pc(cgv::render::context& ctx, const std::vector<vertex>& pc)
{
	if (pc.empty())
		return;
	auto& pr = cgv::render::ref_point_renderer(ctx);
	pr.set_position_array(ctx, &pc.front().point, pc.size(), sizeof(vertex));
	pr.set_color_array(ctx, &pc.front().color, pc.size(), sizeof(vertex));
	if (pr.validate_and_enable(ctx)) {
		glDrawArrays(GL_POINTS, 0, (GLsizei)pc.size());
		pr.disable(ctx);
	}
}
///
void vr_rgbd::draw(cgv::render::context& ctx)
{
	// render the skybox as bkg
	// large enough to contain the whole scene
	float max_scene_extent = 100;
	glDepthMask(GL_FALSE);
	glDisable(GL_CULL_FACE);
	img_tex.enable(ctx, 1);
	skyprog.enable(ctx);
	skyprog.set_uniform(ctx, "img_tex", 1);
	ctx.push_modelview_matrix();
	ctx.mul_modelview_matrix(cgv::math::scale4<double>(
		max_scene_extent, max_scene_extent, max_scene_extent));
	ctx.tesselate_unit_cube();
	ctx.pop_modelview_matrix();
	skyprog.disable(ctx);
	img_tex.disable(ctx);
	glEnable(GL_CULL_FACE);
	glDepthMask(GL_TRUE);

	// render the mesh 
	if (mmesh)
		mmesh->draw(ctx);

	// render the handheld object: the historical ones 
	for (int i = 0; i < scanning_primitive_list.size(); i++) {
		cube_prog.enable(ctx);
		cube_prog.set_uniform(ctx, "mvp", scanning_primitive_list.at(i)->get_mvp());
		switch (scanning_primitive_list.at(i)->get_predef_obj_id()) {
		case 0:
			if (constructive_shaping_mode && scanning_primitive_list.at(i)->get_recorded_posi_list().size() > 0) {
				if ((num_rec_posi % 2 == 1) && (i == (scanning_primitive_list.size() - 1))) {
					// render the first posi and the current one, dynamic 
					ctx.tesselate_box(box3(
						vec3(scanning_primitive_list.at(i)->get_recorded_posi_list().at(0)),
						vec3(objpicklist_translations.at(0))), false, false);
				}
				else {
					// render a fixed object 
					ctx.tesselate_box(box3(
						vec3(scanning_primitive_list.at(i)->get_recorded_posi_list().at(0)),
						vec3(scanning_primitive_list.at(i)->get_recorded_posi_list().at(1))), false, false);
				}
			}
			else
				ctx.tesselate_unit_cube(false, false);
			break;
		case 1:
			if (constructive_shaping_mode) {

			}
			else
				ctx.tesselate_unit_sphere(25, false, false);
			break;
		case 2:
			if (constructive_shaping_mode) {

			}
			else
				ctx.tesselate_unit_prism(false, false);
			break;
		}
		cube_prog.disable(ctx);
	}

	// tesselate + set_uniform style, current muster one
		/*if (objpicklist_rotations.size() > 0) {
			cube_prog.enable(ctx);
			mat4 mvp, R;
			objpicklist_rotations.at(0).put_homogeneous_matrix(R);
			mvp = cgv::math::scale4<double>(0.01 * obj_scale_factor, 0.01 * obj_scale_factor, 0.01 * obj_scale_factor);
			mvp = R * mvp;
			mvp = cgv::math::translate4<double>(objpicklist_translations.at(0)) * mvp;
			cube_prog.set_uniform(ctx, "mvp", mvp);
			switch (predef_obj_id) {
				case 0:
					ctx.tesselate_unit_cube(false, false);
					break;
				case 1:
					ctx.tesselate_unit_sphere(25, false, false);
					break;
				case 2:
					ctx.tesselate_unit_prism(false, false);
					break;
			}
			cube_prog.disable(ctx);
		}*/

		// render the pc
	if (pc_drawable)
		pc_drawable->draw(ctx);

	if (show_points) {
		auto& pr = cgv::render::ref_point_renderer(ctx);
		pr.set_render_style(point_style);
		pr.set_y_view_angle((float)vr_view_ptr->get_y_view_angle());
		draw_pc(ctx, current_pc);

		//
		for (size_t i = 0; i < loaded_pcs.size(); ++i)
			draw_pc(ctx, loaded_pcs[i]);

		//
		size_t begin = 0;
		size_t end = recorded_pcs.size();
		if (end > max_nr_shown_recorded_pcs)
			begin = end - max_nr_shown_recorded_pcs;

		for (size_t i = begin; i < end; ++i)
			draw_pc(ctx, recorded_pcs[i]);
	}

	if (vr_view_ptr) {
		std::vector<vec3> P;
		std::vector<rgb> C;
		const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
		if (state_ptr) {
			for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
				vec3 ray_origin, ray_direction;
				state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
				P.push_back(ray_origin);
				P.push_back(ray_origin + ray_length * ray_direction);
				rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
				C.push_back(c);
				C.push_back(c);
			}
		}
		if (P.size() > 0) {
			cgv::render::shader_program& prog = ctx.ref_default_shader_program();
			int pi = prog.get_position_index();
			int ci = prog.get_color_index();
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
			cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
			cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
			glLineWidth(3);
			prog.enable(ctx);
			glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
			prog.disable(ctx);
			cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
			glLineWidth(1);
		}
	}

	// draw static boxes
	if (boxes.size()) {
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);
	}

	// push btns in boxgui to dynamic box list, treat 
	// boxguibtns the same as moveble boxes
	// render tex. in an other block
	if (construct_boxgui)
		if (boxguibtns.size()) {
			cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
			std::vector<box3> movable_btn;
			std::vector<rgb> movable_btn_colors;
			std::vector<vec3> movable_btn_translations;
			std::vector<quat> movable_btn_rotations;
			for (auto btn : boxguibtns) {
				if (btn.group == which_boxgui_group_is_going_to_be_rendered) {
					movable_btn.push_back(box3(-0.5f * btn.ext, 0.5f * btn.ext));
					movable_btn_colors.push_back(btn.color);
					cur_left_hand_dir.normalize();

					if (btn.has_intersec && !btn.is_static)
						movable_btn_translations.push_back(btn.trans + cur_left_hand_dir * 0.1);
					else
						movable_btn_translations.push_back(btn.trans);
					movable_btn_rotations.push_back(btn.rot);
				}
			}
			renderer.set_render_style(movable_style);
			renderer.set_box_array(ctx, movable_btn);
			renderer.set_color_array(ctx, movable_btn_colors);
			renderer.set_translation_array(ctx, movable_btn_translations);
			renderer.set_rotation_array(ctx, movable_btn_rotations);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)movable_btn.size());
			}
			renderer.disable(ctx);
		}

	// draw dynamic boxes 
	if (!movable_boxes.empty()) {
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(movable_style);
		renderer.set_box_array(ctx, movable_boxes);
		renderer.set_color_array(ctx, movable_box_colors);
		renderer.set_translation_array(ctx, movable_box_translations);
		renderer.set_rotation_array(ctx, movable_box_rotations);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)movable_boxes.size());
		}
		renderer.disable(ctx);
	}

	// yzy, render labels of boxgui 
	for (auto btn : boxguibtns) {
		if (btn.group == which_boxgui_group_is_going_to_be_rendered)
			if (btn.use_label) {
				// points for a label, y-z plane
				vec3 p1(0.5 * btn.ext.x(), 0.5 * btn.ext.y(), 0);
				vec3 p2(-0.5 * btn.ext.x(), 0.5 * btn.ext.y(), 0);
				vec3 p3(0.5 * btn.ext.x(), -0.5 * btn.ext.y(), 0);
				vec3 p4(-0.5 * btn.ext.x(), -0.5 * btn.ext.y(), 0);

				vec3 addi_offset = vec3(0);

				if (btn.has_intersec && !btn.is_static)
					addi_offset = vec3(0, 0, -0.1f); // todo

				p1 = p1 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;
				p2 = p2 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;
				p3 = p3 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;
				p4 = p4 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;

				/*quat tmp(vec3(0, 1, 0), var1);
				tmp.rotate(p1);
				tmp.rotate(p2);
				tmp.rotate(p3);
				tmp.rotate(p4);*/

				// rotate and translate according to the gui boxes
				btn.rot.rotate(p1);
				btn.rot.rotate(p2);
				btn.rot.rotate(p3);
				btn.rot.rotate(p4);

				p1 = p1 + btn.trans;
				p2 = p2 + btn.trans;
				p3 = p3 + btn.trans;
				p4 = p4 + btn.trans;

				cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
				int pi = prog.get_position_index();
				int ti = prog.get_texcoord_index();
				std::vector<vec3> P;
				std::vector<vec2> T;

				P.push_back(p1); T.push_back(vec2(1.0f, 1.0f));
				P.push_back(p2); T.push_back(vec2(0.0f, 1.0f));
				P.push_back(p3); T.push_back(vec2(1.0f, 0.0f));
				P.push_back(p4); T.push_back(vec2(0.0f, 0.0f));

				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
				cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
				prog.enable(ctx);
				btn.labeltex->label_tex.enable(ctx);
				ctx.set_color(rgb(1, 1, 1));
				glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
				btn.labeltex->label_tex.disable(ctx);
				prog.disable(ctx);
				cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
			}
	}

	// draw intersection points
	if (!intersection_points.empty()) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_position_array(ctx, intersection_points);
		sr.set_color_array(ctx, intersection_colors);
		sr.set_render_style(srs);
		if (sr.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)intersection_points.size());
			sr.disable(ctx);
		}
	}
}
///
bool vr_rgbd::handle(cgv::gui::event& e)
{
	// check if vr event flag is not set and don't process events in this case
	if ((e.get_flags() & cgv::gui::EF_VR) == 0)
		return false;
	// check event id
	switch (e.get_kind()) {
	case cgv::gui::EID_KEY:
	{
		cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
		int ci = vrke.get_controller_index();
		if (ci == 0 && vrke.get_key() == vr::VR_DPAD_DOWN) {
			switch (vrke.get_action()) {
			case cgv::gui::KA_PRESS:
				rgbd_2_controller_orientation_start_calib = controller_orientation; // V^0 = V
				rgbd_2_controller_position_start_calib = controller_position;       // r^0 = r
				in_calibration = true;
				update_member(&in_calibration);
				break;
			case cgv::gui::KA_RELEASE:
				rgbd_2_controller_orientation = transpose(rgbd_2_controller_orientation_start_calib) * controller_orientation * rgbd_2_controller_orientation;
				rgbd_2_controller_position = transpose(rgbd_2_controller_orientation_start_calib) * ((controller_orientation * rgbd_2_controller_position + controller_position) - rgbd_2_controller_position_start_calib);
				in_calibration = false;
				update_member(&in_calibration);
				break;
			}
		}
		if (ci == 0 && vrke.get_key() == vr::VR_DPAD_LEFT)
		{
			switch (vrke.get_action()) {
			case cgv::gui::KA_PRESS:
				zoom_in = true;
				update_member(&zoom_in);
				break;
			case cgv::gui::KA_RELEASE:
				zoom_in = false;
				update_member(&zoom_in);
				break;
			}
		}
		if (ci == 0 && vrke.get_key() == vr::VR_DPAD_RIGHT)
		{
			switch (vrke.get_action()) {
			case cgv::gui::KA_PRESS:
				zoom_out = true;
				update_member(&zoom_out);
				break;
			case cgv::gui::KA_RELEASE:
				zoom_out = false;
				update_member(&zoom_out);
				break;
			}
		}
		if (ci == 1 && vrke.get_key() == vr::VR_DPAD_LEFT)
		{
			if (vrke.get_action() == cgv::gui::KA_PRESS)
				vr_view_ptr->set_tracking_rotation(vr_view_ptr->get_tracking_rotation() + 10);
		}
		if (ci == 1 && vrke.get_key() == vr::VR_DPAD_RIGHT)
		{
			if (vrke.get_action() == cgv::gui::KA_PRESS)
				vr_view_ptr->set_tracking_rotation(vr_view_ptr->get_tracking_rotation() - 10);
		}
		if (ci == 0 && vrke.get_key() == vr::VR_MENU)
		{
			switch (vrke.get_action()) {
			case cgv::gui::KA_PRESS:
				clear_all_frames = true;
				update_member(&clear_all_frames);
				break;
			case cgv::gui::KA_RELEASE:
				clear_all_frames = false;
				update_member(&clear_all_frames);
				break;
			}
		}
		if (ci == 1 && vrke.get_key() == vr::VR_GRIP)
		{
			if (vrke.get_action() == cgv::gui::KA_PRESS) {
				vec3 origin, direction;
				vrke.get_state().controller[1].put_ray(&origin(0), &direction(0));
				vec3 posi = compute_ray_plane_intersection_point(origin, direction);
				vr_view_ptr->set_tracking_origin(vec3(posi.x(),
					vr_view_ptr->get_tracking_origin().y(), posi.z()));
			}
		}
		if (ci == 0 && vrke.get_key() == vr::VR_GRIP)
		{
			if (vrke.get_action() == cgv::gui::KA_PRESS) {
				if (which_boxgui_group_is_going_to_be_rendered >= 0)
					which_boxgui_group_is_going_to_be_rendered = -1; //erase all gui stuffs 
				else
					which_boxgui_group_is_going_to_be_rendered = 0;
			}
		}
		return true;
	}
	case cgv::gui::EID_THROTTLE:
	{
		cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
		if ((vrte.get_last_value() <= 0.5f) && (vrte.get_value() > 0.5f)) {
			trigger_is_pressed = (vrte.get_value() > 0.5f);
			update_member(&trigger_is_pressed);
			std::cout << "trigger pressed! " << std::endl;

			//if (constructive_shaping_mode) {
			//	if (predef_obj_id == 0) { // a box, two points are used
			//		int max_rec_posi = 2; // we store maximal two positions to indicate a box 
			//		vec3 cur_posi = objpicklist_translations.at(0);
			//		if (num_rec_posi%2 == 0) { // if is the first position 
			//			// compute current mvp 
			//			mat4 mvp, R;
			//			/*objpicklist_rotations.at(0).put_homogeneous_matrix(R);
			//			mvp = cgv::math::scale4<double>(0.01 * obj_scale_factor, 0.01 * obj_scale_factor, 0.01 * obj_scale_factor);
			//			mvp = R * mvp;
			//			mvp = cgv::math::translate4<double>(objpicklist_translations.at(0)) * mvp;*/

			//			mvp.identity();

			//			cur_obj = new scanning_primitive(predef_obj_id, mvp, true);
			//			cur_obj->record_position(cur_posi);
			//			num_rec_posi++;
			//			scanning_primitive_list.push_back(cur_obj);
			//		}
			//		else if (num_rec_posi%2 == 1) {
			//			cur_obj->record_position(cur_posi);
			//			num_rec_posi++;
			//		}
			//	}
			//}
			//else {
			//	// things will be recorded directly, exact_shaping_mode
			//	// compute current mvp 
			//	mat4 mvp, R;
			//	objpicklist_rotations.at(0).put_homogeneous_matrix(R);
			//	mvp = cgv::math::scale4<double>(0.01 * obj_scale_factor, 0.01 * obj_scale_factor, 0.01 * obj_scale_factor);
			//	mvp = R * mvp;
			//	mvp = cgv::math::translate4<double>(objpicklist_translations.at(0)) * mvp;

			//	cur_obj = new scanning_primitive(predef_obj_id, mvp, false);
			//	scanning_primitive_list.push_back(cur_obj);
			//}
		}
		break;
	}
	case cgv::gui::EID_STICK:
	{
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		switch (vrse.get_action()) {
		case cgv::gui::SA_TOUCH:
			if (state[vrse.get_controller_index()] == IS_OVER)
				state[vrse.get_controller_index()] = IS_GRAB;
			std::cout << "IS_touch" << std::endl;
			if (vrse.get_controller_index() == 1)
				btn_keydown_boxgui = true;
			break;
		case cgv::gui::SA_RELEASE:
			if (state[vrse.get_controller_index()] == IS_GRAB)
				state[vrse.get_controller_index()] = IS_OVER;
			break;
		case cgv::gui::SA_PRESS:
			if ((vrse.get_controller_index() == 1) && (predef_obj_id < predef_obj_maxid) && (vrse.get_x() > 0.5)) {
				predef_obj_id++;
			}
			else if ((vrse.get_controller_index() == 1) && (predef_obj_id > 0) && (vrse.get_x() < -0.5)) {
				predef_obj_id--;
			}
			break;
		}
		return true;
	}
	case cgv::gui::EID_POSE:
		cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
		// check for controller pose events
		int ci = vrpe.get_trackable_index();
		if (ci == rgbd_controller_index) { // always update the controller state! as global vari.
			controller_orientation = vrpe.get_orientation();
			controller_position = vrpe.get_position();
		}
		// left hand event 
		if (ci == 0) {
			// update positions 
			cur_left_hand_posi = vrpe.get_position();
			cur_left_hand_rot = vrpe.get_orientation();
			cur_left_hand_rot_mat = vrpe.get_rotation_matrix();
		}
		if (ci != -1) {
			if (btn_keydown_boxgui) {
				if (intersection_boxgui_indices.size() > 0) {
					// call back functions 
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("save_\nscene")) {
						std::cout << "saved!" << std::endl;
						write_btnposi_to_file();
					}

					// main panel  
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("back")) {
						which_boxgui_group_is_going_to_be_rendered = 0;
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("pc_tools")) {
						which_boxgui_group_is_going_to_be_rendered = 1;
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("mesh_\ntools")) {
						which_boxgui_group_is_going_to_be_rendered = 2;
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("semantic\ntools")) {
						which_boxgui_group_is_going_to_be_rendered = 3;
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("interaction\n_tools")) {
						which_boxgui_group_is_going_to_be_rendered = 4;
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("gui_tools")) {
						which_boxgui_group_is_going_to_be_rendered = 5;
					}

					// sub-panel 1
					/*
						label_list.push_back("read_properties");
						label_list.push_back("load_next_pc");
						label_list.push_back("del_last_pc");
						label_list.push_back("align_pc_\nwith_last\n_frame");
					*/
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("read_properties")) {
						read_properties();
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("load_next_pc")) {
						read_next_pc();
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("load_all_pcs")) {
						read_pc_queue();
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("del_last_pc")) {
						del_last_pc();
					}
					if (boxguibtns.at(intersection_boxgui_indices.front())
						.get_label_text()._Equal("align_pc_\nwith_last\n_frame")) {

					}


				}
				btn_keydown_boxgui = false;
			}
			if (state[0] == IS_GRAB) {
				std::cout << "IS_GRAB" << std::endl;

				// clean again
				intersection_points.clear();
				intersection_box_indices.clear();
				intersection_boxgui_indices.clear();
				for (auto& b : boxguibtns) { // pf- must have &! to modify 
					b.has_intersec = false;
				}
				// in grab mode apply relative transformation to grabbed boxes
				// compute intersections for left hand: drag and move boxes 
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				compute_intersections(origin, direction, true, true);
				// get previous and current controller position
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();
				// get rotation from previous to current orientation
				// this is the current orientation matrix times the
				// inverse (or transpose) of last orientation matrix:
				// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
				mat3 rotation = vrpe.get_rotation_matrix();
				// iterate intersection points of current controller
				for (size_t i = 0; i < intersection_points.size(); ++i) {
					// extract box index
					unsigned bi = intersection_box_indices[i];
					// update translation with position change and rotation
					movable_box_translations[bi] =
						rotation * (movable_box_translations[bi] - last_pos) + pos;
					// update orientation with rotation, note that quaternions
					// need to be multiplied in oposite order. In case of matrices
					// one would write box_orientation_matrix *= rotation
					movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
					// update intersection points
					intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
				}

				if (intersection_boxgui_indices.size() > 0) {
					//intersection_boxgui_controller_indices
					unsigned bi = intersection_boxgui_indices.front();
					// update translation with position change and rotation
					float xval = boxguibtns.at(bi).trans.x();
					boxguibtns.at(bi).trans =
						rotation * (boxguibtns.at(bi).trans - last_pos) + pos;
					boxguibtns.at(bi).trans.x() = xval;
					// update orientation with rotation, note that quaternions
					// need to be multiplied in oposite order. In case of matrices
					// one would write box_orientation_matrix *= rotation

					//boxguibtns.at(bi).rot = quat(rotation) * boxguibtns.at(bi).rot;
				}
				//state[0] = IS_OVER;
			}
			else {
				state[0] = IS_OVER; // init. 
			}
		}
		// always run this block when pose changes 
		{
			// updated with right hand controller, we do not use button local rot now
			vec3 local_z_offset = vec3(0, 0, -1);
			quat rot(cur_left_hand_rot);
			rot.normalize();
			mat3 rot_mat;
			rot.put_matrix(rot_mat);
			vec3 global_offset = rot_mat * local_z_offset;
			cur_left_hand_dir = global_offset;

			for (auto& btn : boxguibtns) {
				vec3 local_offset = btn.local_trans;
				quat rot(cur_left_hand_rot);
				rot.normalize();
				mat3 rot_mat;
				rot.put_matrix(rot_mat);
				vec3 global_offset = rot_mat * local_offset;
				// addi direction vector should be rotated 
				vec3 modi_posi = cur_left_hand_posi + global_offset;

				btn.trans = modi_posi;
				btn.rot = rot * btn.local_rot;
			}

			// for right hand and boxgui intersections 
			// init. for pose event clean, todo 
			intersection_points.clear();
			intersection_box_indices.clear();
			intersection_boxgui_indices.clear();
			for (auto& b : boxguibtns) { // pf- must have &! to modify 
				b.has_intersec = false;
			}

			vec3 origin, direction;
			vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
			compute_intersections(origin, direction, false, true);
			/*vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
			compute_intersections(origin, direction, true, false);*/

			if (intersection_boxgui_indices.size() > 0) {
				boxguibtns.at(intersection_boxgui_indices.front()).has_intersec = true;
				//post_redraw();
			}
			post_redraw();
		}
		return true;
	}

	return false;
}
/// register on device change events
void vr_rgbd::on_device_change(void* kit_handle, bool attach)
{
	post_recreate_gui();
}
///
bool vr_rgbd::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return
		rh.reflect_member("rgbd_controller_index", rgbd_controller_index) &&
		rh.reflect_member("zoom_in", zoom_in) &&
		rh.reflect_member("zoom_out", zoom_out) &&
		rh.reflect_member("save_pc", save_pointcloud) &&
		rh.reflect_member("register_pc", registration_started) &&
		rh.reflect_member("recording_fps", recording_fps) &&
		rh.reflect_member("ray_length", ray_length) &&
		rh.reflect_member("record_frame", record_frame) &&
		rh.reflect_member("record_all_frames", record_all_frames) &&
		rh.reflect_member("clear_all_frames", clear_all_frames) &&
		rh.reflect_member("rgbd_started", rgbd_started) &&
		rh.reflect_member("pointcloud_reading_path", pointcloud_reading_path) &&
		rh.reflect_member("rgbd_protocol_path", rgbd_protocol_path);
}
///
void vr_rgbd::on_set(void* member_ptr)
{
	if (member_ptr == &rgbd_started && rgbd_started != rgbd_inp.is_started()) {
		if (rgbd_started)
			start_rgbd();
		else
			stop_rgbd();
	}
	if (member_ptr == &rgbd_protocol_path) {
		rgbd_inp.stop();
		rgbd_inp.detach();
		rgbd_inp.attach_path(rgbd_protocol_path);
		if (rgbd_started)
			start_rgbd();
	}
	update_member(member_ptr);
	post_redraw();
}
///
void vr_rgbd::create_gui()
{
	/// yzy
	add_decorator("vr_rgbd", "heading", "level=2");
	//read_pc(), upper_pc, lower_pc
	connect_copy(add_button("read_pc")->click, cgv::signal::rebind(this, &vr_rgbd::read_pc));
	connect_copy(add_button("upper_pc")->click, cgv::signal::rebind(this, &vr_rgbd::upper_pc));
	connect_copy(add_button("lower_pc")->click, cgv::signal::rebind(this, &vr_rgbd::lower_pc));
	connect_copy(add_button("load_mesh")->click, cgv::signal::rebind(this, &vr_rgbd::load_mesh));
	connect_copy(add_button("write_posi_2file")->click, cgv::signal::rebind(this, &vr_rgbd::write_btnposi_to_file));
	connect_copy(add_button("read_pc_queue")->click, cgv::signal::rebind(this, &vr_rgbd::read_pc_queue));
	connect_copy(add_button("construct_handheld_gui")->click, cgv::signal::rebind(this, &vr_rgbd::construct_handheld_gui));
	connect_copy(add_button("shuffle_button_group")->click, cgv::signal::rebind(this, &vr_rgbd::shuffle_button_group));
	//read_properties, read_next_pc(), log_cur_num_pc(); del_last_pc
	connect_copy(add_button("read_properties")->click, cgv::signal::rebind(this, &vr_rgbd::read_properties));
	connect_copy(add_button("read_next_pc")->click, cgv::signal::rebind(this, &vr_rgbd::read_next_pc));
	connect_copy(add_button("log_cur_num_pc")->click, cgv::signal::rebind(this, &vr_rgbd::log_cur_num_pc));
	connect_copy(add_button("del_last_pc")->click, cgv::signal::rebind(this, &vr_rgbd::del_last_pc));

	add_member_control(this, "var1", var1, "value_slider", "min=-10;max=10;log=true;ticks=true");
	connect_copy(add_button("test_icp_gen")->click, cgv::signal::rebind(this, &vr_rgbd::test_icp_gen));
	connect_copy(add_button("test_icp")->click, cgv::signal::rebind(this, &vr_rgbd::test_icp));
	connect_copy(add_button("zr_calibration_init")->click, cgv::signal::rebind(this, &vr_rgbd::zr_calibration_init));
	connect_copy(add_button("zr_calibration_do")->click, cgv::signal::rebind(this, &vr_rgbd::zr_calibration_do));
	//calibration_realworld_init
	connect_copy(add_button("calibration_realworld_init")->click, cgv::signal::rebind(this, &vr_rgbd::calibration_realworld_init));
	connect_copy(add_button("calibration_realworld_do")->click, cgv::signal::rebind(this, &vr_rgbd::calibration_realworld_do));
	//gen_voxel_from_pc
	connect_copy(add_button("gen_voxel_from_pc")->click, cgv::signal::rebind(this, &vr_rgbd::gen_voxel_from_pc));

	/// ori.
	add_gui("rgbd_protocol_path", rgbd_protocol_path, "directory", "w=150");
	add_gui("pointcloud_reading_path", pointcloud_reading_path, "directory", "w=150");
	add_gui("pointcloud_writing_path", pointcloud_writing_path, "directory", "w=150");
	add_member_control(this, "constructive_shaping_mode", constructive_shaping_mode, "check");
	add_member_control(this, "rgbd_started", rgbd_started, "check");
	add_member_control(this, "update_voxel", update_voxel, "check");
	add_member_control(this, "record_frame", record_frame, "check");
	add_member_control(this, "record_all_frames", record_all_frames, "check");
	add_member_control(this, "clear_all_frames", clear_all_frames, "check");
	add_member_control(this, "trigger_is_pressed", trigger_is_pressed, "check");
	add_member_control(this, "recording_fps", recording_fps, "value_slider", "min=1;max=30;ticks=true;log=true");
	add_member_control(this, "in_calibration", in_calibration, "check");
	add_member_control(this, "zoom_in", zoom_in, "check");
	add_member_control(this, "zoom_out", zoom_out, "check");
	add_member_control(this, "save_pc", save_pointcloud, "check");
	add_member_control(this, "register_pc", registration_started, "check");

	add_member_control(this, "rgbd_controller_index", rgbd_controller_index, "value_slider", "min=0;max=3;ticks=true");


	add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
	bool show = begin_tree_node("points", show_points, true, "w=100;align=' '");
	add_member_control(this, "show", show_points, "toggle", "w=50");
	if (show) {
		align("\a");
		add_member_control(this, "max_nr_shown_recorded_pcs", max_nr_shown_recorded_pcs, "value_slider", "min=0;max=100;log=true;ticks=true");
		//add_member_control(this, "sort_points", sort_points, "check");
		if (begin_tree_node("point style", point_style)) {
			align("\a");
			add_gui("point_style", point_style);
			align("\b");
			end_tree_node(point_style);
		}
		align("\b");
		end_tree_node(show_points);
	}
}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_rgbd> vr_rgbd_reg("");

///@}