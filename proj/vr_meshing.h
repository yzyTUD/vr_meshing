#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/trigger.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/point_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <rgbd_input.h>
#include <random>
#include <future>
#include <iostream>
#include <chrono>
#include <point_cloud.h>
#include <ICP.h>

#include <gl_point_cloud_drawable.h>

//#include <openmesh/core/io/meshio.hh>
//#include <openmesh/core/mesh/polymesh_arraykernelt.hh>

#include "boxgui.h"
#include "mesh_render.h"
#include "custom_mesh_render_info.h"

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include "intersection.h"
#include <libs\cg_vr\vr_events.h>

#include <random>
#include <fstream>
#include <ICP.cxx>
#include <cgv\gui\file_dialog.h>

// different interaction states for the controllers
enum InteractionState
{
	IS_NONE,
	IS_OVER,
	IS_GRAB
};

struct vertex : public cgv::render::render_types
{
	vec3  point;
	rgba8 color;
};

struct cellpoint : public cgv::render::render_types
{
	double accu_tsdf;
	unsigned int num_points_used_to_updated = 0;
	rgba8 color;
};

class scanning_primitive : public cgv::render::render_types {
private:
	int predef_obj_id;
	mat4 mvp;
	rgb color;
	cgv::render::texture object_tex;
	bool constructive_shaping_mode;
	// 1,2 or multiple points for constructive shaping
	std::vector<vec3> recorded_position_list;
public:
	scanning_primitive(int predefined_id, mat4 modelview_matrix, bool c_shaping_mode) {
		predef_obj_id = predefined_id;
		mvp = modelview_matrix;
		constructive_shaping_mode = c_shaping_mode;
	}
	void record_position(vec3 cur_posi) {
		recorded_position_list.push_back(cur_posi);
	}
	std::vector<vec3> get_recorded_posi_list() {
		return recorded_position_list;
	}
	int get_predef_obj_id() {
		return predef_obj_id;
	}
	mat4 get_mvp() {
		return mvp;
	}
	bool is_constructive_shaping_mode() {
		return constructive_shaping_mode;
	}
};

/// the plugin class vr_rgbd inherits like other plugins from node, drawable and provider
class vr_rgbd :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
protected:
	/*
	* essential stuffs
	*/
	///
	std::future<size_t> future_handle;
	///
	bool rgbd_started;
	std::string rgbd_protocol_path;
	std::string pointcloud_reading_path;
	std::string pointcloud_writing_path;
	/// 
	rgbd::rgbd_input rgbd_inp;
	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;

	// rendering style for boxes
	cgv::render::box_render_style style;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// store the movable boxes
	std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;

	// state of current interaction with boxes for each controller
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;

	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;
	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type; 
	char* cgv_data = getenv("CGV_DATA");
	std::string data_dir = std::string(cgv_data);

	/*
		boxgui related
	*/
	std::vector<boxgui_button> boxguibtns;
	std::vector<int> intersection_boxgui_indices;
	std::vector<int> intersection_boxgui_controller_indices;
	std::vector<vec3> btnposilist;

	/*
		for handheld gui
	*/
	std::vector<box3> objpicklist;
	std::vector<rgb> objpicklist_colors;
	std::vector<vec3> objpicklist_translations;
	std::vector<quat> objpicklist_rotations;
	float obj_scale_factor = 3;
	cgv::render::shader_program cube_prog;
	int which_boxgui_group_is_going_to_be_rendered = 0;

	/*
		pc related
	*/
	/// internal members used for data storage
	rgbd::frame_type color_frame, depth_frame, warped_color_frame;
	rgbd::frame_type color_frame_2, depth_frame_2, ir_frame_2, warped_color_frame_2;
	///
	size_t nr_depth_frames, nr_color_frames;
	/// intermediate point cloud and to be rendered point cloud
	std::vector<vertex> intermediate_pc, current_pc;
	/// list of loaded point clouds
	std::vector<std::vector<vertex> > loaded_pcs;
	/// list of recorded point clouds
	std::vector<std::vector<vertex> > recorded_pcs;
	/// translations of recorded point clouds
	std::vector<quat> rotations;
	/// rotations of recorded point clouds
	std::vector<vec3> translations;
	/// rendering style for points
	cgv::render::point_render_style point_style;
	///counter of pc
	int counter_pc;
	///registration
	bool registration_started;

	int rgbd_controller_index;
	/// current pose of the controller
	mat3 controller_orientation;
	vec3 controller_position;
	/// pose of controller when last point cloud was acquire; this is used for contruction of point cloud in parallel thread
	mat3 controller_orientation_pc;
	vec3 controller_position_pc;
	/// current calibration pose mapping from rgbd coordinates to controller coordinates 
	mat3 rgbd_2_controller_orientation;
	vec3 rgbd_2_controller_position;
	/// calibration pose mapping from rgbd coordinates to controller coordinates stored at the time when freezing the point cloud for calibration
	mat3 rgbd_2_controller_orientation_start_calib;
	vec3 rgbd_2_controller_position_start_calib;
	///
	bool show_points;
	unsigned max_nr_shown_recorded_pcs;
	bool trigger_is_pressed;
	float recording_fps;
	point_cloud* sourcePC;
	point_cloud* targetPC;
	int cur_index_of_pc = 0;

	gl_point_cloud_drawable* pc_drawable = new gl_point_cloud_drawable();

	/*
		hand tracking
	*/
	vec3 cur_left_hand_posi;
	vec3 cur_left_hand_dir;
	mat3 cur_left_hand_rot;
	mat3 cur_left_hand_rot_mat;

	/*
		boolean varibles
	*/
	bool record_frame;
	bool record_all_frames;
	bool clear_all_frames;
	bool in_calibration;
	bool zoom_in;
	bool zoom_out;
	bool save_pointcloud;
	bool update_voxel;
	bool construct_boxgui = true;
	bool btn_keydown_boxgui = false;

	/*
		for mesh rendering
	*/
	float var1;
	std::shared_ptr<SkinningMesh> mmesh;

	/*
		for skybox rendering
	*/
	cgv::render::shader_program skyprog;
	cgv::render::texture img_tex;

	/*
		voxel related
	*/
	std::vector<cellpoint> cell_data;
	std::vector<unsigned char> cell_data_export;
	unsigned int resx;
	unsigned int resy;
	unsigned int resz;
	dvec3 minp;
	dvec3 cell_scaling;
	// define range 
	double max_negative;
	double max_positive;

	/*
		for semantic scanning
	*/
	/*
		0 -- cube
		1 -- sphere
		2 -- cone
		3 -- ...
	*/
	int predef_obj_id = 0;
	int predef_obj_maxid = 2;
	std::vector<scanning_primitive*> scanning_primitive_list;
	bool constructive_shaping_mode = false;
	int num_rec_posi = 0;
	scanning_primitive* cur_obj;


public:
	/*
		basic stuffs
	*/
	vr_rgbd();
	bool init(cgv::render::context& ctx);
	void init_frame(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);
	void draw_pc(cgv::render::context& ctx, const std::vector<vertex>& pc);
	void draw(cgv::render::context& ctx);
	// dynamic stuffs 
	bool handle(cgv::gui::event& e);
	void on_device_change(void* kit_handle, bool attach);
	void timer_event(double t, double dt);
	// gui related stuffs 
	bool self_reflect(cgv::reflect::reflection_handler& rh);
	void on_set(void* member_ptr);
	void create_gui();

	void write_btnposi_to_file();
	void construct_movable_boxgui(std::vector<std::string> llist);
	void compute_intersections(const vec3& origin, const vec3& direction, bool moveboxes, bool boxgui);
	void zr_calibration_init();
	void zr_calibration_do();
	void calibration_realworld_init();
	void calibration_realworld_do();
	void stream_help(std::ostream& os);
	vec3 compute_ray_plane_intersection_point(const vec3& origin, const vec3& direction);
	std::string get_type_name() const;
	void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_table(float tw, float td, float th, float tW);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	/// construct boxes for environment
	void construct_environment(float s, float ew, float ed, float eh, float w, float d, float h);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_movable_boxes(float tw, float td, float th, float tW, size_t nr);

	/*
		gui related
	*/
	///
	void construct_handheld_gui();
	void shuffle_button_group();

	/*
		pc processing 
	*/
	/// 
	void read_next_pc();
	///
	void del_last_pc();
	/// 
	void log_cur_num_pc();
	///
	void log_error(std::string e);
	///
	void read_pc();
	///
	void upper_pc();
	///
	void lower_pc();
	///
	void read_properties();
	///
	void read_pc_queue();
	///
	void generate_rdm_pc(point_cloud& pc1, point_cloud& pc2);
	/// 
	void generate_point_cloud(std::vector<vertex>& pc);
	///
	void registrationPointCloud();
	///
	void test_icp_gen();
	///
	void test_icp();
	///
	void construct_TSDtree();

	/*
		pc cauture 
	*/
	///
	void start_rgbd();
	///
	void stop_rgbd();
	///
	bool record_this_frame(double t);
	///
	void copy_pointcloud(const std::vector<vertex> input, point_cloud& output);
	///
	void pc2vertex(const point_cloud& input, std::vector<vertex>& output);
	///
	size_t construct_point_cloud();
	///
	rgbd::frame_type read_rgb_frame();
	///
	rgbd::frame_type read_depth_frame();
	///
	void write_pcs_to_disk(int i);

	/**
		mesh related stuffs
	*/
	///
	void load_mesh();


	/*
		voxel related stuffs
	*/
	///
	void gen_voxel_from_pc();
	///
	void update_voxel_by_current_pc();
	///
	void save_voxel_cell_data();
};