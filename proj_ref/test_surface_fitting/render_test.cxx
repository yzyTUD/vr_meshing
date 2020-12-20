#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/base/register.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/key_event.h>
#include <cgv/utils/ostream_printf.h>
#include <cgv/gui/provider.h>
#include <cgv/math/ftransform.h>
#include <cgv/media/illum/surface_material.h>

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

class render_test :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	int n = 4;
	std::vector<vec3> P;
	std::vector<vec3> C;
protected:

public:
	/// initialize rotation angle
	render_test()
	{
		connect(get_animation_trigger().shoot, this, &render_test::timer_event);
	}
	/// 
	void on_set(void* member_ptr)
	{
		update_member(member_ptr);
		post_redraw();
	}
	/// self reflection allows to change values in the config file
	bool self_reflect(reflection_handler& rh)
	{
		return true;
	}
	/// return the type name of the class derived from base
	std::string get_type_name() const
	{
		return "simple_cube";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}
	/// overload to handle events, return true if event was processed
	bool handle(event& e)
	{
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
	}
	bool init(context& ctx){
		/*4
			- 0.9 0.0 0.0
			0.0 0.8 0.2
			0.0 - 0.2 0.4
			0.9 0.0 0.6*/
		P.push_back(vec3(-0.9,  0.0, 0.0));
		P.push_back(vec3( 0.0,  0.8, 0.2));
		P.push_back(vec3( 0.0, -0.2, 0.4));
		P.push_back(vec3( 0.9,  0.0, 0.6));
		
		bezier(P, n, 250, C);

		return true;
	}

	void computeCoefficients(int n, int* c) {
		int k, i;
		for (k = 0; k <= n; k++) {
			c[k] = 1;
			for (i = n; i >= k + 1; i--)
				c[k] *= i;
			for (i = n - k; i >= 2; i--)
				c[k] /= i;
		}
	}

	// 
	void computePoint(float u, vec3& pt, int nc, std::vector<vec3> ct, int* c) {
		int k, n = nc - 1;
		float blend;
		pt.x() = pt.y() = pt.z() = 0.0;
		for (k = 0; k < nc; k++) {
			blend = c[k] * powf(u, k) * powf(1 - u, n - k);
			pt.x() += ct[k].x() * blend;
			pt.y() += ct[k].y() * blend;
			pt.z() += ct[k].z() * blend;
		}
	}

	// m total number of points to be gen. 
	void bezier(std::vector<vec3> ct, int nc, int m, std::vector<vec3>& curve) {
		int* c = (int*)malloc(nc * sizeof(int));
		for (int i = 0; i < m; i++)
			curve.push_back(vec3(0));
		computeCoefficients(nc - 1, c);
		for (int i = 0; i < m; i++)
			computePoint(i / (float)m, curve[i], nc, ct, c);
		free(c);
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPointSize(2);
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(1.0, 0.0, 0.0);
		glVertex3f(-1.0, 0.0, 0.0);
		glEnd();
		glColor3f(0.0, 1.0, 0.0);
		glBegin(GL_LINES);
		glVertex3f(0.0, 1.0, 0.0);
		glVertex3f(0.0, -1.0, 0.0);
		glEnd();
		glFlush();
		glColor3f(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(0.0, 0.0, 1.0);
		glVertex3f(0.0, 0.0, -1.0);
		glEnd();
		glFlush();

		glEnable(GL_LINE_STIPPLE);
		glLineStipple(1, 0x00FF); // dashed 
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0, 1.0, 1.0);
		for (int i = 0; i < n; i++)
			glVertex3f(P[i].x(), P[i].y(), P[i].z());
		glEnd();
		glFlush();
		glDisable(GL_LINE_STIPPLE);


		glColor3f(1.0, 1.0, 1.0);
		for (int i = 0; i < 250; i++) {
			glBegin(GL_POINTS);
			glVertex3f(C[i].x(), C[i].y(), C[i].z());
			glEnd();
		}
		glFlush();
	}
	/// overload the create gui method
	void create_gui()
	{
		add_decorator("Simple Cube GUI", "heading", "level=1"); 

	}
};

#include <cgv/base/register.h>
cgv::base::object_registration<render_test> render_test_reg("render_test");
