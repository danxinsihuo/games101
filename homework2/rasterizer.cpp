// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

	Vector3f AB = _v[1] - _v[0], BC = _v[2] - _v[1], CA = _v[0] - _v[1];
	Vector3f P = Vector3f(x, y, 1.0f);
	Vector3f AP = P - _v[0], BP = P - _v[1], CP = P - _v[2];
	float ap = AP.cross(AB).z();
	float bp = BP.cross(BC).z();
	float cp = CP.cross(CA).z();
	if (ap > 0 && bp > 0 && cp > 0 || ap < 0 && bp < 0 && cp < 0) {
		return true;
	}
	return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

// MSAA������һ�������ڲ��ĳ������㣬�ж����Ƿ�����1.��ȸ�С 2.���������ڣ���������ĳ��������ظ��������������ǽ�1�����ص���4�����ص㣬��4������жϡ�λ���жϣ������Ȼ������ɫ����4����С�����ӣ�depth_buf[0]��depth_buf[1]��depth_buf[2]��depth_buf[3]��Ӧ������(0,0)�����ص�4�����������ȡ�n��m��ʾ��������󣬼�1������һ����n*m�������㣬n��������m������
int rst::rasterizer::MSAA(int x, int y, const Triangle& t, int n, int m, float z)
{
	float size_x = 1.0 / n; // the size_x of every super sample pixel
	float size_y = 1.0 / m;

	int blocksinTriangle = 0;
	// ����n*m��������
	for (int i = 0; i < n; ++i)
		for (int j = 0; j < m; ++j)
		{
			float _x = x + i * size_x; // _x is the coordinate of the sample point
			float _y = y + j * size_y;
			if (z < MSAA_depth_buf[get_index(x, y) * 4 + i * n + j] && insideTriangle(_x, _y, t.v))
			{
				// д����Ȼ���
				MSAA_depth_buf[get_index(x, y) * 4 + i * n + j] = z;
				// д����ɫ����
				MSAA_frame_buf[get_index(x, y) * 4 + i * n + j] = t.getColor();
				blocksinTriangle++;
			}
		}
	return blocksinTriangle;
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
		Triangle t;
		Eigen::Vector4f v[] = {
				mvp * to_vec4(buf[i[0]], 1.0f),
				mvp * to_vec4(buf[i[1]], 1.0f),
				mvp * to_vec4(buf[i[2]], 1.0f)
		};
		//Homogeneous division
		for (auto& vec : v) {
			vec /= vec.w();
		}
		//Viewport transformation
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
	}
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	auto v = t.toVector4();

	// TODO : Find out the bounding box of current triangle.
	// iterate through the pixel and find if the current pixel is inside the triangle
	int minX = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
	int maxX = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
	int minY = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
	int maxY = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
	// If so, use the following code to get the interpolated z value.


	// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
	// ��չ�����ò��������
	int m = 2;
	int n = 2;
	for (int i = minX; i <= maxX; i++) {
		for (int j = minY; j <= maxY; j++) {
			int block = 0;
			auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
			float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
			float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
			z_interpolated *= w_reciprocal;
			/*if (z_interpolated < depth_buf[get_index(i, j)] && insideTriangle(i, j, t.v)) {
				set_pixel(Vector3f(i, j, z_interpolated), t.getColor());
			}*/

			if ((block = MSAA(i, j, t, n, m, z_interpolated) > 0)) {
				int idx = get_index(i, j);
				// ������в��������ɫ
				Vector3f mixColor = (MSAA_frame_buf[idx * 4] + MSAA_frame_buf[idx * 4 + 1] + MSAA_frame_buf[idx * 4 + 2] + MSAA_frame_buf[idx * 4 + 3]) / 4.0;
				set_pixel(Eigen::Vector3f(i, j, z_interpolated), mixColor);
			}
		}
	}
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
		std::fill(MSAA_frame_buf.begin(), MSAA_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		std::fill(MSAA_depth_buf.begin(), MSAA_depth_buf.end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
	MSAA_frame_buf.resize(w * h * 4);
	MSAA_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;

}

// clang-format on