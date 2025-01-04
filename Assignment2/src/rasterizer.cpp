// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f p(x + 0.5f, y + 0.5f, 1.0f);

    Vector3f p01 = _v[1] - _v[0];
    Vector3f p12 = _v[2] - _v[1];
    Vector3f p20 = _v[0] - _v[2];

    Vector3f p0 = p - _v[0];
    Vector3f p1 = p - _v[1];
    Vector3f p2 = p - _v[2];

    float z1 = p01.x() * p0.y() - p01.y() * p0.x();
    float z2 = p12.x() * p1.y() - p12.y() * p1.x();
    float z3 = p20.x() * p2.y() - p20.y() * p2.x();

    if(z1 * z2 > 0 && z2 * z3 > 0)
        return true;
    else return false;
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f p(x, y, 1.0f);

    Vector3f p01 = _v[1] - _v[0];
    Vector3f p12 = _v[2] - _v[1];
    Vector3f p20 = _v[0] - _v[2];

    Vector3f p0 = p - _v[0];
    Vector3f p1 = p - _v[1];
    Vector3f p2 = p - _v[2];

    float z1 = p01.x() * p0.y() - p01.y() * p0.x();
    float z2 = p12.x() * p1.y() - p12.y() * p1.x();
    float z3 = p20.x() * p2.y() - p20.y() * p2.x();

    if(z1 * z2 > 0 && z2 * z3 > 0)
        return true;
    else return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
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
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
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
    
    int x1 = width, y1 = height;
    int x2 = 0, y2 = 0;
    for(auto& ver: v)
    {
        if(ver[0] < x1) x1 = ver[0];
        if(ver[0] > x2) x2 = ver[0];
        if(ver[1] < y1) y1 = ver[1];
        if(ver[1] > y2) y2 = ver[1];
    }

    for(int i = x1; i <= x2; i++)
    {
        for(int j = y1; j <= y2; j++)
        {
            int index = get_index(i, j);
#ifndef SS
            if(insideTriangle(i, j, t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(i + 0.5f, j + 0.5f, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(z_interpolated < depth_buf[index])
                {
                    depth_buf[index] = z_interpolated;
                    set_pixel(Vector3f(i, j, 1.0f), t.getColor());
                }
            }
#else
            int k = 0;
            bool fresh = false;
            for(float m = 0.25f; m < 1.0f; m += 0.5f)
            {
                for(float n = 0.25f; n < 1.0f; n += 0.5f)
                {
                    if(insideTriangle(i + m, j + n, t.v))
                    {
                        auto[alpha, beta, gamma] = computeBarycentric2D(i + m, j + n, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        if(z_interpolated < depth_buf_s[k][index])
                        {
                            depth_buf_s[k][index] = z_interpolated;
                            frame_buf_s[k][index] = t.getColor();
                            fresh = true;
                        }
                    }
                    k++;
                }
            }
            if(fresh)
                set_pixel(Vector3f(i, j, 1.0f), (frame_buf_s[0][index] + frame_buf_s[1][index] + frame_buf_s[2][index] + frame_buf_s[3][index]) / 4.0f);
#endif
        }
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
#ifdef SS
    for(auto& fbs: frame_buf_s)
        std::fill(fbs.begin(), fbs.end(), Eigen::Vector3f{0, 0, 0});
    for(auto& dbs: depth_buf_s)
        std::fill(dbs.begin(), dbs.end(), std::numeric_limits<float>::infinity());
#endif
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

#ifdef SS
    for(auto& dbs: depth_buf_s)
        dbs.resize(w * h);
    for(auto& fbs: frame_buf_s)
        fbs.resize(w * h);
#endif
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on