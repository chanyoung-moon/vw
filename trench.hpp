#pragma once

#include <vector>
#include <cmath>

#include "linalg.hpp"

struct inter_status
{
    bool status;
    std::vector<double> pt;
};


std::vector<double> rotate_plane(const std::vector<double> &input,const std::vector<double> &y1, const std::vector<double> &y2)
{
    double rotate_angle = acos(dot(y1,y2)/norm(y1)/norm(y2));
    std::vector<double> cross_vec = cross(y1,y2);      
    if (cross_vec[2] < 0) rotate_angle = - rotate_angle;

    std::vector<std::vector<double>> rot_mat;

    std::vector<double> rot_mat_row1;
    std::vector<double> rot_mat_row2;
    std::vector<double> rot_mat_row3;
    rot_mat_row1.push_back(cos(rotate_angle));
    rot_mat_row1.push_back(-sin(rotate_angle));
    rot_mat_row1.push_back(0);
    rot_mat_row2.push_back(sin(rotate_angle));
    rot_mat_row2.push_back(cos(rotate_angle));
    rot_mat_row2.push_back(0);
    rot_mat_row3.push_back(0);
    rot_mat_row3.push_back(0);
    rot_mat_row3.push_back(1);
    
    rot_mat.push_back(rot_mat_row1);
    rot_mat.push_back(rot_mat_row2);
    rot_mat.push_back(rot_mat_row3);

    return matmul3d(rot_mat,input);   
}

std::vector<std::vector<double>> move_plane(const std::vector<std::vector<double>> &input_arr, const std::vector<double> &vec)
{
    std::vector<std::vector<double>> result;
    int tmp_size_1 = input_arr.size();
    int tmp_size_2 = input_arr[0].size();
    for (int i = 0; i < tmp_size_1;i++)
    {
        std::vector<double> result_row;
        for(int j =0;j<tmp_size_2;j++)
        {
            result_row.push_back(input_arr[i][j] + vec[j]);
        }
        
        result.push_back(result_row);        
    }

    return result;
}

inter_status intersection_check(const std::vector<double> &p,const std::vector<double> &r,const std::vector<double> &q,const std::vector<double> &s)
{
    inter_status tmp_inter_status;
    bool tmp_status;
    std::vector<double> tmp_pt;
    if(abs(cross_2d(r,s)) < 1e-3)
    {
        tmp_status = true;
        tmp_pt = p; 
    } 
    else
    {
        double t = cross_2d(vec_add(q,const_vec_mul(p,-1)),s) / cross_2d(r,s);
        double u = cross_2d(vec_add(q,const_vec_mul(p,-1)),r) / cross_2d(r,s);
        if(abs(cross_2d(r,s))> 1e-4 && (t > 0 || abs(t) < 1e-4) && (1 > t || abs(t-1) < 1e-4) && (u >0 || abs(u) > 1e-4) && (1>u || abs(u-1)<1e-4))
        {
            tmp_status = true;
            tmp_pt = vec_add(const_vec_mul(r,t), p);
        }
        else
        {
            tmp_status = false;
            tmp_pt = {0,0};
        }
    }

    tmp_inter_status.status = tmp_status;
    tmp_inter_status.pt = tmp_pt; 
    return tmp_inter_status;
}

bool TIN_cmp(const std::vector<double> &TIN_1, const std::vector<double> &TIN_2)
{
    bool result;
    std::vector<double> p1 = {(TIN_1[0] + TIN_1[3] + TIN_1[6])/3,(TIN_1[1] + TIN_1[4] + TIN_1[7])/3,(TIN_1[2] + TIN_1[5] + TIN_1[8])/3};
    std::vector<double> p2 = {(TIN_2[0] + TIN_2[3] + TIN_2[6])/3,(TIN_2[1] + TIN_2[4] + TIN_2[7])/3,(TIN_2[2] + TIN_2[5] + TIN_2[8])/3}; 
    if (norm(vec_add(p1,const_vec_mul(p2,-1))) < 1e-4)
    {
        result = true;
    }
    else
    {
        result = false;
    }
    return result;
}

bool TIN_check(const std::vector<double> &TIN_in)
{
    bool result;
    std::vector<double> p1 = {TIN_in[0],TIN_in[1],TIN_in[2]};
    std::vector<double> p2 = {TIN_in[3],TIN_in[4],TIN_in[5]};
    std::vector<double> p3 = {TIN_in[6],TIN_in[7],TIN_in[8]};
    if (norm(vec_add(p1,const_vec_mul(p2,-1))) < 1e-4 || norm(vec_add(p2,const_vec_mul(p3,-1))) < 1e-4 || norm(vec_add(p3,const_vec_mul(p1,-1))) < 1e-4)
    {
        result = true;
    }
    else
    {
        result = false;
    }
    return result;
}

std::vector<std::vector<double>> mesh_to_TIN(const std::vector<std::vector<double>> &mesh_arr_input)
{
    std::vector<std::vector<double>> output_TIN;
    int tmp_size = mesh_arr_input.size();
    for(int i = 0;i<tmp_size;i++)
    {
        std::vector<double> TIN_up;
        std::vector<double> TIN_down;

        TIN_up.push_back(mesh_arr_input[i][0]);
        TIN_up.push_back(mesh_arr_input[i][1]);
        TIN_up.push_back(mesh_arr_input[i][2]);
        TIN_up.push_back(mesh_arr_input[i][3]);
        TIN_up.push_back(mesh_arr_input[i][4]);
        TIN_up.push_back(mesh_arr_input[i][5]);
        TIN_up.push_back(mesh_arr_input[i][6]);
        TIN_up.push_back(mesh_arr_input[i][7]);
        TIN_up.push_back(mesh_arr_input[i][8]);

        TIN_down.push_back(mesh_arr_input[i][0]);
        TIN_down.push_back(mesh_arr_input[i][1]);
        TIN_down.push_back(mesh_arr_input[i][2]);
        TIN_down.push_back(mesh_arr_input[i][6]);
        TIN_down.push_back(mesh_arr_input[i][7]);
        TIN_down.push_back(mesh_arr_input[i][8]);
        TIN_down.push_back(mesh_arr_input[i][9]);
        TIN_down.push_back(mesh_arr_input[i][10]);
        TIN_down.push_back(mesh_arr_input[i][11]);

        output_TIN.push_back(TIN_up);
        output_TIN.push_back(TIN_down);
    }
    return output_TIN;
}