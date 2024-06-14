# pragma once
#include <vector>
#include <cmath>

double dot(const std::vector<double> &vec1, const std::vector<double> &vec2)
{
    int size = vec1.size();
    double result = 0;
    for(int i = 0 ; i < size;i++)
    {
    result = result + vec1[i] * vec2[i];
    }
    return result;
}

double norm(const std::vector<double> &vec1)
{
    int size = vec1.size();
    double result = 0;
    for(int i = 0 ; i < size;i++)
    {
    result = result + pow(vec1[i],2) ;
    }
    return sqrt(result);
}

std::vector<double> cross(const std::vector<double> &vec1, const std::vector<double> &vec2)
{
    std::vector<double> answer;
    answer.push_back(vec1[1]*vec2[2]-vec1[2]*vec2[1]);
    answer.push_back(vec1[2]*vec2[0]-vec1[0]*vec2[2]);
    answer.push_back(vec1[0]*vec2[1]-vec1[1]*vec2[0]);

    return answer;
}

std::vector<double> matmul3d(const std::vector<std::vector<double>> &mat, const std::vector<double> &vec)
{
    std::vector<double> result;
    result.push_back(mat[0][0]*vec[0]+mat[0][1]*vec[1]+mat[0][2]*vec[2]);
    result.push_back(mat[1][0]*vec[0]+mat[1][1]*vec[1]+mat[1][2]*vec[2]);
    result.push_back(mat[2][0]*vec[0]+mat[2][1]*vec[1]+mat[2][2]*vec[2]);
    return result;
}

double find_max(const std::vector<double> &input_arr)
{
    double ans = input_arr[0];
    int tmp_size_1 = input_arr.size();
    for(int i =0;i<tmp_size_1;i ++)
    {

        double val = input_arr[i];
        if (val > ans)
        {
            ans =val;
        }
        
    }
    return ans; 
}

double find_min(const std::vector<double> &input_arr)
{
    double ans = input_arr[0];
    int tmp_size_1 = input_arr.size();
    for(int i =0;i<tmp_size_1;i ++)
    {

        double val = input_arr[i];
        if (val < ans)
        {
            ans =val;
        }
        
    }
    return ans; 
}

double max(const double &a, const double &b)
{
    if (a>b) return a;
    else return b;
}


double cross_2d(const std::vector<double> &v, const std::vector<double> &w)
{
    double result;
    result = v[0] * w[1] - v[1] * w[0];

    return result; 
}

std::vector<double> const_vec_mul(const std::vector<double> &vec, const double &a)
{
    std::vector<double> result;
    int vec_size = vec.size(); 
    for (int i =0;i<vec_size;i++)
    {
        result.push_back(vec[i]*a);
    }
    return result;
}

std::vector<double> vec_add(const std::vector<double> &a,const std::vector<double> &b)
{
    std::vector<double> result;
    int vec_size = a.size();
    for (int i = 0;i<vec_size;i++)
    {
        result.push_back(a[i]+b[i]);
    }
    return result;
}
