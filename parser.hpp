#pragma once

#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <locale>
#include <vector>
#include <cmath>

#include "linalg.hpp"
#include "trench.hpp"

struct Vertex {
    double x;
    double y;
    double z;
};

class Parser {
  private :
    char* dxf_filename;
    char* usermap_filename;

  public :
    std::vector<std::string> split(const std::string &input,const char &delimiter) //입력 문자열을 주어진 구분자(delimiter)를 사용하여 분할
    {
      std::vector<std::string> answer;
      std::stringstream ss(input);
      std::string temp;

      while (getline(ss,temp,delimiter))
      {
        answer.push_back(temp);
      }
      return answer;
    }
    std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> parse_dxf(const char* &dxf_filename) //DXF 파일에서 '3D FACE' 개체의 좌표를 추출하여 반환하는 함수, 입력: DXF 파일 이름, 출력: '3DFACE' 개체의 좌표를 담은 두 개의 벡터 쌍
    {
      std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> output_vec_pair; //output_vec_pair는 최종 반환값으로 두 개의 벡터 쌍을 담음
      std::vector<std::vector<double>> output_vec_final;  //3DFACE 개체의 좌표를 저장하는 벡터
      std::vector<std::vector<double>> offset_arr;    //초기값 포함된 벡터
      std::vector<double> offset_arr_raw = {0, 0, 0}; //초기값을 갖는 벡터
      offset_arr.push_back(offset_arr_raw);
      output_vec_pair.second = offset_arr;

      std::ifstream file;
      try
      {
        file.open(dxf_filename);
      }
      catch(std::exception& e) // 파일열기 실패시 예외처리
      {
        output_vec_pair.first = output_vec_final;
        return output_vec_pair;
      }
      std::string line;
      if(file.is_open())
      {
        std::string input_str("3DFACE");
        std::string start_str("10"); //10을 찾으면 Parse_flag를 설정하고 save_cnt를 0 으로 초기화
        std::string end_str("33"); //13을 찾으면 parse_flag와 module_flag를 해제하고 output_vec(현재 좌표 벡터를)를 output_vec_final에 추가하고 플래그 해제
        bool module_flag = false; //상태 플래그
        bool parse_flag = false;
        std::vector<double> output_vec;
        int save_cnt = 123;
        while(getline(file, line))
        {
          line.erase(std::remove_if(line.begin(), line.end(), std::bind(std::isspace< char > , std::placeholders::_1,std::locale::classic())),line.end());

          if (line.compare(input_str) == 0)
          {
            module_flag = true;
          }
          if (module_flag == true)
          {
            if (parse_flag == true && line.compare(end_str) == 0)
            {
              output_vec_final.push_back(output_vec);
              output_vec.clear();

              parse_flag = false;
              module_flag = false;
            }
            if (line.compare(start_str) == 0)
            {
              parse_flag = true;
              save_cnt = 0;
            }
            if (parse_flag == true)
            {

              if (save_cnt%2 == 1) //save_cnt가 홀수일때 좌표값을 output_vec에 추가
              {
                double tmp_double = std::stod(line);
                output_vec.push_back(tmp_double);
              }
              save_cnt ++;  //읽은 줄 수 카운트
            }
          }
        }
        file.close();
      }

      output_vec_pair.first = output_vec_final; //추출한 좌표 벡터를 'output_vec_pair.first에 저장 후 반환

      std::cout << std::fixed << std::setprecision(6); // 소수점 이하 6자리까지 출력

      // DrawCnt 저장
      double Num_VW = static_cast<double>(output_vec_final.size());
      std::ofstream num_vw_file("/home/user/chanyoung/DrawNum_test.txt");
      num_vw_file << std::fixed << std::setprecision(6) << Num_VW << std::endl;
      num_vw_file.close();

      // 3DFACE X,Y 데이터 출력
      std::cout << "Output Vector Final (3DFACE coordinates):" << std::endl;
      for (const auto& vec : output_vec_pair.first) {
        for (const auto& val : vec) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
       }

       // vw_count 계산 및 출력
        std::vector<double> vw_count;
        for (const auto& vec : output_vec_pair.first) {
            if (vec[6] == vec[9] && vec[7] == vec[10]) {
                vw_count.push_back(3);
            } else {
                vw_count.push_back(4);
            }
        }

        //std::cout << "VW Count:" << std::endl;
        for (const auto& count : vw_count) {
        std::cout << std::fixed << std::setprecision(6) << count << std::endl;
    }

        // vw_count를 파일에 저장
        std::ofstream vw_count_file("/home/user/chanyoung/DrawCnt_test.txt");
        vw_count_file << std::fixed << std::setprecision(6);
        for (const auto& count : vw_count) {
            vw_count_file << count << std::endl;
        }
        vw_count_file.close();

        //x,y text 파일 생성
        std::ofstream draw_x_file("/home/user/chanyoung/DrawX_test.txt");
        std::ofstream draw_y_file("/home/user/chanyoung/DrawY_test.txt");

        //x,y 구분 후 출력  로직
        for (size_t i = 0; i < output_vec_pair.first.size(); ++i) {
    if (vw_count[i] == 3) {
        // VW count is 3: Print 0, 3, 6 x-coordinates and 1, 4, 7 y-coordinates
        std::cout << "Entry " << i+1 << " - X coordinates: " << output_vec_pair.first[i][0] << ", "
                  << output_vec_pair.first[i][3] << ", " << output_vec_pair.first[i][6] << std::endl;
        std::cout << "Entry " << i+1 << " - Y coordinates: " << output_vec_pair.first[i][1] << ", "
                  << output_vec_pair.first[i][4] << ", " << output_vec_pair.first[i][7] << std::endl;

        // Write to DrawX_test.txt
        draw_x_file << std::fixed << std::setprecision(6) << output_vec_pair.first[i][0] << std::endl
                    << output_vec_pair.first[i][3] << std::endl
                    << output_vec_pair.first[i][6] << std::endl;

        // Write to DrawY_test.txt
        draw_y_file << std::fixed << std::setprecision(6) << output_vec_pair.first[i][1] << std::endl
                    << output_vec_pair.first[i][4] << std::endl
                    << output_vec_pair.first[i][7] << std::endl;

    } else if (vw_count[i] == 4) {
        // VW count is 4: Print 0, 3, 6, 9 x-coordinates and 1, 4, 7, 10 y-coordinates
        std::cout << "Entry " << i+1 << " - X coordinates: " << output_vec_pair.first[i][0] << ", "
                  << output_vec_pair.first[i][3] << ", " << output_vec_pair.first[i][6] << ", "
                  << output_vec_pair.first[i][9] << std::endl;
        std::cout << "Entry " << i+1 << " - Y coordinates: " << output_vec_pair.first[i][1] << ", "
                  << output_vec_pair.first[i][4] << ", " << output_vec_pair.first[i][7] << ", "
                  << output_vec_pair.first[i][10] << std::endl;

        // Write to DrawX_test.txt
        draw_x_file << std::fixed << std::setprecision(6) << output_vec_pair.first[i][0] << std::endl
                    << output_vec_pair.first[i][3] << std::endl
                    << output_vec_pair.first[i][6] << std::endl
                    << output_vec_pair.first[i][9] << std::endl;

        // Write to DrawY_test.txt
        draw_y_file << std::fixed << std::setprecision(6) << output_vec_pair.first[i][1] << std::endl
                    << output_vec_pair.first[i][4] << std::endl
                    << output_vec_pair.first[i][7] << std::endl
                    << output_vec_pair.first[i][10] << std::endl;
    }
}

        draw_x_file.close();
        draw_y_file.close();

      return output_vec_pair;

    }

    std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> parse_usr(const char* &usermap_filename) //파일을 읽어 다양한 오프셋을 포함한 3D포인트 데이터 파싱. 파싱된 데이터 기반 트렌치와 평면 데이터를 처리하여 3D좌표 추출하고 반환
    {
      std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> output_pair;
      std::vector<std::vector<double>> output_vec_final;


      int parse_flag = 0;
      double depth_offset = 0, leftright_offset = 0, frontback_offset =0;

      int side_pt_idx = 0;

      std::string type = "Nan";
      std::string name = "Nan";

      std::vector<int> pt_idx_save;
      std::vector<int> path_pt_idx_save;

      std::vector<double>pt_x_save;
      std::vector<double>pt_y_save;
      std::vector<double>pt_z_save;

      std::vector<double>pt_rel_x_save;
      std::vector<double>pt_rel_y_save;
      std::vector<double>pt_rel_z_save;

      std::vector<double>path_pt_x_save;
      std::vector<double>path_pt_y_save;
      std::vector<double>path_pt_z_save;

      std::vector<double>path_pt_rel_x_save;
      std::vector<double>path_pt_rel_y_save;
      std::vector<double>path_pt_rel_z_save;

      std::ifstream file;

      file.open(usermap_filename);

      std::string line;
      if(file.is_open())
      {
        while(getline(file, line))
        {
          std::vector<std::string>line_seg;

          line.erase(std::remove_if(line.begin(), line.end(), std::bind(std::isspace< char > , std::placeholders::_1,std::locale::classic())),line.end());
          line.erase(std::remove(line.begin(),line.end(),'"'),line.end());
          line.erase(std::remove(line.begin(),line.end(),','),line.end());

          line_seg = split(line,':');

          if(line_seg[0].compare("type") == 0)
          {
            type = line_seg[1];
          }
          if(line_seg[0].compare("name") == 0)
          {
            name = line_seg[1];
          }
          if(line_seg[0].compare("depthOffset") == 0)
          {
            depth_offset = std::stod(line_seg[1]);
          }
          if(line_seg[0].compare("leftRightOffset") == 0)
          {
            leftright_offset = std::stod(line_seg[1]);
          }
          if(line_seg[0].compare("frontBackOffset") == 0)
          {
            frontback_offset = std::stod(line_seg[1]);
          }

          if(line_seg[0].compare("points") == 0)
          {
            parse_flag = 1;
          }
          else if(line_seg[0].compare("pathPoints") == 0)
          {
            parse_flag = 2;
          }

          if(line_seg[0].compare("index") == 0)
          {
            if(parse_flag==1)
            {
              pt_idx_save.push_back(std::stoi(line_seg[1]));
            }
            if(parse_flag==2)
            {
              path_pt_idx_save.push_back(std::stoi(line_seg[1]));
            }
          }

          if(line_seg[0].compare("xPos") == 0)
          {
            if(parse_flag==1)
            {
              pt_rel_x_save.push_back(std::stod(line_seg[1]));
            }
            if(parse_flag==2)
            {
              path_pt_rel_x_save.push_back(std::stod(line_seg[1]));
            }
          }
          if(line_seg[0].compare("xRawPos") == 0)
          {
            if(parse_flag==1)
            {
              pt_x_save.push_back(std::stod(line_seg[1]));
            }
            if(parse_flag==2)
            {
              path_pt_x_save.push_back(std::stod(line_seg[1]));
            }
          }

          if(line_seg[0].compare("yPos") == 0)
          {
            if(parse_flag==1)
            {
              pt_rel_z_save.push_back(std::stod(line_seg[1]));
            }
            if(parse_flag==2)
            {
              path_pt_rel_z_save.push_back(std::stod(line_seg[1]));
            }
          }
          if(line_seg[0].compare("yRawPos") == 0)
          {
            if(parse_flag==1)
            {
              pt_z_save.push_back(std::stod(line_seg[1]));
            }
            if(parse_flag==2)
            {
              path_pt_z_save.push_back(std::stod(line_seg[1]));
            }
          }

          if(line_seg[0].compare("zPos") == 0)
          {
            if(parse_flag==1)
            {
              pt_rel_y_save.push_back(std::stod(line_seg[1]));
            }
            if(parse_flag==2)
            {
              path_pt_rel_y_save.push_back(std::stod(line_seg[1]));
            }
          }
          if(line_seg[0].compare("zRawPos") == 0)
          {
            if(parse_flag==1)
            {
              pt_y_save.push_back(std::stod(line_seg[1]));
            }
            if(parse_flag==2)
            {
              path_pt_y_save.push_back(std::stod(line_seg[1]));
            }
          }

          if(line_seg[0].compare("sidePointIndex") == 0)
          {
            side_pt_idx = std::stoi(line_seg[1]);
          }
        }
        file.close();
      }
      else
      {
        std::vector<std::vector<double>> offset_arr;
        output_pair.first = output_vec_final;
        output_pair.second = offset_arr;
        return output_pair;
      }

      ////////////// generate TIN ///////////////
      int pt_idx_size = pt_idx_save.size();

      if (type.compare("plane") == 0)
      {

        int triangle_num = pt_idx_save.size()-2;

        for(int i= 0;i<triangle_num;i++)
        {
          std::vector<double> output_vec;
          output_vec.push_back(pt_x_save[0]);
          output_vec.push_back(pt_y_save[0]);
          output_vec.push_back(pt_z_save[0]);
          output_vec.push_back(pt_x_save[i+1]);
          output_vec.push_back(pt_y_save[i+1]);
          output_vec.push_back(pt_z_save[i+1]);
          output_vec.push_back(pt_x_save[i+2]);
          output_vec.push_back(pt_y_save[i+2]);
          output_vec.push_back(pt_z_save[i+2]);
          output_vec_final.push_back(output_vec);
        }
      }

      else if (type.compare("profile") == 0)
      {
        std::vector<std::vector<double>> mesh;
        double add_len = 10000;
        int mesh_num = pt_idx_save.size()-1;

        if (pt_idx_size > 1)
        {
          std::vector<double> vec1 = {pt_x_save[1]-pt_x_save[0],pt_y_save[1]-pt_y_save[0],pt_z_save[1]-pt_z_save[0]};
          std::vector<double> vec2 = {0,0,1};
          std::vector<double> dir_vec = const_vec_mul(cross(vec1,vec2),1/norm(cross(vec1,vec2)));
          dir_vec[2] = 0;

          std::vector<std::vector<double>> target_pt_arr;
          target_pt_arr.push_back(pt_x_save);
          target_pt_arr.push_back(pt_y_save);
          target_pt_arr.push_back(pt_z_save);

          std::vector<std::vector<double>> tmp_plane1;
          std::vector<std::vector<double>> tmp_plane2;
          for(int i = 0;i<pt_idx_size;i++)
          {
            std::vector<double> ext_vec = const_vec_mul(dir_vec,add_len);
            std::vector<double> target_pt_arr_col = {target_pt_arr[0][i],target_pt_arr[1][i],target_pt_arr[2][i]};
            tmp_plane1.push_back(vec_add(target_pt_arr_col,ext_vec));
            tmp_plane2.push_back(vec_add(target_pt_arr_col,const_vec_mul(ext_vec,-1)));
          }

          for(int i =0;i<mesh_num;i++)
          {
            std::vector<double> mesh_row;
            mesh_row.push_back(tmp_plane1[i][0]);
            mesh_row.push_back(tmp_plane1[i][1]);
            mesh_row.push_back(tmp_plane1[i][2]);
            mesh_row.push_back(tmp_plane2[i][0]);
            mesh_row.push_back(tmp_plane2[i][1]);
            mesh_row.push_back(tmp_plane2[i][2]);
            mesh_row.push_back(tmp_plane2[i+1][0]);
            mesh_row.push_back(tmp_plane2[i+1][1]);
            mesh_row.push_back(tmp_plane2[i+1][2]);
            mesh_row.push_back(tmp_plane1[i+1][0]);
            mesh_row.push_back(tmp_plane1[i+1][1]);
            mesh_row.push_back(tmp_plane1[i+1][2]);
            mesh.push_back(mesh_row);
          }
          output_vec_final=mesh_to_TIN(mesh);
        }
      }

      else if(type.compare("trench") == 0)
      {
        std::vector<std::vector<double>> output_TIN_raw;

        if (pt_idx_size > 1)
        {
          //// 1st step : generate cross plane

          std::vector<std::vector<double>> mesh_arr;

          std::vector<std::vector<double>> target_path_pt_arr;
          target_path_pt_arr.push_back(path_pt_x_save);
          target_path_pt_arr.push_back(path_pt_y_save);
          target_path_pt_arr.push_back(path_pt_z_save);

          std::vector<std::vector<double>> target_pt_arr;
          target_pt_arr.push_back(pt_x_save);
          target_pt_arr.push_back(pt_y_save);
          target_pt_arr.push_back(pt_z_save);

          std::vector<double> start_vec = {target_path_pt_arr[0][1]-target_path_pt_arr[0][0],target_path_pt_arr[1][1]-target_path_pt_arr[1][0],target_path_pt_arr[2][1]-target_path_pt_arr[2][0]};
          std::vector<double> proj_vec = {start_vec[0], start_vec[1], 0};

          std::vector<double> v1 = {target_pt_arr[0][1]-target_pt_arr[0][0],target_pt_arr[1][1]-target_pt_arr[1][0],target_pt_arr[2][1]-target_pt_arr[2][0]};
          std::vector<double> v2 = {0,0,1};

          std::vector<double> zero_vec = cross(v2,v1);
          zero_vec = const_vec_mul(zero_vec,1/norm(zero_vec));

          std::vector<double> init_y;
          double norm_proj = norm(proj_vec);
          init_y.push_back(proj_vec[0]/norm_proj);
          init_y.push_back(proj_vec[1]/norm_proj);
          init_y.push_back(proj_vec[2]/norm_proj);

          std::vector<double> start_pt = {target_path_pt_arr[0][0],target_path_pt_arr[1][0],target_path_pt_arr[2][0]};
          int start_idx = side_pt_idx;

          std::vector<std::vector<double>> tmp_plane;
          std::vector<std::vector<double>> start_plane;
          int idx_size = pt_idx_save.size();
          for(int i =0;i<idx_size;i++)
          {
            std::vector<double> tmp_plane_row;
            tmp_plane_row.push_back(target_pt_arr[0][i]-target_pt_arr[0][start_idx]);
            tmp_plane_row.push_back(target_pt_arr[1][i]-target_pt_arr[1][start_idx]);
            tmp_plane_row.push_back(target_pt_arr[2][i]-target_pt_arr[2][start_idx]);

            std::vector<double> start_plane_row;
            std::vector<double> rot_plane_row;
            rot_plane_row = rotate_plane(tmp_plane_row,zero_vec,init_y);
            start_plane_row.push_back(rot_plane_row[0]+start_pt[0]);
            start_plane_row.push_back(rot_plane_row[1]+start_pt[1]);
            start_plane_row.push_back(rot_plane_row[2]+start_pt[2]);

            tmp_plane.push_back(tmp_plane_row);
            start_plane.push_back(start_plane_row);
          }

          /// generate consequtive cross planes
          std::vector<std::vector<std::vector<double>>> cross_plane_list_front;
          std::vector<std::vector<std::vector<double>>> cross_plane_list_end;

          int path_pt_idx_size = path_pt_idx_save.size();
          for(int i =0;i<path_pt_idx_size-1;i++)
          {
            if(i == 0)
            {
              cross_plane_list_front.push_back(start_plane);
              cross_plane_list_end.push_back(move_plane(start_plane,start_vec));
            }
            else
            {
              std::vector<double> prev_vec;
              prev_vec.push_back(target_path_pt_arr[0][i]-target_path_pt_arr[0][i-1]);
              prev_vec.push_back(target_path_pt_arr[1][i]-target_path_pt_arr[1][i-1]);
              prev_vec.push_back(target_path_pt_arr[2][i]-target_path_pt_arr[2][i-1]);

              std::vector<double> prev_vec_proj;
              prev_vec_proj.push_back(prev_vec[0]);
              prev_vec_proj.push_back(prev_vec[1]);
              prev_vec_proj.push_back(0);

              std::vector<double> prev_y = const_vec_mul(prev_vec_proj,1/norm(prev_vec_proj));

              std::vector<double> cur_vec;
              cur_vec.push_back(target_path_pt_arr[0][i+1]-target_path_pt_arr[0][i]);
              cur_vec.push_back(target_path_pt_arr[1][i+1]-target_path_pt_arr[1][i]);
              cur_vec.push_back(target_path_pt_arr[2][i+1]-target_path_pt_arr[2][i]);

              std::vector<double> cur_vec_proj;
              cur_vec_proj.push_back(cur_vec[0]);
              cur_vec_proj.push_back(cur_vec[1]);
              cur_vec_proj.push_back(0);

              std::vector<double> cur_y = const_vec_mul(cur_vec_proj,1/norm(cur_vec_proj));
              std::vector<std::vector<double>> tmp_plane_2;
              std::vector<std::vector<double>> tmp_plane_3;

              for(int j =0;j<idx_size;j++)
              {
                std::vector<double> tmp_plane_2_row;
                tmp_plane_2_row.push_back(cross_plane_list_end[i-1][j][0]-target_path_pt_arr[0][i]);
                tmp_plane_2_row.push_back(cross_plane_list_end[i-1][j][1]-target_path_pt_arr[1][i]);
                tmp_plane_2_row.push_back(cross_plane_list_end[i-1][j][2]-target_path_pt_arr[2][i]);

                tmp_plane_2.push_back(tmp_plane_2_row);

                std::vector<double> tmp_plane_3_row;
                std::vector<double> rot_plane_2_row;

                rot_plane_2_row = rotate_plane(tmp_plane_2[j],prev_y,cur_y);

                tmp_plane_3_row.push_back(rot_plane_2_row[0]+target_path_pt_arr[0][i]);
                tmp_plane_3_row.push_back(rot_plane_2_row[1]+target_path_pt_arr[1][i]);
                tmp_plane_3_row.push_back(rot_plane_2_row[2]+target_path_pt_arr[2][i]);

                tmp_plane_3.push_back(tmp_plane_3_row);

              }

              std::vector<std::vector<double>> tmp_plane_4 = move_plane(tmp_plane_3,cur_vec);

              ///// fold-intersection check
              std::vector<std::vector<double>> inter_plane_prev;
              std::vector<std::vector<double>> inter_plane_after;
              bool inter_flag = false;
              for(int j =0;j<idx_size;j++)
              {
                std::vector<std::vector<double>> cur_seg;
                cur_seg.push_back(tmp_plane_3[j]);
                cur_seg.push_back(tmp_plane_4[j]);

                std::vector<std::vector<double>> prev_seg;
                prev_seg.push_back(cross_plane_list_front[i-1][j]);
                prev_seg.push_back(cross_plane_list_end[i-1][j]);

                std::vector<double> cur_seg_0 = {cur_seg[0][0],cur_seg[0][1]};
                std::vector<double> cur_seg_1 = {cur_seg[1][0],cur_seg[1][1]};

                std::vector<double> prev_seg_0 = {prev_seg[0][0],prev_seg[0][1]};
                std::vector<double> prev_seg_1 = {prev_seg[1][0],prev_seg[1][1]};

                inter_status inter_result = intersection_check(cur_seg_0,vec_add(cur_seg_1,const_vec_mul(cur_seg_0,-1)),prev_seg_0,vec_add(prev_seg_1,const_vec_mul(prev_seg_0,-1)));

                if (inter_result.status==true)
                {
                  inter_flag = true;
                  std::vector<double> inter_plane_prev_row = {inter_result.pt[0], inter_result.pt[1], tmp_plane_3[j][2]};
                  inter_plane_prev.push_back(inter_plane_prev_row);
                  inter_plane_after.push_back(inter_plane_prev_row);
                }
                else
                {
                  inter_plane_prev.push_back(cross_plane_list_end[i-1][j]);
                  inter_plane_after.push_back(tmp_plane_3[j]);
                }
              }
              if (inter_flag == true)
              {
                cross_plane_list_end.pop_back();
                cross_plane_list_end.push_back(inter_plane_prev);

                cross_plane_list_front.push_back(inter_plane_after);
              }
              else
              {
                cross_plane_list_front.push_back(tmp_plane_3);
              }
              cross_plane_list_end.push_back(move_plane(tmp_plane_3,cur_vec));
            }
          }


          ///// generate mesh : connect generated planes
          for(int i =0;i<path_pt_idx_size-1;i++)
          {
            if (i > 0)
            {
              for(int j = 0; j<idx_size-1;j++)
              {
                std::vector<std::vector<double>> mesh_elm_2;
                mesh_elm_2.push_back(cross_plane_list_front[i][j]);
                mesh_elm_2.push_back(cross_plane_list_end[i-1][j]);
                mesh_elm_2.push_back(cross_plane_list_end[i-1][j+1]);
                mesh_elm_2.push_back(cross_plane_list_front[i][j+1]);
                std::vector<double> mesh_arr_row;
                for(int a = 0 ; a < 4;a++)
                {
                  for(int b = 0 ; b < 3;b++)
                  {
                    mesh_arr_row.push_back(mesh_elm_2[a][b]);
                  }
                }
                mesh_arr.push_back(mesh_arr_row);
              }
            }

            for(int j=0;j<idx_size-1;j++)
            {
              std::vector<std::vector<double>> mesh_elm;
              mesh_elm.push_back(cross_plane_list_end[i][j]);
              mesh_elm.push_back(cross_plane_list_front[i][j]);
              mesh_elm.push_back(cross_plane_list_front[i][j+1]);
              mesh_elm.push_back(cross_plane_list_end[i][j+1]);
              std::vector<double> mesh_arr_row2;
              for(int a = 0 ; a < 4;a++)
              {
                for(int b = 0 ; b < 3;b++)
                {
                  mesh_arr_row2.push_back(mesh_elm[a][b]);
                }
              }
              mesh_arr.push_back(mesh_arr_row2);
            }
          }

          output_TIN_raw = mesh_to_TIN(mesh_arr);

          ////// Refine TIN

          int output_TIN_size = output_TIN_raw.size();
          std::vector<std::vector<double>> raw_TIN;

          for(int i =0;i<output_TIN_size;i++)
          {
            bool match_flag= false;
            bool error_flag= false;
            std::vector<double> iter_TIN = output_TIN_raw[i];

            int raw_TIN_size = raw_TIN.size();
            for(int j =0;j<raw_TIN_size;j++)
            {
              if (TIN_cmp(iter_TIN,raw_TIN[j]))
              {
                match_flag = true;
              }
            }
            if (TIN_check(iter_TIN))
            {
              error_flag = true;
            }
            if (!match_flag && !error_flag)
            {
              raw_TIN.push_back(iter_TIN);
            }
          }
          output_vec_final=raw_TIN;
        }

      }

      output_pair.first = output_vec_final;
      std::vector<double> offset_arr_raw = {depth_offset, frontback_offset, leftright_offset};
      std::vector<std::vector<double>> offset_arr;
      offset_arr.push_back(offset_arr_raw);
      output_pair.second = offset_arr;

      return output_pair;
    }
    std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> gen_zone(const std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> & input_arr) //추출된 3D좌표를 사용하여 ZONE 개체 생성
    {
      std::vector<std::vector<double>> parsed_arr = input_arr.first;

      //find max x. y val

      std::vector<double> x_parsed_arr;
      std::vector<double> y_parsed_arr;

      int parsed_row = parsed_arr.size();

      for(int i = 0 ; i < 3 ;i++)
      {
        for(int j =0 ; j<parsed_row ;j++)
        {
          x_parsed_arr.push_back(parsed_arr[j][3*i]);
        }
      }

      for(int i = 0 ; i < 3 ;i++)
      {
        for(int j =0 ; j<parsed_row ;j++)
        {
          y_parsed_arr.push_back(parsed_arr[j][3*i+1]);
        }
      }

      double x_max = find_max(x_parsed_arr) + 1;
      double x_min = find_min(x_parsed_arr) - 1;

      double y_max = find_max(y_parsed_arr) + 1;
      double y_min = find_min(y_parsed_arr) - 1;

      //choose grid size
      std::vector<double> x_diff_arr;

      for(int i =0 ; i<parsed_row ;i++)
      {
        x_diff_arr.push_back(parsed_arr[i][3]-parsed_arr[i][0]);
        x_diff_arr.push_back(parsed_arr[i][6]-parsed_arr[i][3]);
      }

      std::vector<double> y_diff_arr;

      for(int i =0 ; i<parsed_row ;i++)
      {
        y_diff_arr.push_back(parsed_arr[i][4]-parsed_arr[i][1]);
        y_diff_arr.push_back(parsed_arr[i][7]-parsed_arr[i][4]);
      }

      double dx_max = find_max(x_diff_arr);
      double dy_max = find_max(y_diff_arr);

      double grid_size = max(dx_max,dy_max);

      int x_n = (x_max - x_min) / grid_size +4;
      int y_n = (y_max - y_min) / grid_size +4;

      //generate zone x,y points
      std::vector<double> x_points;
      std::vector<double> y_points;

      for(int i =0;i<x_n;i++)
      {
        x_points.push_back(x_min + grid_size*(i-1));
      }

      for(int i =0;i<y_n;i++)
      {
        y_points.push_back(y_min + grid_size*(i-1));
      }

      //generate zone

      std::vector<std::vector<double>> zone;

      for(int i = 0;i < y_n-1;i++)
      {
        for(int k = 0; k < x_n-1;k++)
        {
          std::vector<double> zone_row;
          zone_row.push_back(x_points[k]);
          zone_row.push_back(y_points[i]);
          zone_row.push_back(x_points[k]);
          zone_row.push_back(y_points[i+1]);
          zone_row.push_back(x_points[k+1]);
          zone_row.push_back(y_points[i]);
          zone_row.push_back(x_points[k+1]);
          zone_row.push_back(y_points[i+1]);
          zone.push_back(zone_row);
        }
      }

      //find zone number of parsed_arr
      std::vector<std::vector<double>> add_arr;
      std::vector<double> zone_idx_arr;
      for (int i = 0; i < parsed_row;i++)
      {
        double temp_x = (parsed_arr[i][0]+parsed_arr[i][3]+parsed_arr[i][6])/3;
        double temp_y = (parsed_arr[i][1]+parsed_arr[i][4]+parsed_arr[i][7])/3;

        int zone_size = zone.size();
        for (int k =0 ;k<zone_size;k++)
        {
          if (temp_x <= zone[k][6] &&  temp_x > zone[k][0] && temp_y <= zone[k][7] && temp_y > zone[k][1]) zone_idx_arr.push_back(k);
        }
      }
      zone_idx_arr.push_back(x_n);
      add_arr.push_back(zone_idx_arr);

      std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> result_pair;
      result_pair.first = zone;
      result_pair.second = add_arr;

      return result_pair;

    }

};
