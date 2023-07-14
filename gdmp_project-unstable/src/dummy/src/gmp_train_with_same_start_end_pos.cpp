#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <armadillo>

#include <ros/package.h>

#include <gmp_lib/gmp_lib.h>

// ===============================================================

using namespace as64_;

int main(int argc, char **argv)
{
    std::string path = ros::package::getPath("dummy") + "/data/";
    
    gmp_::FileIO fid(path + "train_data.bin");

    YAML::Node config = YAML::LoadFile(base_path + "config.yaml");

    std::vector<int> v;
    if ( !YAML::getParam(config, "my_vector", v) ) throwParamLoadError("my_vector");
    printStdVector(v, "my_vector");

    std::vector<int> v2;
    if ( !YAML::getParam(config, "other_vector", v2) ) throwParamLoadError("other_vector");
    printStdVector(v2, "other_vector");
    
    // YAML::Node node;
    // if ( !YAML::getParam(config, "my_struct", node) ) throwParamLoadError("my_struct");
    // struct MyStruct my_struct;
    // if ( !YAML::getParam(node, "name", my_struct.name) ) throwParamLoadError("my_struct.name");
    // if ( !YAML::getParam(node, "passcode", my_struct.passcode) ) throwParamLoadError("my_struct.passcode");
    // if ( !YAML::getParam(node, "nums", my_struct.nums) ) throwParamLoadError("my_struct.nums");
    // std::cout << my_struct << "\n";

    struct MyStruct my_struct;
    if ( !YAML::getParam(config, "my_struct", my_struct) ) throwParamLoadError("my_struct");
    std::cout << my_struct << "\n";

    arma::vec arm_vec;
    if ( !YAML::getParam(config, "arm_vec", arm_vec) ) throwParamLoadError("arm_vec");
    std::cout << "arm_vec: " << arm_vec.t() << "\n";

    arma::rowvec arm_row_vec;
    if ( !YAML::getParam(config, "arm_row_vec", arm_row_vec) ) throwParamLoadError("arm_row_vec");
    std::cout << "arm_row_vec: " << arm_row_vec << "\n";

    arma::mat arma_mat;
    if ( !YAML::getParam(config, "arma_mat", arma_mat) ) throwParamLoadError("arma_mat");
    std::cout << "arma_mat:\n" << arma_mat << "\n";

    std::ofstream fout( (base_path + "out_config.yaml").c_str() );
    fout << config;

}

