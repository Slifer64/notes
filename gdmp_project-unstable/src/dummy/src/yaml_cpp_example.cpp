#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <type_traits>

#include <armadillo>

#include <ros/package.h>

#include <yaml-cpp/yaml.h>

struct MyStruct
{
    std::string name;
    int passcode;
    arma::mat nums;
};

template<typename T>
void printStdVector(const std::vector<T> &v, const std::string &name)
{
    std::cout << name << ": ";
    for (int i=0; i<v.size()-1; i++) std::cout << v[i] << ", ";
    std::cout << v.back() << "\n";
}

std::ostream &operator<<(std::ostream &out, const struct MyStruct &s)
{
    out << "struct:\n"
        << "    name: " << s.name << "\n"
        << "    passcode: " << s.passcode << "\n"
        << "    nums: \n[";
    // for (int i=0; i<s.nums.size()-1;i++) out << s.nums[i] << ", ";
    // out << s.nums.back() << "]";
    out << s.nums << "]";
    return out;
}

void throwParamLoadError(const std::string &param)
{
    throw std::runtime_error("Failed to load param \"" + param + "\"...");
}

// ===============================================================

namespace YAML
{
    template<>
    struct convert<MyStruct> 
    {
        static Node encode(const MyStruct& rhs) 
        {
            Node node;
            node["name"] = rhs.name;
            node["passcode"] = rhs.passcode;
            node["nums"] = rhs.nums;
            // for (int i=0; i<rhs.nums.size(); i++) node["nums"].push_back(rhs.nums[i]);
            return node;
        }

        static bool decode(const Node& node, MyStruct& rhs) 
        {
            if( !node.IsMap() ) return false;

            if ( !YAML::getParam(node, "name", rhs.name) ) return false;
            if ( !YAML::getParam(node, "passcode", rhs.passcode) ) return false;
            if ( !YAML::getParam(node, "nums", rhs.nums) ) return false;
            return true;
        }
    };

} // namespace YAML

// ===============================================================

int main(int argc, char **argv)
{
    std::string base_path = ros::package::getPath("dummy") + "/config/";

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

