#include <matlab_lib/mat_file_reader.h>
#include <matlab_lib/mat_file_writer.h>

void test_write(const std::string &filename)
{
    std::cout << "\33[1;36m ======= Test WRITE ==========\33[0m\n";

    arma::mat m1 = {{-1.2, -2.4, -4.8}, {1.2, 2.4, 4.8}};

    Eigen::VectorXd v(6);
    v << 1, 2, 3, 4, 5, 6;

    Eigen::MatrixXd m2(3, 2);
    m2 << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;

    arma::uvec arma_uv = {10, 20, 30, 40, 50};

    std::vector<int> std_uv = {1, 2, 3, 4, 5};

    double a = 5.3;

    std::string str = "Hello World!";

    MatFileWriter fid(filename);

    fid.write("m1", m1);
    fid.write("m2", m2);
    fid.write("arma_uv", arma_uv);
    fid.write("std_uv", std_uv);
    fid.write("v", v);
    fid.write("a", a);
    fid.write("str", str);

    arma::Row<int> print_std_uv = std_uv;

    std::cerr << "\33[1;36m=====> Wrote the following data:\n\33[0m";
    std::cerr << "\n m1 = \n" << m1 << "\n";
    std::cerr << "\n arma_uv = \n" << arma_uv << "\n";
    std::cerr << "\n std_uv = \n" << print_std_uv << "\n";
    std::cerr << "\n v = \n" << v << "\n";
    std::cerr << "\n a = \n" << a << "\n";
    std::cerr << "\n str = \n" << str << "\n";

    std::cout << "\33[1;36m ======= DONE ==========\33[0m\n";
}

void test_read(const std::string &filename)
{   
    std::cout << "\33[1;34m ======= Test READ ==========\33[0m\n";

    arma::mat m1;
    Eigen::MatrixXd m2;
    Eigen::VectorXd v;
    arma::uvec arma_uv;
    std::vector<int> std_uv;
    std::string str;
    double a;

    MatFileReader fid(filename);

    fid.printVariablesInfo();

    fid.read("m1", m1);
    fid.read("m2", m2);
    fid.read("arma_uv", arma_uv);
    fid.read("std_uv", std_uv);
    fid.read("v", v);
    fid.read("a", a);
    fid.read("str", str);

    arma::Row<int> print_std_uv = std_uv;

    std::cerr << "\33[1;36m=====> Read the following data:\n\33[0m";
    std::cerr << "\n m1 = \n" << m1 << "\n";
    std::cerr << "\n arma_uv = \n" << arma_uv << "\n";
    std::cerr << "\n std_uv = \n" << print_std_uv << "\n";
    std::cerr << "\n v = \n" << v << "\n";
    std::cerr << "\n a = \n" << a << "\n";
    std::cerr << "\n str = \n" << str << "\n";

    // std::cout << "\33[1;34m ======= DONE ==========\33[0m\n";
}

int main(int argc, char **argv)
{
    std::string filename = "my_data.mat";

    test_write(filename);

    test_read(filename);

    return 0;
}
