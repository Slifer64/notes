#ifndef AS64_MATLAB_LIB_MAT_FILE_WRITER_H
#define AS64_MATLAB_LIB_MAT_FILE_WRITER_H

#include <matlab_lib/utils.h>

class MatFileWriter
{
public:
    // Constructor that takes a string as input and creates a MATLAB mat file
    MatFileWriter(const std::string &filename)
    {
        this->filename = filename;
        open();
    }

    ~MatFileWriter()
    {
        close();
    }

    // write scalar
    template <typename T>
    void write(const std::string name, const T &data)
    {
        mxArray *mxData = mxCreateNumericMatrix(1, 1, mxClassIDFromType<T>(), mxREAL);
        if (mxData == NULL)
            throw std::runtime_error("\33[1;31mERROR:\33[0m Could not create mxArray for data\n");
        *reinterpret_cast<T *>(mxGetData(mxData)) = data;
        if (matPutVariable(file, name.c_str(), mxData) != 0)
        {
            throw std::runtime_error("\33[1;31mERROR:\33[0m Could not write variable '" + name + "' to MATLAB mat file\n");
            mxDestroyArray(mxData);
        }
        mxDestroyArray(mxData);
    }

    // write string
    void write(const std::string &name, const std::string &str)
    {
        write(name, str.c_str());
    }

    void write(const std::string &name, const char *str)
    {
        mxArray *strMatArray = mxCreateString(str);
        if (matPutVariable(file, name.c_str(), strMatArray) != 0)
        {
            throw std::runtime_error("\33[1;31mERROR:\33[0m Could not write variable to .mat file\n");
            mxDestroyArray(strMatArray);
        }
        mxDestroyArray(strMatArray);
    }

    // Overload of write for std::vector
    template <typename T>
    void write(const std::string name, const std::vector<T> &data)
    {
        mxArray *mxData = mxCreateNumericMatrix(data.size(), 1, mxClassIDFromType<T>(), mxREAL);
        if (mxData == NULL)
        {
            throw std::runtime_error("\33[1;31mERROR:\33[0m Could not create mxArray for data\n");
        }
        std::copy(data.begin(), data.end(), reinterpret_cast<T *>(mxGetData(mxData)));
        if (matPutVariable(file, name.c_str(), mxData) != 0)
        {
            throw std::runtime_error("\33[1;31mERROR:\33[0m Could not write variable '" + name + "' to MATLAB mat file\n");
            mxDestroyArray(mxData);
        }
        mxDestroyArray(mxData);
    }

    // Overload of write for Armadillo data types
    template <typename T>
    void write(const std::string name, const arma::Col<T> &data)
    {
        __write__(name, data.memptr(), data.n_elem, 1);
    }

    template <typename T>
    void write(const std::string name, const arma::Row<T> &data)
    {
        __write__(name, data.memptr(), 1, data.n_elem);
    }

    template <typename T>
    void write(const std::string name, const arma::Mat<T> &data)
    {
        __write__(name, data.memptr(), data.n_rows, data.n_cols);
    }

    // Overload of write for Eigen data types
    template <typename T>
    void write(const std::string name, const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &data)
    {
        __write__(name, data.data(), data.rows(), data.cols());
    }

    template <typename T>
    void write(const std::string name, const Eigen::Matrix<T, 1, Eigen::Dynamic> &data)
    {
        __write__(name, data.data(), 1, data.size());
    }

    template <typename T>
    void write(const std::string name, const Eigen::Matrix<T, Eigen::Dynamic, 1> &data)
    {
        __write__(name, data.data(), data.size(), 1);
    }

private:
    MATFile *file;
    std::string filename;

    // Open the .mat file
    void open()
    {
        file = matOpen(filename.c_str(), "w");
        if (!file)
            throw std::ios_base::failure("\33[1;31mERROR:\33[0m Failed to create '" + filename + "'\n");
    }

    // Close the .mat file
    void close()
    {
        if (file && matClose(file) != 0)
            throw std::ios_base::failure("\33[1;31mERROR:\33[0m Failed to close file '" + filename + "'\n");
    }

    // Function that writes data to the mat file
    template <typename T>
    void __write__(const std::string name, const T *data, const size_t n_rows, const size_t n_cols)
    {
        // mxArray *mxData = mxCreateNumericMatrix(nElements, 1, mxClassIDFromType<typeid(*reinterpret_cast<const double *>(data))>(), mxREAL);
        mxArray *mxData = mxCreateNumericMatrix(n_rows, n_cols, mxClassIDFromType<T>(), mxREAL);
        if (mxData == NULL)
            throw std::runtime_error("\33[1;31mERROR:\33[0m Could not create mxArray for data\n");

        std::memcpy(mxGetData(mxData), data, n_rows * n_cols * sizeof(T));
        if (matPutVariable(file, name.c_str(), mxData) != 0)
        {
            throw std::runtime_error("\33[1;31mERROR:\33[0m Could not write variable '" + name + "' to MATLAB mat file\n");
            mxDestroyArray(mxData);
        }
        mxDestroyArray(mxData);
    }
};

#endif // AS64_MATLAB_LIB_MAT_FILE_WRITER_H