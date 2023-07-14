#ifndef AS64_MATLAB_LIB_MAT_FILE_READER_H
#define AS64_MATLAB_LIB_MAT_FILE_READER_H

#include <matlab_lib/utils.h>

class MatFileReader
{
public:
    // Constructor
    MatFileReader(const std::string &filename)
    {
        this->filename = filename;
        open();
    }

    // Destructor
    ~MatFileReader()
    {
        close();
    }

    // Read Armadillo matrix
    template <typename T>
    void read(const std::string &name, arma::Mat<T> &m)
    {
        size_t n_rows, n_cols;
        mxArray *var = __read__<T>(name, &n_rows, &n_cols);
        m = arma::Mat<T>(static_cast<T *>(mxGetData(var)), n_rows, n_cols, true);
        mxDestroyArray(var);
    }

    // Read Armadillo column vector
    template <typename T>
    void read(const std::string &name, arma::Col<T> &c)
    {
        size_t n_elem;
        mxArray *var = __read__<T>(name, &n_elem, NULL);
        c = arma::Col<T>(static_cast<T *>(mxGetData(var)), n_elem);
        mxDestroyArray(var);
    }

    // Read Armadillo row vector
    template <typename T>
    void read(const std::string &name, arma::Row<T> &r)
    {
        size_t n_elem;
        mxArray *var = __read__<T>(name, NULL, &n_elem);
        r = arma::Col<T>(static_cast<T *>(mxGetData(var)), n_elem);
        mxDestroyArray(var);
    }

    // Read Eigen matrix
    template <typename T>
    void read(const std::string &name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m)
    {
        size_t n_rows, n_cols;
        mxArray *var = __read__<T>(name, &n_rows, &n_cols);
        m = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>(static_cast<T *>(mxGetData(var)), n_rows, n_cols);
        mxDestroyArray(var);
    }

    template <typename T>
    void read(const std::string &name, Eigen::Matrix<T, Eigen::Dynamic, 1> &c)
    {
        size_t n_elem;
        mxArray *var = __read__<T>(name, &n_elem, NULL);
        c = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>(static_cast<T *>(mxGetData(var)), n_elem);
        mxDestroyArray(var);
    }

    template <typename T>
    void read(const std::string &name, Eigen::Matrix<T, 1, Eigen::Dynamic> &r)
    {
        size_t n_elem;
        mxArray *var = __read__<T>(name, &n_elem, NULL);
        r = Eigen::Map<Eigen::Matrix<T, 1, Eigen::Dynamic>>(static_cast<T *>(mxGetData(var)), n_elem);
        mxDestroyArray(var);
    }

    // Read std::vector
    template <typename T>
    void read(const std::string &name, std::vector<T> &v)
    {
        size_t n_rows, n_cols;
        mxArray *var = __read__<T>(name, &n_rows, &n_cols);
        T *ptr = static_cast<T *>(mxGetData(var));
        v = std::vector<T>(ptr, ptr + (n_rows * n_cols));
        mxDestroyArray(var);
    }

    // Read std::string
    void read(const std::string &name, std::string &s)
    {
        // Get the MATLAB array with the given name
        mxArray *var = matGetVariable(pmat, name.c_str());

        assert_exists(var, name);
        assert_is_string(var, name);

        // Get the length of the MATLAB character array
        size_t len = mxGetNumberOfElements(var);

        // Allocate memory for the string and copy the data
        char *data = mxArrayToString(var);
        s = std::string(data, len);

        // Clean up the MATLAB array and the allocated memory
        mxDestroyArray(var);
        mxFree(data);
    }

    // Read scalar
    template <typename T>
    bool read(const std::string &name, T &val)
    {
        // Get the MATLAB array with the given name
        mxArray *var = matGetVariable(pmat, name.c_str());

        assert_exists(var, name);
        assert_is_scalar(var, name);
        assert_is_same_class<T>(var, name);

        // Copy the data into the output scalar
        val = *reinterpret_cast<T *>(mxGetData(var));

        // Clean up the MATLAB array
        mxDestroyArray(var);

        return true;
    }

    void printVariablesInfo(std::ostream &out=std::cout)
    {
        const char **dir;
        int ndir;
        dir = (const char **)matGetDir(pmat, &ndir);
        if (dir == NULL)
            throw std::runtime_error("Error reading contents of file " + filename + "\n");
        mxFree(dir);

        /* In order to use matGetNextXXX correctly, reopen file to read in headers. */
        close();
        open();

        /* Get headers of all variables */
        const char *name;
        mxArray *pVar;
        int name_len = 0;
        int type_len = 0;
        std::vector<std::string> var_name;
        std::vector<std::string> var_type;
        for (int i = 0; i < ndir; i++)
        {
            pVar = matGetNextVariableInfo(pmat, &name);
            if (pVar == NULL)
                throw std::runtime_error("Error reading contents of file " + filename + "\n");

            int len = strlen(name);
            if (len > name_len) name_len = len;
            var_name.push_back(name);

            std::string type = mxArrayInfo(pVar);
            if (type.length() > type_len) type_len = type.length();
            var_type.push_back(type);

            mxDestroyArray(pVar);
        }
        name_len += 5;
        type_len += 5;

        std::string horiz_line(name_len + type_len, '-');
        std::string line2(int((horiz_line.length() - filename.length() - 2)/2) , '#');
        std::string sp_(name_len - strlen("name"), ' ');

        out << line2 << " \33[1m" + filename + "\33[0m " << line2 << "\n";
        // out << horiz_line << "\n";
        out << "\33[1mname\33[0m" << sp_ << "\33[1mtype (size)\33[0m" << std::endl
            << horiz_line << std::endl;
        for (int k=0; k<var_name.size(); k++)
            out << std::setw(name_len) << std::left << var_name[k]
                << std::setw(type_len) << std::left << var_type[k] << "\n";
        out << horiz_line << "\n";
    }

private:

    // Open the .mat file
    void open()
    {
        pmat = matOpen(filename.c_str(), "r");
        if (!pmat)
            throw std::ios_base::failure("\33[1;31mERROR:\33[0m Failed to open '" + filename + "'\n");
    }

    // Close the .mat file
    void close()
    {
        if (pmat && matClose(pmat) != 0)
            throw std::ios_base::failure("\33[1;31mERROR:\33[0m Failed to close file '" + filename + "'\n");
    }

    void assert_exists(mxArray *var, const std::string &name)
    {
        if (var == nullptr)
            throw std::runtime_error("\33[1;31mERROR:\33[0m Variable '" + name + "': does't exist.\n");
    }

    void assert_is_array(mxArray *var, const std::string &name)
    {
        if (!mxIsNumeric(var))
            throw std::runtime_error("\33[1;31mERROR:\33[0m Variable '" + name + "': does not contain numbers.\n");
    }

    void assert_is_string(mxArray *var, const std::string &name)
    {
        if (!mxIsChar(var))
            throw std::runtime_error("\33[1;31mERROR:\33[0m Variable '" + name + "': does not contain characters.\n");
    }

    void assert_is_scalar(mxArray *var, const std::string &name)
    {
        if (!mxIsScalar(var))
            throw std::runtime_error("\33[1;31mERROR:\33[0m Variable '" + name + "': is not scalar.\n");
    }

    template <typename T>
    void assert_is_same_class(mxArray *var, const std::string &name)
    {
        // Check that the variable has the expected type
        mxClassID source_class = mxGetClassID(var);
        mxClassID dest_class = mxClassIDFromType<T>();
        if (source_class != dest_class)
            throw std::runtime_error("\33[1;31mERROR:\33[0m Variable '" + name + "': incompatible types: source is '" + mxClassIDToStr(source_class) + "', dest is '" + mxClassIDToStr(dest_class) + "'\n");
    }

    template <typename T>
    mxArray *__read__(const std::string &name, size_t *n_rows, size_t *n_cols)
    {
        // Get the MATLAB array with the given name
        mxArray *var = matGetVariable(pmat, name.c_str());

        assert_exists(var, name);
        assert_is_array(var, name);
        assert_is_same_class<T>(var, name);

        // Get the size and data pointer of the MATLAB array
        mwSize numDims = mxGetNumberOfDimensions(var);
        const mwSize *dims = mxGetDimensions(var);
        // T* data = static_cast<T*>(mxGetData(var));

        if (!n_rows && dims[0] > 1)
            throw std::runtime_error("\33[1;31mERROR:\33[0m Cannot read '" + name + "' which is " + std::to_string(dims[0]) + " x " + std::to_string(dims[1]) + " and write it to a column vector...\n");
        if (n_rows) *n_rows = dims[0];

        if (!n_cols && dims[1] > 1)
            throw std::runtime_error("\33[1;31mERROR:\33[0m Cannot read '" + name + "' which is " + std::to_string(dims[0]) + " x " + std::to_string(dims[1]) + " and write it to a row vector...\n");
        if (n_cols) *n_cols = dims[1];

        return var;
    }

    // Pointer to the MATFile struct
    MATFile *pmat = nullptr;
    std::string filename;
};

#endif // AS64_MATLAB_LIB_MAT_FILE_READER_H