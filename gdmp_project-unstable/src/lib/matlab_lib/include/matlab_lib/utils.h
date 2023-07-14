#ifndef AS64_MATLAB_LIB_H
#define AS64_MATLAB_LIB_H

#include <iostream>
#include <type_traits> // std::is_class
#include <iomanip>
#include <string>
#include <vector>
#include <exception>
#include <armadillo>
#include <Eigen/Dense>
#include <mat.h>

#define MAT_IO_PRINT_ERR_MSG

template <typename T>
mxClassID mxClassIDFromType()
{
    if (std::is_same<T, double>::value)
        return mxDOUBLE_CLASS;
    else if (std::is_same<T, float>::value)
        return mxSINGLE_CLASS;
    else if (std::is_same<T, int8_t>::value)
        return mxINT8_CLASS;
    else if (std::is_same<T, uint8_t>::value)
        return mxUINT8_CLASS;
    else if (std::is_same<T, int16_t>::value)
        return mxINT16_CLASS;
    else if (std::is_same<T, uint16_t>::value)
        return mxUINT16_CLASS;
    else if (std::is_same<T, int32_t>::value)
        return mxINT32_CLASS;
    else if (std::is_same<T, uint32_t>::value)
        return mxUINT32_CLASS;
    else if (std::is_same<T, int64_t>::value)
        return mxINT64_CLASS;
    else if (std::is_same<T, long long>::value)
        return mxINT64_CLASS;
    else if (std::is_same<T, uint64_t>::value)
        return mxUINT64_CLASS;
    else if (std::is_same<T, unsigned long long>::value)
        return mxUINT64_CLASS;
    else if (std::is_same<T, std::string>::value)
        return mxCHAR_CLASS;
    else if (std::is_same<T, bool>::value)
        return mxLOGICAL_CLASS;
    else if (std::is_class<T>::value)
        return mxSTRUCT_CLASS;
    // else if (std::is_same<T, std::vector>::value)
    //     return mxCELL_CLASS;
    else
    {   
        #ifdef MAT_IO_PRINT_ERR_MSG
        std::cerr << "\33[1;31mERROR:\33[0m mxUNKNOWN_CLASS: Unknown data type..." << std::endl;
        #endif
        return mxUNKNOWN_CLASS;
    }
}

std::string mxClassIDToStr(mxClassID id);

std::string mxArrayInfo(mxArray *pVar);

#endif // AS64_MATLAB_LIB_H