#include <matlab_lib/utils.h>

std::string mxClassIDToStr(mxClassID id)
{
    if (id == mxDOUBLE_CLASS) return "double";
    else if (id == mxSINGLE_CLASS) return "float";
    else if (id == mxINT8_CLASS) return "int8";
    else if (id == mxUINT8_CLASS) return "uint8";
    else if (id == mxINT16_CLASS) return "int16";
    else if (id == mxUINT16_CLASS) return "uint16";
    else if (id == mxINT32_CLASS) return "int32";
    else if (id == mxUINT32_CLASS) return "uint32";
    else if (id == mxINT64_CLASS) return "int64";
    else if (id == mxUINT64_CLASS) return "uint64";
    else if (id == mxCHAR_CLASS) return "string";
    else if (id == mxLOGICAL_CLASS) return "bool";
    else if (id == mxSTRUCT_CLASS) return "struct";
    else if (id == mxCELL_CLASS) return "cell";
    else return "unknown";
}

std::string mxArrayInfo(mxArray *pVar)
{
    std::string info = mxClassIDToStr(mxGetClassID(pVar));
    if (mxIsNumeric(pVar) && !mxIsScalar(pVar))
    {
        int numDims = mxGetNumberOfDimensions(pVar);
        const mwSize *dims = mxGetDimensions(pVar);
        info = "array<" + info + ">(" + std::to_string(dims[0]);
        for (int i = 1; i < numDims; i++) info += ", " + std::to_string(dims[i]);
        info += ")";
    }
    
    return info;
}
