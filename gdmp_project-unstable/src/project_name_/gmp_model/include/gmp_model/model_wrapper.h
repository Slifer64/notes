#ifndef $_PROJECT_384$_MODEL_WRAPPER_H
#define $_PROJECT_384$_MODEL_WRAPPER_H

#include <io_lib/file_io.h>
#include <gmp_model/model.h>
#include <gmp_model/target_model.h>

#include <ros/ros.h>

#define ModelWrapper_fun_ std::string("[ModelWrapper::") + __func__ + "]: "


template<typename T>
class su_ptr
{
public:
  explicit su_ptr(T *p=NULL)
  {
    sp2.reset(new std::unique_ptr<T>());
    if (p) reset(p);
  }

  void reset(T *p)
  {
    sp2.get()->reset(p);
  }

  T *operator()()
  {
    return sp2.get()->get();
  }

  const T *operator()() const
  {
    return sp2.get()->get();
  }

  T * operator->()
  {
    return sp2.get()->get();
  }

  const T * operator->() const
  {
    return sp2.get()->get();
  }

protected:
  std::shared_ptr<std::unique_ptr<T>> sp2;
};

class ModelWrapper : public su_ptr<Model>
{
public:

  ModelWrapper()
  {
    reset(new Model());
  }

  bool loadModel(const std::string &path, std::string *err_msg)
  {
    try
    {
      io_::FileIO fid(path, io_::FileIO::in);
      int type_;
      fid.read("type", type_);

      //std::cerr << "[ModelWrapper::loadModel]: type = " << type_ << "\n";

      this->reset((Model::Type)type_);
      return  (*this)->load(path, err_msg);
    }
    catch(std::exception &e){
      PRINT_ERROR_MSG(ModelWrapper_fun_ + e.what() + "\n");
      if (err_msg) *err_msg = ModelWrapper_fun_ + e.what();
      return false;
    }
  }

  using su_ptr<Model>::reset;

  void reset(Model::Type type)
  {
    if (type == Model::Type::STD) reset(new Model());
    else if (type == Model::Type::TARGET) reset(new TargetModel());
    else throw std::runtime_error(ModelWrapper_fun_ + "Unknown model type: " + std::to_string((int)type));
  }
};

#endif // $_PROJECT_384$_MODEL_WRAPPER_H
