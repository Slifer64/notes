#ifndef YAML_CONVERTIONS_H_62B23520_7C8E_11DE_8A39_0800200C9A66
#define YAML_CONVERTIONS_H_62B23520_7C8E_11DE_8A39_0800200C9A66

#include "yaml-cpp/node/iterator.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/type.h"

#include <cstring>
#include <vector>
#include <array>
#include <armadillo>
#include <Eigen/Dense>

namespace YAML
{
    template<typename T>
    bool getParam(const YAML::Node &node, const std::string &name, T &val)
    {
        if (!node[name]) return false; //throw std::runtime_error("Failed to load param \"" + name + "\"...");
        val = node[name].as<T>();
        return true;
    }

    // =============================================
    // ================ std::vector  ===============
    // =============================================

    template<typename T>
    struct convert< std::vector<T> > 
    {
        static Node encode(const std::vector<T> &rhs) 
        {
            Node node;
            for (int i=0; i<rhs.size(); i++) node.push_back(rhs[i]);
            return node;
        }

        static bool decode(const Node& node, std::vector<T> &rhs) 
        {
            if( !node.IsSequence() ) return false;

            rhs.resize(node.size());
            for (int i=0;i<rhs.size();i++) rhs[i] = node[i].as<T>();

            return true;
        }
    };

    template<typename T>
    struct convert< std::vector<std::vector<T>> > 
    {
        static Node encode(const std::vector<std::vector<T>> &rhs) 
        {
            Node node;
            for (int i=0; i<rhs.size(); i++)
            {
                std::vector<T> row_i = rhs[i];
                node.push_back(row_i);
            }
            return node;
        }

        static bool decode(const Node& node, std::vector<std::vector<T>> &rhs) 
        {
            if( !node.IsSequence() ) return false;

            int n_rows = node.size();
            if (!n_rows)
            {
                rhs.clear();
                return true;
            }

            rhs.resize(n_rows);

            for (int i=0;i<n_rows;i++)
            {
                int n_cols = node[i].size();
                if (!node[i].IsSequence()) return false; // throw some exception?
                rhs[i] = node[i].as<std::vector<T>>();
            }

            return true;
        }
    };

    // ============================================
    // ================ std::array  ===============
    // ============================================

    // Already defined...
    // template<typename T, size_t N>
    // struct convert< std::array<T,N> > 
    // {
    //     static Node encode(const std::array<T,N> &rhs) 
    //     {
    //         Node node;
    //         for (int i=0; i<rhs.size(); i++) node.push_back(rhs[i]);
    //         return node;
    //     }

    //     static bool decode(const Node& node, std::array<T,N> &rhs) 
    //     {
    //         if( !node.IsSequence() || node.size() != N ) return false;

    //         for (int i=0;i<N;i++) rhs[i] = node[i].as<T>();
    //         return true;
    //     }
    // };
    
    // =======================================
    // ================ Eigen  ===============
    // =======================================

    template<typename T>
    struct convert< Eigen::Matrix<T,3,1> > 
    {
        static Node encode(const Eigen::Matrix<T,3,1> &rhs) 
        {
            Node node;
            for (int i=0; i<rhs.rows(); i++) node.push_back(rhs(i));
            return node;
        }

        static bool decode(const Node& node, Eigen::Matrix<T,3,1> &rhs) 
        {
            if( !node.IsSequence() ) return false;

            for (int i=0;i<rhs.rows();i++) rhs(i) = node[i].as<T>();
            return true;
        }
    };

    template<typename T>
    struct convert< Eigen::Quaternion<T> > 
    {
        static Node encode(const Eigen::Quaternion<T> &rhs) 
        {
            Node node;
            node.push_back(rhs.w());
            node.push_back(rhs.x());
            node.push_back(rhs.y());
            node.push_back(rhs.z());
            return node;
        }

        static bool decode(const Node& node, Eigen::Quaternion<T> &rhs) 
        {
            if( !node.IsSequence() ) return false;

            rhs.w() = node[0].as<T>();
            rhs.x() = node[1].as<T>();
            rhs.y() = node[2].as<T>();
            rhs.z() = node[3].as<T>();
            return true;
        }
    };

    // ==========================================
    // =============  armadillo  ================
    // ==========================================


    template<typename T>
    struct convert< arma::Col<T> > 
    {
        static Node encode(const arma::Col<T> &rhs) 
        {
            Node node;
            for (int i=0; i<rhs.size(); i++) node.push_back(rhs(i));
            return node;
        }

        static bool decode(const Node& node, arma::Col<T> &rhs) 
        {
            if( !node.IsSequence() ) return false;

            rhs.resize(node.size());
            for (int i=0;i<rhs.size();i++) rhs(i) = node[i].as<T>();

            return true;
        }
    };

    template<typename T>
    struct convert< arma::Row<T> > 
    {
        static Node encode(const arma::Row<T> &rhs) 
        {
            Node node;
            for (int i=0; i<rhs.size(); i++) node.push_back(rhs(i));
            return node;
        }

        static bool decode(const Node& node, arma::Row<T> &rhs) 
        {
            if( !node.IsSequence() ) return false;

            rhs.resize(node.size());
            for (int i=0;i<rhs.size();i++) rhs(i) = node[i].as<T>();

            return true;
        }
    };

    template<typename T>
    struct convert< arma::Mat<T> > 
    {
        static Node encode(const arma::Mat<T> &rhs) 
        {
            Node node;
            for (int i=0; i<rhs.n_rows; i++)
            {
                arma::Row<T> row_i = rhs.row(i);
                node.push_back(row_i);
            }
            return node;
        }

        static bool decode(const Node& node, arma::Mat<T> &rhs) 
        {
            if( !node.IsSequence() ) return false;

            int n_rows = node.size();
            if (!n_rows)
            {
                rhs.clear();
                return true;
            }

            int n_cols = node[0].size();
            rhs.resize(n_rows, n_cols);

            for (int i=0;i<n_rows;i++)
            {
                if (!node[i].IsSequence() || node[i].size() != n_cols) return false; // throw some exception?
                rhs.row(i) = node[i].as<arma::Row<T>>();
            }

            return true;
        }
    };

} // namespace YAML

#endif // YAML_CONVERTIONS_H_62B23520_7C8E_11DE_8A39_0800200C9A66
