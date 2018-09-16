#ifndef KMEANS_SOLVER_H_INCLUDED
#define KMEANS_SOLVER_H_INCLUDED

/**
 * @brief implimentation of kmeans/k-means++ method in C++
 * @sa https://qiita.com/NoriakiOshita/items/cbd46d907d196efe64a3
 * @file kmeans_solver.h
 * @author Masaya Kataoka
 * @date 2018-09-16
 */

//headers in Eigen
#include <Eigen/Core>

namespace kmeans
{
    struct point
    {
        const Eigen::VectorXd value;
        const int index;
        point(Eigen::VectorXd v, int idx) : value(v), index(idx){};
    };

    class cluster
    {
    public:
        cluster();
        ~cluster();
    private:
    };

    class solver
    {
    public:
        solver();
        ~solver();
        Eigen::VectorXd run(Eigen::MatrixXd data, int num_clusters);
        //Eigen::VectorXd run(Eigen::MatrixXd data, int num_clusters);
    };
}
#endif  //KMEANS_SOLVER_H_INCLUDED