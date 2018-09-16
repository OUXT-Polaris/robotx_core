#ifndef XMEANS_SOLVER_H_INCLUDED
#define XMEANS_SOLVER_H_INCLUDED

//headers in this package
#include <kmeans_solver.h>
//headers in Eigen
#include <Eigen/Core>

namespace xmeans
{
    class solver
    {
    public:
        solver();
        ~solver();
        Eigen::VectorXd run(Eigen::MatrixXd data);
    private:
       kmeans::solver _kmeans_solver; 
    };
}

#endif  //XMEANS_SOLVER_H_INCLUDED