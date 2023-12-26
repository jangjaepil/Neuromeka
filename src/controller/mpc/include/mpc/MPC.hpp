#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>

class MPC  
{
public:
    double getErrorNorm(const Eigen::Matrix<double, 12, 1>& x, const Eigen::Matrix<double, 12, 1>& xRef);
    void updateConstraintVectors(const Eigen::Matrix<double, 12, 1>& x0,
                                            Eigen::VectorXd& lowerBound,
                                            Eigen::VectorXd& upperBound);
    void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 12, 1>& xMax,
                                        const Eigen::Matrix<double, 12, 1>& xMin,
                                        const Eigen::Matrix<double, 4, 1>& uMax,
                                        const Eigen::Matrix<double, 4, 1>& uMin,
                                        const Eigen::Matrix<double, 12, 1>& x0,
                                        int mpcWindow,
                                        Eigen::VectorXd& lowerBound,
                                        Eigen::VectorXd& upperBound);
    void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 12, 12>& dynamicMatrix,
                                 const Eigen::Matrix<double, 12, 4>& controlMatrix,
                                 int mpcWindow,
                                 Eigen::SparseMatrix<double>& constraintMatrix);

    void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 12>& Q,
                         const Eigen::Matrix<double, 12, 1>& xRef,
                         int mpcWindow,
                         Eigen::VectorXd& gradient);

    void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 12>& Q,
                        const Eigen::DiagonalMatrix<double, 4>& R,
                        int mpcWindow,
                        Eigen::SparseMatrix<double>& hessianMatrix);
    void setWeightMatrices(Eigen::DiagonalMatrix<double, 12>& Q, Eigen::DiagonalMatrix<double, 4>& R);
    void setInequalityConstraints(Eigen::Matrix<double, 12, 1>& xMax,
                              Eigen::Matrix<double, 12, 1>& xMin,
                              Eigen::Matrix<double, 4, 1>& uMax,
                              Eigen::Matrix<double, 4, 1>& uMin);
    void setDynamicsMatrices(Eigen::Matrix<double, 12, 12>& a, Eigen::Matrix<double, 12, 4>& b);
    bool init();
    bool solveProblem();
    Eigen::Matrix<double, 12, 1> getStates();
    Eigen::Vector4d getCtr();
    void setRef(Eigen::Matrix<double, 12, 1> dx);

private:
    int mpcWindow = 20;
    int numberOfSteps = 50;
    Eigen::VectorXd QPSolution;
    Eigen::Vector4d ctr;
    OsqpEigen::Solver solver;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 12, 12> a;
    Eigen::Matrix<double, 12, 4> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 12, 1> xMax;
    Eigen::Matrix<double, 12, 1> xMin;
    Eigen::Matrix<double, 4, 1> uMax;
    Eigen::Matrix<double, 4, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 12> Q;
    Eigen::DiagonalMatrix<double, 4> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 12, 1> x0;
    Eigen::Matrix<double, 12, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

};