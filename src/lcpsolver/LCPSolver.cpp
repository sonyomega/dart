#include "LCPSolver.h"
#include "Lemke.h"
#include <cstdio>
#include "lcp.h"
#include "misc.h"

namespace dart
{
namespace lcpsolver
{

LCPSolver::LCPSolver()
{

}
LCPSolver::~LCPSolver()
{

}

bool LCPSolver::Solve(const MatrixXd& _A, const VectorXd& _b, VectorXd& _x, int _numContacts, double _mu, int _numDir, bool _bUseODESolver)
{
    if (!_bUseODESolver)
    {
        int err = Lemke(_A, _b, _x);
        return (err == 0);
    }
    else
    {
        assert(_numDir >= 4);
        MatrixXd AODE;
        VectorXd bODE;
        transferToODEFormulation(_A, _b, AODE, bODE, _numDir, _numContacts);
        double* A, *b, *x, *w, *lo, *hi;
        int n = AODE.rows();

        int nSkip = dPAD(n);

        A = new double[n * nSkip];
        b = new double[n];
        x = new double[n];
        w = new double[n];
        lo = new double[n];
        hi = new double[n];
        int* findex = new int[n];

        memset(A, 0, n * nSkip * sizeof(double));
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                A[i * nSkip + j] = AODE(i, j);
            }
        }
        for (int i = 0; i < n; ++i)
        {
            b[i] = -bODE[i];
            x[i] = w[i] = lo[i] = 0;
            hi[i] = dInfinity;
            findex[i] = -1;
        }
        for (int i = 0; i < _numContacts; ++i)
        {
            findex[_numContacts + i * 2 + 0] = i;
            findex[_numContacts + i * 2 + 1] = i;

            lo[_numContacts + i * 2 + 0] = -_mu;
            lo[_numContacts + i * 2 + 1] = -_mu;

            hi[_numContacts + i * 2 + 0] = _mu;
            hi[_numContacts + i * 2 + 1] = _mu;

        }
        //		dClearUpperTriangle (A,n);
        dSolveLCP (n, A, x, b, w, 0, lo, hi, findex);
        /*
                  for (int i = 0; i < n; i++) {
                  if (w[i] < 0.0 && abs(x[i] - hi[i]) > 0.000001)
                  cout << "w[" << i << "] is negative, but x is " << x[i] << endl;
                  else if (w[i] > 0.0 && abs(x[i] - lo[i]) > 0.000001)
                  cout << "w[" << i << "] is positive, but x is " << x[i] << " lo is " <<  lo[i] << endl;
                  else if (abs(w[i]) < 0.000001 && (x[i] > hi[i] || x[i] < lo[i]))
                  cout << "w[i] " << i << " is zero, but x is " << x[i] << endl;
                  }
                */
        VectorXd xODE = VectorXd::Zero(n);
        for (int i = 0; i < n; ++i)
        {
            xODE[i] = x[i];
        }
        transferSolFromODEFormulation(xODE, _x, _numDir, _numContacts);

        //		checkIfSolution(reducedA, reducedb, _x);

        delete[] A;
        delete[] b;
        delete[] x;
        delete[] w;
        delete[] lo;
        delete[] hi;
        delete[] findex;
        return 1;
    }

}

void LCPSolver::transferToODEFormulation(const MatrixXd& _A, const VectorXd& _b, MatrixXd& _AOut, VectorXd& _bOut, int _numDir, int _numContacts)
{
    int numOtherConstrs = _A.rows() - _numContacts * (2 + _numDir);
    int n = _numContacts * 3 + numOtherConstrs;
    MatrixXd AIntermediate = MatrixXd::Zero(n, _A.cols());
    _AOut = MatrixXd::Zero(n, n);
    _bOut = VectorXd::Zero(n);
    int offset = _numDir / 4;
    for (int i = 0; i < _numContacts; ++i)
    {
        AIntermediate.row(i) = _A.row(i);
        _bOut[i] = _b[i];

        AIntermediate.row(_numContacts + i * 2 + 0) = _A.row(_numContacts + i * _numDir + 0);
        AIntermediate.row(_numContacts + i * 2 + 1) = _A.row(_numContacts + i * _numDir + offset);
        _bOut[_numContacts + i * 2 + 0] = _b[_numContacts + i * _numDir + 0];
        _bOut[_numContacts + i * 2 + 1] = _b[_numContacts + i * _numDir + offset];
    }
    for (int i = 0; i < numOtherConstrs; i++) {
        AIntermediate.row(_numContacts * 3 + i) = _A.row(_numContacts * (_numDir + 2) + i);
        _bOut[_numContacts * 3 + i] = _b[_numContacts * (_numDir + 2) + i];
    }
    for (int i = 0; i < _numContacts; ++i)
    {
        _AOut.col(i) = AIntermediate.col(i);
        _AOut.col(_numContacts + i * 2 + 0) = AIntermediate.col(_numContacts + i * _numDir + 0);
        _AOut.col(_numContacts + i * 2 + 1) = AIntermediate.col(_numContacts + i * _numDir + offset);
    }
    for (int i = 0; i < numOtherConstrs; i++)
        _AOut.col(_numContacts * 3 + i) = AIntermediate.col(_numContacts * (_numDir + 2) + i);

}

void LCPSolver::transferSolFromODEFormulation(const VectorXd& _x, VectorXd& _xOut, int _numDir, int _numContacts)
{
    int numOtherConstrs = _x.size() - _numContacts * 3;
    _xOut = VectorXd::Zero(_numContacts * (2 + _numDir) + numOtherConstrs);

    _xOut.head(_numContacts) = _x.head(_numContacts);

    int offset = _numDir / 4;
    for (int i = 0; i < _numContacts; ++i)
    {
        _xOut[_numContacts + i * _numDir + 0] = _x[_numContacts + i * 2 + 0];
        _xOut[_numContacts + i * _numDir + offset] = _x[_numContacts + i * 2 + 1];
    }
    for (int i = 0; i < numOtherConstrs; i++)
        _xOut[_numContacts * (2 + _numDir) + i] = _x[_numContacts * 3 + i];
}
bool LCPSolver::checkIfSolution(const MatrixXd& _A, const VectorXd& _b, const VectorXd& _x)
{
    const double threshold = 1e-4;
    int n = _x.size();

    VectorXd w = _A * _x + _b;
    for (int i = 0; i < n; ++i)
    {
        if (w(i) < -threshold || _x(i) < -threshold)
            return false;
        if (abs(w(i) * _x(i)) > threshold)
            return false;
    }
    return true;
}

} // namespace lcpsolver
} // namespace dart
