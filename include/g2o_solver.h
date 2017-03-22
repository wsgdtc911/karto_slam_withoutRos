#ifndef KARTO_G2OSOLVER_H
#define KARTO_G2OSOLVER_H

#include <Mapper.h>

namespace g2o
{
  class VertexSE2;
  class SparseOptimizer;
}

class G2OSolver : public karto::ScanSolver
{
public:
  G2OSolver();
  virtual ~G2OSolver();

public:
  virtual void Clear();
  virtual void Compute();
  virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

private:
  karto::ScanSolver::IdPoseVector corrections_;

  g2o::SparseOptimizer* optimizer_;

  std::vector<g2o::VertexSE2*> vertices_;

  std::ofstream outfile;
};

#endif // KARTO_G2OSOLVER_H

