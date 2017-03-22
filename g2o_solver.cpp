/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <Karto.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/factory.h>

#include <g2o/types/slam2d/types_slam2d.h>

G2O_USE_TYPE_GROUP(slam2d)


#include "g2o_solver.h"


//#include "ros/console.h"

G2OSolver::G2OSolver()
{
  optimizer_ = new g2o::SparseOptimizer();

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solverGauss   = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  // OptimizationAlgorithmLevenberg* solverLevenberg = new OptimizationAlgorithmLevenberg(blockSolver);
  optimizer_->setAlgorithm(solverGauss);

  outfile.open("../log/g2oInfo.log");

}

G2OSolver::~G2OSolver()
{
    delete optimizer_;
    outfile.close();
}

void G2OSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector& G2OSolver::GetCorrections() const
{
  return corrections_;
}

void G2OSolver::Compute()
{
  std::cout << "G2OSolver::Compute(): saving graph..." << std::flush;
  optimizer_->save("before_optimization.g2o");
  std::cout << "done." << std::endl;
  std::cout << "G2OSolver::Compute(): running optimizer..." << std::flush;
  corrections_.clear();
  //typedef std::vector<sba::Node2d, Eigen::aligned_allocator<sba::Node2d> > NodeVector;

  optimizer_->initializeOptimization();
  optimizer_->optimize(50);

  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    const g2o::SE2& estimate = vertices_[i]->estimate();
    karto::Pose2 pose(estimate.translation().x(), estimate.translation().y(), estimate.rotation().angle());
    corrections_.push_back(std::make_pair(vertices_[i]->id(), pose));
  }

  std::cout << "done." << std::endl;
  optimizer_->save("after_optimization.g2o");
        
  /*
  g2o::OptimizableGraph::VertexContainer points;
  for (
      g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer_->vertices().begin(); 
      it != optimizer_->vertices().end(); ++it) 
  {
    g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
    const g2o::SE2& estimate = v->estimate();
    karto::Pose2 pose(estimate.translation().x(), estimate.translation().y(), estimate.rotation().angle());
  }
  */
}

void G2OSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  //Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
  //m_Spa.addNode(vector, pVertex->GetObject()->GetUniqueId());
  g2o::VertexSE2* vertex = new g2o::VertexSE2();
  vertex->setId(pVertex->GetObject()->GetUniqueId());
  g2o::SE2 p(pose.GetX(), pose.GetY(), pose.GetHeading());
  vertex->setEstimate(p);
  // fix first vertex
  if (vertices_.size() == 0)
  {
    vertex->setFixed(true);
    outfile << "Fixed ";
  }
  optimizer_->addVertex(vertex);
  //TODO Memory management of vertices and edges?
  vertices_.push_back(vertex);

  outfile << "vertex : " << pVertex->GetObject()->GetUniqueId() << "; pose : " << pose.GetX() << " " << pose.GetY() << " " << pose.GetHeading() << std::endl;
}

void G2OSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{
  karto::LocalizedRangeScan* pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan* pTarget = pEdge->GetTarget()->GetObject();
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  //Eigen::Vector3d mean(diff.GetX(), diff.GetY(), diff.GetHeading());
  g2o::SE2 motion(diff.GetX(), diff.GetY(), diff.GetHeading());

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double,3,3> m;
  m(0,0) = precisionMatrix(0,0);
  m(0,1) = m(1,0) = precisionMatrix(0,1);
  m(0,2) = m(2,0) = precisionMatrix(0,2);
  m(1,1) = precisionMatrix(1,1);
  m(1,2) = m(2,1) = precisionMatrix(1,2);
  m(2,2) = precisionMatrix(2,2);

  outfile << "Source : " << pSource->GetUniqueId() << "; Target : " << pTarget->GetUniqueId() << "; motion : " << diff.GetX() << " " << diff.GetY() << " " << diff.GetHeading();
  outfile << "; Inf : " << m(0, 0) << " " << m(1, 1) << " " << m(2, 2) << std::endl;

  //m_Spa.addConstraint(pSource->GetUniqueId(), pTarget->GetUniqueId(), mean, m);

  g2o::EdgeSE2* edge = new g2o::EdgeSE2();
  edge->vertices()[0] = optimizer_->vertices().find(pSource->GetUniqueId())->second;
  edge->vertices()[1] = optimizer_->vertices().find(pTarget->GetUniqueId())->second;
  edge->setMeasurement(motion);
  // TODO
  edge->setInformation(m);
  //edge->setInformation(Eigen::Matrix3d::Identity());

  optimizer_->addEdge(edge);
}

