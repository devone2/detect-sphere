#include "ceres/ceres.h"
#include "glog/logging.h"
#include <algorithm>
#include <iterator>
#include "fkresidual.h"

#define _USE_MATH_DEFINES
#include <math.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

std::string join(const double *ds, int len);

// Function prototypes
std::string showAffine3d(const Eigen::Affine3d& trans);
std::vector<LinkParam<double>> makeModLinks(const std::vector<double>& linkModifications);
Eigen::Matrix<double, 3, 1> calcCartezPosition(const double* camera, const std::vector<double>& linkModifications, const std::vector<double>& joints);

std::string showAffine3d(const Eigen::Affine3d& trans)
{
  Eigen::AngleAxisd ang;
  ang.fromRotationMatrix(trans.rotation());

  std::stringstream sstm;
  sstm <<  "T: " << trans.translation().transpose() << ", R: "
       << "Axis: " << ang.axis().transpose() << " Angle: " <<ang.angle();

  return sstm.str();
}
Eigen::Affine3d calculateFK(const std::vector<LinkParam<double>>& links, const std::vector<double>& joints) {
   return FKResidual::calculateFK(links, joints);
}

Eigen::Affine3d calculateModFK(const std::vector<double>& joints, const std::vector<double>& linkModifications) {
    const std::vector<LinkParam<double>> links = makeModLinks(linkModifications);
    return calculateFK(links, joints);
}

std::vector<LinkParam<double>> makeModLinks(const std::vector<double>& linkModifications) {

    return FKResidual::makeModLinks(&linkModifications[0]);
}
std::vector<double> genJoints();
double fRand(double fMin, double fMax);
void genObservation(const double* camera, const std::vector<double>& linkModifications, std::vector<Observation>& observations, int obs_count);

std::vector<double> genJoints() {
    std::vector<double> result;
    for(int i=0;i<8;i++) {
        result.push_back(fRand(-M_PI, M_PI));
    }
    return result;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void genObservation(const double* camera, const std::vector<double>& linkModifications, std::vector<Observation>& observations, int obs_count) {
    std::cout << "Observation linkModification:" << join(linkModifications.data(), 8) << "\n";
    std::vector<LinkParam<double>> links = makeModLinks(linkModifications);
    Eigen::Affine3d G = Eigen::Translation<double, 3>(camera[0],camera[1],camera[2])
            * Eigen::AngleAxisd(camera[3], Eigen::Matrix<double, 3, 1>(camera[4],camera[5], camera[6]));

    Eigen::Matrix<double, 3, 1> t(10,20,0.005);

    for(int i=0;i<obs_count;i++) {
        auto joints = genJoints();
        Eigen::Affine3d H = calculateFK(links, joints);
        Observation o;
        o.joints = joints;
        o.obsPos = G.inverse()*H*t;
        observations.push_back(o);
    }
}

Eigen::Matrix<double, 3, 1> calcCartezPosition(const double* camera, const std::vector<double>& linkModifications, const std::vector<double>& joints)
{
    std::vector<LinkParam<double>> links = makeModLinks(linkModifications);
    Eigen::Affine3d G = Eigen::Translation<double, 3>(camera[0],camera[1],camera[2])
            * Eigen::AngleAxisd(camera[3], Eigen::Matrix<double, 3, 1>(camera[4],camera[5], camera[6]));

    Eigen::Matrix<double, 3, 1> t(10,20,0.005);

    Eigen::Affine3d H = calculateFK(links, joints);
    auto position = G.inverse()*H*t;

    return position;
}

class LoggingCallback : public ceres::IterationCallback {
 public:
    explicit LoggingCallback(const double * const camera, const double * const xmods) :
        camera(camera), xmods(xmods) {}

  ~LoggingCallback() {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
      std::cout << "camera : " << join(camera,7) << " \n"
                << "mods: " <<  join(xmods, 8) << " \n";

    return ceres::SOLVER_CONTINUE;
  }

 private:
  const double * const camera;
  const double * const xmods;
};


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  std::vector<Observation> observations;

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double camera[7] = {0,0,3,M_PI/4,1,1,1};
  double xmods[8] = {1,1,1,1,1,1,1,1};

  //const double initial_c[7] = {0.3,0.5,0.9,1,0,0};
  const double real_c[7]= {5,7,3,M_PI/2,1,0,0};
  std::vector<double> real_mod = {1.18,1,1.016,1.02,1,1,0.999,1};
  genObservation(real_c,real_mod, observations,10);
  // Build the problem.
  Problem problem;

  for(int i=0;i<observations.size();i++) {
      for(int j=0;j<observations.size(); j++) {
          CostFunction* cost_function =
              new AutoDiffCostFunction<FKResidual,3,7,8>(new FKResidual(observations[i], observations[j]));
          problem.AddResidualBlock(cost_function, NULL, camera, xmods);
      }
  }

  problem.SetParameterLowerBound(camera, 3, -M_PI);
  problem.SetParameterUpperBound(camera, 3, 2*M_PI);

  for(int i=0;i<8;i++) {
      problem.SetParameterLowerBound(xmods, i, 0.8);
      problem.SetParameterUpperBound(xmods, i, 1.2);
  }

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 200;
  options.update_state_every_iteration  = true;
  options.callbacks.push_back(new LoggingCallback(camera, xmods));
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  std::cout << "camera : " << join(real_c,7) << " -> \n"
            << join(camera,7) << "\n"
            << "mods: " <<  join(&real_mod[0], real_mod.size()) << " -> \n"
            << join(xmods,8) << "\n";

  // calculate cost function for original values

  std::vector<double> calcMods(std::begin(xmods), std::end(xmods));

  for(int i=0;i<observations.size();i++) {
      auto calcPos = calcCartezPosition(camera, calcMods, observations[i].joints);
      std::cout << i << ". observed: " << observations[i].obsPos.transpose() << " vs "
                << calcPos.transpose() << "\n";

      std:: cout << "Diff: " << (observations[i].obsPos - calcPos).transpose() << "\n";
  }


  double orig_cost = -1.0;
  problem.Evaluate(Problem::EvaluateOptions(), &orig_cost, NULL, NULL, NULL);
  std::cout << "Cost at current value: " << orig_cost << "\n";
  std::copy(std::begin(real_c), std::end(real_c), std::begin(camera));
  std::copy(std::begin(real_mod), std::end(real_mod), std::begin(xmods));
  problem.Evaluate(Problem::EvaluateOptions(), &orig_cost, NULL, NULL, NULL);
  std::cout << "Cost at orig value: " << orig_cost << "\n";

  return 0;
}

std::string join(const double *ds, int len) {
    std::stringstream ss;
    for(int i=0;i<len;i++) {
        ss << ds[i] << ", ";
    }
    return ss.str();
}
