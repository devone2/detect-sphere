#include "ceres/ceres.h"
#include "glog/logging.h"
#include <algorithm>
#include <iterator>

#define _USE_MATH_DEFINES
#include <math.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

std::string join(const double *ds, int len);

template <typename T>
struct LinkParam {
    Eigen::Transform<T,3,Eigen::Affine> origin;
    Eigen::Matrix<T, 3, 1> rotationVector;

    /*
    LinkParam(Eigen::Translation<double, 3> originTranslation, Eigen::AngleAxisd originRotation, Eigen::Vectord3d rotationVector) {
        this->origin = originTranslation * originRotation;
        this->rotationVector = rotationVector;
    }*/
};

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

struct Observation {
    Eigen::Matrix<double, 3, 1> obsPos;
    std::vector<double> joints;
};

struct FKResidual {
    FKResidual(Observation obs1, Observation obs2) {
        _obs1 = obs1;
        _obs2 = obs2;
    }

  template <typename T> bool operator()(const T* const camera, const T* const mods, T* residual) const {
    typedef Eigen::Matrix<T, 3, 1> Vec3;

    Eigen::Transform<T,3,Eigen::Affine> G = Eigen::Translation<T, 3>(camera[0],camera[1],camera[2]) * Eigen::AngleAxis<T>(camera[3], Vec3(camera[4],camera[5], camera[6]));
    Vec3 P1 = convertToVec3<T>(_obs1.obsPos);
    Vec3 P2 = convertToVec3<T>(_obs2.obsPos);

    std::vector<LinkParam<T>> modLinks = makeModLinks<T>(mods);
    Eigen::Transform<T,3,Eigen::Affine> H1 = calculateFK<T>(modLinks, _obs1.joints);
    Eigen::Transform<T,3,Eigen::Affine> H2 = calculateFK<T>(modLinks, _obs2.joints);

    Vec3 L1 = H2 * H1.inverse() * G * P1;
    Vec3 L2 = G * P2;

    Vec3 result = L1 - L2;

    residual[0] = result(0);
    residual[1] = result(1);
    residual[2] = result(2);

    return true;
  }

    template <typename T> Eigen::Matrix<T, 3, 1> convertToVec3(Eigen::Matrix<double, 3, 1> pos) const {
        return Eigen::Matrix<T, 3, 1>(T(pos(0)),T(pos(1)),T(pos(2)));
    }

    template <typename T> Eigen::Transform<T,3,Eigen::Affine> calculateFK(const std::vector<LinkParam<T>>& links, const std::vector<double>& joints) const {
        Eigen::Transform<T,3,Eigen::Affine> pose = Eigen::Transform<T,3,Eigen::Affine>::Identity();

        for(auto i=0; i<links.size();i++) {
           pose = pose * links[i].origin * Eigen::AngleAxis<T>(T(joints[i]), links[i].rotationVector);
        }

        return pose;
    }

    template <typename T> std::vector<LinkParam<T>> makeModLinks(const T * const linkModifications) const {

        Eigen::Matrix<T, 3, 1> unitZ(T(0),T(0),T(1));
        Eigen::Matrix<T, 3, 1> unitX(T(1),T(0),T(0));
        Eigen::Matrix<T, 3, 1> zero(T(0),T(0),T(0));

        std::vector<LinkParam<T>> links;
        links.push_back( {Eigen::Translation<T, 3>(T(0),T(0),T(0.333) * linkModifications[0]) * Eigen::AngleAxis<T>::Identity(), unitZ});
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(0),T(0)) * Eigen::AngleAxis<T>( T(-M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(-0.316) * linkModifications[2],T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0.0825) * linkModifications[3],T(0),T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(-0.0825),T(0.384) * linkModifications[4],T(0)) * Eigen::AngleAxis<T>( T(-M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(0),T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0.088) * linkModifications[6],T(0),T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(0),T(0.107) * linkModifications[7]) * Eigen::AngleAxis<T>::Identity(), zero });

        return links;
    }

  private:
    Observation _obs1;
    Observation _obs2;
};

std::string showAffine3d(const Eigen::Affine3d& trans);
std::vector<LinkParam<double>> makeModLinks(const std::vector<double>& linkModifications);
Eigen::Affine3d calculateModFK(const std::vector<double>& joints, const std::vector<double>& linkModifications);


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
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();

    for(auto i=0; i<links.size();i++) {
       pose = pose * links[i].origin * Eigen::AngleAxisd(joints[i], links[i].rotationVector);
    }

    return pose;
}

Eigen::Affine3d calculateModFK(const std::vector<double>& joints, const std::vector<double>& linkModifications) {
    const std::vector<LinkParam<double>> links = makeModLinks(linkModifications);
    return calculateFK(links, joints);
}

std::vector<LinkParam<double>> makeModLinks(const std::vector<double>& linkModifications) {

    Eigen::Vector3d unitZ(0,0,1);
    Eigen::Vector3d unitX(1,0,0);
    Eigen::Vector3d zero(0,0,0);

    std::vector<LinkParam<double>> links;
    links.push_back( {Eigen::Translation<double, 3>(0,0,0.333 * linkModifications[0]) * Eigen::AngleAxisd::Identity(), unitZ});
    links.push_back({ Eigen::Translation<double, 3>(0,0,0) * Eigen::AngleAxisd( -M_PI/2, unitX), unitZ });
    links.push_back({ Eigen::Translation<double, 3>(0,-0.316 * linkModifications[2],0) * Eigen::AngleAxisd( M_PI/2, unitX), unitZ });
    links.push_back({ Eigen::Translation<double, 3>(0.0825* linkModifications[3],0,0) * Eigen::AngleAxisd( M_PI/2, unitX), unitZ });
    links.push_back({ Eigen::Translation<double, 3>(-0.0825,0.384* linkModifications[4],0) * Eigen::AngleAxisd( -M_PI/2, unitX), unitZ });
    links.push_back({ Eigen::Translation<double, 3>(0,0,0) * Eigen::AngleAxisd( M_PI/2, unitX), unitZ });
    links.push_back({ Eigen::Translation<double, 3>(0.088 * linkModifications[6],0,0) * Eigen::AngleAxisd( M_PI/2, unitX), unitZ });
    links.push_back({ Eigen::Translation<double, 3>(0.0,0,0.107* linkModifications[7]) * Eigen::AngleAxisd::Identity(), zero });

    return links;
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
    Eigen::Affine3d G = Eigen::Translation<double, 3>(camera[0],camera[1],camera[2]) * Eigen::AngleAxisd(camera[3], Eigen::Matrix<double, 3, 1>(camera[4],camera[5], camera[6]));

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

  std::vector<double> mods = {2,1,1,1,1,1,1,1};

  std::vector<double> joints;
  for(auto i=0; i<mods.size();i++) {
      joints.push_back(0);
  }


 Eigen::Affine3d pose = calculateModFK(joints, mods);

 std::cout << "Final pose: " << showAffine3d(pose) << "\n";

  std::vector<Observation> observations;


  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double camera[7] = {0,0,0,0,1,0,0};
  double xmods[8] = {1,1,1,1,1,1,1,1};

  const double initial_c[7] = {0.3,0.5,0.9,1,0,0};
  //const double initial_c[7]= {0,0,0,0,1,0,0};
  const double initial_m[8] = {1,1,1,1,1,1,1,1};
  std::vector<double> real_mod = {1.015,1,1.016,1.02,1,1,0.999,1};
  genObservation(initial_c,real_mod, observations,10);
  // Build the problem.
  Problem problem;
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).

  for(int i=0;i<observations.size();i++) {
      for(int j=i+1;j<observations.size(); j++) {
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
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 200;
  options.update_state_every_iteration  = true;
  options.callbacks.push_back(new LoggingCallback(camera, xmods));
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  std::cout << "camera : " << join(initial_c,7) << " -> \n"
            << join(camera,7) << "\n"
            << "mods: " <<  join(initial_m, 8) << " -> \n"
            << join(xmods,8) << "\n";

  // calculate cost function for original values
  double orig_cost = -1.0;
  problem.Evaluate(Problem::EvaluateOptions(), &orig_cost, NULL, NULL, NULL);
  std::cout << "Cost at current value: " << orig_cost << "\n";
  std::copy(std::begin(initial_c), std::end(initial_c), std::begin(camera));
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
