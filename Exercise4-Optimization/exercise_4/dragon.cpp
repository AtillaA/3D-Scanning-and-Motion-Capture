#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// -----------------------------------------------------------------------------------
// TODO: Implement the cost function
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& original_, const Point2D& transformed_, const Weight& weight_) : original(original_), transformed(transformed_), weight(weight_){}

	template<typename T>
	bool operator()(const T* const deg, const T* const tx, const T* const ty, T* residual) const {
		T rad = *deg * T(M_PI / 180.0);
		T x = T(cos(rad) * original.x - sin(rad) * original.y) + *tx - T(transformed.x);
		T y = T(sin(rad) * original.x + cos(rad) * original.y) + *ty - T(transformed.y);
			
		residual[0] = T(weight.w * (x * x + y * y));

		return true;
	}

private:
	const Weight weight;
	const Point2D original;
	const Point2D transformed;
};
// -----------------------------------------------------------------------------------


int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "../data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);
	
	const double angle_initial = 40;
	const double tx_initial = 1000;
	const double ty_initial = 50;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// ----------------------------------------------------------------------------------
	// TODO: For each weighted correspondence create one residual block
	for (int i = 0; i < original.size(); i++) {
		problem.AddResidualBlock(new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
			new RegistrationCostFunction(original[i], transformed[i], weights[i])),nullptr, &deg, &tx, &ty);
	}
	// ----------------------------------------------------------------------------------

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}
