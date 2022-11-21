#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


struct GaussianCostFunction
{
	GaussianCostFunction(const Point2D& point_)
		: point(point_)
	{
	}

	template<typename T>
	bool operator()(const T* const mu, const T* const sigma, T* residual) const
	{
		// TODO: Implement the cost function
		T y = 1.0 / (ceres::sqrt(T(2) * M_PI * ceres::pow(sigma[0], T(2.0)))) * ceres::exp(-ceres::pow(point.x - mu[0], T(2.0)) / (T(2) * ceres::pow(sigma[0], T(2.0))));
		residual[0] = ceres::pow(y - point.y, T(2.0));
		//residual[0] = ceres::abs(y - point.y);
		return true;
	}

private:
	const Point2D point;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points
	const std::string file_path = "C:/Users/adilm/Desktop/3DScanning/Data/points_gaussian.txt";
	const auto points = read_points_from_file<Point2D>(file_path);

	// Good initial values make the optimization easier
	const double mu_initial = 0.0;
	const double sigma_initial = 1.0;

	double mu = mu_initial;
	double sigma = sigma_initial;

	ceres::Problem problem;

	// For each data point create one residual block
	for (auto& point : points)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<GaussianCostFunction, 1, 1, 1>(
				new GaussianCostFunction(point)
			),
			nullptr, &mu, &sigma
		);
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 10;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	std::cout << "Initial mu: " << mu_initial << "\tsigma: " << sigma_initial << std::endl;
	std::cout << "Final mu: " << mu << "\tsigma: " << sigma << std::endl;


	system("pause");
	return 0;
}
