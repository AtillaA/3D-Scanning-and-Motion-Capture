#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);
		
		// -----------------------------------------------------------------------------------------------------
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements

		Matrix4f estimatedPose = Matrix4f::Identity();
		
		// translation between the means of both sets of points (-Rx + t + x)
		Vector3f movement = -rotation * sourceMean + translation + sourceMean;

		estimatedPose.block<3,3>(0,0) = rotation;
		estimatedPose.block<3,1>(0,3) = movement;

		return estimatedPose;
		// -----------------------------------------------------------------------------------------------------
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// -----------------------------------------------------------------------------------------------------
		// TODO: Compute the mean of input points.

		Vector3f mean = Vector3f::Zero();
		size_t nr_points = points.size();

		for (auto point : points) mean += point;

		std::cout << "-- Mean of the point [x, y, z] == {" << mean[0] <<", "<< mean[1] << ", " << mean[2] << "} --" << std::endl;
		
		return mean;
		// -----------------------------------------------------------------------------------------------------
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// -----------------------------------------------------------------------------------------------------
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.

		Matrix3f rotation = Matrix3f::Identity();
		Matrix3f C = Matrix3f::Zero();

		for (size_t i = 0; i < sourcePoints.size(); ++i)
			C +=  (targetPoints[i] - targetMean) * (sourcePoints[i] - sourceMean).transpose();

		Eigen::JacobiSVD<Matrix3f> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
		
		rotation = svd.matrixU() * svd.matrixV().transpose();

		return rotation;
		// -----------------------------------------------------------------------------------------------------
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// -----------------------------------------------------------------------------------------------------
		// TODO: Compute the translation vector from source to target points.

        Vector3f translation = Vector3f::Zero();

		translation = targetMean - sourceMean;

		return translation;
		// -----------------------------------------------------------------------------------------------------
	}
};
