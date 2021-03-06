#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/abb_irb1600_145_fk_fast.hpp>

int main(int argc, char** argv)
{
    printf("%d arguments\n", argc);
    for (int idx = 0; idx < argc; idx++)
    {
        printf("Argument %d: %s\n", idx, argv[idx]);
    }
    std::cout << "Testing PrettyPrints..." << std::endl;
    std::cout << PrettyPrint::PrettyPrint(Eigen::Affine3d::Identity()) << std::endl;
    std::cout << PrettyPrint::PrettyPrint(Eigen::Vector3d(0.0, 0.0, 0.0)) << std::endl;
    std::cout << PrettyPrint::PrettyPrint(std::vector<bool>{true, false, true, false}) << std::endl;
    std::cout << "...done" << std::endl;
    std::vector<double> base_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    EigenHelpers::VectorAffine3d link_transforms = ABB_IRB1600_145_FK_FAST::GetLinkTransforms(base_config);
    std::cout << "Link transforms:\n" << PrettyPrint::PrettyPrint(link_transforms, false, "\n") << std::endl;

    // Test Vector3d averaging
    EigenHelpers::VectorVector3d testvecs(8, Eigen::Vector3d::Zero());
    testvecs[0] = Eigen::Vector3d(-1.0, -1.0, -1.0);
    testvecs[1] = Eigen::Vector3d(-1.0, -1.0, 1.0);
    testvecs[2] = Eigen::Vector3d(-1.0, 1.0, -1.0);
    testvecs[3] = Eigen::Vector3d(-1.0, 1.0, 1.0);
    testvecs[4] = Eigen::Vector3d(1.0, -1.0, -1.0);
    testvecs[5] = Eigen::Vector3d(1.0, -1.0, 1.0);
    testvecs[6] = Eigen::Vector3d(1.0, 1.0, -1.0);
    testvecs[7] = Eigen::Vector3d(1.0, 1.0, 1.0);
    std::cout << "Individual vectors: " << PrettyPrint::PrettyPrint(testvecs) << std::endl;
    Eigen::Vector3d averagevec = EigenHelpers::AverageEigenVector3d(testvecs);
    std::cout << "Average vector: " << PrettyPrint::PrettyPrint(averagevec) << std::endl;
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937_64 prng(seed);
    arc_helpers::TruncatedNormalDistribution dist(0.0, 1.0, -5.0, 5.0);
    std::vector<double> test_trunc_normals(100000, 0.0);
    for (size_t idx = 0; idx < test_trunc_normals.size(); idx++)
    {
        test_trunc_normals[idx] = dist(prng);
    }
    std::cout << "Truncated normal test:\n" << PrettyPrint::PrettyPrint(test_trunc_normals, false, ",") << std::endl;

    // Test weighted dot product functions
    Eigen::Vector3d weights(1.0, 2.0, 3.0);
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[0]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[0], weights) << std::endl;
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[7]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[7], weights) << std::endl;
    std::cout << "Weighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[0], testvecs[7], weights) << std::endl;
    std::cout << "Unweighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[0], testvecs[7], Eigen::Vector3d::Ones()) << std::endl;
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[1]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[1], weights) << std::endl;
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[2]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[2], weights) << std::endl;
    std::cout << "Weighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[1], testvecs[2], weights) << std::endl;
    std::cout << "Unweighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[1], testvecs[2], Eigen::Vector3d::Ones()) << std::endl;

    // Test multivariate gaussian
    std::cout << "MVN Gaussian test:\n";
    Eigen::Vector2d mean(0.0, 1.0);
    Eigen::Matrix2d covar;
    covar << 10.0, 5.0,
             5.0, 10.0;
    arc_helpers::MultivariteGaussianDistribution mvn_dist(mean, covar);
    std::vector<Eigen::VectorXd> mvn_gaussians(3000);
    for (size_t idx = 0; idx < mvn_gaussians.size(); idx++)
    {
        mvn_gaussians[idx] = mvn_dist(prng);
        std::cout << mvn_gaussians[idx].transpose() << std::endl;
    }

    return 0;
}
