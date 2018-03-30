#ifndef KALMAN_FILTER_MULTIARM_BANDIT_HPP
#define KALMAN_FILTER_MULTIARM_BANDIT_HPP

#include <assert.h>
#include <vector>
#include <random>
#include <utility>

#include <Eigen/Eigenvalues>

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/eigen_helpers.hpp>

namespace smmap
{
    template <typename Generator = std::mt19937_64>
    class KalmanFilterMANB
    {
        public:
            KalmanFilterMANB(
                    const Eigen::VectorXd& prior_mean = Eigen::VectorXd::Zero(1),
                    const Eigen::VectorXd& prior_var = Eigen::VectorXd::Ones(1))
                : num_bandits_(prior_mean.rows())
                , arm_mean_(prior_mean)
                , arm_var_(prior_var)
            {
                assert(arm_mean_.cols() == arm_var_.cols());
            }

            /**
             * @brief selectArmToPull Perform Thompson sampling on the bandits,
             *                        and select the bandit with the largest sample.
             * @param generator
             * @return
             */
            ssize_t selectArmToPull(Generator& generator)
            {
                // Sample from the current distribuition
                std::normal_distribution<double> normal_dist(0.0, 1.0);

                ssize_t best_arm = -1;
                double best_sample = -std::numeric_limits<double>::infinity();
                for (ssize_t arm_ind = 0; arm_ind < num_bandits_; arm_ind++)
                {
                    const double sample = std::sqrt(arm_var_(arm_ind)) * normal_dist(generator) + arm_mean_(arm_ind);

                    if (sample > best_sample)
                    {
                        best_arm = arm_ind;
                        best_sample = sample;
                    }
                }

                assert(best_arm >= 0);
                return best_arm;
            }

            bool generateAllModelActions() const
            {
                return false;
            }

            /**
             * @brief updateArms
             * @param transition_variance
             * @param arm_pulled
             * @param observed_reward
             * @param observation_variance
             */
            void updateArms(
                    const Eigen::VectorXd& transition_variance,
                    const ssize_t arm_pulled,
                    const double observed_reward,
                    const double observation_variance)
            {
                for (ssize_t arm_ind = 0; arm_ind < num_bandits_; arm_ind++)
                {
                    if (arm_ind != arm_pulled)
                    {
                        arm_var_(arm_ind) += transition_variance(arm_ind);
                    }
                    else
                    {
                        arm_mean_(arm_ind) = ((arm_var_(arm_ind) + transition_variance(arm_ind)) * observed_reward + observation_variance * arm_mean_(arm_ind))
                                            / (arm_var_(arm_ind) + transition_variance(arm_ind) + observation_variance);

                        arm_var_(arm_ind) = (arm_var_(arm_ind) + transition_variance(arm_ind)) * observation_variance
                                           / (arm_var_(arm_ind) + transition_variance(arm_ind) + observation_variance);
                    }
                }
            }

            const Eigen::VectorXd& getMean() const
            {
                return arm_mean_;
            }

            Eigen::VectorXd getMean()
            {
                return arm_mean_;
            }

            const Eigen::VectorXd& getVariance() const
            {
                return arm_var_;
            }

            Eigen::VectorXd getVariance()
            {
                return arm_var_;
            }

        private:
            ssize_t num_bandits_;

            Eigen::VectorXd arm_mean_;
            Eigen::VectorXd arm_var_;
    };

    template<typename Generator = std::mt19937_64>
    class KalmanFilterMANDB
    {
        public:
            KalmanFilterMANDB(
                    const Eigen::VectorXd& prior_mean = Eigen::VectorXd::Ones(1),
                    const Eigen::MatrixXd& prior_covar = Eigen::MatrixXd::Identity(1, 1))
                : arm_mean_(prior_mean)
                , arm_covar_(prior_covar)
            {
                assert(arm_covar_.rows() == arm_covar_.cols());
                assert(arm_covar_.rows() == arm_mean_.rows());
            }

            /**
             * @brief selectArmToPull Perform Thompson sampling on the bandits,
             *                        and select the bandit with the largest sample.
             * @param generator
             * @return
             */
            ssize_t selectArmToPull(Generator& generator)
            {
                // Sample from the current distribuition
                arc_helpers::MultivariteGaussianDistribution distribution(arm_mean_, arm_covar_);
                const Eigen::VectorXd sample = distribution(generator);

                // Find the arm with the highest sample
                ssize_t best_arm = -1;
                sample.maxCoeff(&best_arm);

                return best_arm;
            }

            bool generateAllModelActions() const
            {
                return true;
            }

            /**
             * @brief updateArms
             * @param transition_covariance
             * @param arm_pulled
             * @param obs_reward
             * @param obs_var
             */
            void updateArms(
                    const Eigen::MatrixXd& transition_covariance,
                    const Eigen::MatrixXd& observation_matrix,
                    const Eigen::VectorXd& observed_reward,
                    const Eigen::MatrixXd& observation_covariance)
            {
                #pragma GCC diagnostic push
                #pragma GCC diagnostic ignored "-Wconversion"
                const Eigen::MatrixXd& C = observation_matrix;

                // Kalman predict
                const Eigen::VectorXd& predicted_mean = arm_mean_;                      // No change to mean
                const auto predicted_covariance = arm_covar_ + transition_covariance;   // Add process noise

                // Kalman update - symbols from wikipedia article
                const auto innovation = observed_reward - C * predicted_mean;                                            // tilde y_k
                const auto innovation_covariance = C * predicted_covariance.selfadjointView<Eigen::Lower>() * C.transpose() + observation_covariance;    // S_k
                const auto kalman_gain = predicted_covariance.selfadjointView<Eigen::Lower>() * C.transpose() * innovation_covariance.inverse();         // K_k

                arm_mean_ = predicted_mean + kalman_gain * innovation;                                                              // hat x_k|k
                arm_covar_ = predicted_covariance - kalman_gain * C * predicted_covariance.selfadjointView<Eigen::Lower>();         // P_k|k
                #pragma GCC diagnostic pop

                // Numerical problems fixing
                arm_covar_ = ((arm_covar_ + arm_covar_.transpose()) * 0.5).selfadjointView<Eigen::Lower>();

                assert(!(arm_mean_.unaryExpr([] (const double &val) { return std::isnan(val); })).any() && "NaN Found in arm_mean_ in kalman banidt!");
                assert(!(arm_mean_.unaryExpr([] (const double &val) { return std::isinf(val); })).any() && "Inf Found in arm_mean_ in kalman banidt!");
                assert(!(arm_covar_.unaryExpr([] (const double &val) { return std::isinf(val); })).any() && "NaN Found in arm_covar_ in kalman bandit!");
                assert(!(arm_covar_.unaryExpr([] (const double &val) { return std::isinf(val); })).any() && "Inf Found in arm_covar_ in kalman bandit!");
            }

            const Eigen::VectorXd& getMean() const
            {
                return arm_mean_;
            }

            Eigen::VectorXd getMean()
            {
                return arm_mean_;
            }

            const Eigen::MatrixXd& getCovariance() const
            {
                return arm_covar_;
            }

            Eigen::MatrixXd getCovariance()
            {
                return arm_covar_;
            }

        private:
            Eigen::VectorXd arm_mean_;
            Eigen::MatrixXd arm_covar_;
    };
}

#endif // KALMAN_FILTER_MULTIARM_BANDIT_HPP
