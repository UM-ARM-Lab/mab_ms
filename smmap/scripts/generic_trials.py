#!/usr/bin/python

from run_trial import *


def run_trials(experiment, run_baseline=False, run_UCB=False, run_KFMANB=False, run_KFMANDB=False, generate_screenshots="false", log_prefix=""):

    if run_baseline:
        # Note that this is 0 to 25 as range does [start, stop), thus we get 0:4:24 in Matlab speak
        deform_range = range(0, 25, 4)

        # Run the single model baseline
        for translational_deform in deform_range:
            for rotational_deform in deform_range:
                run_trial(experiment=experiment,
                          logging_enabled="true",
                          start_bullet_viewer=generate_screenshots,
                          screenshots_enabled=generate_screenshots,
                          test_id=log_prefix + "single_model_baseline/" + "trans_" + str(translational_deform) + "_rot_" + str(rotational_deform),
                          multi_model="false",
                          deformability_override="true",
                          translational_deformability=translational_deform,
                          rotational_deformability=rotational_deform)

        # Note that this is 0 to 16 as range does [start, stop), thus we get 0:1:10 in Matlab speak
        adaptive_range = range(0, 11, 1)

        # Run the single model baseline
        for adaptive_exponent in adaptive_range:
            adaptive_model_learning_rate = 10.0 ** (-adaptive_exponent)
            run_trial(experiment=experiment,
                      logging_enabled="true",
                      start_bullet_viewer=generate_screenshots,
                      screenshots_enabled=generate_screenshots,
                      test_id=log_prefix + "single_model_baseline/" + "adaptive_1e-" + str(adaptive_exponent),
                      multi_model="false",
                      use_adaptive_model="true",
                      adaptive_model_learning_rate=adaptive_model_learning_rate)

    # Run the multi-model trials last, once for UCB, 10 per non-deterministic bandit algorithm
    if run_UCB:
        run_trial(experiment=experiment,
                  logging_enabled="true",
                  start_bullet_viewer=generate_screenshots,
                  screenshots_enabled=generate_screenshots,
                  test_id=log_prefix + "/UCB_regret",
                  bandit_algorithm="UCB",
                  multi_model="true",
                  calculate_regret="true")

    if run_KFMANB:
        for i in range(0, 10):
            run_trial(experiment=experiment,
                      logging_enabled="true",
                      start_bullet_viewer=generate_screenshots,
                      screenshots_enabled=generate_screenshots,
                      test_id=log_prefix + "KFMANB_regret_" + str(i),
                      bandit_algorithm="KFMANB",
                      multi_model="true",
                      calculate_regret="true",
                      use_random_seed="true")

    if run_KFMANDB:
        for i in range(0, 10):
            run_trial(experiment=experiment,
                      logging_enabled="true",
                      start_bullet_viewer="true",
                      screenshots_enabled="true",
                      test_id=log_prefix + "KFMANDB_regret_" + str(i),
                      bandit_algorithm="KFMANDB",
                      multi_model="true",
                      calculate_regret="true",
                      use_random_seed="true")
