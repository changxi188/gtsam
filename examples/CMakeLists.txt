set (excluded_examples
    elaboratePoint2KalmanFilter.cpp
)


#gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
gtsamAddExamplesGlob("self_test_SOn.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
gtsamAddExamplesGlob("self_test_between_factor.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
gtsamAddExamplesGlob("self_test_isam2_bayes_tree.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
gtsamAddExamplesGlob("self_test_incremental_fixed_lag_smoother.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
gtsamAddExamplesGlob("self_test_basic_data_struct.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
gtsamAddExamplesGlob("VisualISAMExample.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
gtsamAddExamplesGlob("VisualISAM2Example.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")

add_subdirectory(self_test)
