#ifndef TASK_ENUMS_H
#define TASK_ENUMS_H

// TODO: rename TASK_ENUMS to something else, as this file now has more ENUMs that just task enums

namespace smmap
{
    enum DeformableType
    {
        ROPE,
        CLOTH
    };

    enum TaskType
    {
        ROPE_CYLINDER_COVERAGE,
        ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS,
        CLOTH_CYLINDER_COVERAGE,
        CLOTH_TABLE_COVERAGE,
        CLOTH_COLAB_FOLDING,
        CLOTH_WAFR,
        CLOTH_SINGLE_POLE,
        CLOTH_WALL,
        CLOTH_DOUBLE_SLIT,
        ROPE_MAZE,
        ROPE_ZIG_MATCH,

        // Model accuracy tests
        ROPE_TABLE_LINEAR_MOTION,   // directional rigidity
        CLOTH_TABLE_LINEAR_MOTION,  // constraint violation
        ROPE_TABLE_PENTRATION,      // constraint violation
        CLOTH_PLACEMAT_LINEAR_MOTION, // Live robot directional rigidity

        // Live robot experiments
        CLOTH_PLACEMAT_LIVE_ROBOT,
        CLOTH_MFLAG_LIVE_ROBOT,
        ROPE_SIMPLE_COVERAGE_TWO_GRIPPERS,

        // Hooks
        ROPE_HOOKS,
        CLOTH_HOOKS,
        ROPE_HOOKS_DATA_GENERATION
    };

    enum TrialType
    {
        DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_AVOIDANCE_CONTROLLER,
        ADAPTIVE_JACOBIAN_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_AVOIDANCE_CONTROLLER,
        DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_CONSTRAINT_CONTROLLER,
        CONSTRAINT_SINGLE_MODEL_CONSTRAINT_CONTROLLER,
        DIMINISHING_RIGIDITY_SINGLE_MODEL_CONSTRAINT_CONTROLLER,
        MULTI_MODEL_BANDIT_TEST,
        MULTI_MODEL_CONTROLLER_TEST,
        MULTI_MODEL_ACCURACY_TEST
    };

    enum MABAlgorithm
    {
        UCB1Normal,
        KFMANB,
        KFMANDB
    };

    enum StretchingConstraintControllerSolverType
    {
        RANDOM_SAMPLING,
        NOMAD_OPTIMIZATION,
        GRADIENT_DESCENT
    };

}

#endif // TASK_ENUMS_H
