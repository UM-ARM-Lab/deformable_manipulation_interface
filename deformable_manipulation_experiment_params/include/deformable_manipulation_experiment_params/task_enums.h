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

        // NEWLY ADD FOR TEST, Mengyao
        ROPE_DRAG_OPPOSITE_TABLE,       // directional rigidity
        ROPE_DRAG_ALONG_TABLE,          // directional rigidity
        ROPE_TOWARD_TABLE,              // mask, constraint violation
        ROPE_CROSS,                     // directional rigidity use Dale's directly

        // Live robot experiments
        CLOTH_PLACEMAT_LIVE_ROBOT
    };

    enum PlannerTrialType
    {
        DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_CONTROLLER,
        ADAPTIVE_JACOBIAN_SINGLE_MODEL_LEAST_SQUARES_CONTROLLER,
        CONSTRAINT_SINGLE_MODEL_CONSTRAINT_CONTROLLER,
        DIMINISHING_RIGIDITY_SINGLE_MODEL_CONSTRAINT_CONTROLLER,
        MULTI_MODEL_BANDIT_TEST,
        MULTI_MODEL_CONTROLLER_TEST
    };

    enum GripperControllerType
    {
        RANDOM_SAMPLING,
        NOMAD_OPTIMIZATION
    };

}

#endif // TASK_ENUMS_H
