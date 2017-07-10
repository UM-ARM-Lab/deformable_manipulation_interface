#ifndef TASK_ENUMS_H
#define TASK_ENUMS_H

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
        ROPE_CYLINDER_COVERAGE_DOUBLE_GRIPPER,
        CLOTH_CYLINDER_COVERAGE,
        CLOTH_TABLE_COVERAGE,
        CLOTH_COLAB_FOLDING,
        CLOTH_WAFR,
        CLOTH_SINGLE_POLE,
        CLOTH_WALL,
        CLOTH_DOUBLE_SLIT,
        ROPE_MAZE
    };
}

#endif // TASK_ENUMS_H
