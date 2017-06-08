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
        CLOTH_CYLINDER_COVERAGE,
        CLOTH_TABLE_COVERAGE,
        CLOTH_COLAB_FOLDING,
        CLOTH_WAFR,
        CLOTH_SINGLE_POLE,
        CLOTH_WALL,
        CLOTH_DOUBLE_SLIT,
        ROPE_MAZE,

        // NEWLY ADD FOR TEST, Mengyao
        ROPE_DRAG_OPPOSITE_TABLE,      // directional rigidity
        ROPE_DRAG_ALONG_TABLE,         // directional rigidity
        ROPE_TOWARD_TABLE,            // mask, constraint violation
        ROPE_CROSS                    // directional rigidity use Dale's directly
    };
}

#endif // TASK_ENUMS_H