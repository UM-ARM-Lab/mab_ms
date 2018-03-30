#include <arc_utilities/arc_helpers.hpp>
#include "smmap_experiment_params/xyzgrid.h"

using namespace smmap;

XYZGrid::XYZGrid(const double world_x_min,
                 const double world_x_step,
                 const int64_t world_x_num_steps,

                 const double world_y_min,
                 const double world_y_step,
                 const int64_t world_y_num_steps,

                 const double world_z_min,
                 const double world_z_step,
                 const int64_t world_z_num_steps)
    : world_x_min_(world_x_min)
    , world_x_step_(world_x_step)
    , world_x_num_steps_(world_x_num_steps)
    , world_y_min_(world_y_min)
    , world_y_step_(world_y_step)
    , world_y_num_steps_(world_y_num_steps)
    , world_z_min_(world_z_min)
    , world_z_step_(world_z_step)
    , world_z_num_steps_(world_z_num_steps)
{}


ssize_t XYZGrid::xyzIndexToGridIndex(const ssize_t x_ind, const ssize_t y_ind, const ssize_t z_ind) const
{
    // If the point is in the grid, return the index
    if ((0 <= x_ind && x_ind < world_x_num_steps_)
        && (0 <= y_ind && y_ind < world_y_num_steps_)
        && (0 <= z_ind && z_ind < world_z_num_steps_))
    {
        return (x_ind * world_y_num_steps_ + y_ind) * world_z_num_steps_ + z_ind;
    }
    // Otherwise return -1
    else
    {
        return -1;
    }
}

ssize_t XYZGrid::worldPosToGridIndex(const double x, const double y, const double z) const
{
    const int64_t x_ind = std::lround((x - world_x_min_) / world_x_step_);
    const int64_t y_ind = std::lround((y - world_y_min_) / world_y_step_);
    const int64_t z_ind = std::lround((z - world_z_min_) / world_z_step_);

    return xyzIndexToGridIndex(x_ind, y_ind, z_ind);
}

ssize_t XYZGrid::worldPosToGridIndex(const Eigen::Vector3d& vec) const
{
    return worldPosToGridIndex(vec(0), vec(1), vec(2));
}

ssize_t XYZGrid::worldPosToGridIndexClamped(const double x, const double y, const double z) const
{
    const int64_t x_ind = std::lround((x - world_x_min_) / world_x_step_);
    const int64_t y_ind = std::lround((y - world_y_min_) / world_y_step_);
    const int64_t z_ind = std::lround((z - world_z_min_) / world_z_step_);

    return xyzIndexToGridIndex(
                arc_helpers::ClampValue(x_ind, 0L, world_x_num_steps_ - 1),
                arc_helpers::ClampValue(y_ind, 0L, world_y_num_steps_ - 1),
                arc_helpers::ClampValue(z_ind, 0L, world_z_num_steps_ - 1));
}

ssize_t XYZGrid::worldPosToGridIndexClamped(const Eigen::Vector3d& vec) const
{
    return worldPosToGridIndexClamped(vec(0), vec(1), vec(2));
}

double XYZGrid::xIndToWorldX(ssize_t x_ind) const
{
    return world_x_min_ + world_x_step_ * (double)x_ind;
}

double XYZGrid::yIndToWorldY(ssize_t y_ind) const
{
    return world_y_min_ + world_y_step_ * (double)y_ind;
}

double XYZGrid::zIndToWorldZ(ssize_t z_ind) const
{
    return world_z_min_ + world_z_step_ * (double)z_ind;
}

double XYZGrid::minStepDimension() const
{
     return std::min({world_x_step_, world_y_step_, world_z_step_});
}
