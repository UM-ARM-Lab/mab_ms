#ifndef XYZGRID_H
#define XYZGRID_H

#include <Eigen/Dense>

namespace smmap
{
    class XYZGrid
    {
        public:
            XYZGrid(const double world_x_min,
                    const double world_x_step,
                    const int64_t world_x_num_steps,

                    const double world_y_min,
                    const double world_y_step,
                    const int64_t world_y_num_steps,

                    const double world_z_min,
                    const double world_z_step,
                    const int64_t world_z_num_steps_);

            ssize_t xyzIndexToGridIndex(const ssize_t x_ind, const ssize_t y_ind, const ssize_t z_ind) const;
            ssize_t worldPosToGridIndex(const double x, const double y, const double z) const;
            ssize_t worldPosToGridIndex(const Eigen::Vector3d& vec) const;
            ssize_t worldPosToGridIndexClamped(const double x, const double y, const double z) const;
            ssize_t worldPosToGridIndexClamped(const Eigen::Vector3d& vec) const;

            double xIndToWorldX(ssize_t x_ind) const;
            double yIndToWorldY(ssize_t y_ind) const;
            double zIndToWorldZ(ssize_t z_ind) const;

            double minStepDimension() const;

            double getXMin() const
            {
                return world_x_min_;
            }
            double getYMin() const
            {
                return world_y_min_;
            }
            double getZMin() const
            {
                return world_z_min_;
            }

            double getXMax() const
            {
                return xIndToWorldX(world_x_num_steps_ - 1);
            }
            double getYMax() const
            {
                return yIndToWorldY(world_y_num_steps_ - 1);
            }
            double getZMax() const
            {
                return zIndToWorldZ(world_z_num_steps_ - 1);
            }

            int64_t getXNumSteps() const
            {
                return world_x_num_steps_;
            }

            int64_t getYNumSteps() const
            {
                return world_y_num_steps_;
            }

            int64_t getZNumSteps() const
            {
                return world_z_num_steps_;
            }

            int64_t getNumCells() const
            {
                return world_x_num_steps_ * world_y_num_steps_ * world_z_num_steps_;
            }

        private:
            /// Variables describing the extents of the graph
            const double world_x_min_;
            const double world_x_step_;
            const int64_t world_x_num_steps_;

            const double world_y_min_;
            const double world_y_step_;
            const int64_t world_y_num_steps_;

            const double world_z_min_;
            const double world_z_step_;
            const int64_t world_z_num_steps_;
    };
}

#endif // XYZGRID_H
