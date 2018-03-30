#pragma once
#include "simulation/environment.h"
#include "simulation/basicobjects.h"
#include <btBulletDynamicsCommon.h>
#include <vector>

class CapsuleRope : public CompoundObject<BulletObject>
{
    private:
        const float angStiffness;
        const float angDamping;
        const float linDamping;
        const float angLimit;

    protected:
        virtual void init();
        virtual void destroy();

    public:
        typedef boost::shared_ptr<CapsuleRope> Ptr;
        std::vector<boost::shared_ptr<btCollisionShape> > shapes;
        std::vector<BulletConstraint::Ptr> joints;
        const btScalar radius;
        const int nLinks;

        CapsuleRope(
                const std::vector<btVector3>& ctrlPoints,
                float radius_, float angStiffness_=.1f,
                float angDamping_=1, float linDamping_=.75f, float angLimit_=.4f);

        std::vector<btVector3> getNodes() const;
        std::vector<btVector3> getControlPoints() const;
};
