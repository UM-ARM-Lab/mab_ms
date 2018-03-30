#ifndef GRIPPER_KINEMATIC_OBJECT_H
#define GRIPPER_KINEMATIC_OBJECT_H

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"

class GripperKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<GripperKinematicObject> Ptr;

        GripperKinematicObject(const std::string& name_input, float apperture_input, btVector4 color = btVector4(0.6f, 0.6f, 0.6f, 0.9f));

        void translate(btVector3 transvec);
        void applyTransform(btTransform tm);
        void setWorldTransform(btTransform tm);
        btTransform getWorldTransform();
        void getWorldTransform(btTransform& in);

        // used only for the rope
        void rigidGrab(btRigidBody* prb, size_t objectnodeind, Environment::Ptr env_ptr);

        // Toggles open/close
        void toggleOpen();

        // Toggles attached/not attached (btSoftBody term)
        // if radius = 0, we are not using radius, we are using an alternate method
        void toggleAttach(btSoftBody * psb, double radius = 0);

        void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts);
        void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence = 1);
        void releaseAllAnchors(btSoftBody * psb);

        const std::vector<size_t>& getAttachedNodeIndices() const;

        const btVector3& getHalfExtents() const;
        float getGripperRadius() const;

        // Used by the manual grippers for cloth
        void step_openclose(btSoftBody * psb);

        EnvironmentObject::Ptr copy(Fork &f) const;
        void internalCopy(GripperKinematicObject::Ptr o, Fork &f) const;

        friend std::ostream& operator<< (std::ostream& stream, const GripperKinematicObject& gripper);

    private:
        std::string name;
        btVector3 halfextents;

        btTransform cur_tm;

        enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

        GripperState state;
        bool bOpen;         // used only for cloth (I think)
        float apperture;
        float closed_gap;   // used only for cloth (I think)

        bool bAttached;
        std::vector<size_t> vattached_node_inds;

        boost::shared_ptr<btGeneric6DofConstraint> rope_cnt;
};

#endif
