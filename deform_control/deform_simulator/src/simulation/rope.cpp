#include "simulation/rope.h"
#include "simulation/basicobjects.h"
#include <iostream>
#include <boost/make_shared.hpp>

using namespace std;

boost::shared_ptr<btGeneric6DofSpringConstraint>  createBendConstraint(
        btScalar len,
        const boost::shared_ptr<btRigidBody> rbA,
        const boost::shared_ptr<btRigidBody>& rbB,
        float damping, float stiffness, float limit)
{
    btTransform tA, tB;
    tA.setIdentity();
    tB.setIdentity();
    tA.setOrigin(btVector3(len / 2, 0,0));
    tB.setOrigin(btVector3(-len / 2, 0,0));

    boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr =
            boost::make_shared<btGeneric6DofSpringConstraint>(*rbA, *rbB, tA, tB, false);
    for (int i = 3; i <= 5; i++)
    {
        springPtr->enableSpring(i, true);
        springPtr->setStiffness(i, stiffness);
        springPtr->setDamping(i, damping);
    }

    springPtr->setAngularLowerLimit(btVector3(-limit, -limit, -limit));
    springPtr->setAngularUpperLimit(btVector3(limit, limit, limit));
    return springPtr;
}

void createRopeTransforms(
        vector<btTransform>& transforms,
        vector<btScalar>& lengths,
        const vector<btVector3>& ctrlPoints)
{
    const int nLinks = ctrlPoints.size() - 1;
    for (int i = 0; i < nLinks; i++)
    {
        btVector3 pt0 = ctrlPoints[i];
        btVector3 pt1 = ctrlPoints[i+1];
        btVector3 midpt = (pt0+pt1)/2;
        btVector3 diff = (pt1-pt0);

        btQuaternion q;
        btScalar ang = diff.angle(btVector3(1,0,0));
        if (ang * ang > 1e-4)
        {
            btVector3 ax = diff.cross(btVector3(1,0,0));
            q = btQuaternion(ax, -ang);
        }
        else
        {
            q = btQuaternion(0, 0, 0, 1);
        }
        btTransform trans(q, midpt);

        float len = diff.length();
        transforms.push_back(trans);
        lengths.push_back(len);
    }
}


CapsuleRope::CapsuleRope(
        const vector<btVector3>& ctrlPoints,
        btScalar radius_, float angStiffness_,
        float angDamping_, float linDamping_, float angLimit_)
    : angStiffness(angStiffness_)
    , angDamping(angDamping_)
    , linDamping(linDamping_)
    , angLimit(angLimit_)
    , radius(radius_)
    , nLinks(ctrlPoints.size() - 1)
{
    vector<btTransform> transforms;
    vector<btScalar> lengths;
    createRopeTransforms(transforms, lengths, ctrlPoints);
    for (int i = 0; i < nLinks; i++)
    {
        btTransform trans = transforms[i];
        btScalar len = lengths[i];
        float mass = 1;
        CapsuleObject::Ptr child(new CapsuleObject(1,radius,len,trans));
        child->rigidBody->setDamping(linDamping,angDamping);
        child->rigidBody->setFriction(1);

        children.push_back(child);

        if (i > 0)
        {
            auto jointPtr = boost::make_shared<btPoint2PointConstraint>(
                        *children[i-1]->rigidBody,
                        *children[i]->rigidBody,
                        btVector3(len / 2.0, 0, 0),
                        btVector3(-len / 2.0, 0, 0));
            joints.push_back(boost::make_shared<BulletConstraint>(jointPtr, true));

            auto springPtr = createBendConstraint(
                        len,
                        children[i-1]->rigidBody,
                        children[i]->rigidBody,
                        angDamping,
                        angStiffness,
                        angLimit);
            joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtr, true)));
        }
    }
}

void CapsuleRope::init()
{
    CompoundObject<BulletObject>::init();

    for (int i = 0; i< joints.size(); i++)
    {
        getEnvironment()->addConstraint(joints[i]);
    }
}

void CapsuleRope::destroy()
{
    CompoundObject<BulletObject>::destroy();
}

vector<btVector3> CapsuleRope::getNodes() const
{
    vector<btVector3> out(children.size());
    for (int i = 0; i < children.size(); i++)
    {
        out[i] = children[i]->rigidBody->getCenterOfMassPosition();
    }
    return out;
}

vector<btVector3> CapsuleRope::getControlPoints() const
{
    vector<btVector3> out;
    out.reserve(children.size()+1);
    for (int i = 0; i < children.size(); i++)
    {
        btRigidBody* body = children[i]->rigidBody.get();
        btCapsuleShape* capsule = dynamic_cast<btCapsuleShapeX*>(body->getCollisionShape());
        btTransform tf = body->getCenterOfMassTransform();
        if (i == 0)
        {
            out.push_back(tf.getOrigin() + btMatrix3x3(tf.getRotation()) * btVector3(-capsule->getHalfHeight(), 0, 0));
        }
        out.push_back(tf.getOrigin() + btMatrix3x3(tf.getRotation()) * btVector3(capsule->getHalfHeight(), 0, 0));
    }
    return out;
}
