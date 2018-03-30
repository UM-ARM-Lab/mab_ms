#include <osg/BlendFunc>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgwTools/Shapes.h>
#include <osgbCollision/CollisionShapes.h>
#include <Serialize/BulletFileLoader/btBulletFile.h>
#include <boost/scoped_array.hpp>

#include "simulation/SetColorsVisitor.h"
#include "simulation/basicobjects.h"
#include "simulation/config_bullet.h"

#define MAX_RAYCAST_DISTANCE 100.0

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Bullet Object                                                              //
////////////////////////////////////////////////////////////////////////////////

BulletObject::BulletObject(CI ci, const btTransform &initTrans, bool isKinematic_)
    : isKinematic(isKinematic_)
{
    assert(ci.m_motionState == nullptr);
    if (isKinematic)
    {
        ci.m_mass = 0;
        ci.m_localInertia = btVector3(0, 0, 0);
    }
    motionState.reset(new MotionState(*this, initTrans));
    ci.m_motionState = motionState.get();
    collisionShape.reset(ci.m_collisionShape);
    rigidBody.reset(new btRigidBody(ci));
    if (isKinematic)
    {
        rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    }
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
}

BulletObject::BulletObject(btScalar mass, btCollisionShape *cs, const btTransform &initTrans, bool isKinematic_)
    : isKinematic(isKinematic_)
{
    motionState.reset(new MotionState(*this, initTrans));
    collisionShape.reset(cs);

    btVector3 fallInertia(0, 0, 0);
    if (!isKinematic)
    {
        collisionShape->calculateLocalInertia(mass, fallInertia);
    }
    CI ci(mass, cs, fallInertia);
    ci.m_motionState = motionState.get();

    rigidBody.reset(new btRigidBody(ci));
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
}

BulletObject::BulletObject(const BulletObject &o)
    : isKinematic(o.isKinematic)
{
    // we need to access lots of private members of btRigidBody and etc
    // the easiest way to do this is to use serialization

    // first copy over the collisionShape. This isn't a real deep copy,
    // but we can share collisionShapes so this should be fine
    collisionShape = o.collisionShape;

    // then copy the motionstate
    motionState = o.motionState->clone(*this);

    // then serialize the rigid body
    boost::shared_ptr<btDefaultSerializer> serializer(new btDefaultSerializer());
    int len; btChunk *chunk; const char *structType;
    // http://www.bulletphysics.com/Bullet/BulletFull/btDiscreteDynamicsWorld_8cpp_source.html#l01147
    serializer->startSerialization();
        // rigid body
        len = o.rigidBody->calculateSerializeBufferSize();
        chunk = serializer->allocate(len, 1);
        structType = o.rigidBody->serialize(chunk->m_oldPtr, serializer.get());
        serializer->finalizeChunk(chunk, structType, BT_RIGIDBODY_CODE, o.rigidBody.get());
        // collision shape
//        len = collisionShape->calculateSerializeBufferSize();
//        chunk = serializer->allocate(len, 1);
//        structType = collisionShape->serialize(chunk->m_oldPtr, serializer.get());
//        serializer->finalizeChunk(chunk, structType, BT_SHAPE_CODE, collisionShape);
        // TODO: constraints?
    serializer->finishSerialization();

    // read the data that the serializer just wrote
    int bufSize = serializer->getCurrentBufferSize();
    boost::scoped_array<char> buf(new char[bufSize]);
    memcpy(buf.get(), serializer->getBufferPointer(), bufSize);
    boost::shared_ptr<bParse::btBulletFile> bulletFile(
        new bParse::btBulletFile(buf.get(), bufSize));
    bulletFile->parse(false);
    // create a new rigidBody with the data
    BOOST_ASSERT(bulletFile->m_rigidBodies.size() == 1);
    if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION)
    {
        // double precision not supported
        BOOST_ASSERT(false);
    }
    else
    {
        // single precision
        btRigidBodyFloatData *data = reinterpret_cast<btRigidBodyFloatData *> (bulletFile->m_rigidBodies[0]);
        btScalar mass = btScalar(data->m_inverseMass? 1.f/data->m_inverseMass : 0.f);
        btVector3 localInertia; localInertia.setZero();
        btTransform startTransform;
        startTransform.deSerializeFloat(data->m_collisionObjectData.m_worldTransform);
        //	startTransform.setBasis(btMatrix3x3::getIdentity());
        if (collisionShape->isNonMoving())
            mass = 0.f;
        if (mass)
            collisionShape->calculateLocalInertia(mass, localInertia);

        // fill in btRigidBody params
        btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(), collisionShape.get(), localInertia);
        ci.m_linearDamping = data->m_linearDamping;
        ci.m_angularDamping = data->m_angularDamping;
        ci.m_additionalDampingFactor = data->m_additionalDampingFactor;
        ci.m_additionalLinearDampingThresholdSqr = data->m_additionalLinearDampingThresholdSqr;
        ci.m_additionalAngularDampingThresholdSqr = data->m_additionalAngularDampingThresholdSqr;
        ci.m_additionalAngularDampingFactor = data->m_additionalAngularDampingFactor;
        ci.m_linearSleepingThreshold = data->m_linearSleepingThreshold;
        ci.m_angularSleepingThreshold = data->m_angularSleepingThreshold;
        ci.m_additionalDamping = data->m_additionalDamping;
        rigidBody.reset(new btRigidBody(ci));

        // extra "active" params
        rigidBody->setLinearVelocity(o.rigidBody->getLinearVelocity());
        rigidBody->setAngularVelocity(o.rigidBody->getAngularVelocity());
        rigidBody->applyCentralForce(o.rigidBody->getTotalForce());
        rigidBody->applyTorque(o.rigidBody->getTotalTorque());

        // fill in btCollisionObject params for the rigid body
        btCollisionObject *colObj = rigidBody.get();
        btCollisionObjectFloatData &colObjData = data->m_collisionObjectData;
        btVector3 temp;
        temp.deSerializeFloat(colObjData.m_anisotropicFriction);
        colObj->setAnisotropicFriction(temp);
        colObj->setContactProcessingThreshold(colObjData.m_contactProcessingThreshold);
        colObj->setFriction(colObjData.m_friction);
        colObj->setRestitution(colObjData.m_restitution);
        colObj->setCollisionFlags(colObjData.m_collisionFlags);
        colObj->setHitFraction(colObjData.m_hitFraction);
        colObj->setActivationState(colObjData.m_activationState1);
    }
}

EnvironmentObject::Ptr BulletObject::copy(Fork &f) const
{
    Ptr o(new BulletObject(*this));
    internalCopy(o, f);
    return o;
}

void BulletObject::internalCopy(BulletObject::Ptr o, Fork &f) const
{
    f.registerCopy(rigidBody.get(), o->rigidBody.get());
}

void BulletObject::init()
{
    getEnvironment()->bullet->dynamicsWorld->addRigidBody(rigidBody.get());
    node = createOSGNode();
    transform = new osg::MatrixTransform();
    transform->addChild(node.get());
    getEnvironment()->osg->root->addChild(transform.get());

    osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
    blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    osg::StateSet *ss = node->getOrCreateStateSet();
    ss->setAttributeAndModes(blendFunc);
    setColorAfterInit();
}

void BulletObject::preDraw()
{
    // before drawing, we must copy the orientation/position
    // of the object from Bullet to OSG
    btTransform btTrans;
    rigidBody->getMotionState()->getWorldTransform(btTrans);

    btScalar m[16];
    btTrans.getOpenGLMatrix(m);
    //transform->setMatrix(osg::Matrix(identityIfBad(m)));
    transform->setMatrix(osg::Matrix(m));
}

void BulletObject::destroy()
{
    getEnvironment()->bullet->dynamicsWorld->removeRigidBody(rigidBody.get());
    getEnvironment()->osg->root->removeChild(transform.get());
}

osg::ref_ptr<osg::Node> BulletObject::createOSGNode()
{
    return osg::ref_ptr<osg::Node>(osgbCollision::osgNodeFromBtCollisionShape(collisionShape.get()));
}

BulletObject::MoveAction::Ptr BulletObject::createMoveAction()
{
    return MoveAction::Ptr(new MoveAction(this));
}

BulletObject::MoveAction::Ptr BulletObject::createMoveAction(const btTransform &start, const btTransform &end, float time)
{
    return MoveAction::Ptr(new MoveAction(this, start, end, time));
}

void BulletObject::setColor(float r, float g, float b, float a)
{
    m_color.reset(new osg::Vec4f(r,g,b,a));
    if (node)
    {
        setColorAfterInit();
    }
}

void BulletObject::setColorAfterInit()
{
    if (m_color)
    {
        if (m_color->a() != 1.0f)
        {
            osg::StateSet *ss = node->getOrCreateStateSet();
            ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        }

        SetColorsVisitor visitor(m_color->r(), m_color->g(), m_color->b(), m_color->a());
        node->accept(visitor);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Constraint Object                                                          //
////////////////////////////////////////////////////////////////////////////////

BulletConstraint::BulletConstraint(btTypedConstraint *cnt_, bool disableCollisionsBetweenLinkedBodies_)
    : cnt(cnt_)
    , disableCollisionsBetweenLinkedBodies(disableCollisionsBetweenLinkedBodies_)
{}

BulletConstraint::BulletConstraint(boost::shared_ptr<btTypedConstraint> cnt_, bool disableCollisionsBetweenLinkedBodies_)
    : cnt(cnt_)
    , disableCollisionsBetweenLinkedBodies(disableCollisionsBetweenLinkedBodies_)
{}

void BulletConstraint::init()
{
    getEnvironment()->bullet->dynamicsWorld->addConstraint(cnt.get(), disableCollisionsBetweenLinkedBodies);
}

void BulletConstraint::destroy()
{
    getEnvironment()->bullet->dynamicsWorld->removeConstraint(cnt.get());
}

EnvironmentObject::Ptr BulletConstraint::copy(Fork &f) const
{
    // serialize the current constraint
    boost::shared_ptr<btDefaultSerializer> serializer(new btDefaultSerializer());
    serializer->startSerialization();
    int len = cnt->calculateSerializeBufferSize();
    btChunk *chunk = serializer->allocate(len, 1);
    const char *structType = cnt->serialize(chunk->m_oldPtr, serializer.get());
    serializer->finalizeChunk(chunk, structType, BT_CONSTRAINT_CODE, cnt.get());
    serializer->finishSerialization();

    // read the data that the serializer just wrote
    int bufSize = serializer->getCurrentBufferSize();
    boost::scoped_array<char> buf(new char[bufSize]);
    memcpy(buf.get(), serializer->getBufferPointer(), bufSize);
    boost::shared_ptr<bParse::btBulletFile> bulletFile(
        new bParse::btBulletFile(buf.get(), bufSize));
    bulletFile->parse(false);

    BOOST_ASSERT(bulletFile->m_constraints.size() == 1);
    if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION)
    {
        // double precision not supported
        BOOST_ASSERT(false);
        return Ptr();
    }

    // get equivalents of rigid bodies in the new cloned world
    btRigidBody *rbA = (btRigidBody *) f.copyOf(&cnt->getRigidBodyA());
    btRigidBody *rbB = (btRigidBody *) f.copyOf(&cnt->getRigidBodyB());

    // create new constraint
    boost::shared_ptr<btTypedConstraint> newcnt;
    btTypedConstraintData *constraintData = (btTypedConstraintData*)bulletFile->m_constraints[0];
    if (constraintData->m_objectType == POINT2POINT_CONSTRAINT_TYPE)
    {
        btPoint2PointConstraintFloatData* p2pData = (btPoint2PointConstraintFloatData*)constraintData;
        if (rbA && rbB)
        {
            btVector3 pivotInA,pivotInB;
            pivotInA.deSerializeFloat(p2pData->m_pivotInA);
            pivotInB.deSerializeFloat(p2pData->m_pivotInB);
            newcnt.reset(new btPoint2PointConstraint(*rbA, *rbB, pivotInA, pivotInB));
        }
        else
        {
            btVector3 pivotInA;
            pivotInA.deSerializeFloat(p2pData->m_pivotInA);
            newcnt.reset(new btPoint2PointConstraint(*rbA, pivotInA));
        }

    }
    else if (constraintData->m_objectType == HINGE_CONSTRAINT_TYPE)
    {
        btHingeConstraintFloatData* hingeData = (btHingeConstraintFloatData*)constraintData;
        btHingeConstraint *hinge = 0;
        if (rbA && rbB)
        {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
            rbBFrame.deSerializeFloat(hingeData->m_rbBFrame);
            hinge = new btHingeConstraint(*rbA, *rbB, rbAFrame, rbBFrame, hingeData->m_useReferenceFrameA != 0);
        }
        else
        {
            btTransform rbAFrame;
            rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
            hinge = new btHingeConstraint(*rbA, rbAFrame, hingeData->m_useReferenceFrameA != 0);
        }
        if (hingeData->m_enableAngularMotor)
        {
            hinge->enableAngularMotor(true, hingeData->m_motorTargetVelocity, hingeData->m_maxMotorImpulse);
        }
        hinge->setAngularOnly(hingeData->m_angularOnly != 0);
        hinge->setLimit(btScalar(hingeData->m_lowerLimit), btScalar(hingeData->m_upperLimit), btScalar(hingeData->m_limitSoftness), btScalar(hingeData->m_biasFactor), btScalar(hingeData->m_relaxationFactor));
        newcnt.reset(hinge);

    }
    else if (constraintData->m_objectType == CONETWIST_CONSTRAINT_TYPE)
    {
        btConeTwistConstraintData* coneData = (btConeTwistConstraintData*)constraintData;
        btConeTwistConstraint* coneTwist = 0;
        if (rbA&& rbB)
        {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
            rbBFrame.deSerializeFloat(coneData->m_rbBFrame);
            coneTwist = new btConeTwistConstraint(*rbA, *rbB, rbAFrame, rbBFrame);
        }
        else
        {
            btTransform rbAFrame;
            rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
            coneTwist = new btConeTwistConstraint(*rbA,rbAFrame);
        }
        coneTwist->setLimit(coneData->m_swingSpan1, coneData->m_swingSpan2, coneData->m_twistSpan, coneData->m_limitSoftness, coneData->m_biasFactor, coneData->m_relaxationFactor);
        coneTwist->setDamping(coneData->m_damping);
        newcnt.reset(coneTwist);

    }
    else if (constraintData->m_objectType == D6_SPRING_CONSTRAINT_TYPE)
    {
        btGeneric6DofSpringConstraintData* dofData = (btGeneric6DofSpringConstraintData*)constraintData;
        btGeneric6DofSpringConstraint* dof = 0;
        if (rbA && rbB)
        {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(dofData->m_6dofData.m_rbAFrame);
            rbBFrame.deSerializeFloat(dofData->m_6dofData.m_rbBFrame);
            dof = new btGeneric6DofSpringConstraint(*rbA, *rbB, rbAFrame, rbBFrame, dofData->m_6dofData.m_useLinearReferenceFrameA != 0);
        }
        else
        {
            printf("Error in btWorldImporter::createGeneric6DofSpringConstraint: requires rbA && rbB\n");
            BOOST_ASSERT(false);
        }
        if (dof)
        {
            btVector3 angLowerLimit,angUpperLimit, linLowerLimit,linUpperlimit;
            angLowerLimit.deSerializeFloat(dofData->m_6dofData.m_angularLowerLimit);
            angUpperLimit.deSerializeFloat(dofData->m_6dofData.m_angularUpperLimit);
            linLowerLimit.deSerializeFloat(dofData->m_6dofData.m_linearLowerLimit);
            linUpperlimit.deSerializeFloat(dofData->m_6dofData.m_linearUpperLimit);

            dof->setAngularLowerLimit(angLowerLimit);
            dof->setAngularUpperLimit(angUpperLimit);
            dof->setLinearLowerLimit(linLowerLimit);
            dof->setLinearUpperLimit(linUpperlimit);

            for (int i = 0; i < 6; i++)
            {
                dof->setStiffness(i, dofData->m_springStiffness[i]);
                dof->setEquilibriumPoint(i, dofData->m_equilibriumPoint[i]);
                dof->enableSpring(i, dofData->m_springEnabled[i] != 0);
                dof->setDamping(i, dofData->m_springDamping[i]);
            }
        }
        newcnt.reset(dof);

    }
    else if (constraintData->m_objectType == D6_CONSTRAINT_TYPE)
    {
        btGeneric6DofConstraintData* dofData = (btGeneric6DofConstraintData*)constraintData;
        btGeneric6DofConstraint* dof = 0;
        if (rbA&& rbB)
        {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(dofData->m_rbAFrame);
            rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
            dof = new btGeneric6DofConstraint(*rbA, *rbB, rbAFrame, rbBFrame, dofData->m_useLinearReferenceFrameA != 0);
        }
        else
        {
            if (rbB)
            {
                btTransform rbBFrame;
                rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
                dof = new btGeneric6DofConstraint(*rbB, rbBFrame, dofData->m_useLinearReferenceFrameA != 0);
            }
            else
            {
                printf("Error in btWorldImporter::createGeneric6DofConstraint: missing rbB\n");
                BOOST_ASSERT(false);
            }
        }
        if (dof)
        {
            btVector3 angLowerLimit,angUpperLimit, linLowerLimit,linUpperlimit;
            angLowerLimit.deSerializeFloat(dofData->m_angularLowerLimit);
            angUpperLimit.deSerializeFloat(dofData->m_angularUpperLimit);
            linLowerLimit.deSerializeFloat(dofData->m_linearLowerLimit);
            linUpperlimit.deSerializeFloat(dofData->m_linearUpperLimit);

            dof->setAngularLowerLimit(angLowerLimit);
            dof->setAngularUpperLimit(angUpperLimit);
            dof->setLinearLowerLimit(linLowerLimit);
            dof->setLinearUpperLimit(linUpperlimit);
        }
        newcnt.reset(dof);

    }
    else if (constraintData->m_objectType == SLIDER_CONSTRAINT_TYPE)
    {
        btSliderConstraintData* sliderData = (btSliderConstraintData*)constraintData;
        btSliderConstraint* slider = 0;
        if (rbA&& rbB)
        {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(sliderData->m_rbAFrame);
            rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
            slider = new btSliderConstraint(*rbA, *rbB, rbAFrame, rbBFrame, sliderData->m_useLinearReferenceFrameA != 0);
        }
        else
        {
            btTransform rbBFrame;
            rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
            slider = new btSliderConstraint(*rbB, rbBFrame, sliderData->m_useLinearReferenceFrameA != 0);
        }
        slider->setLowerLinLimit(sliderData->m_linearLowerLimit);
        slider->setUpperLinLimit(sliderData->m_linearUpperLimit);
        slider->setLowerAngLimit(sliderData->m_angularLowerLimit);
        slider->setUpperAngLimit(sliderData->m_angularUpperLimit);
        slider->setUseFrameOffset(sliderData->m_useOffsetForConstraintFrame != 0);
        newcnt.reset(slider);

    }
    else
    {
        printf("unknown constraint type\n");
    }

    if (newcnt)
    {
        f.registerCopy(cnt.get(), newcnt.get());
    }

    return Ptr(new BulletConstraint(newcnt, disableCollisionsBetweenLinkedBodies));
}
