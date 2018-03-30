#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
// Struct used to create and interact with an OpenSceneGraph.                 //
////////////////////////////////////////////////////////////////////////////////

struct OSGInstance
{
    typedef boost::shared_ptr<OSGInstance> Ptr;

    osg::ref_ptr<osg::Group> root;

    OSGInstance();
};

////////////////////////////////////////////////////////////////////////////////
// Struct used to create and interact with a Bullet dynamics world.           //
////////////////////////////////////////////////////////////////////////////////

struct BulletInstance
{
    typedef boost::shared_ptr<BulletInstance> Ptr;

    btBroadphaseInterface *broadphase;
    btSoftBodyRigidBodyCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btSequentialImpulseConstraintSolver *solver;
    btSoftRigidDynamicsWorld *dynamicsWorld;
    btSoftBodyWorldInfo softBodyWorldInfo;

    BulletInstance();
    ~BulletInstance();

    void setGravity(const btVector3 &gravity);

    // Populates out with all objects colliding with obj, possibly ignoring some objects
    // dynamicsWorld->updateAabbs() must be called before contactTest
    // see http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=4850
    typedef std::set<const btCollisionObject *> CollisionObjectSet;
    void contactTest(btCollisionObject *obj, CollisionObjectSet &out, const CollisionObjectSet *ignore = nullptr);
};

struct Environment;
class Fork;

////////////////////////////////////////////////////////////////////////////////
// Base object type used to define what an object in OSG/Bullet has to be     //
// able to do.                                                                //
////////////////////////////////////////////////////////////////////////////////

class EnvironmentObject
{
    private:
        Environment *env;

    public:
        typedef boost::shared_ptr<EnvironmentObject> Ptr;

        EnvironmentObject()
            : env(nullptr)
        {}

        EnvironmentObject(Environment *env_)
            : env(env_)
        {}

        virtual ~EnvironmentObject()
        {}

        Environment *getEnvironment()
        {
            return env;
        }

        // These are for environment forking.
        // copy() should return a copy of the object suitable for addition
        // into the environment contained by f. This should NOT add objects to f.env;
        // however it should call f.registerCopy() for each Bullet
        // collision object that it makes.
        virtual EnvironmentObject::Ptr copy(Fork &f) const = 0;
        // postCopy is called after all objects are copied and inserted into f.env.
        // This is useful for updating constraints, anchors, etc.
        // You're free to use f.forkOf()  or f.copyOf() to get equivalent objects in the new env.
        virtual void postCopy(EnvironmentObject::Ptr copy, Fork &f) const
        {
            (void)copy;
            (void)f;
        }

        // methods only to be called by the Environment
        void setEnvironment(Environment *env_)
        {
            env = env_;
        }
        virtual void init() {}
        virtual void prePhysics() {}
        virtual void preDraw() {}
        virtual void destroy() {}
};

////////////////////////////////////////////////////////////////////////////////
// Struct used to manage the collections of objects in both Bullet and        //
// OpenSceneGraph at the same time.                                           //
////////////////////////////////////////////////////////////////////////////////

struct Environment
{
    typedef boost::shared_ptr<Environment> Ptr;

    BulletInstance::Ptr bullet;
    OSGInstance::Ptr osg;

    typedef std::vector<EnvironmentObject::Ptr> ObjectList;
    ObjectList objects;

    typedef std::vector<EnvironmentObject::Ptr> ConstraintList;
    ConstraintList constraints;

    Environment(BulletInstance::Ptr bullet_, OSGInstance::Ptr osg_);
    ~Environment();

    // objects are reponsible for adding themselves
    // to the dynamics world and the osg root via init()
    void add(EnvironmentObject::Ptr obj);
    void remove(EnvironmentObject::Ptr obj);

    // objects are reponsible for adding themselves
    // to the dynamics world and the osg root via init()
    void addConstraint(EnvironmentObject::Ptr cnt);
    void removeConstraint(EnvironmentObject::Ptr cnt);

    // Returns the amount of time that has passed in the simulator
    double step(btScalar dt, int maxSubSteps, btScalar fixedTimeStep);
};

////////////////////////////////////////////////////////////////////////////////
// An Environment Fork is a wrapper around an Environment with an operator    //
// that associates copied objects with their original ones                    //
////////////////////////////////////////////////////////////////////////////////

class Fork
{
    public:
        typedef boost::shared_ptr<Fork> Ptr;

        const Environment *parentEnv;
        Environment::Ptr env;

        // maps object in parentEnv to object in env
        typedef std::map<EnvironmentObject::Ptr, EnvironmentObject::Ptr> ObjectMap;
        ObjectMap objMap;

        typedef std::map<const void *, void *> DataMap;
        DataMap dataMap;
        void registerCopy(const void *orig, void *copy)
        {
            // make sure that the original doesn't exist in the map already
            assert(orig != nullptr && copy != nullptr && copyOf(orig) == nullptr);
            dataMap.insert(std::make_pair(orig, copy));
        }

        Fork(const Environment *parentEnv_, BulletInstance::Ptr bullet, OSGInstance::Ptr osg)
            : parentEnv(parentEnv_)
            , env(new Environment(bullet, osg))
        {
            copyObjects();
        }

        Fork(const Environment::Ptr parentEnv_, BulletInstance::Ptr bullet, OSGInstance::Ptr osg)
            : Fork(parentEnv_.get(), bullet, osg)
        {}

        void* copyOf(const void* orig) const
        {
            DataMap::const_iterator ittr = dataMap.find(orig);
            return ittr == dataMap.end() ? nullptr : ittr->second;
        }

        EnvironmentObject::Ptr forkOf(EnvironmentObject::Ptr orig) const
        {
            ObjectMap::const_iterator ittr = objMap.find(orig);
            return ittr == objMap.end() ? EnvironmentObject::Ptr() : ittr->second;
        }

    private:
        void copyObjects()
        {
            // copy objects first
            for (Environment::ObjectList::const_iterator ittr = parentEnv->objects.begin(); ittr != parentEnv->objects.end(); ++ittr)
            {
                EnvironmentObject::Ptr copy = (*ittr)->copy(*this);
                env->add(copy);
                objMap[*ittr] = copy;
            }
            // some objects might need processing after all objects have been added
            // e.g. anchors and joints for soft bodies
            for (Environment::ObjectList::const_iterator ittr = parentEnv->objects.begin(); ittr != parentEnv->objects.end(); ++ittr)
            {
                (*ittr)->postCopy(objMap[*ittr], *this);
            }

            // copy constraints
            for (Environment::ConstraintList::const_iterator ittr = parentEnv->constraints.begin(); ittr != parentEnv->constraints.end(); ++ittr)
            {
                EnvironmentObject::Ptr copy = (*ittr)->copy(*this);
                env->addConstraint(copy);
                objMap[*ittr] = copy;
            }
            for (Environment::ConstraintList::const_iterator ittr = parentEnv->constraints.begin(); ittr != parentEnv->constraints.end(); ++ittr)
            {
                (*ittr)->postCopy(objMap[*ittr], *this);
            }
        }
};

class Action
{
    public:
        typedef boost::shared_ptr<Action> Ptr;

        Action()
            : Action(1.0f)
        {}

        Action(float execTime_)
            : timeElapsed(0.0f)
            , execTime(execTime_)
            , isDone(false)
        {}

        virtual bool done() const
        {
            return timeElapsed >= execTime || isDone;
        }

        virtual void reset()
        {
            timeElapsed = 0.; setDone(false);
        }

        virtual void setExecTime(float t)
        {
            execTime = t;
        }

        virtual void step(float dt) = 0;

    protected:
        float timeElapsed;
        float execTime;
        bool isDone;
        int plotOnly;

        void setDone(bool b)
        {
            isDone = b;
        }

        void stepTime(float dt)
        {
            timeElapsed += dt;
        }

        float fracElapsed() const
        {
            return std::min(timeElapsed / execTime, 1.0f);
        }
};

#endif // _ENVIRONMENT_H_
