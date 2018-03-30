#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "utils/util.h"

#include <iostream>
#include <boost/bind.hpp>
using namespace std;

Scene::Scene() : drawingOn(false), syncTime(false), simTime(0) {
    osg.reset(new OSGInstance());
    bullet.reset(new BulletInstance());
    bullet->setGravity(BulletConfig::gravity);

    env.reset(new Environment(bullet, osg));

    // plots for debugging
    plotPoints.reset(new PlotPoints(GeneralConfig::scale * 0.5));
    env->add(plotPoints);
    plotLines.reset(new PlotLines(GeneralConfig::scale * 0.5));
    env->add(plotLines);

    // populate the scene with some basic objects
    ground.reset(new PlaneStaticObject(btVector3(0., 0., 1.), 0., btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)), 75.0f));
    env->add(ground);

    // default callbacks
    addVoidKeyCallback('p', boost::bind(&Scene::toggleIdle, this));
    addVoidKeyCallback('d', boost::bind(&Scene::toggleDebugDraw, this));
}

void Scene::startViewer() {
    drawingOn = true;
    loopState.looping = loopState.paused = loopState.debugDraw = loopState.skip_step = false;

    dbgDraw.reset(new osgbCollision::GLDebugDrawer());
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    dbgDraw->setEnabled(false);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw.get());
    osg->root->addChild(dbgDraw->getSceneGraph());

    viewer.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    manip = new EventHandler(*this);
    manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition), util::toOSGVector(ViewerConfig::pointCameraLooksAt), osg::Z_AXIS);
    viewer.setCameraManipulator(manip);
    viewer.setSceneData(osg->root.get());
    viewer.realize();
    step(0);
}

void Scene::toggleDebugDraw() {
    loopState.debugDraw = !loopState.debugDraw;
    dbgDraw->setEnabled(loopState.debugDraw);
}

void Scene::step(float dt, int maxsteps, float internaldt) {
    static float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;

    if (drawingOn && loopState.debugDraw && !dbgDraw->getActive())
    {
        dbgDraw->BeginDraw();
    }

    if (syncTime && drawingOn)
        endTime = viewer.getFrameStamp()->getSimulationTime();

    // run pre-step callbacks
    for (size_t i = 0; i < prestepCallbacks.size(); ++i)
        prestepCallbacks[i]();

    simTime += env->step(dt, maxsteps, internaldt);
    // Note that this only steps the registered forks
    for (std::set<Fork::Ptr>::iterator i = forks.begin(); i != forks.end(); ++i)
        (*i)->env->step(dt, maxsteps, internaldt);

    // run post-step callbacks
    for (size_t i = 0; i < poststepCallbacks.size(); ++i)
        poststepCallbacks[i]();

    draw();

    if (syncTime && drawingOn) {
        float timeLeft = dt - (endTime - startTime);
        idleFor(timeLeft);
        startTime = endTime + timeLeft;
    }
}

void Scene::step(float dt) {
    step(dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
}

// Steps for a time interval
void Scene::stepFor(float dt, float time) {
    while (time > 0) {
        step(dt);
        time -= dt;
    }
}

// Idles for a time interval. Physics will not run.
void Scene::idleFor(float time) {
    if (!drawingOn || !syncTime || time <= 0.f)
        return;
    float endTime = time + viewer.getFrameStamp()->getSimulationTime();
    while (viewer.getFrameStamp()->getSimulationTime() < endTime && !viewer.done()) {
      draw();
      usleep(10000);
    }
}

void Scene::draw() {
    if (!drawingOn)
        return;
    if (loopState.debugDraw) {
        // This call was moved to the start of the step() function as part of the bullet collision pipeline involves drawing things when debug draw is enabled
        if (!dbgDraw->getActive()) {
            dbgDraw->BeginDraw();
        }
        bullet->dynamicsWorld->debugDrawWorld();
        dbgDraw->EndDraw();
    }
    viewer.frame();
}

void Scene::startLoop() {
    bool oldSyncTime = syncTime;
    syncTime = false;
    loopState.looping = true;
    loopState.prevTime = loopState.currTime =
        viewer.getFrameStamp()->getSimulationTime();
    while (loopState.looping && drawingOn && !viewer.done()) {
        loopState.currTime = viewer.getFrameStamp()->getSimulationTime();
        step(loopState.currTime - loopState.prevTime);
        loopState.prevTime = loopState.currTime;
    }
    syncTime = oldSyncTime;
}

void Scene::startFixedTimestepLoop(float dt) {
    loopState.looping = true;
    while (loopState.looping && drawingOn && !viewer.done())
    {
        if(loopState.skip_step)
        {
            if(drawingOn && !viewer.done())
                draw();
            usleep(10000);
        }
        else
            step(dt);
    }
}

void Scene::stopLoop() {
    loopState.looping = false;
}

void Scene::idle(bool b) {
    loopState.paused = b;
    while (loopState.paused && drawingOn && !viewer.done()) {
      draw();
      usleep(10000);
    }
    loopState.prevTime = loopState.currTime = viewer.getFrameStamp()->getSimulationTime();
}

void Scene::toggleIdle() {
    idle(!loopState.paused);
}

void Scene::runAction(Action &a, float dt) {
    while (!a.done()) {
        a.step(dt);
        step(dt);
    }
}

void Scene::addKeyCallback(char c, Callback cb) {
    if (keyCallbacks.count(c) != 0)
        cout << "warning: key " << c << " is bound to multiple callbacks" << endl;
    keyCallbacks.insert(make_pair(c, cb));
}

void Scene::addVoidCallback(osgGA::GUIEventAdapter::EventType t, VoidCallback cb) {
    addCallback(t, boost::bind<bool>(VoidCallbackWrapper(cb)));
}

void Scene::addVoidKeyCallback(char c, VoidCallback cb) {
    addKeyCallback(c, boost::bind<bool>(VoidCallbackWrapper(cb)));
}

void Scene::addPreStepCallback(VoidCallback cb) {
    prestepCallbacks.push_back(cb);
}

void Scene::addPostStepCallback(VoidCallback cb) {
    poststepCallbacks.push_back(cb);
}

void EventHandler::getTransformation(osg::Vec3d &eye, osg::Vec3d &center, osg::Vec3d &up) const {
    center = _center;
    eye = _center + _rotation * osg::Vec3d(0., 0., _distance);
    up = _rotation * osg::Vec3d(0., 1., 0.);
}

bool EventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    bool suppressDefault = false;
    osgGA::GUIEventAdapter::EventType t = ea.getEventType();

    // keypress handlers (for convenience)
    if (t == osgGA::GUIEventAdapter::KEYDOWN) {
        pair<Scene::KeyCallbackMap::const_iterator, Scene::KeyCallbackMap::const_iterator> range =
            scene.keyCallbacks.equal_range(ea.getKey());
        for (Scene::KeyCallbackMap::const_iterator i = range.first; i != range.second; ++i)
            suppressDefault |= i->second(ea);
    }

    // general handlers
    pair<Scene::CallbackMap::const_iterator, Scene::CallbackMap::const_iterator> range =
        scene.callbacks.equal_range(t);
    for (Scene::CallbackMap::const_iterator i = range.first; i != range.second; ++i)
        suppressDefault |= i->second(ea);

    if (!suppressDefault)
        return osgGA::TrackballManipulator::handle(ea, aa);

    return false;
}
