#include "custom_scene/custom_scene.h"

#include <omp.h>

#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <future>
#include <ros/callback_queue.h>
#include <smmap_experiment_params/ros_params.hpp>

#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>

#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <bullet_helpers/bullet_internal_conversions.hpp>
#include <bullet_helpers/bullet_ros_conversions.hpp>
#include <bullet_helpers/bullet_math_helpers.hpp>
#include <bullet_helpers/bullet_pretty_print.hpp>

#include "utils/util.h"
#include "custom_scene/internal_utils.hpp"

using namespace BulletHelpers;
using namespace smmap;

////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor
////////////////////////////////////////////////////////////////////////////////

CustomScene::CustomScene(ros::NodeHandle& nh,
                         const DeformableType deformable_type,
                         const TaskType task_type)
    : screen_recorder_(nullptr)
    , plot_points_(boost::make_shared<PlotPoints>(0.1f * METERS))
    , plot_lines_(boost::make_shared<PlotLines>(0.25f * METERS))
    , deformable_type_(deformable_type)
    , task_type_(task_type)
    , advance_grippers_(true)
    , free_space_grid_(GetWorldXMin(nh) * METERS, GetWorldXStep(nh) * METERS, GetWorldXNumSteps(nh),
                       GetWorldYMin(nh) * METERS, GetWorldYStep(nh) * METERS, GetWorldYNumSteps(nh),
                       GetWorldZMin(nh) * METERS, GetWorldZStep(nh) * METERS, GetWorldZNumSteps(nh))
    , free_space_graph_((size_t)free_space_grid_.getNumCells() + 1000)
    , num_graph_edges_(0)
    , nh_(nh)
    , ph_("~")
    , feedback_covariance_(GetFeedbackCovariance(nh_))
    , test_grippers_poses_as_(nh_, GetTestGrippersPosesTopic(nh_), boost::bind(&CustomScene::testGripperPosesExecuteCallback, this, _1), false)
    , num_timesteps_to_execute_per_gripper_cmd_(GetNumSimstepsPerGripperCommand(ph_))
{
    ROS_INFO("Building the world");
    // Build the world
    // TODO: make this setable/resetable via a ROS service call
    switch (deformable_type_)
    {
        case DeformableType::ROPE:
            makeRopeWorld();
            break;

        case DeformableType::CLOTH:
            makeClothWorld();
            break;

        default:
        {
            ROS_FATAL_STREAM("Unknown deformable type " << deformable_type_);
            throw_arc_exception(std::invalid_argument, "Unknown deformable object type " + std::to_string(deformable_type_));
        }
    };

    // Store the initial configuration as it will be needed by other libraries
    // TODO: find a better way to do this that exposes less internals
    object_initial_configuration_ = toRosPointVector(getDeformableObjectNodes(), METERS);

    ROS_INFO("Creating subscribers and publishers");
    // Publish to the feedback channel
    simulator_fbk_pub_ = nh_.advertise<smmap_msgs::SimulatorFeedback>(
            GetSimulatorFeedbackTopic(nh_), 20);

    ROS_INFO("Creating services");
    // Create a service to let others know the internal gripper names
    gripper_names_srv_ = nh_.advertiseService(
            GetGripperNamesTopic(nh_), &CustomScene::getGripperNamesCallback, this);

    // Create a service to let others know what nodes the grippers are attached too
    gripper_attached_node_indices_srv_ = nh_.advertiseService(
            GetGripperAttachedNodeIndicesTopic(nh_), &CustomScene::getGripperAttachedNodeIndicesCallback, this);

    // Create a service to let others know the current gripper pose
    gripper_pose_srv_ = nh_.advertiseService(
            GetGripperPoseTopic(nh_), &CustomScene::getGripperPoseCallback, this);

    // Create a service to let others know the current gripper pose
    gripper_collision_check_srv_ = nh_.advertiseService(
            GetGripperCollisionCheckTopic(nh_), &CustomScene::gripperCollisionCheckCallback, this);

    // Create a service to let others know the cover points
    cover_points_srv_ = nh_.advertiseService(
            GetCoverPointsTopic(nh_), &CustomScene::getCoverPointsCallback, this);

    // Create a service to let others know the mirror line data
    mirror_line_srv_ = nh_.advertiseService(
            GetMirrorLineTopic(nh_), &CustomScene::getMirrorLineCallback, this);

    // Create a service to let others know what the free space of the world looks like
    free_space_graph_srv_ = nh_.advertiseService(
            GetFreeSpaceGraphTopic(nh_), &CustomScene::getFreeSpaceGraphCallback, this);

    // Create a service to let others know the object initial configuration
    object_initial_configuration_srv_ = nh_.advertiseService(
            GetObjectInitialConfigurationTopic(nh_), &CustomScene::getObjectInitialConfigurationCallback, this);

    // Create a service to let others know the object current configuration
    object_current_configuration_srv_ = nh_.advertiseService(
                GetObjectCurrentConfigurationTopic(nh_), &CustomScene::getObjectCurrentConfigurationCallback, this);

    // Create a service to listen to in order to know when to shutdown
    terminate_sim_srv_ = nh_.advertiseService(
                GetTerminateSimulationTopic(nh_), &CustomScene::terminateSimulationCallback, this);



    // Create a service to listen to in order to move the grippers and advance sim time
    execute_gripper_movement_srv_ = nh_.advertiseService(
                GetExecuteGrippersMovementTopic(nh_), &CustomScene::executeGripperMovementCallback, this);



    // Create a subscriber to take visualization instructions
    visualization_marker_sub_ = nh_.subscribe(
            GetVisualizationMarkerTopic(nh_), 20, &CustomScene::visualizationMarkerCallback, this);

    // Create a subscriber to take visualization instructions
    visualization_marker_array_sub_ = nh_.subscribe(
            GetVisualizationMarkerArrayTopic(nh_), 20, &CustomScene::visualizationMarkerArrayCallback, this);
}

////////////////////////////////////////////////////////////////////////////////
// Main function that makes things happen
////////////////////////////////////////////////////////////////////////////////

void CustomScene::run(const bool drawScene, const bool syncTime)
{
    // Run the startup code for the viewer and everything else
    if (ros::ok())
    {
        // Note that viewer cleans up this memory
        viewer.addEventHandler(new CustomKeyHandler(*this));

        // When the viewer closes, shutdown ROS
        addVoidCallback(osgGA::GUIEventAdapter::EventType::CLOSE_WINDOW,
                        boost::bind(&CustomScene::terminateSimulationCallback, this, std_srvs::Empty::Request(), std_srvs::Empty::Response()));

        addPreStepCallback(boost::bind(&CustomScene::drawAxes, this));

        // if syncTime is set, the simulator blocks until the real time elapsed
        // matches the simulator time elapsed, or something, it's not clear
        setSyncTime(syncTime);
        setDrawing(drawScene);
        if (drawScene)
        {
            startViewer();
        }
        screen_recorder_ = std::make_shared<ScreenRecorder>(&viewer, smmap::GetScreenshotsEnabled(ph_), smmap::GetScreenshotFolder(nh_));

        // Create a thread to create the free space graph while the object settles
        auto free_space_graph_future = std::async(std::launch::async, &CustomScene::createFreeSpaceGraph, this, false);
        // Let the object settle before anything else happens
        ROS_INFO("Waiting for the scene to settle");
        stepFor(BulletConfig::dt, 4.0);
        // Wait for the graph to be finished being made
        while (free_space_graph_future.wait_for(std::chrono::microseconds(10000)) != std::future_status::ready)
        {
            step(0);
        }
        free_space_graph_future.get();

        base_sim_time_ = simTime;

        // Startup the action server
        test_grippers_poses_as_.start();
    }

    // TODO: remove this hardcoded spin rate
    std::thread spin_thread(&ROSHelpers::Spin, 100.0 / BulletConfig::dt);
    ROS_INFO("Simulation ready.");

    // Run the simulation - this loop only redraws the scene, actual work is done in service callbacks
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            step(0);
            simulator_fbk_pub_.publish(createSimulatorFbk());

//            osg::Vec3d eye, center, up;
//            manip->getTransformation(eye, center, up);

//            std::cout << eye.x()/METERS    << " " << eye.y()/METERS    << " " << eye.z()/METERS << "    "
//                      << center.x()/METERS << " " << center.y()/METERS << " " << center.z()/METERS << "    "
//                      << up.x()/METERS     << " " << up.y()/METERS     << " " << up.z()/METERS << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }

    // clean up the extra thread we started
    spin_thread.join();
}

////////////////////////////////////////////////////////////////////////////////
// Construction helper functions
////////////////////////////////////////////////////////////////////////////////

void CustomScene::makeTable()
{
    // table parameters
    const btVector3 table_surface_position =
            btVector3(GetTableSurfaceX(nh_),
                      GetTableSurfaceY(nh_),
                      GetTableSurfaceZ(nh_)) * METERS;
    const btVector3 table_half_extents =
            btVector3(GetTableHalfExtentsX(nh_),
                      GetTableHalfExtentsY(nh_),
                      GetTableThickness(nh_) / 2.0f ) * METERS;

    const btVector3 table_com = table_surface_position - btVector3(0, 0, table_half_extents.z());

    // create the table
    BoxObject::Ptr table = boost::make_shared<BoxObject> (
                0, table_half_extents,
                btTransform(btQuaternion(0, 0, 0, 1), table_com));
    table->setColor(0.4f, 0.4f, 0.4f, 1.0f);
    table->rigidBody->setFriction(1.0f);

    env->add(table);
    world_objects_["table"] = table;

    // if we are doing a table coverage task, create the table coverage points
    if (task_type_ == TaskType::TABLE_COVERAGE)
    {
        #pragma message "Cloth table cover points step size magic number"
        const float stepsize = 0.0125f * METERS;
        btTransform table_tf = table->rigidBody->getCenterOfMassTransform();

        const float cloth_collision_margin = cloth_->softBody->getCollisionShape()->getMargin();
        std::vector<btVector3> cloth_coverage_lines;
        for (float y = -table->halfExtents.y(); y <= table->halfExtents.y(); y += stepsize)
        {
            // Add a coverage line to the visualization
            cloth_coverage_lines.push_back(
                    table_tf * btVector3 (-table->halfExtents.x(), y, table->halfExtents.z() + cloth_collision_margin));

            cloth_coverage_lines.push_back(
                    table_tf * btVector3 (+table->halfExtents.x(), y, table->halfExtents.z() + cloth_collision_margin));

            // Add many coverage points along the coverage line
            for (float x = -table->halfExtents.x(); x <= table->halfExtents.x(); x += stepsize)
            {
                cover_points_.push_back(table_tf * btVector3(x, y, table->halfExtents.z() + cloth_collision_margin));
            }
        }
        ROS_INFO_STREAM("Number of cover points: " << cover_points_.size());

        std::vector<btVector4> cloth_coverage_color(cloth_coverage_lines.size(), btVector4(1, 0, 0, 1));
        plot_lines_->setPoints(cloth_coverage_lines, cloth_coverage_color);
        env->add(plot_lines_);
    }
}

void CustomScene::makeCylinder()
{
    // cylinder parameters
    const btVector3 cylinder_com_origin =
        btVector3(GetCylinderCenterOfMassX(nh_),
                  GetCylinderCenterOfMassY(nh_),
                  GetCylinderCenterOfMassZ(nh_)) * METERS;

    const btScalar cylinder_radius = GetCylinderRadius(nh_) * METERS;
    const btScalar cylinder_height = GetCylinderHeight(nh_) * METERS;

    // create a cylinder
    CylinderStaticObject::Ptr cylinder = boost::make_shared<CylinderStaticObject>(
                0, cylinder_radius, cylinder_height,
                btTransform(btQuaternion(0, 0, 0, 1), cylinder_com_origin));
    cylinder->setColor(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 0.5f);

    // add the cylinder to the world
    env->add(cylinder);
    world_objects_["cylinder"] = cylinder;

    if (deformable_type_ == DeformableType::ROPE && task_type_ == TaskType::CYLINDER_COVERAGE)
    {
        #pragma message "Magic numbers - discretization level of cover points"
        // consider 21 points around the cylinder
        for (float theta = 0; theta < 2.0f * M_PI; theta += 0.3f)
        // NOTE: this 0.3 ought to be 2*M_PI/21=0.299199... however that chops off the last value, probably due to rounding
        {
            // 31 points per theta
            for (float h = -cylinder_height / 2.0f; h < cylinder_height / 2.0f; h += cylinder_height / 30.0f)
            {
                cover_points_.push_back(
                        cylinder_com_origin
                        + btVector3((cylinder_radius + rope_->radius / 2.0f) * std::cos(theta),
                                     (cylinder_radius + rope_->radius / 2.0f) * std::sin(theta),
                                     h));
            }
        }
    }
    else if (deformable_type_ == DeformableType::CLOTH && (task_type_ == TaskType::CYLINDER_COVERAGE || task_type_ == TaskType::WAFR))
    {
        const float cloth_collision_margin = cloth_->softBody->getCollisionShape()->getMargin();

        #pragma message "Magic numbers - discretization level of cover points"
        for (float x = -cylinder_radius; x <= cylinder_radius; x += cylinder_radius / 8.0f)
        {
            for (float y = -cylinder_radius; y <= cylinder_radius; y += cylinder_radius / 8.0f)
            {
                // Only accept those points that are within the bounding circle
                if (x * x + y * y < cylinder_radius * cylinder_radius)
                {
                    cover_points_.push_back(
                                cylinder_com_origin
                                + btVector3(x, y, cylinder_height / 2.0f + cloth_collision_margin));
                }
            }
        }


        ////////////////////////////////////////////////////////////////////////
        // Horizontal Cylinder above first cylinder
        ////////////////////////////////////////////////////////////////////////

        const btVector3 horizontal_cylinder_com_origin = cylinder_com_origin +
                btVector3(-0.15f, 0.0f, 0.20f) * METERS;

        CylinderStaticObject::Ptr horizontal_cylinder = boost::make_shared<CylinderStaticObject>(
                    0, cylinder_radius / 4.0f, cylinder_height * 1.9f,
                    btTransform(btQuaternion(btVector3(1, 0, 0), (float)M_PI/2.0f), horizontal_cylinder_com_origin));
        horizontal_cylinder->setColor(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 0.5f);

        // add the cylinder to the world
        env->add(horizontal_cylinder);
        world_objects_["horizontal_cylinder"] = horizontal_cylinder;

        #pragma message "Magic numbers - discretization level of cover points"
        for (float theta = 1.0f * (float)M_PI - 0.524f; theta <= 2.0f * M_PI; theta += 0.523f)
        {
            const float cover_points_radius = horizontal_cylinder->getRadius() + cloth_collision_margin + (btScalar)GetRobotMinGripperDistance() * METERS;

            for (float h = -horizontal_cylinder->getHeight()/2.0f; h <= horizontal_cylinder->getHeight()/1.99f; h += horizontal_cylinder->getHeight() / 30.0f)
            {
                cover_points_.push_back(
                            horizontal_cylinder_com_origin
                            + btVector3(
                                cover_points_radius * std::sin(theta),
                                h,
                                cover_points_radius * std::cos(theta)));
            }
        }
    }

    std::vector<btVector4> coverage_color(cover_points_.size(), btVector4(1, 0, 0, 1));
    plot_points_->setPoints(cover_points_, coverage_color);
    env->add(plot_points_);
}

void CustomScene::makeRope()
{
    // find the needed table parameters
    const btVector3 rope_com =
            btVector3(GetRopeCenterOfMassX(nh_),
                       GetRopeCenterOfMassY(nh_),
                       GetRopeCenterOfMassZ(nh_)) * METERS;

    const float rope_segment_length = GetRopeSegmentLength(nh_) * METERS;
    const size_t num_links = (size_t)GetRopeNumLinks(nh_);

    // make the rope
    std::vector<btVector3> control_points(num_links);
    for (size_t n = 0; n < num_links; n++)
    {
        control_points[n] = rope_com
                + btVector3(((float)n - (float)(num_links) / 2.0f) * rope_segment_length, 0, 0);

    }
    rope_ = boost::make_shared<CapsuleRope>(control_points, GetRopeRadius(nh_) * METERS);

    // color the rope
    std::vector<BulletObject::Ptr> children = rope_->getChildren();
    for (size_t j = 0; j < children.size(); j++)
    {
        children[j]->setColor(0.15f, 0.65f, 0.15f, 1.0f);
    }

    // add the table and rope to the world
    env->add(rope_);
}

void CustomScene::makeCloth()
{
    // cloth parameters
    const btVector3 cloth_center = btVector3(
                GetClothCenterOfMassX(nh_),
                GetClothCenterOfMassY(nh_),
                GetClothCenterOfMassZ(nh_)) * METERS;

    const btScalar cloth_x_half_side_length = GetClothXSize(nh_) * METERS / 2.0f;
    const btScalar cloth_y_half_side_length = GetClothYSize(nh_) * METERS / 2.0f;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        cloth_center + btVector3(-cloth_x_half_side_length, -cloth_y_half_side_length, 0),
        cloth_center + btVector3(+cloth_x_half_side_length, -cloth_y_half_side_length, 0),
        cloth_center + btVector3(-cloth_x_half_side_length, +cloth_y_half_side_length, 0),
        cloth_center + btVector3(+cloth_x_half_side_length, +cloth_y_half_side_length, 0),
        GetClothNumDivsX(nh_), GetClothNumDivsY(nh_),
        0, true);
    psb->setTotalMass(0.1f, true);

//    psb->m_cfg.viterations = 10;     // Velocity solver iterations   - default 0 - changing this to 10 causes the cloth to just pass through objects it seems
    psb->m_cfg.piterations = 10;     // Positions solver iterations  - default 1 - DmitrySim 10
//    psb->m_cfg.diterations = 10;     // Drift solver iterations      - default 0 - changing this to 10 or 100 doesn't seem to cause any meaningful change
    psb->m_cfg.citerations = 10;     // Cluster solver iterations    - default 4

    psb->m_cfg.collisions   =
            btSoftBody::fCollision::CL_SS |         // Cluster collisions, soft vs. soft
            btSoftBody::fCollision::CL_RS |         // Cluster collisions, soft vs. rigid
            btSoftBody::fCollision::CL_SELF;        // Cluster collisions, self - requires CL_SS

    psb->getCollisionShape()->setMargin(0.0025f * METERS); // default 0.25 - DmitrySim 0.05


    psb->m_cfg.kDP          = 0.05f;    // Damping coeffient [0, +inf]          - default 0
    psb->m_cfg.kDF          = 1.0f;     // Dynamic friction coefficient [0,1]   - default 0.2
//    psb->m_cfg.kMT          = 0.0f;     // Pose matching coefficient [0,1]      - default 0
//    psb->m_cfg.kKHR         = 1.0f;     // Kinetic contacts hardness [0,1]      - default 0.1


    btSoftBody::Material *pm = psb->m_materials[0];
    pm->m_kLST = GetClothLinearStiffness(nh_);     // Linear stiffness coefficient [0,1]       - default is 1 - 0.2 makes it rubbery (handles self collisions better)
//    pm->m_kAST = 0.90f;     // Area/Angular stiffness coefficient [0,1] - default is 1
//    pm->m_kVST = 1;         // Volume stiffness coefficient [0,1]       - default is 1
    const int distance = 2; // node radius for creating constraints
    psb->generateBendingConstraints(distance, pm);
    psb->randomizeConstraints();


    if (task_type_ == TaskType::TABLE_COVERAGE)
    {
        psb->generateClusters(0);
    }
    else
    {
        // splits the soft body volume up into the given number of small, convex clusters,
        // which consecutively will be used for collision detection with other soft bodies or rigid bodies.
        // Sending '0' causes the function to use the number of tetrahedral/face elemtents as the number of clusters
//        psb->generateClusters(num_divs * num_divs / 25);
        psb->generateClusters(500);
        for (int i = 0; i < psb->m_clusters.size(); ++i)
        {
            psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.001f; // default 0.01
//            psb->m_clusters[i]->m_maxSelfCollisionImpulse = 100.0f; // maximum self impulse that is *ignored (I think)* - default 100
        }
    }

    cloth_ = boost::make_shared<BulletSoftObject>(psb);
    // note that we need to add the cloth to the environment before setting the
    // color, otherwise we get a segfault
    env->add(cloth_);
    cloth_->setColor(0.15f, 0.65f, 0.15f, 1.0f);

    findClothCornerNodes();
}

void CustomScene::makeRopeWorld()
{
    makeRope();

    // Here we assume that we are already working with a rope object
    switch (task_type_)
    {
        case TaskType::CYLINDER_COVERAGE:
        {
            makeTable();
            makeCylinder();

            // add a single auto gripper to the world
            grippers_["gripper"] = boost::make_shared< GripperKinematicObject>(
                        "gripper",
                        GetRopeGripperApperture(nh_) * METERS,
                        btVector4(0.0f, 0.0f, 0.6f, 1.0f));
            grippers_["gripper"]->setWorldTransform(
                    rope_->getChildren()[0]->rigidBody->getCenterOfMassTransform());
            grippers_["gripper"]->rigidGrab(rope_->getChildren()[0]->rigidBody.get(), 0, env);

            auto_grippers_.push_back("gripper");

            break;
        }
        default:
        {
            ROS_FATAL_STREAM("Unknown task type for a ROPE object " << task_type_);
            throw_arc_exception(std::invalid_argument, "Unknown task type for a ROPE object " + std::to_string(task_type_));
        }
    }

    for (auto& gripper: grippers_)
    {
        gripper_axes_[gripper.first] = boost::make_shared<PlotAxes>();

        // Add the gripper and it's axis to the world
        env->add(gripper.second);
        env->add(gripper_axes_[gripper.first]);
    }

    // Add a gripper that is in the same state as used for the rope experiments
    collision_check_gripper_ = boost::make_shared< GripperKinematicObject>(
                "collision_check_gripper",
                GetRopeGripperApperture(nh_) * METERS,
                btVector4(0, 0, 0, 0));
    collision_check_gripper_->setWorldTransform(btTransform());
    // Don't add this gripper to the world, we can do collision checks for it vs other objects without adding it
//    env->add(collision_check_gripper_);
}

void CustomScene::makeClothWorld()
{
    makeCloth();

    // add 2 auto grippers to the world
    {
        btVector3 gripper_half_extents;

        // auto gripper0
        grippers_["auto_gripper0"] = boost::make_shared< GripperKinematicObject>(
                    "auto_gripper0",
                    GetClothGripperApperture(nh_) * METERS,
                    btVector4(0.0f, 0.0f, 0.6f, 1.0f));
        gripper_half_extents = grippers_["auto_gripper0"]->getHalfExtents();
        grippers_["auto_gripper0"]->setWorldTransform(
                btTransform(btQuaternion(0, 0, 0, 1),
                             cloth_->softBody->m_nodes[cloth_corner_node_indices_[0]].m_x
                             + btVector3(gripper_half_extents.x(), gripper_half_extents.y(), 0)));

        auto_grippers_.push_back("auto_gripper0");

        // auto gripper1
        grippers_["auto_gripper1"] = boost::make_shared< GripperKinematicObject>(
                    "auto_gripper1",
                    GetClothGripperApperture(nh_) * METERS,
                    btVector4(0.0f, 0.0f, 0.6f, 1.0f));
        gripper_half_extents = grippers_["auto_gripper1"]->getHalfExtents();
        grippers_["auto_gripper1"]->setWorldTransform(
                btTransform(btQuaternion(0, 0, 0, 1),
                             cloth_->softBody->m_nodes[cloth_corner_node_indices_[1]].m_x
                             + btVector3(gripper_half_extents.x(), -gripper_half_extents.y(), 0)));

        auto_grippers_.push_back("auto_gripper1");
    }

    switch (task_type_)
    {
        case TaskType::TABLE_COVERAGE:
        {
            makeTable();
            break;
        }
        case TaskType::CYLINDER_COVERAGE:
        {
            makeCylinder();
            break;
        }
        case TaskType::COLAB_FOLDING:
        {
            // add 2 manual grippers to the world
            {
                btVector3 gripper_half_extents;

                // manual gripper0
                grippers_["manual_gripper0"] = boost::make_shared< GripperKinematicObject>(
                            "manual_gripper0",
                            GetClothGripperApperture(nh_) * METERS,
                            btVector4(0.6f, 0.6f, 0.6f, 0.4f));
                gripper_half_extents = grippers_["manual_gripper0"]->getHalfExtents();
                grippers_["manual_gripper0"]->setWorldTransform(
                            btTransform(btQuaternion(0, 0, 0, 1),
                                         cloth_->softBody->m_nodes[cloth_corner_node_indices_[2]].m_x
                        + btVector3(-gripper_half_extents.x(), gripper_half_extents.y(), 0)));

                manual_grippers_.push_back("manual_gripper0");
                manual_grippers_paths_.push_back(ManualGripperPath(grippers_["manual_gripper0"], &gripperPath0));

                // manual gripper1
                grippers_["manual_gripper1"] = boost::make_shared< GripperKinematicObject>(
                            "manual_gripper1",
                            GetClothGripperApperture(nh_) * METERS,
                            btVector4(0.6f, 0.6f, 0.6f, 0.4f));
                gripper_half_extents = grippers_["manual_gripper1"]->getHalfExtents();
                grippers_["manual_gripper1"]->setWorldTransform(
                            btTransform(btQuaternion(0, 0, 0, 1),
                                         cloth_->softBody->m_nodes[cloth_corner_node_indices_[3]].m_x
                        + btVector3(-gripper_half_extents.x(), -gripper_half_extents.y(), 0)));

                manual_grippers_.push_back("manual_gripper1");
                manual_grippers_paths_.push_back(ManualGripperPath(grippers_["manual_gripper1"], &gripperPath1));
            }
            break;
        }
        case TaskType::WAFR:
        {
            makeCylinder();
            break;
        }
        default:
        {
            ROS_FATAL_STREAM("Unknown task type for a CLOTH object " << task_type_);
            throw_arc_exception(std::invalid_argument, "Unknown task type for a CLOTH object " + std::to_string(task_type_));
        }
    }

    for (auto& gripper: grippers_)
    {
        // Grip the cloth
        gripper.second->toggleOpen();
        gripper.second->toggleAttach(cloth_->softBody.get());

        gripper_axes_[gripper.first] = boost::make_shared<PlotAxes>();

        // Add the gripper and its axis to the world
        env->add(gripper.second);
        env->add(gripper_axes_[gripper.first]);

        // Add a callback in case this gripper gets step_openclose activated on it
        // This is a state machine whose input comes from CustomKeyHandler
        addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, gripper.second, cloth_->softBody.get()));
    }

    // Add a gripper that is in the same state as used for the cloth experiments
    collision_check_gripper_ = boost::make_shared< GripperKinematicObject>(
                "collision_check_gripper",
                GetClothGripperApperture(nh_) * METERS,
                btVector4(0, 0, 0, 0));
    collision_check_gripper_->setWorldTransform(btTransform());
    collision_check_gripper_->toggleOpen();
    // Don't add this gripper to the world, we can do collision checks for it vs other objects without adding it
//    env->add(collision_check_gripper_);
}

/**
 * Sets cloth_corner_node_indices_ to the extremal points of the mesh
 *
 * cloth_corner_node_indices_[0] = minx_miny
 * cloth_corner_node_indices_[1] = minx_maxy
 * cloth_corner_node_indices_[2] = maxx_miny
 * cloth_corner_node_indices_[3] = maxx_maxy
 */
void CustomScene::findClothCornerNodes()
{
    cloth_corner_node_indices_.resize(4, 0);
    // Setup defaults for doing the max and min operations inside of the loop
    std::vector<btVector3> corner_node_positions(4);

    // min_x, min_y
    corner_node_positions[0] = btVector3(
            std::numeric_limits<btScalar>::infinity(),
            std::numeric_limits<btScalar>::infinity(),
            0);

    // min_x, max_y
    corner_node_positions[1] = btVector3(
            std::numeric_limits<btScalar>::infinity(),
            -std::numeric_limits<btScalar>::infinity(),
            0);

    // max_x, min_y
    corner_node_positions[2] = btVector3(
            -std::numeric_limits<btScalar>::infinity(),
            std::numeric_limits<btScalar>::infinity(),
            0);

    // max_x, max_y
    corner_node_positions[3] = btVector3(
            -std::numeric_limits<btScalar>::infinity(),
            -std::numeric_limits<btScalar>::infinity(),
            0);

    btSoftBody::tNodeArray cloth_nodes = cloth_->softBody->m_nodes;

    // Itterate through the nodes in the cloth, finding the extremal points
    for (int ind = 0; ind < cloth_nodes.size(); ind++)
    {
        if (cloth_nodes[ind].m_x.x() <= corner_node_positions[0].x() &&
                cloth_nodes[ind].m_x.y() <= corner_node_positions[0].y())
        {
            cloth_corner_node_indices_[0] = ind;
            corner_node_positions[0] = cloth_nodes[ind].m_x;
        }
        else if (cloth_nodes[ind].m_x.x() <= corner_node_positions[1].x() &&
                cloth_nodes[ind].m_x.y() >= corner_node_positions[1].y())
        {
            cloth_corner_node_indices_[1] = ind;
            corner_node_positions[1] = cloth_nodes[ind].m_x;
        }
        else if (cloth_nodes[ind].m_x.x() >= corner_node_positions[2].x() &&
                cloth_nodes[ind].m_x.y() <= corner_node_positions[2].y())
        {
            cloth_corner_node_indices_[2] = ind;
            corner_node_positions[2] = cloth_nodes[ind].m_x;
        }
        else if (cloth_nodes[ind].m_x.x() >= corner_node_positions[3].x() &&
                cloth_nodes[ind].m_x.y() >= corner_node_positions[3].y())
        {
            cloth_corner_node_indices_[3] = ind;
            corner_node_positions[3] = cloth_nodes[ind].m_x;
        }
    }

    // Create a mirror line if we are doing colaborative folding
    if (task_type_ == TaskType::COLAB_FOLDING)
    {
        mirror_line_data_.min_y = corner_node_positions[0].y() / METERS;
        mirror_line_data_.max_y = corner_node_positions[3].y() / METERS;
        mirror_line_data_.mid_x = (corner_node_positions[0].x() +
                (corner_node_positions[3].x() - corner_node_positions[0].x()) / 2)  / METERS;

        std::vector<btVector3> mirror_line_points;
        mirror_line_points.push_back(btVector3((float)mirror_line_data_.mid_x, (float)mirror_line_data_.min_y, 0.8f) * METERS);
        mirror_line_points.push_back(btVector3((float)mirror_line_data_.mid_x, (float)mirror_line_data_.max_y, 0.8f) * METERS);
        std::vector<btVector4> mirror_line_colors;
        mirror_line_colors.push_back(btVector4(1,0,0,1));

        PlotLines::Ptr line_strip = boost::make_shared<PlotLines>(0.1f * METERS);
        line_strip->setPoints(mirror_line_points,
                               mirror_line_colors);
        visualization_line_markers_["mirror_line"] = line_strip;

        env->add(line_strip);
    }
}

void CustomScene::createEdgesToNeighbours(const int64_t x_starting_ind, const int64_t y_starting_ind, const int64_t z_starting_ind)
{
    const int64_t starting_ind = free_space_grid_.xyzIndexToGridIndex(x_starting_ind, y_starting_ind, z_starting_ind);
    assert(starting_ind >= 0);
    const double x = free_space_grid_.xIndToWorldX(x_starting_ind);
    const double y = free_space_grid_.yIndToWorldY(y_starting_ind);
    const double z = free_space_grid_.zIndToWorldZ(z_starting_ind);
    const btVector3 starting_pos((btScalar)x, (btScalar)y, (btScalar)z);

    // Note that these are in [min, max) form - i.e. exclude the max
    const int64_t x_min_ind = std::max(0L, x_starting_ind - 1);
    const int64_t x_max_ind = std::min(free_space_grid_.getXNumSteps(), x_starting_ind + 2);
    const int64_t y_min_ind = std::max(0L, y_starting_ind - 1);
    const int64_t y_max_ind = std::min(free_space_grid_.getYNumSteps(), y_starting_ind + 2);
    const int64_t z_min_ind = std::max(0L, z_starting_ind - 1);
    const int64_t z_max_ind = std::min(free_space_grid_.getZNumSteps(), z_starting_ind + 2);

    SphereObject::Ptr test_sphere = boost::make_shared<SphereObject>(0, 0.001*METERS, btTransform(), true);

    for (int64_t x_loop_ind = x_min_ind; x_loop_ind < x_max_ind; x_loop_ind++)
    {
        const double x_loop = free_space_grid_.xIndToWorldX(x_loop_ind);
        for (int64_t y_loop_ind = y_min_ind; y_loop_ind < y_max_ind; y_loop_ind++)
        {
            const double y_loop = free_space_grid_.yIndToWorldY(y_loop_ind);
            for (int64_t z_loop_ind = z_min_ind; z_loop_ind < z_max_ind; z_loop_ind++)
            {
                // Only count a neighbour if it's not the same as the starting position
                if (x_starting_ind != x_loop_ind || y_starting_ind != y_loop_ind || z_starting_ind != z_loop_ind)
                {
                    const double z_loop = free_space_grid_.zIndToWorldZ(z_loop_ind);

                    test_sphere->motionState->setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), btVector3((btScalar)x_loop, (btScalar)y_loop, (btScalar)z_loop)));

                    // If we are not in collision already, then check for neighbours
                    if (collisionHelper(test_sphere).m_distance >= 0.0)
                    {
                        const btVector3 loop_pos((btScalar)x_loop, (btScalar)y_loop, (btScalar)z_loop);
                        const btScalar dist = (loop_pos - starting_pos).length();

                        const int64_t loop_ind = free_space_grid_.xyzIndexToGridIndex(x_loop_ind, y_loop_ind, z_loop_ind);
                        assert(loop_ind >= 0);
                        free_space_graph_.AddEdgeBetweenNodes(starting_ind, loop_ind, dist);
                        num_graph_edges_ += 2;
                    }
                }
            }
        }
    }
}

void CustomScene::createFreeSpaceGraph(const bool draw_graph_corners)
{
    ROS_INFO("Creating free space graph");
    if (draw_graph_corners)
    {
        // Debugging - draw the corners of the grid
        std::cout << "Drawing the 8 corners of the graph world\n";
        graph_corners_.reserve((8));
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wfloat-conversion"
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMin(), free_space_grid_.getYMin(), free_space_grid_.getZMin())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMin(), free_space_grid_.getYMin(), free_space_grid_.getZMax())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMin(), free_space_grid_.getYMax(), free_space_grid_.getZMin())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMin(), free_space_grid_.getYMax(), free_space_grid_.getZMax())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMax(), free_space_grid_.getYMin(), free_space_grid_.getZMin())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMax(), free_space_grid_.getYMin(), free_space_grid_.getZMax())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMax(), free_space_grid_.getYMax(), free_space_grid_.getZMin())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(free_space_grid_.getXMax(), free_space_grid_.getYMax(), free_space_grid_.getZMax())), 1.0f));
        #pragma GCC diagnostic pop
        for (auto& corner: graph_corners_)
        {
            env->add(corner);
        }
    }

    // First add all of the cells to the graph with no edges
    for (int64_t x_ind = 0; x_ind < free_space_grid_.getXNumSteps(); x_ind++)
    {
        const double x = free_space_grid_.xIndToWorldX(x_ind);
        for (int64_t y_ind = 0; y_ind < free_space_grid_.getYNumSteps(); y_ind++)
        {
            const double y = free_space_grid_.yIndToWorldY(y_ind);
            for (int64_t z_ind = 0; z_ind < free_space_grid_.getZNumSteps(); z_ind++)
            {
                const double z = free_space_grid_.zIndToWorldZ(z_ind);
                const int64_t node_ind = free_space_graph_.AddNode(btVector3((btScalar)x, (btScalar)y, (btScalar)z));
                // Double checking my math here
                assert(node_ind == free_space_grid_.worldPosToGridIndex(x, y, z));
            }
        }
    }

    // Next add edges between adjacent nodes that are not in collision
    for (int64_t x_ind = 0; x_ind < free_space_grid_.getXNumSteps(); x_ind++)
    {
        const double x = free_space_grid_.xIndToWorldX(x_ind);
        for (int64_t y_ind = 0; y_ind < free_space_grid_.getYNumSteps(); y_ind++)
        {
            const double y = free_space_grid_.yIndToWorldY(y_ind);
            for (int64_t z_ind = 0; z_ind < free_space_grid_.getZNumSteps(); z_ind++)
            {
                SphereObject::Ptr test_sphere = boost::make_shared<SphereObject>(0, 0.001*METERS, btTransform(), true);

                const double z = free_space_grid_.zIndToWorldZ(z_ind);
                test_sphere->motionState->setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), btVector3((btScalar)x, (btScalar)y, (btScalar)z)));

                createEdgesToNeighbours(x_ind, y_ind, z_ind);
            }
        }
    }

    // Last connect the cover points to the graph
    cover_ind_to_free_space_graph_ind_.resize(cover_points_.size());
    for (size_t cover_ind = 0; cover_ind < cover_points_.size(); cover_ind++)
    {
        const btVector3& cover_point = cover_points_[cover_ind];

        // Find the nearest node in the graph due to the grid
        btVector3 nearest_node_point = cover_point;
        int64_t graph_ind = free_space_grid_.worldPosToGridIndex(nearest_node_point.x(), nearest_node_point.y(), nearest_node_point.z());
        assert(graph_ind >= 0);

        const double step_size = free_space_grid_.minStepDimension() / 4.0;
        SphereObject::Ptr test_sphere = boost::make_shared<SphereObject>(0, 0.001*METERS, btTransform(), true);

        // If the node is in collision, find the nearest grid node that is not in collision
        while (free_space_graph_.GetNodeImmutable(graph_ind).GetInEdgesImmutable().size() == 0)
        {
            // Find the direction to move
            test_sphere->motionState->setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), nearest_node_point));
            btPointCollector collision_result = collisionHelper(test_sphere);

            // Move a step out of collision
            nearest_node_point = nearest_node_point + collision_result.m_normalOnBInWorld * (btScalar)step_size;

            graph_ind = free_space_grid_.worldPosToGridIndex(nearest_node_point.x(), nearest_node_point.y(), nearest_node_point.z());
            assert(graph_ind >= 0);
            assert(graph_ind < free_space_grid_.getNumCells());
        }

        // Add an edge between the cover point and the nearest point on the grid
        const btVector3& graph_point = free_space_graph_.GetNodeImmutable(graph_ind).GetValueImmutable();
        const double dist = (graph_point - cover_point).length();
        const int64_t cover_node_ind = free_space_graph_.AddNode(cover_point);
        free_space_graph_.AddEdgesBetweenNodes(cover_node_ind, graph_ind, dist);
        cover_ind_to_free_space_graph_ind_[cover_ind] = graph_ind;
    }

    assert(free_space_graph_.CheckGraphLinkage());
}

////////////////////////////////////////////////////////////////////////////////
// Internal helper functions
////////////////////////////////////////////////////////////////////////////////

void CustomScene::SetGripperTransform(const std::map<std::string, GripperKinematicObject::Ptr>& grippers_map, const std::string& name, const geometry_msgs::Pose& pose)
{
    GripperKinematicObject::Ptr gripper = grippers_map.at(name);

    const btTransform tf = toBulletTransform(pose, METERS);

    assert(-100.0f < tf.getOrigin().x() && tf.getOrigin().x() < 100.0f);
    assert(-100.0f < tf.getOrigin().y() && tf.getOrigin().y() < 100.0f);
    assert(-100.0f < tf.getOrigin().z() && tf.getOrigin().z() < 100.0f);

    gripper->setWorldTransform(tf);
}


smmap_msgs::SimulatorFeedback CustomScene::createSimulatorFbk() const
{
    assert(num_timesteps_to_execute_per_gripper_cmd_ > 0);

    smmap_msgs::SimulatorFeedback msg;

    // fill out the object configuration data
    msg.object_configuration = toRosPointVector(getDeformableObjectNodes(), METERS);
    if (feedback_covariance_ > 0)
    {
        for (auto& point: msg.object_configuration)
        {
            point.x += BoxMuller(0, feedback_covariance_);
            point.y += BoxMuller(0, feedback_covariance_);
            point.z += BoxMuller(0, feedback_covariance_);
        }
    }

    // fill out the gripper data
    for (const std::string &gripper_name: auto_grippers_)
    {
        const GripperKinematicObject::Ptr gripper = grippers_.at(gripper_name);
        msg.gripper_names.push_back(gripper_name);
        msg.gripper_poses.push_back(toRosPose(gripper->getWorldTransform(), METERS));

        btPointCollector collision_result = collisionHelper(gripper);

        if (collision_result.m_hasResult)
        {
            msg.gripper_distance_to_obstacle.push_back(collision_result.m_distance / METERS);
            msg.obstacle_surface_normal.push_back(toRosVector3(collision_result.m_normalOnBInWorld, 1));
            msg.gripper_nearest_point_to_obstacle.push_back(toRosPoint(
                        collision_result.m_pointInWorld
                        + collision_result.m_normalOnBInWorld * collision_result.m_distance, METERS));
        }
        else
        {
            msg.gripper_distance_to_obstacle.push_back(std::numeric_limits<double>::infinity());
            msg.obstacle_surface_normal.push_back(toRosVector3(btVector3(1.0f, 0.0f, 0.0f), 1));
            msg.gripper_nearest_point_to_obstacle.push_back(toRosPoint(btVector3(0.0f, 0.0f, 0.0f), 1));
        }
    }

    // update the sim_time
    msg.sim_time = (simTime - base_sim_time_) / (double)num_timesteps_to_execute_per_gripper_cmd_;

    return msg;
}

smmap_msgs::SimulatorFeedback CustomScene::createSimulatorFbk(const SimForkResult& result) const
{
    smmap_msgs::SimulatorFeedback msg;

    msg.object_configuration = toRosPointVector(getDeformableObjectNodes(result), METERS);

    // fill out the gripper data
    for (const std::string &gripper_name: auto_grippers_)
    {
        const GripperKinematicObject::Ptr gripper = grippers_.at(gripper_name);
        msg.gripper_names.push_back(gripper_name);
        msg.gripper_poses.push_back(toRosPose(gripper->getWorldTransform(), METERS));

        btPointCollector collision_result = collisionHelper(gripper);

        if (collision_result.m_hasResult)
        {
            msg.gripper_distance_to_obstacle.push_back(collision_result.m_distance / METERS);
            msg.obstacle_surface_normal.push_back(toRosVector3(collision_result.m_normalOnBInWorld, 1));
            msg.gripper_nearest_point_to_obstacle.push_back(toRosPoint(
                        collision_result.m_pointInWorld
                        + collision_result.m_normalOnBInWorld * collision_result.m_distance, METERS));
        }
        else
        {
            msg.gripper_distance_to_obstacle.push_back(std::numeric_limits<double>::infinity());
            msg.obstacle_surface_normal.push_back(toRosVector3(btVector3(1.0f, 0.0f, 0.0f), 1));
            msg.gripper_nearest_point_to_obstacle.push_back(toRosPoint(btVector3(0.0f, 0.0f, 0.0f), 1));
        }

        // update the sim_time
        // TODO: I think this math is wrong - simTime won't be updated yet
        msg.sim_time = (simTime - base_sim_time_) / (double)num_timesteps_to_execute_per_gripper_cmd_;
    }

    return msg;
}


std::vector<btVector3> CustomScene::getDeformableObjectNodes() const
{
    std::vector<btVector3> nodes;

    switch (deformable_type_)
    {
        case DeformableType::ROPE:
            nodes = rope_->getNodes();
            break;

        case DeformableType::CLOTH:
            nodes = nodeArrayToNodePosVector(cloth_->softBody->m_nodes);
            break;
    }

    return nodes;
}

std::vector<btVector3> CustomScene::getDeformableObjectNodes(const SimForkResult& result) const
{
    std::vector<btVector3> nodes;

    switch (deformable_type_)
    {
        case DeformableType::ROPE:
            nodes = result.rope_->getNodes();
            break;

        case DeformableType::CLOTH:
            nodes = nodeArrayToNodePosVector(result.cloth_->softBody->m_nodes);
            break;
    }

    return nodes;
}

/**
 * @brief Invoke bullet's collision detector to find the points on the gripper
 * and cylinder/table that are nearest each other.
 *
 * @param gripper_name The name of the gripper to check for collision
 *
 * @return The result of the collision check
 */
btPointCollector CustomScene::collisionHelper(const GripperKinematicObject::Ptr& gripper) const
{
    assert(gripper);

    // Note that gjkOutput initializes to hasResult = false and m_distance = BT_LARGE_FLOAT
    btPointCollector gjkOutput_min;

    // find the distance to any objects in the world
    for (auto ittr = world_objects_.begin(); ittr != world_objects_.end(); ++ittr)
    {
        BulletObject::Ptr obj = ittr->second;
        for (size_t gripper_child_ind = 0; gripper_child_ind < gripper->getChildren().size(); gripper_child_ind++)
        {
            // TODO: how much (if any) of this should be static/class members?
            btGjkEpaPenetrationDepthSolver epaSolver;
            btVoronoiSimplexSolver sGjkSimplexSolver;
            btPointCollector gjkOutput;

            btGjkPairDetector convexConvex(dynamic_cast<btBoxShape*>(gripper->getChildren()[gripper_child_ind]->collisionShape.get()),
                    dynamic_cast<btConvexShape*>(obj->collisionShape.get()), &sGjkSimplexSolver, &epaSolver);

            btGjkPairDetector::ClosestPointInput input;

            gripper->getChildren()[gripper_child_ind]->motionState->getWorldTransform(input.m_transformA);
            obj->motionState->getWorldTransform(input.m_transformB);
            input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);
            convexConvex.getClosestPoints(input, gjkOutput, nullptr);

            if (gjkOutput.m_distance < gjkOutput_min.m_distance)
            {
                gjkOutput_min = gjkOutput;
            }
        }
    }

    gjkOutput_min.m_normalOnBInWorld.normalize();
    return gjkOutput_min;
}

/**
 * @brief CustomScene::collisionHelper
 * @param sphere
 * @return
 */
btPointCollector CustomScene::collisionHelper(const SphereObject::Ptr& sphere) const
{
    assert(sphere);
    // Note that gjkOutput initializes to hasResult = false and m_distance = BT_LARGE_FLOAT
    btPointCollector gjkOutput_min;

    // find the distance to any objects in the world
    for (auto ittr = world_objects_.begin(); ittr != world_objects_.end(); ++ittr)
    {
        BulletObject::Ptr obj = ittr->second;

        // TODO: how much (if any) of this should be static/class members?
        btGjkEpaPenetrationDepthSolver epaSolver;
        btVoronoiSimplexSolver sGjkSimplexSolver;
        btPointCollector gjkOutput;

        btGjkPairDetector convexConvex(dynamic_cast<btSphereShape*>(sphere->collisionShape.get()),
                dynamic_cast<btConvexShape*>(obj->collisionShape.get()), &sGjkSimplexSolver, &epaSolver);

        btGjkPairDetector::ClosestPointInput input;
        sphere->motionState->getWorldTransform(input.m_transformA);
        obj->motionState->getWorldTransform(input.m_transformB);
        input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);
        convexConvex.getClosestPoints(input, gjkOutput, nullptr);

        if (gjkOutput.m_distance < gjkOutput_min.m_distance)
        {
            gjkOutput_min = gjkOutput;
        }
    }

    gjkOutput_min.m_normalOnBInWorld.normalize();
    return gjkOutput_min;
}

////////////////////////////////////////////////////////////////////////////////
// Fork and Fork-visualization management
////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<ViewerData> CustomScene::createVisualizerForFork(SimForkResult& fork_result)
{
    std::shared_ptr<ViewerData> viewer(new ViewerData(*this, fork_result));

    viewer->viewer_.addEventHandler(new CustomKeyHandler(*this));

    viewer->dbgDraw_.reset(new osgbCollision::GLDebugDrawer());
    viewer->dbgDraw_->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    viewer->dbgDraw_->setEnabled(false);
    fork_result.bullet_->dynamicsWorld->setDebugDrawer(viewer->dbgDraw_.get());
    fork_result.osg_->root->addChild(viewer->dbgDraw_->getSceneGraph());

    viewer->viewer_.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    viewer->manip_ = new EventHandler(*this);
    viewer->manip_->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition), util::toOSGVector(ViewerConfig::pointCameraLooksAt), osg::Z_AXIS);
    viewer->viewer_.setCameraManipulator(viewer->manip_);
    viewer->viewer_.setSceneData(fork_result.osg_->root.get());
    viewer->viewer_.realize();

    return viewer;
}

SimForkResult CustomScene::createForkWithNoSimulationDone(
        const Environment::Ptr environment_to_fork,
        BulletSoftObject::Ptr cloth_to_fork,
        boost::shared_ptr<CapsuleRope> rope_to_fork,
        std::map<std::string, GripperKinematicObject::Ptr> grippers_to_fork)
{
    assert(task_type_ != TaskType::COLAB_FOLDING && "This does not yet work with colab folding - due to manual gripper path stuff");

    SimForkResult result;
    result.bullet_.reset(new BulletInstance());
    result.bullet_->setGravity(BulletConfig::gravity);
    result.osg_.reset(new OSGInstance());
    result.fork_.reset(new Fork(environment_to_fork, result.bullet_, result.osg_));
    result.cloth_ = boost::static_pointer_cast<BulletSoftObject>(result.fork_->forkOf(cloth_to_fork));
    result.rope_ = boost::static_pointer_cast<CapsuleRope>(result.fork_->forkOf(rope_to_fork));

    for (const auto& gripper: grippers_to_fork)
    {
        GripperKinematicObject::Ptr gripper_copy = boost::static_pointer_cast<GripperKinematicObject>(result.fork_->forkOf(gripper.second));
        assert(gripper_copy);
        result.grippers_[gripper.first] = gripper_copy;
    }

    // If we have a rope, regrasp with the gripper
    if (result.rope_)
    {
        assert(result.grippers_.size() == 1);
        result.grippers_[auto_grippers_[0]]->rigidGrab(result.rope_->getChildren()[0]->rigidBody.get(), 0, result.fork_->env);
    }

    return result;
}

SimForkResult CustomScene::createForkWithNoSimulationDone(
        const std::vector<std::string>& gripper_names,
        const std::vector<geometry_msgs::Pose>& gripper_poses)
{
    assert(gripper_names.size() == gripper_poses.size());

    SimForkResult result = createForkWithNoSimulationDone(env, cloth_, rope_, grippers_);

    for (size_t gripper_ind = 0; gripper_ind < gripper_names.size(); gripper_ind++)
    {
        SetGripperTransform(result.grippers_, gripper_names[gripper_ind], gripper_poses[gripper_ind]);
    }

    return result;
}

SimForkResult CustomScene::simulateInNewFork(
        const std::vector<std::string>& gripper_names,
        const std::vector<geometry_msgs::Pose>& gripper_poses)
{
    SimForkResult result = createForkWithNoSimulationDone(gripper_names, gripper_poses);

    for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
    {
        result.fork_->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks - Setting visualization markers
////////////////////////////////////////////////////////////////////////////////

// TODO: be able to delete markers and have a timeout
void CustomScene::visualizationMarkerCallback(
        visualization_msgs::Marker marker)
{
    std::string id = marker.ns + std::to_string(marker.id);

    // TODO: make this mutex not quite so "global" around this switch
    std::lock_guard<std::mutex> lock(sim_mutex_);

    switch (marker.type)
    {
        case visualization_msgs::Marker::POINTS:
        {
            if (visualization_point_markers_.count(id) == 0)
            {
                PlotPoints::Ptr points = boost::make_shared<PlotPoints>();
                points->setPoints(toOsgRefVec3Array(marker.points, METERS),
                                   toOsgRefVec4Array(marker.colors));
                visualization_point_markers_[id] = points;

                env->add(points);
            }
            else
            {
                PlotPoints::Ptr points = visualization_point_markers_[id];
                points->setPoints(toOsgRefVec3Array(marker.points, METERS),
                                   toOsgRefVec4Array(marker.colors));
            }
            break;
        }
        case visualization_msgs::Marker::SPHERE:
        {
            if (visualization_sphere_markers_.count(id) == 0)
            {
                PlotSpheres::Ptr spheres = boost::make_shared<PlotSpheres>();
                spheres->plot(toOsgRefVec3Array(marker.points, METERS),
                               toOsgRefVec4Array(marker.colors),
                               std::vector<float>(marker.points.size(), (float)marker.scale.x * METERS));
                visualization_sphere_markers_[id] = spheres;

                env->add(spheres);
            }
            else
            {
                PlotSpheres::Ptr spheres = visualization_sphere_markers_[id];
                spheres->plot(toOsgRefVec3Array(marker.points, METERS),
                               toOsgRefVec4Array(marker.colors),
                               std::vector<float>(marker.points.size(), (float)marker.scale.x * METERS));
            }
            break;
        }
        case visualization_msgs::Marker::LINE_STRIP:
        {
            convertLineStripToLineList(marker);
        }
        case visualization_msgs::Marker::LINE_LIST:
        {
            // if the object is new, add it
            if (visualization_line_markers_.count(id) == 0)
            {
                PlotLines::Ptr line_strip = boost::make_shared<PlotLines>((float)marker.scale.x * METERS);
                line_strip->setPoints(toBulletPointVector(marker.points, METERS),
                                       toBulletColorArray(marker.colors));
                visualization_line_markers_[id] = line_strip;

                env->add(line_strip);
            }
            else
            {
                PlotLines::Ptr line_strip = visualization_line_markers_[id];
                line_strip->setPoints(toBulletPointVector(marker.points, METERS),
                                       toBulletColorArray(marker.colors));
            }
            break;
        }
        default:
        {
            ROS_ERROR_STREAM_NAMED("visualization",
                    "Marker type " << marker.type << " not implemented " << id);
        }
    }
}

void CustomScene::visualizationMarkerArrayCallback(
        visualization_msgs::MarkerArray marker_array)
{
    for (visualization_msgs::Marker marker: marker_array.markers)
    {
        visualizationMarkerCallback(marker);
    }
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks - Getting information
////////////////////////////////////////////////////////////////////////////////

bool CustomScene::getGripperNamesCallback(
        smmap_msgs::GetGripperNames::Request& req,
        smmap_msgs::GetGripperNames::Response& res)
{
    (void)req;
    res.names = auto_grippers_;
    return true;
}

bool CustomScene::getGripperAttachedNodeIndicesCallback(
        smmap_msgs::GetGripperAttachedNodeIndices::Request& req,
        smmap_msgs::GetGripperAttachedNodeIndices::Response& res)
{
    GripperKinematicObject::Ptr gripper = grippers_.at(req.name);
    res.indices = gripper->getAttachedNodeIndices();
    return true;
}

bool CustomScene::getGripperPoseCallback(
        smmap_msgs::GetGripperPose::Request& req,
        smmap_msgs::GetGripperPose::Response& res)
{
    GripperKinematicObject::Ptr gripper = grippers_.at(req.name);
    res.pose = toRosPose(gripper->getWorldTransform(), METERS);
    return true;
}

bool CustomScene::gripperCollisionCheckCallback(
        smmap_msgs::GetGripperCollisionReport::Request& req,
        smmap_msgs::GetGripperCollisionReport::Response& res)
{
    size_t num_checks = req.pose.size();

    res.gripper_distance_to_obstacle.resize(num_checks);
    res.gripper_nearest_point_to_obstacle.resize(num_checks);
    res.obstacle_surface_normal.resize(num_checks);

    for (size_t pose_ind = 0; pose_ind < num_checks; pose_ind++)
    {
        collision_check_gripper_->setWorldTransform(toBulletTransform(req.pose[pose_ind], METERS));
        btPointCollector collision_result = collisionHelper(collision_check_gripper_);

        if (collision_result.m_hasResult)
        {
            res.gripper_distance_to_obstacle[pose_ind] = collision_result.m_distance / METERS;
        }
        else
        {
            res.gripper_distance_to_obstacle[pose_ind] = std::numeric_limits<double>::infinity();
        }

        res.obstacle_surface_normal[pose_ind] = toRosVector3(collision_result.m_normalOnBInWorld, 1);
        res.gripper_nearest_point_to_obstacle[pose_ind] = toRosPoint(
                    collision_result.m_pointInWorld
                    + collision_result.m_normalOnBInWorld * collision_result.m_distance, METERS);
    }

    return true;
}

bool CustomScene::getCoverPointsCallback(
        smmap_msgs::GetPointSet::Request& req,
        smmap_msgs::GetPointSet::Response& res)
{
    (void)req;
    res.points = toRosPointVector(cover_points_, METERS);
    return true;
}

bool CustomScene::getMirrorLineCallback(
        smmap_msgs::GetMirrorLine::Request& req,
        smmap_msgs::GetMirrorLine::Response& res)
{
    (void)req;
    if (task_type_ == TaskType::COLAB_FOLDING &&
         deformable_type_ == DeformableType::CLOTH)
    {
        res = mirror_line_data_;
        return true;
    }
    else
    {
        res.mid_x = std::numeric_limits<double>::infinity();
        res.min_y = std::numeric_limits<double>::infinity();
        res.max_y = std::numeric_limits<double>::infinity();
        return false;
    }
}

bool CustomScene::getFreeSpaceGraphCallback(
        smmap_msgs::GetFreeSpaceGraphRequest& req,
        smmap_msgs::GetFreeSpaceGraphResponse& res)
{
    (void)req;

    // First serialze the graph itself
    {
        // Allocate a data buffer to store the results in
        res.graph_data_buffer.clear();
        size_t expected_data_len = 0;
        expected_data_len += sizeof(size_t); // Graph node vector header
        expected_data_len += free_space_graph_.GetNodesImmutable().size() * 3 * sizeof(btScalar); // Value item for each graph node
        expected_data_len += free_space_graph_.GetNodesImmutable().size() * sizeof(double); // Distance value for each graph node
        expected_data_len += free_space_graph_.GetNodesImmutable().size() * 2 * sizeof(size_t); // Edge vector overhead
        expected_data_len += num_graph_edges_ * sizeof(arc_dijkstras::GraphEdge); // Total number of edges to store
        res.graph_data_buffer.reserve(expected_data_len);

        // Define the graph value serialization function
        const auto value_serializer_with_scaling_fn = [] (const btVector3& value, std::vector<uint8_t>& buffer)
        {
            const uint64_t start_buffer_size = buffer.size();
            uint64_t running_total = 0;

            running_total += arc_helpers::SerializeFixedSizePOD<btScalar>(value.x() / METERS, buffer);
            running_total += arc_helpers::SerializeFixedSizePOD<btScalar>(value.y() / METERS, buffer);
            running_total += arc_helpers::SerializeFixedSizePOD<btScalar>(value.z() / METERS, buffer);

            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;

            assert(running_total == bytes_written);

            return bytes_written;
//            return 0;
        };

        const auto value_serializer_fn = [] (const btVector3& value, std::vector<uint8_t>& buffer)
        {
            const uint64_t start_buffer_size = buffer.size();
            uint64_t running_total = 0;

            running_total += arc_helpers::SerializeFixedSizePOD<btScalar>(value.x(), buffer);
            running_total += arc_helpers::SerializeFixedSizePOD<btScalar>(value.y(), buffer);
            running_total += arc_helpers::SerializeFixedSizePOD<btScalar>(value.z(), buffer);

            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;

            assert(running_total == bytes_written);

            return bytes_written;
//            return 0;
        };

        // Define the graph value serialization function
        const auto value_deserializer_fn = [] (const std::vector<uint8_t>& buffer, const int64_t current)
        {
            uint64_t current_position = current;

            // Deserialze 3 floats, converting into doubles afterwards
            std::pair<btScalar, uint64_t> x = arc_helpers::DeserializeFixedSizePOD<btScalar>(buffer, current_position);
            current_position += x.second;
            std::pair<btScalar, uint64_t> y = arc_helpers::DeserializeFixedSizePOD<btScalar>(buffer, current_position);
            current_position += y.second;
            std::pair<btScalar, uint64_t> z = arc_helpers::DeserializeFixedSizePOD<btScalar>(buffer, current_position);
            current_position += z.second;

            const btVector3 deserialized(x.first, y.first, z.first);

            // Figure out how many bytes were read
            const uint64_t bytes_read = current_position - current;
            return std::make_pair(deserialized, bytes_read);
//            return std::make_pair(btVector3(), 0L);
        };

        // Resize the edge weights by a factor of METERS
        free_space_graph_.SerializeSelf(res.graph_data_buffer, value_serializer_with_scaling_fn);
        auto resized_graph = arc_dijkstras::Graph<btVector3>::Deserialize(res.graph_data_buffer, 0, value_deserializer_fn).first;

        for (auto& node: resized_graph.GetNodesMutable())
        {
            for (auto& edge: node.GetInEdgesMutable())
            {
                edge.SetWeight(edge.GetWeight() / METERS);
            }
            for (auto& edge: node.GetOutEdgesMutable())
            {
                edge.SetWeight(edge.GetWeight() / METERS);
            }
        }
        // Ensure we still have a valid graph
        assert(free_space_graph_.CheckGraphLinkage());
        assert(resized_graph.CheckGraphLinkage());

        // Finally, serialze the graph
        res.graph_data_buffer.clear();
        res.graph_data_buffer.reserve(expected_data_len);
        resized_graph.SerializeSelf(res.graph_data_buffer, value_serializer_fn);
    }

    // Next add the mapping between cover indices and graph indices
    res.cover_point_ind_to_graph_ind = cover_ind_to_free_space_graph_ind_;

    return true;
}

bool CustomScene::getObjectInitialConfigurationCallback(
        smmap_msgs::GetPointSet::Request& req,
        smmap_msgs::GetPointSet::Response& res)
{
    (void)req;
    res.points = object_initial_configuration_;
    return true;
}

bool CustomScene::getObjectCurrentConfigurationCallback(
        smmap_msgs::GetPointSet::Request& req,
        smmap_msgs::GetPointSet::Response& res)
{
    (void)req;
    res.points = toRosPointVector(getDeformableObjectNodes(), METERS);
    return true;
}

bool CustomScene::terminateSimulationCallback(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res)
{
    (void)req;
    (void)res;

    if (screen_recorder_)
    {
        screen_recorder_->zipScreenshots();
    }
    ros::shutdown();

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks - Simulation actions
////////////////////////////////////////////////////////////////////////////////

bool CustomScene::executeGripperMovementCallback(
        smmap_msgs::ExecuteGripperMovement::Request& req,
        smmap_msgs::ExecuteGripperMovement::Response& res)
{
    assert(req.grippers_names.size() == req.gripper_poses.size());

    std::lock_guard<std::mutex> lock(sim_mutex_);
    ROS_INFO("Executing gripper command");
    for (size_t gripper_ind = 0; gripper_ind < req.grippers_names.size(); gripper_ind++)
    {
        SetGripperTransform(grippers_, req.grippers_names[gripper_ind], req.gripper_poses[gripper_ind]);
    }

    for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
    {
        screen_recorder_->snapshot();
        step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
        simulator_fbk_pub_.publish(createSimulatorFbk());
    }

    res.sim_state = createSimulatorFbk();
    return true;
}

void CustomScene::testGripperPosesExecuteCallback(
        const smmap_msgs::TestGrippersPosesGoalConstPtr& goal)
{
    const size_t num_tests = goal->poses_to_test.size();

    std::lock_guard<std::mutex> lock (sim_mutex_);
    #pragma omp parallel for
    for (size_t test_ind = 0; test_ind < num_tests; test_ind++)
    {
        ROS_INFO_STREAM_NAMED("test_gripper_poses", "Testing gripper pose " << test_ind);
        const SimForkResult result = simulateInNewFork(goal->gripper_names, goal->poses_to_test[test_ind].pose);

        // Create a feedback message
        smmap_msgs::TestGrippersPosesFeedback fbk;
        fbk.sim_state = createSimulatorFbk(result);
        fbk.test_id = test_ind;
        // Send feedback
        test_grippers_poses_as_.publishFeedback(fbk);
    }

    test_grippers_poses_as_.setSucceeded();
}

////////////////////////////////////////////////////////////////////////////////
// Pre-step Callbacks
////////////////////////////////////////////////////////////////////////////////

void CustomScene::drawAxes()
{
    for (auto& axes: gripper_axes_)
    {
        axes.second->setup(grippers_[axes.first]->getWorldTransform(), 1);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Post-step Callbacks
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////
// Key Handler for our Custom Scene
////////////////////////////////////////////////////////////////////////

CustomScene::CustomKeyHandler::CustomKeyHandler(CustomScene &scene)
    : scene_(scene)
    , current_gripper_(nullptr)
    , translate_gripper_(false)
    , rotate_gripper_(false)
{}

bool CustomScene::CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter & aa)
{
    (void)aa;
    bool suppress_default_handler = false;

    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
        {
            switch (ea.getKey())
            {
                // Gripper translate/rotate selection keybinds
                {
                    case '1':
                    {
                        current_gripper_ = getGripper(0);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'q':
                    {
                        current_gripper_ = getGripper(0);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '2':
                    {
                        current_gripper_ = getGripper(1);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'w':
                    {
                        current_gripper_ = getGripper(1);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '3':
                    {
                        current_gripper_ = getGripper(2);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'e':
                    {
                        current_gripper_ = getGripper(2);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '4':
                    {
                        current_gripper_ = getGripper(3);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'r':
                    {
                        current_gripper_ = getGripper(3);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }
                }

                // Simulation behaviour keybinds
                {
                    case 'a':
                    {
                        scene_.advance_grippers_.store(!scene_.advance_grippers_.load());
                        break;
                    }
                }

/*
                case '[':
                {
                    scene.left_gripper1->toggle();
                    scene.left_gripper1->toggleattach(scene.clothPtr->softBody.get());
                    if(scene.num_auto_grippers > 1)
                    {
                        scene.left_gripper2->toggle();
                        scene.left_gripper2->toggleattach(scene.clothPtr->softBody.get());
                    }
                    break;
                }

                case ']':
                {
                    if(scene.num_auto_grippers > 1)
                        scene.corner_number_ += 2;
                    else
                        scene.corner_number_ += 1;
                    if(scene.corner_number_ > 3)
                        scene.corner_number_ = 0;
                    scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                        scene.clothPtr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number_]].m_x));
                    if(scene.num_auto_grippers > 1)
                        scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                            scene.clothPtr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number_+1]].m_x));
                    break;
                }
*/
/*
                case 's':
                    scene.left_gripper2->toggle();
                    break;

                case 'z':
                    scene.left_gripper1->toggleattach(scene.clothPtr->softBody.get());
                    break;

                case 'x':
                    scene.left_gripper2->toggleattach(scene.clothPtr->softBody.get());
                    break;


                case 'c':
                    scene.regraspWithOneGripper(scene.left_gripper1,scene.left_gripper2);
                    break;

                case 'v':
                    scene.regraspWithOneGripper(scene.right_gripper1,scene.right_gripper2);
                    break;
*/
/*
                case 'f':
                    // scene.regraspWithOneGripper(scene.right_gripper1,scene.left_gripper1);
                    // scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
                    scene.testRelease(scene.left_gripper1);
                    break;

                case 't':
                    scene.testRelease2(scene.left_gripper2);
                    break;scene.inputState.startDragging = true;

                case 'y':
                    scene.testRegrasp2(scene.left_gripper2);
                    break;

                case 'g':
                    // scene.regraspWithOneGripper(scene.right_gripper2,scene.left_gripper2);
                    // scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,110)));
                    scene.testRegrasp(scene.left_gripper1);
                    break;

                case 'k':
                    std::cout << "try to adjust first gripper" << std::endl;
                    scene.testAdjust(scene.left_gripper1);
                    break;

                case 'l':
                    // std::cout << "try to adjust second gripper" << std::endl;
                    scene.switchTarget();
                    break;
*/
/*
                 case 'k':
                     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*
                         scene.right_gripper1->getWorldTransform().getRotation(),
                         scene.right_gripper1->getWorldTransform().getOrigin()));
                     break;

                 case ',':
                     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*
                         scene.right_gripper1->getWorldTransform().getRotation(),
                         scene.right_gripper1->getWorldTransform().getOrigin()));
                     break;


                 case 'l':
                     scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*
                         scene.right_gripper2->getWorldTransform().getRotation(),
                         scene.right_gripper2->getWorldTransform().getOrigin()));
                     break;

                case '.':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*
                        scene.right_gripper2->getWorldTransform().getRotation(),
                        scene.right_gripper2->getWorldTransform().getOrigin()));
                    break;


                 case 'y':
                     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*
                         scene.right_gripper1->getWorldTransform().getRotation(),
                         scene.right_gripper1->getWorldTransform().getOrigin()));
                     break;


                case 'u':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*
                        scene.right_gripper2->getWorldTransform().getRotation(),
                        scene.right_gripper2->getWorldTransform().getOrigin()));
                    break;
*/
/*
                case 'i':
                    scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                        scene.clothPtr->softBody->m_nodes[scene.robot_mid_point_ind].m_x));
                    break;

                case 'o':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                        scene.clothPtr->softBody->m_nodes[scene.user_mid_point_ind].m_x));
                    break;


                case 'b':
                    if(scene.right_gripper2->bOpen)
                        scene.right_gripper2->state = GripperState_CLOSING;
                    else
                        scene.right_gripper2->state = GripperState_OPENING;

                    break;

                case 'n':
                    if(scene.left_gripper2->bOpen)
                        scene.left_gripper2->state = GripperState_CLOSING;
                    else
                        scene.left_gripper2->state = GripperState_OPENING;

                    break;

                case 'j':
                {
                    /// by bFirstTrackingIteration
                    scene.getDeformableObjectNodes(scene.prev_node_pos);
                    scene.bTracking = !scene.bTracking;
                    if(scene.bTracking)
                    {
                        scene.bFirstTrackingIteration = true;
                        scene.itrnumber = 0;
                    }
                    if(!scene.bTracking)
                    {
                        scene.plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());
                    }

                    break;
                }
*/
            }
            break;
        }

        case osgGA::GUIEventAdapter::KEYUP:
        {
            switch (ea.getKey())
            {
                case '1':
                case 'q':
                case '2':
                case 'w':
                case '3':
                case 'e':
                case '4':
                case 'r':
                {
                    current_gripper_ = nullptr;
                    translate_gripper_ = false;
                    rotate_gripper_ = false;
                }
            }
            break;
        }

        case osgGA::GUIEventAdapter::PUSH:
        {
            start_dragging_ = true;
            break;
        }

        case osgGA::GUIEventAdapter::DRAG:
        {
            // drag the active manipulator in the plane of view
            if ((ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) && current_gripper_)
            {
                // if we've just started moving, reset our internal position trackers
                if (start_dragging_)
                {
                    mouse_last_x_ = ea.getXnormalized();
                    mouse_last_y_ = ea.getYnormalized();
                    start_dragging_ = false;
                }
                float dx = mouse_last_x_ - ea.getXnormalized();
                float dy = ea.getYnormalized() - mouse_last_y_;

                mouse_last_x_ = ea.getXnormalized();
                mouse_last_y_ = ea.getYnormalized();

                // get our current view
                osg::Vec3d osgCenter, osgEye, osgUp;
                scene_.manip->getTransformation(osgCenter, osgEye, osgUp);
                btVector3 from(util::toBtVector(osgEye));
                btVector3 to(util::toBtVector(osgCenter));
                btVector3 up(util::toBtVector(osgUp)); up.normalize();

                // compute basis vectors for the plane of view
                // (the plane normal to the ray from the camera to the center of the scene)
                btVector3 camera_normal_vector = (to - from).normalized();
                btVector3 camera_y_axis = (up - (up.dot(camera_normal_vector))*camera_normal_vector).normalized(); //TODO: FIXME: is this necessary with osg?
                btVector3 camera_x_axis = camera_normal_vector.cross(camera_y_axis);
                btVector3 drag_vector = SceneConfig::mouseDragScale * (dx*camera_x_axis + dy*camera_y_axis);

                btTransform current_transform = current_gripper_->getWorldTransform();
                btTransform next_transform(current_transform);

                if (translate_gripper_)
                {
                    // if moving the manip, just set the origin appropriately
                    next_transform.setOrigin(drag_vector + current_transform.getOrigin());
                }
                else if (rotate_gripper_)
                {
                    // if we're rotating, the axis is perpendicular to the
                    // direction the mouse is dragging
                    btVector3 axis = camera_normal_vector.cross(drag_vector);
                    btScalar angle = drag_vector.length();
                    btQuaternion rot(axis, angle);
                    // we must ensure that we never get a bad rotation quaternion
                    // due to really small (effectively zero) mouse movements
                    // this is the easiest way to do this:
                    if (rot.length() > 0.99f && rot.length() < 1.01f)
                    {
                        next_transform.setRotation(rot * current_transform.getRotation());
                    }
                }

                // We don't need to lock here, as this is called inside of draw()
                // at which point we've already locked
                current_gripper_->setWorldTransform(next_transform);
                suppress_default_handler = true;
            }
            break;
        }

        default:
        {
            break;
        }
    }
    return suppress_default_handler;
}

GripperKinematicObject::Ptr CustomScene::CustomKeyHandler::getGripper(size_t gripper_num)
{
    if (gripper_num < scene_.auto_grippers_.size())
    {
        return scene_.grippers_[scene_.auto_grippers_[gripper_num]];
    }
    else if (gripper_num - scene_.auto_grippers_.size() < scene_.manual_grippers_.size())
    {
        gripper_num -= scene_.auto_grippers_.size();
        return scene_.grippers_[scene_.manual_grippers_[gripper_num]];
    }
    else
    {
        std::cerr << "Invalid gripper number: " << gripper_num << std::endl;
        std::cerr << "Existing auto grippers:   " << PrettyPrint::PrettyPrint(scene_.auto_grippers_) << std::endl;
        std::cerr << "Existing manual grippers: " << PrettyPrint::PrettyPrint(scene_.manual_grippers_) << std::endl;
        return nullptr;
    }
}
