#include "simulation/recording.h"
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "utils/my_assert.h"
#include <smmap_experiment_params/ros_params.hpp>

using namespace std;

namespace fs = boost::filesystem;

bool yesOrNo(const string& message)
{
    while (true)
    {
        cout << message << " (y/n): ";
        char yn;
        cin >> yn;
        if (yn == 'y')
        {
            return true;
        }
        else if (yn == 'n')
        {
            return false;
        }
    }
}

void askToResetDir(fs::path p)
{
    if (fs::exists(p))
    {
        bool consent = yesOrNo(p.string() + "already exists. Delete it?");
        if (consent)
        {
            cout << "deleting " << p.string() << endl;
            fs::remove_all(p);
        }
        else throw IOError();
    }
    if (!fs::is_directory(p))
    {
        std::cerr << "\x1b[33;1m" << p << " does not exist! Creating ... ";

        // NOTE: create_directories should be able to return true in this case
        // however due to a bug related to a trailing '/' this is not currently
        // the case in my version of boost
        // https://svn.boost.org/trac/boost/ticket/7258
        fs::create_directories(p);
        if (fs::is_directory(p))
//            if (boost::filesystem::create_directories(p))
        {
            std::cerr << "Succeeded!\x1b[37m\n";
        }
        else
        {
            std::cerr << "\x1b[31;1mFailed!\x1b[37m\n";
        }
    }

    ENSURE(fs::is_directory(p));
}

void resetDir(fs::path p)
{
    if (fs::exists(p))
    {
        fs::remove_all(p);
    }

    if (!fs::is_directory(p))
    {
        std::cerr << "\x1b[33;1m" << p << " does not exist! Creating ... ";

        // NOTE: create_directories should be able to return true in this case
        // however due to a bug related to a trailing '/' this is not currently
        // the case in my version of boost
        // https://svn.boost.org/trac/boost/ticket/7258
        fs::create_directories(p);
        if (fs::is_directory(p))
//            if (boost::filesystem::create_directories(p))
        {
            std::cerr << "Succeeded!\x1b[37m\n";
        }
        else
        {
            std::cerr << "\x1b[31;1mFailed!\x1b[37m\n";
        }
    }

    ENSURE(fs::is_directory(p));
}

ScreenRecorder::ScreenRecorder(osgViewer::Viewer* const viewer, const bool screenshots_enabled, const std::string& screenshot_dir)
    : m_screenshotsEnabled(screenshots_enabled)
    , m_screenshotDir(screenshot_dir)
    , m_viewer(viewer)
    , m_captureOperation(nullptr)
    , m_captureHandler(nullptr)
{
    if (m_screenshotsEnabled)
    {
        assert(m_viewer != nullptr);
        std::cout << "Clearning destination folder";
        if (fs::exists(m_screenshotDir))
        {
            fs::remove_all(m_screenshotDir);
        }

        std::cout << "Clearing temporary screenshot folder " << "/tmp/smmap_screenshots/" << std::endl;
        resetDir("/tmp/smmap_screenshots/");

        m_captureOperation = std::make_shared<osgViewer::ScreenCaptureHandler::WriteToFile>("/tmp/smmap_screenshots/img", "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER);
        m_captureHandler = std::make_shared<osgViewer::ScreenCaptureHandler>(m_captureOperation.get());
        m_viewer->addEventHandler(m_captureHandler.get());
    }
    else
    {
        std::cout << "Screenshots disabled\n";
    }
}

void ScreenRecorder::zipScreenshots()
{
    if (m_screenshotsEnabled)
    {
        int rv;
        const std::string video_file = m_screenshotDir.substr(0, m_screenshotDir.length() - 1) + ".avi";
        std::cout << "Creating output video " << video_file << std::endl;
        rv = system(("cd /tmp/smmap_screenshots && imToMov.sh 100 " + video_file + " jpg").c_str());
        if (rv < 0)
        {
            std::cerr << "Unable to create video; return value " << rv << std::endl;
        }

        const std::string zip_file = m_screenshotDir.substr(0, m_screenshotDir.length() - 1) + ".tar.gz";
        std::cout << "Zipping screenshots and moving to" << zip_file << std::endl;
        const std::string sys_cmd = "tar -czf ";
        rv = system((sys_cmd + zip_file + " -C /tmp/smmap_screenshots .").c_str());
        if (rv < 0)
        {
            std::cerr << "Unable to zip and move screenshots; return value " << rv << std::endl;
        }
    }
}

void ScreenRecorder::snapshot()
{
    if (m_screenshotsEnabled)
    {
        std::cout << "Taking screenshot (actual rate is half - Viewer is not rendering every frame)\n";
        m_captureHandler->captureNextFrame(*m_viewer);
    }
}
