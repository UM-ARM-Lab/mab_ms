#pragma once
#include "utils/config.h"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <memory>

class ScreenRecorder
{
    public:
        ScreenRecorder(osgViewer::Viewer* const viewer, const bool screenshots_enabled, const std::string& screenshot_dir);
        void zipScreenshots();

        void snapshot(); //call this BEFORE scene's step() method

    private:
        const bool m_screenshotsEnabled;
        const std::string m_screenshotDir;
        osgViewer::Viewer* const m_viewer;
        std::shared_ptr<osgViewer::ScreenCaptureHandler::CaptureOperation> m_captureOperation;
        std::shared_ptr<osgViewer::ScreenCaptureHandler> m_captureHandler;
};
