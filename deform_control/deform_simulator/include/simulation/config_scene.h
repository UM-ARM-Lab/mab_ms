#pragma once
#include "utils/config.h"

struct SceneConfig : Config
{
    static float mouseDragScale;
    SceneConfig() : Config()
    {
        params.push_back(new Parameter<float>("mouseDragScale", &mouseDragScale, "scaling factor for mouse control for IK"));
    }
};
