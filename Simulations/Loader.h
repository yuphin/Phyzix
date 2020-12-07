#pragma once
/*
    This is a modified version of the original code
    Link to original code: https://github.com/mmacklin/tinsel
*/

#include <cstring>
#include "Scene.h"
#include "Options.h"
class Scene;
bool load_scene(const std::string& filename, Scene* scene, Options* render_opts);

