#pragma once

#include "Spade/Core/Objects.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace Spade {

    class InteractionSystem {
    public:
        void Update(Universe& universe, GLFWwindow* window, float deltaTime);
    };

}
