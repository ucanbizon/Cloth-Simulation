#include "Cloth.h"
#include "FastBox.h"
#include "Sphere.h"
#include "Plane.h"
#include "Renderer.h"

int main() {
    Cloth cloth(20, 1.0f, 100.0f, 80.0f);
    FastBox fastbox(cloth);
    Sphere sphere(Eigen::Vector3f(0.0, -50.0,0.0), 50.0f);
    Plane ground(-35.0f);
    Renderer renderer(800, 600, cloth, sphere, ground);
    while (!glfwWindowShouldClose(renderer.GetWindow())){
        cloth.Run(20);
        fastbox.PlaneBoxCollision(cloth, ground, true);
        cloth.PlaneCollision(fastbox, ground);
        fastbox.SphereBoxCollision(cloth, sphere, true);
        cloth.SphereCollision(fastbox, sphere);
        fastbox.SelfBoxCollision(cloth);
        cloth.SelfCollision(fastbox);
        cloth.ComputeNormals();
        renderer.Render(cloth, sphere, ground);
    }
    return 0;
}


