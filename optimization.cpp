#include "cloth.h"
#include "integrator.h"
#include "controller.h"
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <iostream>
#include <memory>

using namespace ClothOpt;

int main() {
    // Initialize Polyscope
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::init();
    
    polyscope::view::setUpDir(polyscope::UpDir::YUp);
    polyscope::view::setFrontDir(polyscope::FrontDir::ZFront);
    
    // Create 10x10 cloth
    ClothMesh cloth;
    cloth.createGrid(10, 10, 0.1);  // 10x10 grid with 0.1 spacing
    
    double clothWidth = 9 * 0.1;  // 9 intervals = 0.9 units
    double clothCenter = clothWidth / 2.0;
    
    // Start cloth flat at a reasonable height
    for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
        Eigen::Vector3d pos = cloth.getVertex(i).position;
        pos.y() = 0.5;  // Start at height 0.5
        cloth.setVertexPosition(i, pos);
    }
    
    std::cout << "=== SIMPLE EDGE PULL DEMO ===" << std::endl;
    std::cout << "Cloth: 10x10 grid, size: " << clothWidth << " x " << clothWidth << std::endl;
    std::cout << "Pulling one edge (10 vertices) in the same direction" << std::endl;
    
    // Cloth properties - flexible for nice deformation
    cloth.properties.stiffness = 800.0;
    cloth.properties.bendingStiffness = 20.0;
    cloth.properties.damping = 0.9;
    cloth.properties.friction = 0.8;
    cloth.properties.gravity = Eigen::Vector3d(0, -9.81, 0);
    
    // Create integrator
    auto integrator = std::make_unique<SemiImplicitEulerIntegrator>();
    integrator->enableDebug(false);
    
    // Create controller
    ClothController controller;
    controller.enableDebug(true);
    
    // Get all vertices on one edge (bottom edge: j=9)
    std::vector<size_t> edgeVertices;
    for (int i = 0; i < 10; ++i) {
        size_t vertexIndex = cloth.getGridIndex(i, 9);  // Bottom edge
        edgeVertices.push_back(vertexIndex);
    }
    
    std::cout << "Edge vertices to control: ";
    for (size_t idx : edgeVertices) {
        std::cout << idx << " ";
    }
    std::cout << std::endl;
    
    // Visualization
    auto* psMesh = polyscope::registerSurfaceMesh("Cloth", cloth.getVertexMatrix(), cloth.getTriangleMatrix());
    psMesh->setSurfaceColor({0.3, 0.7, 0.9});  // Nice blue cloth
    psMesh->setEdgeWidth(1.0);
    psMesh->setMaterial("wax");
    
    // Ground plane
    std::vector<Eigen::Vector3d> groundVertices = {
        {-0.2, 0.0, -0.2}, 
        {clothWidth + 0.2, 0.0, -0.2}, 
        {clothWidth + 0.2, 0.0, clothWidth + 0.2}, 
        {-0.2, 0.0, clothWidth + 0.2}
    };
    std::vector<std::array<int, 3>> groundTriangles = {{0, 1, 2}, {0, 2, 3}};
    auto* psGround = polyscope::registerSurfaceMesh("Ground", groundVertices, groundTriangles);
    psGround->setSurfaceColor({0.5, 0.5, 0.5});
    
    // Control points visualization - show the edge being controlled
    std::vector<Eigen::Vector3d> controlPoints;
    for (size_t idx : edgeVertices) {
        controlPoints.push_back(cloth.getVertex(idx).position);
    }
    auto* psControlPoints = polyscope::registerPointCloud("Control Edge", controlPoints);
    psControlPoints->setPointRadius(0.015);
    psControlPoints->setPointColor({1.0, 0.0, 0.0});  // Red for controlled edge
    
    // Camera position
    polyscope::view::lookAt(
        {clothCenter + 0.8, 0.8, clothCenter + 0.8},
        {clothCenter, 0.3, clothCenter},
        {0.0, 1.0, 0.0}
    );
    
    // Simulation variables
    double dt = 0.002;
    int frameCount = 0;
    double simulationTime = 0.0;
    
    // Control variables
    bool applyControl = false;
    float pullDistance = 0.3f;      // How far to pull
    float pullHeight = 0.2f;        // How high to lift
    float controlStrength = 1200.0f;
    
    // Pull direction options
    enum PullDirection {
        PULL_FORWARD,   // +Z direction
        PULL_BACKWARD,  // -Z direction
        PULL_RIGHT,     // +X direction
        PULL_LEFT,      // -X direction
        PULL_UP         // +Y direction
    };
    
    PullDirection currentDirection = PULL_FORWARD;
    const char* directionNames[] = {"Forward (+Z)", "Backward (-Z)", "Right (+X)", "Left (-X)", "Up (+Y)"};
    
    polyscope::state::userCallback = [&]() {
        simulationTime += dt;
        
        // Apply control if enabled
        if (applyControl) {
            controller.applyControls(cloth, dt);
        }
        
        // Run physics simulation
        for (int i = 0; i < 3; ++i) {
            integrator->step(cloth, dt);
        }
        
        // Update visualization
        psMesh->updateVertexPositions(cloth.getVertexMatrix());
        
        // Update control points visualization
        std::vector<Eigen::Vector3d> updatedControlPoints;
        for (size_t idx : edgeVertices) {
            updatedControlPoints.push_back(cloth.getVertex(idx).position);
        }
        psControlPoints->updatePointPositions(updatedControlPoints);
        
        frameCount++;
        
        // GUI
        if (ImGui::Begin("Edge Pull Demo")) {
            ImGui::Text("Simulation Time: %.1f s", simulationTime);
            ImGui::Text("Frame: %d", frameCount);
            ImGui::Text("Controlled vertices: %zu", edgeVertices.size());
            
            ImGui::Separator();
            
            // Direction selection
            ImGui::Text("Pull Direction:");
            if (ImGui::RadioButton("Forward (+Z)", currentDirection == PULL_FORWARD)) {
                currentDirection = PULL_FORWARD;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Backward (-Z)", currentDirection == PULL_BACKWARD)) {
                currentDirection = PULL_BACKWARD;
            }
            
            if (ImGui::RadioButton("Right (+X)", currentDirection == PULL_RIGHT)) {
                currentDirection = PULL_RIGHT;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Left (-X)", currentDirection == PULL_LEFT)) {
                currentDirection = PULL_LEFT;
            }
            
            if (ImGui::RadioButton("Up (+Y)", currentDirection == PULL_UP)) {
                currentDirection = PULL_UP;
            }
            
            ImGui::Separator();
            
            // Control parameters
            ImGui::SliderFloat("Pull Distance", &pullDistance, 0.1f, 0.8f);
            ImGui::SliderFloat("Pull Height", &pullHeight, 0.0f, 0.5f);
            ImGui::SliderFloat("Control Strength", &controlStrength, 500.0f, 3000.0f);
            
            ImGui::Separator();
            
            // Control buttons
            if (ImGui::Button("Start Pulling")) {
                applyControl = true;
                controller.removeAllControls();
                
                // Calculate pull direction vector
                Eigen::Vector3d pullDirection(0, 0, 0);
                switch (currentDirection) {
                    case PULL_FORWARD:  pullDirection = Eigen::Vector3d(0, 0, pullDistance); break;
                    case PULL_BACKWARD: pullDirection = Eigen::Vector3d(0, 0, -pullDistance); break;
                    case PULL_RIGHT:    pullDirection = Eigen::Vector3d(pullDistance, 0, 0); break;
                    case PULL_LEFT:     pullDirection = Eigen::Vector3d(-pullDistance, 0, 0); break;
                    case PULL_UP:       pullDirection = Eigen::Vector3d(0, pullDistance, 0); break;
                }
                
                // Add control to all edge vertices
                for (size_t idx : edgeVertices) {
                    Eigen::Vector3d currentPos = cloth.getVertex(idx).position;
                    Eigen::Vector3d targetPos = currentPos + pullDirection + Eigen::Vector3d(0, pullHeight, 0);
                    
                    controller.addPositionControl(idx, targetPos, controlStrength, 100.0);
                }
                
                std::cout << "Started pulling edge in direction: " << directionNames[currentDirection] << std::endl;
                std::cout << "Pull vector: " << pullDirection.transpose() << std::endl;
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Stop Pulling")) {
                applyControl = false;
                controller.removeAllControls();
                std::cout << "Stopped pulling" << std::endl;
            }
            
            ImGui::Separator();
            
            if (ImGui::Button("Reset Cloth")) {
                applyControl = false;
                controller.removeAllControls();
                
                // Reset all vertices to original grid positions
                for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
                    int gridX = i % 10;
                    int gridY = i / 10;
                    Eigen::Vector3d pos(gridX * 0.1, 0.5, gridY * 0.1);
                    cloth.setVertexPosition(i, pos);
                    cloth.velocities[i] = Eigen::Vector3d::Zero();
                }
                
                std::cout << "Reset cloth to original position" << std::endl;
            }
            
            ImGui::Separator();
            
            // Debug info
            ImGui::Text("Debug Info:");
            ImGui::Text("Control active: %s", applyControl ? "YES" : "NO");
            ImGui::Text("Active controls: %zu", controller.getControlCount());
            
            if (ImGui::CollapsingHeader("Edge Vertices Positions")) {
                for (size_t i = 0; i < edgeVertices.size(); ++i) {
                    size_t idx = edgeVertices[i];
                    Eigen::Vector3d pos = cloth.getVertex(idx).position;
                    ImGui::Text("Vertex %zu: (%.2f, %.2f, %.2f)", idx, pos.x(), pos.y(), pos.z());
                }
            }
        }
        ImGui::End();
    };
    
    std::cout << "\nStarting edge pull demo..." << std::endl;
    std::cout << "Use the GUI to control the bottom edge of the cloth!" << std::endl;
    std::cout << "- Select pull direction" << std::endl;
    std::cout << "- Adjust pull distance and height" << std::endl;
    std::cout << "- Click 'Start Pulling' to apply control" << std::endl;
    
    polyscope::show();
    return 0;
}