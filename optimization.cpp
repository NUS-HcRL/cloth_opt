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
    // Initialize Polyscope with larger GUI
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::view::windowWidth = 1600;   // Wider window
    polyscope::view::windowHeight = 1000;  // Taller window
    polyscope::init();
    
    // Scale up ImGui fonts for better readability
    ImGuiIO& io = ImGui::GetIO();
    io.FontGlobalScale = 3.0f;  // 30% larger text
    
    polyscope::view::setUpDir(polyscope::UpDir::YUp);
    polyscope::view::setFrontDir(polyscope::FrontDir::ZFront);
    
    // Create 2x2 cloth
    ClothMesh cloth;
    cloth.createGrid(2, 2, 0.2);
    
    // Position cloth slightly above the ground plane
    double groundHeight = 0.0;
    double clothStartHeight = 0.05;  // Start just above ground
    
    for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
        Eigen::Vector3d pos = cloth.getVertex(i).position;
        pos.y() = clothStartHeight;  // Place cloth near ground
        cloth.setVertexPosition(i, pos);
    }
    
    // Print vertex layout
    std::cout << "=== 2x2 CLOTH DEMO ===" << std::endl;
    std::cout << "2 --- 3" << std::endl;
    std::cout << "|     |" << std::endl;
    std::cout << "0 --- 1" << std::endl;
    
    // Print initial positions
    for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
        Eigen::Vector3d pos = cloth.getVertex(i).position;
        std::cout << "Vertex " << i << ": (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
    }
    
    // Simple cloth properties
    cloth.properties.stiffness = 500.0;
    cloth.properties.damping = 0.8;
    cloth.properties.gravity = Eigen::Vector3d(0, -9.81, 0);  // Standard gravity
    
    // Create integrator and controller
    auto integrator = std::make_unique<SemiImplicitEulerIntegrator>();
    ClothController controller;
    
    // Pin vertex 0 as anchor
    cloth.pinVertex(0);
    std::cout << "Pinned vertex 0 (anchor)" << std::endl;
    std::cout << "Will control vertex 3 (top-right)" << std::endl;
    
    // Create ground plane
    std::vector<Eigen::Vector3d> groundVertices = {
        {-0.5, groundHeight, -0.5},     // Bottom-left
        {0.7, groundHeight, -0.5},      // Bottom-right  
        {0.7, groundHeight, 0.7},       // Top-right
        {-0.5, groundHeight, 0.7}       // Top-left
    };
    std::vector<std::array<int, 3>> groundTriangles = {
        {0, 1, 2},  // First triangle
        {0, 2, 3}   // Second triangle
    };
    
    // Visualization
    auto* psMesh = polyscope::registerSurfaceMesh("Cloth", cloth.getVertexMatrix(), cloth.getTriangleMatrix());
    psMesh->setSurfaceColor({0.2, 0.8, 0.4});  // Green cloth
    psMesh->setEdgeWidth(3.0);
    psMesh->setMaterial("wax");
    
    // Register ground plane
    auto* psGround = polyscope::registerSurfaceMesh("Ground", groundVertices, groundTriangles);
    psGround->setSurfaceColor({0.7, 0.7, 0.7});  // Gray ground
    psGround->setMaterial("flat");
    
    // Vertex visualization
    std::vector<Eigen::Vector3d> vertexPositions;
    for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
        vertexPositions.push_back(cloth.getVertex(i).position);
    }
    auto* psVertices = polyscope::registerPointCloud("Vertices", vertexPositions);
    psVertices->setPointRadius(0.008);
    psVertices->setPointColor({1.0, 0.0, 0.0});  // Red vertices
    
    // Control point visualization - initialize with dummy point to avoid size mismatch
    std::vector<Eigen::Vector3d> initialControlPoint = {Eigen::Vector3d(0, 0, 0)};
    auto* psControlPoints = polyscope::registerPointCloud("Control Points", initialControlPoint);
    psControlPoints->setPointRadius(0.01);
    psControlPoints->setPointColor({1.0, 1.0, 0.0});  // Yellow for controlled
    psControlPoints->setEnabled(false);  // Hide initially
    
    // Camera position - better view of cloth on ground
    polyscope::view::lookAt(
        {0.5, 0.4, 0.5},    // Camera position
        {0.1, 0.1, 0.1},    // Look at point (center of cloth)
        {0.0, 1.0, 0.0}     // Up direction
    );
    
    // Simulation variables
    double dt = 0.01;
    bool controlActive = false;
    float targetX = 0.2f, targetY = 0.3f, targetZ = 0.2f;  // Default target above ground
    float strength = 800.0f;
    int controlVertex = 3;  // Control vertex 3 instead of 0
    
    polyscope::state::userCallback = [&]() {

        ImGuiIO& io = ImGui::GetIO();
        io.FontGlobalScale = 2.0f; 

        // Apply control
        if (controlActive) {
            controller.applyControls(cloth, dt);
        }
        
        // Physics
        integrator->step(cloth, dt);
        
        // Simple ground collision (prevent vertices from going below ground)
        for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
            Eigen::Vector3d pos = cloth.getVertex(i).position;
            if (pos.y() < groundHeight + 0.01) {  // Small offset to prevent penetration
                pos.y() = groundHeight + 0.01;
                cloth.setVertexPosition(i, pos);
                // Damp velocity in Y direction
                cloth.velocities[i].y() *= 0.3;
            }
        }
        
        // Update visualization
        psMesh->updateVertexPositions(cloth.getVertexMatrix());
        
        // Update vertex visualization
        std::vector<Eigen::Vector3d> updatedPositions;
        for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
            updatedPositions.push_back(cloth.getVertex(i).position);
        }
        psVertices->updatePointPositions(updatedPositions);
        
        // Update control points visualization - FIX: Always update with exactly 1 point
        if (controlActive) {
            std::vector<Eigen::Vector3d> controlPoints = {cloth.getVertex(controlVertex).position};
            psControlPoints->updatePointPositions(controlPoints);
            psControlPoints->setEnabled(true);
        } else {
            // Keep the same size but hide the point cloud
            std::vector<Eigen::Vector3d> hiddenPoint = {Eigen::Vector3d(0, -10, 0)};  // Move far away
            psControlPoints->updatePointPositions(hiddenPoint);
            psControlPoints->setEnabled(false);
        }
        
        // Larger GUI window with better spacing
        ImGui::SetNextWindowSize(ImVec2(450, 700), ImGuiCond_FirstUseEver);  // Set window size
        ImGui::SetNextWindowPos(ImVec2(50, 50), ImGuiCond_FirstUseEver);     // Set position
        
        if (ImGui::Begin("2x2 Cloth Controller", nullptr, ImGuiWindowFlags_None)) {
            
            // Header section with bigger text
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 12));  // More spacing
            
            ImGui::Text("=== CLOTH LAYOUT ===");
            ImGui::Text("2 --- 3");
            ImGui::Text("|         |");
            ImGui::Text("0 --- 1");
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Vertex 0 is pinned (anchor)");
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Cloth rests on gray ground plane");
            
            ImGui::Separator();
            ImGui::Spacing();
            
            // Vertex selection with bigger buttons
            ImGui::Text("Control which vertex:");
            ImGui::Spacing();
            
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12, 8));  // Bigger buttons
            if (ImGui::RadioButton("Vertex 1##v1", controlVertex == 1)) controlVertex = 1;
            ImGui::SameLine();
            if (ImGui::RadioButton("Vertex 2##v2", controlVertex == 2)) controlVertex = 2;
            ImGui::SameLine();
            if (ImGui::RadioButton("Vertex 3##v3", controlVertex == 3)) controlVertex = 3;
            ImGui::PopStyleVar();
            
            ImGui::Separator();
            ImGui::Spacing();
            
            // Target position controls with more space
            ImGui::Text("Target position for vertex %d:", controlVertex);
            ImGui::Spacing();
            
            ImGui::PushItemWidth(250);  // Wider sliders
            ImGui::SliderFloat("X Position", &targetX, -0.3f, 0.5f, "%.3f");
            ImGui::SliderFloat("Y Position", &targetY, 0.02f, 0.8f, "%.3f");
            ImGui::SliderFloat("Z Position", &targetZ, -0.3f, 0.5f, "%.3f");
            ImGui::Spacing();
            ImGui::SliderFloat("Control Strength", &strength, 200.0f, 2000.0f, "%.0f");
            ImGui::PopItemWidth();
            
            ImGui::Separator();
            ImGui::Spacing();
            
            // Control buttons - larger and more prominent
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(20, 12));  // Bigger buttons
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));  // Green start button
            if (ImGui::Button("START CONTROL", ImVec2(150, 0))) {
                controlActive = true;
                controller.removeAllControls();
                Eigen::Vector3d target(targetX, targetY, targetZ);
                controller.addPositionControl(controlVertex, target, strength, 100.0);
                std::cout << "Controlling vertex " << controlVertex << " to: " << target.transpose() << std::endl;
            }
            ImGui::PopStyleColor();
            
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));  // Red stop button
            if (ImGui::Button("STOP", ImVec2(100, 0))) {
                controlActive = false;
                controller.removeAllControls();
                std::cout << "Stopped control" << std::endl;
            }
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
            
            ImGui::Spacing();
            
            // Action buttons
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(15, 8));
            if (ImGui::Button("Reset Cloth", ImVec2(120, 0))) {
                controlActive = false;
                controller.removeAllControls();
                for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
                    int x = i % 2, z = i / 2;
                    cloth.setVertexPosition(i, Eigen::Vector3d(x * 0.2, clothStartHeight, z * 0.2));
                    cloth.velocities[i] = Eigen::Vector3d::Zero();
                }
                cloth.pinVertex(0);
                std::cout << "Reset cloth" << std::endl;
            }
            
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.6f, 0.2f, 1.0f));  // Orange drop button
            if (ImGui::Button("Drop Cloth", ImVec2(120, 0))) {
                controlActive = false;
                controller.removeAllControls();
                for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
                    int x = i % 2, z = i / 2;
                    cloth.setVertexPosition(i, Eigen::Vector3d(x * 0.2, 0.3, z * 0.2));
                    cloth.velocities[i] = Eigen::Vector3d::Zero();
                }
                cloth.pinVertex(0);
                std::cout << "Dropped cloth from height" << std::endl;
            }
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
            
            ImGui::Separator();
            ImGui::Spacing();
            
            // Status section
            ImGui::Text("=== STATUS ===");
            ImGui::TextColored(controlActive ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(1.0f, 0.2f, 0.2f, 1.0f), 
                              "Control: %s", controlActive ? "ACTIVE" : "STOPPED");
            ImGui::Text("Selected vertex: %d", controlVertex);
            ImGui::Text("Ground height: %.2f", groundHeight);
            
            ImGui::Separator();
            ImGui::Spacing();
            
            // Vertex positions with better formatting
            ImGui::Text("=== VERTEX POSITIONS ===");
            for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
                Eigen::Vector3d pos = cloth.getVertex(i).position;
                const char* status = (i == 0) ? " [PINNED]" : (i == controlVertex && controlActive) ? " [CONTROLLED]" : "";
                
                ImVec4 color = (i == 0) ? ImVec4(1.0f, 0.6f, 0.6f, 1.0f) :  // Pink for pinned
                              (i == controlVertex && controlActive) ? ImVec4(0.6f, 1.0f, 0.6f, 1.0f) :  // Green for controlled
                              ImVec4(1.0f, 1.0f, 1.0f, 1.0f);  // White for others
                
                ImGui::TextColored(color, "V%zu: (%.3f, %.3f, %.3f)%s", i, pos.x(), pos.y(), pos.z(), status);
            }
            
            ImGui::Separator();
            ImGui::Spacing();
            
            // Physics settings with collapsible header
            if (ImGui::CollapsingHeader("Physics Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushItemWidth(200);
                static float stiffness = cloth.properties.stiffness;
                if (ImGui::SliderFloat("Stiffness", &stiffness, 100.0f, 1500.0f, "%.0f")) {
                    cloth.properties.stiffness = stiffness;
                }
                
                static float damping = cloth.properties.damping;
                if (ImGui::SliderFloat("Damping", &damping, 0.3f, 0.95f, "%.2f")) {
                    cloth.properties.damping = damping;
                }
                ImGui::PopItemWidth();
            }
            
            ImGui::PopStyleVar();  // Pop ItemSpacing
        }
        ImGui::End();
    };
    
    std::cout << "\nCloth placed on ground plane!" << std::endl;
    std::cout << "Use GUI to control vertices and watch cloth interact with ground!" << std::endl;
    std::cout << "Try the 'Drop Cloth' button to see gravity in action!" << std::endl;
    
    polyscope::show();
    return 0;
}