#include "cloth.h"
#include "integrator.h"
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <iostream>
#include <memory>
#include <chrono>

using namespace ClothOpt;

int main() {
    // Initialize Polyscope with PERFORMANCE SETTINGS
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::init();
    
    // Set up camera and view
    polyscope::view::setUpDir(polyscope::UpDir::YUp);
    polyscope::view::setFrontDir(polyscope::FrontDir::ZFront);
    polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Turntable);
    
    // Create cloth - REDUCE RESOLUTION FOR FASTER RENDERING
    ClothMesh cloth;
    cloth.createGrid(8, 8, 0.125);  // Grid spans (0,0,0) to (0.875, 0, 0.875)
    
    // Calculate cloth center for proper sphere positioning
    double clothWidth = 7 * 0.125;  // 7 intervals * spacing
    double clothCenter = clothWidth / 2.0;  // Center of cloth
    
    std::cout << "Cloth spans: (0,0,0) to (" << clothWidth << ",0," << clothWidth << ")" << std::endl;
    std::cout << "Cloth center X,Z: " << clothCenter << std::endl;
    
    // Start above ground
    for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
        Eigen::Vector3d pos = cloth.getVertex(i).position;
        pos.y() = 0.8;  // Higher start
        cloth.setVertexPosition(i, pos);
    }
    
    // Pin corners
    cloth.pinVertex(cloth.getGridIndex(0, 0));
    cloth.pinVertex(cloth.getGridIndex(0, 7));  // Updated for 8x8 grid
    
    // Properties
    cloth.properties.stiffness = 2000.0;
    cloth.properties.bendingStiffness = 200.0;
    cloth.properties.damping = 0.98;
    cloth.properties.friction = 0.7;
    cloth.properties.gravity = Eigen::Vector3d(0, -9.81, 0);
    
    // Collision sphere - PROPERLY CENTERED IN CLOTH
    Eigen::Vector3d sphereCenter(clothCenter, 0.5, clothCenter);  // Center in cloth coordinates
    double sphereRadius = 0.12;  // Slightly smaller radius
    cloth.addSphere(sphereCenter, sphereRadius);
    
    // Create integrator - OPTIMIZED SETTINGS
    auto integrator = std::make_unique<SemiImplicitEulerIntegrator>();
    integrator->enableDebug(false);  // Enable debug to see collisions
    integrator->setDebugFrequency(30);  // Debug every 30 frames
    
    std::cout << "PERFORMANCE TEST - " << cloth.getVertexCount() << " vertices" << std::endl;
    std::cout << "Sphere center: " << sphereCenter.transpose() << std::endl;
    std::cout << "Sphere radius: " << sphereRadius << std::endl;
    std::cout << "Sphere top: y = " << (sphereCenter.y() + sphereRadius) << std::endl;
    std::cout << "Sphere bottom: y = " << (sphereCenter.y() - sphereRadius) << std::endl;
    
    // Visualization - OPTIMIZED FOR PERFORMANCE
    auto* psMesh = polyscope::registerSurfaceMesh("Cloth", cloth.getVertexMatrix(), cloth.getTriangleMatrix());
    psMesh->setSurfaceColor({0.2, 0.6, 1.0});
    psMesh->setEdgeWidth(0.5);  // Thinner edges for faster rendering
    psMesh->setMaterial("wax");  // Simpler material
    
    // Simplified ground - MATCH CLOTH DIMENSIONS
    double groundExtent = clothWidth + 0.3;  // Slightly larger than cloth
    std::vector<Eigen::Vector3d> groundVertices = {
        {-0.15, 0.0, -0.15}, 
        {groundExtent, 0.0, -0.15}, 
        {groundExtent, 0.0, groundExtent}, 
        {-0.15, 0.0, groundExtent}
    };
    std::vector<std::array<int, 3>> groundTriangles = {{0, 1, 2}, {0, 2, 3}};
    auto* psGround = polyscope::registerSurfaceMesh("Ground", groundVertices, groundTriangles);
    psGround->setSurfaceColor({0.8, 0.8, 0.8});
    psGround->setMaterial("flat");  // Flat material for ground
    
    // Sphere - ENSURE EXACT POSITIONING MATCH
    std::vector<Eigen::Vector3d> sphereCenters = {sphereCenter};
    auto* psSphere = polyscope::registerPointCloud("Collision Sphere", sphereCenters);
    psSphere->setPointRadius(sphereRadius/2.0);
    psSphere->setPointColor({1.0, 0.2, 0.2});  // Bright red
    psSphere->setPointRenderMode(polyscope::PointRenderMode::Sphere);
    
    // Set camera position to see everything
    polyscope::view::lookAt(
        {clothCenter + 0.5, 0.6, clothCenter + 0.5},  // Camera position
        {clothCenter, 0.2, clothCenter},              // Look at center
        {0.0, 1.0, 0.0}                               // Up direction
    );
    
    // Performance tracking
    auto lastTime = std::chrono::high_resolution_clock::now();
    int frameCount = 0;
    double totalSimTime = 0.0;
    double totalRenderTime = 0.0;
    
    // Simulation loop - OPTIMIZED UPDATE FREQUENCY
    double dt = 0.003;  // Slightly larger timestep
    int renderSkip = 0;  // Skip rendering some frames
    
    polyscope::state::userCallback = [&]() {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Run simulation steps - MORE STEPS PER FRAME
        for (int i = 0; i < 8; ++i) {  // 8 substeps per frame for smoother simulation
            integrator->step(cloth, dt);
        }
        
        auto simEndTime = std::chrono::high_resolution_clock::now();
        auto simDuration = std::chrono::duration_cast<std::chrono::microseconds>(simEndTime - startTime);
        totalSimTime += simDuration.count() / 1000.0;
        
        // SKIP SOME RENDER UPDATES FOR PERFORMANCE
        renderSkip++;
        if (renderSkip >= 2) {  // Only update render every 2 frames
            psMesh->updateVertexPositions(cloth.getVertexMatrix());
            renderSkip = 0;
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        auto totalDuration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        totalRenderTime += totalDuration.count() / 1000.0;
        
        frameCount++;
        if (frameCount % 60 == 0) {
            double avgSimTime = totalSimTime / frameCount;
            double avgTotalTime = totalRenderTime / frameCount;
            double avgRenderTime = avgTotalTime - avgSimTime;
            
            std::cout << "Frame " << frameCount << std::endl;
            std::cout << "  Sim time: " << std::fixed << std::setprecision(2) << avgSimTime << "ms" << std::endl;
            std::cout << "  Render time: " << avgRenderTime << "ms" << std::endl;
            std::cout << "  Total time: " << avgTotalTime << "ms" << std::endl;
            std::cout << "  Est. FPS: " << (int)(1000.0 / avgTotalTime) << std::endl;
            
            // Check cloth height and sphere interaction
            double avgHeight = 0;
            int nearSphere = 0;
            for (size_t i = 0; i < cloth.getVertexCount(); ++i) {
                Eigen::Vector3d pos = cloth.getVertex(i).position;
                avgHeight += pos.y();
                
                // Check distance to sphere
                double distToSphere = (pos - sphereCenter).norm();
                if (distToSphere <= sphereRadius + 0.02) {  // Within 2cm
                    nearSphere++;
                }
            }
            avgHeight /= cloth.getVertexCount();
            std::cout << "  Average height: " << avgHeight << std::endl;
            std::cout << "  Vertices near sphere: " << nearSphere << std::endl;
            
            // Performance breakdown
            double simPercent = (avgSimTime / avgTotalTime) * 100;
            double renderPercent = (avgRenderTime / avgTotalTime) * 100;
            std::cout << "  Performance: " << (int)simPercent << "% sim, " 
                      << (int)renderPercent << "% render" << std::endl << std::endl;
        }
    };
    
    std::cout << "\nStarting RENDER-OPTIMIZED simulation..." << std::endl;
    std::cout << "Optimizations:" << std::endl;
    std::cout << "- Cloth grid: 8x8, centered at (" << clothCenter << ", 0, " << clothCenter << ")" << std::endl;
    std::cout << "- Sphere: center (" << sphereCenter.transpose() << "), radius " << sphereRadius << std::endl;
    std::cout << "- Yellow dots show sphere boundary for verification" << std::endl;
    std::cout << "- Debug enabled to track collisions" << std::endl;
    std::cout << "Watch performance breakdown in console!" << std::endl;
    
    polyscope::show();
    return 0;
}