#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <Eigen/Core>
#include <vector>
#include <iostream>

// Function to create a cloth grid mesh
std::pair<Eigen::MatrixXd, Eigen::MatrixXi> createClothGrid(int width, int height, double spacing = 1.0) {
    // Create vertices
    Eigen::MatrixXd vertices(width * height, 3);
    int vertex_idx = 0;
    
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            vertices(vertex_idx, 0) = j * spacing;  // x
            vertices(vertex_idx, 1) = 0.0;          // y (flat cloth)
            vertices(vertex_idx, 2) = i * spacing;  // z
            vertex_idx++;
        }
    }
    
    // Create triangular faces
    std::vector<std::array<int, 3>> faces_vec;
    
    for (int i = 0; i < height - 1; ++i) {
        for (int j = 0; j < width - 1; ++j) {
            int top_left = i * width + j;
            int top_right = i * width + (j + 1);
            int bottom_left = (i + 1) * width + j;
            int bottom_right = (i + 1) * width + (j + 1);
            
            // First triangle
            faces_vec.push_back({top_left, bottom_left, top_right});
            // Second triangle
            faces_vec.push_back({top_right, bottom_left, bottom_right});
        }
    }
    
    // Convert to Eigen matrix
    Eigen::MatrixXi faces(faces_vec.size(), 3);
    for (size_t i = 0; i < faces_vec.size(); ++i) {
        faces(i, 0) = faces_vec[i][0];
        faces(i, 1) = faces_vec[i][1];
        faces(i, 2) = faces_vec[i][2];
    }
    
    return {vertices, faces};
}

// Function to add some deformation
Eigen::MatrixXd addWaveDeformation(const Eigen::MatrixXd& vertices, double amplitude = 0.5, double frequency = 2.0) {
    Eigen::MatrixXd deformed_vertices = vertices;
    
    for (int i = 0; i < vertices.rows(); ++i) {
        double x = vertices(i, 0);
        double z = vertices(i, 2);
        
        // Add sinusoidal wave deformation
        deformed_vertices(i, 1) = amplitude * std::sin(frequency * x) * std::cos(frequency * z);
    }
    
    return deformed_vertices;
}

int main() {
    // Initialize Polyscope
    polyscope::init();
    
    // Set some nice defaults
    polyscope::options::autocenterStructures = true;
    polyscope::view::windowWidth = 1024;
    polyscope::view::windowHeight = 768;
    
    // Create cloth grid
    int cloth_width = 20;
    int cloth_height = 20;
    double spacing = 0.1;
    
    auto [vertices, faces] = createClothGrid(cloth_width, cloth_height, spacing);
    
    Eigen::MatrixXd deformed_vertices = addWaveDeformation(vertices, 0.2, 3.0);
    
    polyscope::SurfaceMesh* cloth_mesh = polyscope::registerSurfaceMesh("Cloth Grid", deformed_vertices, faces);
    
    cloth_mesh->setSurfaceColor({0.8, 0.2, 0.2});  // Red-ish color
    cloth_mesh->setEdgeColor({0.1, 0.1, 0.1});     // Dark edges
    cloth_mesh->setEdgeWidth(1.0);
    
    polyscope::SurfaceMesh* flat_cloth = polyscope::registerSurfaceMesh("Flat Cloth", vertices, faces);
    flat_cloth->setSurfaceColor({0.2, 0.8, 0.2});  // Green color
    flat_cloth->setEnabled(false);  // Start with it disabled
    
    cloth_mesh->addVertexScalarQuantity("Y Position", deformed_vertices.col(1));
    
    Eigen::VectorXd height_colors = deformed_vertices.col(1);
    cloth_mesh->addVertexScalarQuantity("Height Color", height_colors);
    
    std::cout << "Cloth mesh created with:" << std::endl;
    std::cout << "  Vertices: " << vertices.rows() << std::endl;
    std::cout << "  Faces: " << faces.rows() << std::endl;
    std::cout << "  Dimensions: " << cloth_width << "x" << cloth_height << std::endl;
    
    polyscope::show();
    
    return 0;
}