#include "open3d/Open3D.h"

#include <iostream>

#include <chrono>

#include "spdlog/spdlog.h"

std::shared_ptr<open3d::geometry::PointCloud> remove_outliers(std::shared_ptr<open3d::geometry::PointCloud> pointCloud){
    
    if (!pointCloud) {
        return nullptr;
    }

    //Apply statistical outlier removal
    auto [filteredCloud, ind] = pointCloud->RemoveStatisticalOutliers(1000, 1.0);

    return filteredCloud;

}

void test() {
    std::string inputFile = "1.ply";   // Input PLY file
    std::string outputFile = "out.ply";  // Output PLY file

    // auto cloud = remove_outliers(inputFile, outputFile);

    auto cloud = open3d::io::CreatePointCloudFromFile(inputFile);
    if (!cloud) {
        std::cerr << "Failed to load point cloud from " << inputFile << std::endl;
        return;
    }

    std::cout << "Loaded point cloud with " << cloud->points_.size() << " points." << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    auto [result, size] = cloud->SegmentPlane(1, 3, 1000);

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end - start;

    std::cout << std::format("duration = {}\n", duration.count());
    
    // Extract inliers (points that belong to the plane)
    auto inliers = cloud->SelectByIndex(size);

    std::cout << "Number of inlier points: " << inliers->points_.size() << std::endl;

    // Save the inlier points (plane points)
    if (!open3d::io::WritePointCloud(outputFile, *inliers)) {
        std::cerr << "Failed to save inlier point cloud to " << outputFile << std::endl;
        return;
    }

    std::cout << "Inlier point cloud saved to " << outputFile << std::endl;

    // Optionally visualize the results

    auto cloudPlane = open3d::io::CreatePointCloudFromFile("out.ply");

    

    // cloudPlane->GetRotationMatrixFromAxisAngle();

    // fitCircle(cloudPlane);

    // cloud

    open3d::visualization::DrawGeometries({ cloudPlane}, "Point Cloud and Plane");
}

void fitCircle(std::shared_ptr<open3d::geometry::PointCloud> cloud){
    std::vector<Eigen::Vector2d> points2D;
    for (const auto& point : cloud->points_) {
        std::cout << std::format("coords x = {}, y = {}, z = {}\n", point.x(), point.y(), point.z());
        points2D.emplace_back(point.x(), point.y());
    }
}

void printPoints(std::shared_ptr<open3d::geometry::PointCloud> cloud) {
    for(const auto& point : cloud->points_){
        std::cout << std::format("x = {}, y = {}, z = {}\n", point.x(), point.y(), point.z());
    }

}

std::optional<Eigen::Vector3d> findCenter(std::shared_ptr<open3d::geometry::PointCloud> cloud){
    
    if (cloud->points_.empty()) {
        // std::cerr << "The point cloud is empty!" << std::endl;
        spdlog::error("The point cloud is empty");
        return std::nullopt;
    }

    // Calculate the center of mass (centroid)
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& point : cloud->points_) {
        centroid += point;
    }
    centroid /= cloud->points_.size();

    // Output the centroid
    // std::cout << "Center of Mass (Centroid): " << centroid.transpose() << std::endl;

    return centroid;
}

void writeToFile(std::shared_ptr<open3d::geometry::PointCloud> cloud, const std::string& fileName){
    if (cloud->points_.empty()) {
        std::cerr << "The point cloud is empty!" << std::endl;
        return;
    }

    std::ofstream f(fileName, std::ios::out);

    if(!f){
        spdlog::error("Couldn't open file");
        return;
    }

    if(!cloud->points_.size()){
        spdlog::error("Zero points in cloud");
        return;
    }

    for(const auto& p : cloud->points_){
        f << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
    }

    f.close();
}   


std::shared_ptr<open3d::geometry::PointCloud> rotate(std::shared_ptr<open3d::geometry::PointCloud> cloud){
    auto point_cloud = cloud;
    // Fit a plane to the point cloud
    auto [plane_model, inliers] = point_cloud->SegmentPlane(0.01, 3, 1000);
    // auto new_cloud2 = point_cloud->SelectByIndex(inliers);
    // printPoints(new_cloud2);

    // The normal of the plane is the first three elements of plane_model (a, b, c)
    Eigen::Vector3d normal = plane_model.head<3>();

    // Define the target normal (Z-axis)
    Eigen::Vector3d target_normal(0, 0, 1);

    // Compute the rotation between the current normal and the Z-axis
    Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(normal, target_normal);

    // Convert the quaternion to a rotation matrix
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

    // Apply the rotation to the point cloud
    point_cloud->Rotate(rotation_matrix, findCenter(point_cloud).value());

    return point_cloud;

    
}

std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, Eigen::Vector3d> processCloud(std::shared_ptr<open3d::geometry::PointCloud> cloud, double threshold = 0.85) {
    
    auto [plane_model2, inliers2] = cloud->SegmentPlane(threshold, 3, 1000);

    auto new_cloud = cloud->SelectByIndex(inliers2);

    new_cloud = remove_outliers(new_cloud);

    auto center = findCenter(new_cloud);

    if(!center.has_value()){
        spdlog::error("no value");
        return std::make_tuple(nullptr, Eigen::Vector3d());
    }
    
    auto p = center.value();

    // printPoints(new_cloud);

    // spdlog::info("x = {}, y = {}, z = {}", p.x(), p.y(), p.z());

    return std::make_tuple(new_cloud, p);
    // Visualize the result
    // open3d::visualization::DrawGeometries({new_cloud}, "Aligned Point Cloud");
    

}

double distance(const Eigen::Vector3d& center, const Eigen::Vector3d& p){
    return std::sqrt(std::pow(center.x() - p.x(), 2) + std::pow(center.y() - p.y(), 2));
}

int calculatePoints(std::shared_ptr<open3d::geometry::PointCloud> cloud, const Eigen::Vector3d& center, double radius){
    
    int amount = 0;

    for(const auto& p : cloud->points_){
        if(distance(center, p) <= radius){
            ++amount;
        }
    }

    return amount;
}

int main(int argc, char **argv) {
    try{
    
        std::string inputFile = "1.ply";   // Input PLY file
        std::string outputFile = "out.ply";  // Output PLY file
        double time = 0;
        double rad = 6.5;
        auto cloud = open3d::io::CreatePointCloudFromFile(inputFile);

        for(int i = 0; i < 50; ++i){
            auto start = std::chrono::high_resolution_clock::now();

            auto rotatedCloud = rotate(cloud);

            auto [flatCloud, center] = processCloud(rotatedCloud, 1);
            auto [maxCloud, center2] = processCloud(rotatedCloud, 0.25);


            double amCur = calculatePoints(maxCloud, center, rad);
            double amBest = calculatePoints(flatCloud, center, rad);

            auto end = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> duration = end - start;
            time += duration.count();   
        }
        
        spdlog::info("time = {}", time / 50);
        //double ratio = amCur / amBest;

        // spdlog::info("{} {}", amCur, amBest);
        // spdlog::info("ratio = {}", ratio);
    }
    catch(const std::exception& exp){
        spdlog::error("{}", exp.what());
    }
    // open3d::io::WritePointCloud(outputFile, *rotatedCloud);

    // writeToFile(pCloud, "../data/points.txt");
    // open3d::io::WritePointCloud("../data/out.ply", *pCloud);

}