#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <opencv4/opencv2/opencv.hpp>
// #include <opencv4/opencv2/quality.hpp>
#include <GL/gl.h>
#include <GL/glu.h>


#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/quality/qualityssim.hpp>

#include <iostream>
#include <filesystem>

// Function to load a PLY file into a point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& filePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(filePath, *cloud) == -1) {
        PCL_ERROR("Couldn't read the file\n");
        return nullptr;
    }
    return cloud;
}

// Function to set the camera view
void setCameraView(pcl::visualization::PCLVisualizer::Ptr viewer, 
                   const std::vector<float>& pos, 
                   const std::vector<float>& view, 
                   const std::vector<float>& up) {
    viewer->setCameraPosition(pos[0], pos[1], pos[2],
                              view[0], view[1], view[2],
                              up[0], up[1], up[2]);
}

// Function to render the point cloud and capture the image
void renderPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                      const std::vector<float>& cameraPosition, 
                      const cv::Size& imageSize,
                      cv::Mat& image) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addPointCloud(cloud);

    setCameraView(viewer, cameraPosition, {0, 0, 0}, {0, 1, 0});

    viewer->setSize(imageSize.width, imageSize.height);
    viewer->setBackgroundColor(1, 1, 1);

    viewer->spinOnce(10, true);

    viewer->saveScreenshot("point_cloud_render.png");

    image = cv::imread("point_cloud_render.png");
    // cv::remove("point_cloud_render.png");
}


// Function to calculate the SSIM between two images
double calculateSSIM(const cv::Mat& img1, const cv::Mat& img2) {
    cv::Mat grayA, grayB;
    cv::cvtColor(img1, grayA, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, grayB, cv::COLOR_BGR2GRAY);
    
    std::vector<double> ssim;
    auto mssim = cv::quality::QualitySSIM::compute(grayA, grayB, ssim);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_ply_file1> <path_to_ply_file2>\n";
        return -1;
    }

    std::string filePath1 = argv[1];
    std::string filePath2 = argv[2];

    // Load the point cloud files
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = loadPointCloud(filePath1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = loadPointCloud(filePath2);

    if (!cloud1 || !cloud2) {
        std::cerr << "Error loading point clouds\n";
        return -1;
    }

    // Define camera position (pos, view, up)
    std::vector<float> cameraPosition = {0, 0, 10};

    // Render the point clouds and capture images
    cv::Mat image1, image2;
    renderPointCloud(cloud1, cameraPosition, {640, 480}, image1);
    renderPointCloud(cloud2, cameraPosition, {640, 480}, image2);

    // Calculate SSIM between the images
    double ssimScore = calculateSSIM(image1, image2);

    std::cout << "SSIM Score: " << ssimScore << std::endl;

    // Display the images
    cv::imshow("Point Cloud 1", image1);
    cv::imshow("Point Cloud 2", image2);
    cv::waitKey(0);

    return 0;
}
