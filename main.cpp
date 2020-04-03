#include <iostream>
#include <benchmark/benchmark.h>
#include<pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "voxel_grid_fix.hpp"

using namespace std;

static void OriginalVoxelGrid(benchmark::State& state) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader Reader;
    Reader.read("table_scene.pcd", *cloud);

    for (auto _ : state)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*cloud_filtered);
    }
}
BENCHMARK(OriginalVoxelGrid);

static void FixedVoxelGrid(benchmark::State& state) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader Reader;
    Reader.read("table_scene.pcd", *cloud);

    for (auto _ : state)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGridFixed<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*cloud_filtered);
    }
}
BENCHMARK(FixedVoxelGrid);


BENCHMARK_MAIN();
