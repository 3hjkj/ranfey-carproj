#ifndef PERCEPTION_LIDAR_POINTS_TYPES_H_
#define PERCEPTION_LIDAR_POINTS_TYPES_H_

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>

namespace perception
{
    // namespace lidar_cells
    // {
    namespace pcl_util
    {

        typedef pcl::PointIndices PointIndices;
        typedef pcl::PointIndices::Ptr PointIndicesPtr;
        typedef pcl::PointXY Point2d;
        typedef pcl::PointCloud<Point2d> PointCloud2d;
        typedef pcl::PointCloud<Point2d>::Ptr PointCloud2dPtr;
        typedef pcl::PointCloud<Point2d>::ConstPtr PointCloud2dConstPtr;

        struct PointXYZIART;
        // struct PointXYZIRT;

        typedef PointXYZIART Point;
        // typedef PointXYZIRT Point;
        typedef pcl::PointCloud<Point> PointCloud;
        typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
        typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;
        typedef pcl::PointXYZRGB CPoint;
        typedef pcl::PointCloud<CPoint> CPointCloud;
        typedef pcl::PointCloud<CPoint>::Ptr CPointCloudPtr;
        typedef pcl::PointCloud<CPoint>::ConstPtr CPointCloudConstPtr;
        typedef pcl::KdTreeFLANN<Point> KdTree;

        struct Point2i
        {
            Point2i()
                : x(0), y(0) {}

            Point2i(const int _x, const int _y)
                : x(_x), y(_y) {}

            int x;
            int y;
        };

        inline std::ostream &operator<<(std::ostream &os, const Point2i &p)
        {
            os << "(" << p.x << "," << p.y << ")";
            return os;
        }

        // using double type to define x, y, z.
        struct PointD
        {
            double x;
            double y;
            double z;
            std::uint8_t intensity;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } EIGEN_ALIGN16;

        typedef ::pcl::PointCloud<PointD> PointDCloud;
        typedef ::pcl::PointCloud<PointD>::Ptr PointDCloudPtr;
        typedef ::pcl::PointCloud<PointD>::ConstPtr PointDCloudConstPtr;

        struct PointXYZIH
        {
            PCL_ADD_POINT4D;
            float intensity;
            float h; // height from ground

            PointXYZIH()
            {
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
                h = 0.0f;
                intensity = 0.0f;
                data[3] = 1.0f;
            }

            explicit PointXYZIH(float intensity_)
            {
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
                data[3] = 1.0f;
                intensity = intensity_;
            }
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } EIGEN_ALIGN16;

        struct PointXYZIRT
        {
            float x;
            float y;
            float z;
            unsigned char intensity;
            unsigned char ring;
            double timestamp;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
        } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

        struct PointXYZIART
        {
            PCL_ADD_POINT4D
            PCL_ADD_RGB
            float intensity;
            float angle;
            std::uint8_t ring;
            double timestamp;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
        } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

        struct PointXYZIT
        {
            float x;
            float y;
            float z;
            unsigned char intensity;
            double timestamp;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
        } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

        struct RawPointXYZIT
        {
            PCL_ADD_POINT4D
            std::uint8_t intensity;
            double timestamp;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
        } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

        struct PointXYZIRTd
        {
            double x;
            double y;
            double z;
            unsigned char intensity;
            unsigned char ring;
            double timestamp;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
        } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

        struct PointXYZITd
        {
            double x;
            double y;
            double z;
            unsigned char intensity;
            double timestamp;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
        } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

        struct PointXYZRGBRCG
        {
            PCL_ADD_POINT4D
            PCL_ADD_RGB
            std::uint32_t row;
            std::uint32_t column;
            float angle;
            float radius;
            float radial_dis_diff;
            float nearby_diff_r;
            float nearby_diff_z;
            float pre_gradient;
            float next_gradient;
            float up_gradient;
            float down_gradient;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
        } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

    } // namespace pcl_util
    // }     // namespace perception
} // namespace avos

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointD,
                                  (double, x, x)(double, y, y)(double, z, z)(std::uint8_t, intensity, intensity))

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointXYZIH,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, h, h))

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp))
POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::RawPointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(std::uint8_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointXYZIART,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, angle, angle)(std::uint8_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointXYZITd,
                                  (double, x, x)(double, y, y)(double, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointXYZIRTd,
                                  (double, x, x)(double, y, y)(double, z, z)(std::uint8_t, intensity, intensity)(std::uint8_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::pcl_util::PointXYZRGBRCG,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(std::uint32_t, row, row)(std::uint32_t, column, column)(float, angle, angle)(float, radius, radius)(float, radial_dis_diff, radial_dis_diff)(float, nearby_diff_r, nearby_diff_r)(float, nearby_diff_z, nearby_diff_z)(float, pre_gradient, pre_gradient)(float, next_gradient, next_gradient)(float, up_gradient, up_gradient)(float, down_gradient, down_gradient))

#endif // PERCEPTION_PERCEPTION_LIB_PCL_TYPES_H_
