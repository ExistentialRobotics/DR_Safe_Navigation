
#ifndef __ERL_GRID_MSG_GENERATOR_
#define __ERL_GRID_MSG_GENERATOR_

//#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <cmath>
#include <string>

namespace erl
{
    std_msgs::ColorRGBA heightMapColor(double h)
    {

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:
                color.r = v;
                color.g = n;
                color.b = m;
                break;
            case 1:
                color.r = n;
                color.g = v;
                color.b = m;
                break;
            case 2:
                color.r = m;
                color.g = v;
                color.b = n;
                break;
            case 3:
                color.r = m;
                color.g = n;
                color.b = v;
                break;
            case 4:
                color.r = n;
                color.g = m;
                color.b = v;
                break;
            case 5:
                color.r = v;
                color.g = m;
                color.b = n;
                break;
            default:
                color.r = 1;
                color.g = 0.5;
                color.b = 0.5;
                break;
        }

        return color;
    }

    class GridMsgGenerator
    {
    public:
      unsigned n_markers = 1;
      /**
       * Initialize the GridMsgGenerator.
       * @param frame_id The frame ID.
       * @param resolution The resolution or size of each grid point.
       * @param type The type of marker Array. Defaults to Cubes.
       * @param r Red value of the color.
       * @param g Green value of the color.
       * @param b Blue value of the color.
       * @param a The alpha of the fill.
       */
        GridMsgGenerator(std::string frame_id,
            const std::vector<double>& resolution, uint32_t type=visualization_msgs::Marker::CUBE_LIST,
            double r=0.0, double g=0.0, double b=1.0, double a=1.0, int depth=1)

            : msg(new visualization_msgs::MarkerArray), markerarray_frame_id(frame_id)
        {
            //pub = nh.advertise<visualization_msgs::MarkerArray>(topic, 1, true);
            n_markers = depth;
            msg->markers.resize(depth); // TODO Determine why there were '10' markers in this list before???
            for (int i = 0; i < depth; ++i)
            {
                msg->markers[i].header.frame_id = markerarray_frame_id;
                msg->markers[i].ns = "grid_visualization";
                msg->markers[i].id = i;
                msg->markers[i].type = type;
                msg->markers[i].scale.x = resolution[0] * pow(2, i);
                msg->markers[i].scale.y = resolution[1] * pow(2, i);
                msg->markers[i].scale.z = resolution[2] * pow(2, i);

                msg->markers[i].pose.orientation.x = 0.0;
                msg->markers[i].pose.orientation.y = 0.0;
                msg->markers[i].pose.orientation.z = 0.0;
                msg->markers[i].pose.orientation.w = 1.0;
                std_msgs::ColorRGBA color;
                color.r = r;
                color.g = g;
                color.b = b;
                color.a = a;
                msg->markers[i].color = color;
            }
        }

        void insert_point3d(float x, float y, float z, float min_z, float max_z, float size)
        {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            if (size > 0)
                depth = (int) log2(size / 0.1);
            msg->markers[depth].points.push_back(center);

            if (min_z < max_z) {
                double h = (1.0 - std::min(std::max((z - min_z) / (max_z - min_z), 0.0f), 1.0f)) * 0.8;
                msg->markers[depth].colors.push_back(heightMapColor(h));
            }
        }

        void insert_point3d(float x, float y, float z, float min_z, float max_z)
        { insert_point3d(x, y, z, min_z, max_z, -1.0f); }

        void insert_point3d(float x, float y, float z)
        { insert_point3d(x, y, z, 1.0f, 0.0f, -1.0f); }

        void insert_color_point3d(int x, int y, int z, double min_v, double max_v, double v)
        {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            msg->markers[depth].points.push_back(center);


            double h = (1.0 - std::min(std::max((v - min_v) / (max_v - min_v), 0.0), 1.0)) * 0.8;
            msg->markers[depth].colors.push_back(heightMapColor(h));
        }

        void clear()
        {
            for (int i = 0; i < n_markers; ++i) {
                msg->markers[i].points.clear();
                msg->markers[i].colors.clear();
            }
        }

        visualization_msgs::MarkerArray::Ptr get_msg() const
        {
          msg->markers[0].header.stamp = ros::Time::now();
          for (int i = 1; i < n_markers; ++i)
            msg->markers[i].header.stamp = msg->markers[0].header.stamp;
          return msg;
        }

        /*
        void publish() const
        {
          msg->markers[0].header.stamp = ros::Time::now();
          pub.publish(*msg);
          //ros::spinOnce();
        }
        */

    private:
        //ros::NodeHandle nh;
        //ros::Publisher pub;
        visualization_msgs::MarkerArray::Ptr msg;
        std::string markerarray_frame_id;
        //std::string topic;
        //float resolution;
        //std::vector<double> resolution;
    };
}
#endif

