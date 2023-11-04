
#include <dbcsan/dbscan_ros.h>
#include <boost/program_options.hpp>


namespace po = boost::program_options;



void argparse(int& argc, char** argv, float& tol, int& corepoints, int& minpoints, int& maxpoints){
    
    po::options_description desc("the dbscan parameters");



    // here you can put your default values like this,
    // po::value<float>()->default_value(YOUR_VALUE)
    desc.add_options()
        ("help", "Should produce help messages")
        ("tol", po::value<float>(),"the euclidean tolerance")
        ("corepoints", po::value<int>(), "the core points neighbourhood")
        ("minpoints", po::value<int>, "the minimum number of points")
        ("maxpoints", po::value<int>, "the maximum number of points");

    po::variables_map vm;

    po::store(po::parse_command_line(argc, argv, desc), vm);




    if (vm.count("help")) ROS_INFO("--tol euclidean_tol --corepoints int_points --minpoints mp --maxpoints np")
   
    if (vm.count("tol"))
        tol=vm["tol"];
    else{
        ROS_INFO("please provide the tolerance value for dbscan");
        return;
    }

    if (vm.count("corepoints"))
        corepoints=vm["corepoints"];
    else{
        ROS_INFO("please provide corepoints value for dbscan");
        return;
    }

    if (vm.count("minpoints"))
        minpoints=vm["minpoints"];
    else{
        ROS_INFO("please provide the minimum points for dbscan")
        return;
    }

    if (vm.count("maxpoints"))
        maxpoints=vm["maxpoints"];
    else{
        ROS_INFO("please provide the maxpoints for dbscan")
    }

}

int main(int argc, char** argv){
    
    float tol; int corepoints; int minpoints; int maxpoints;
    
    // you can remove this call if you want to change the variables
    // yourself
    argparse(argc, argv, tol, corepoints, minpoints, maxpoints);
    
    ros_dbscan ros_dbscan_node;

    return 0;

}
