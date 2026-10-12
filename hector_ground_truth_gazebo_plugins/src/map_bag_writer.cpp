#include <filesystem>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <map_bag/map_bag.hpp>
#include <map_bag_msgs/msg/map.hpp>
#include <map_bag_ros/message_conversions/map.hpp>
#include <map_bag/io.hpp>

/*!
 * Writes every map published on ground_truth/elevation_map to its own map bag file.
 *
 * Parameters:
 * * write_path: Directory the files are written to. Created if it does not exist.
 * * file_prefix: Base name of the files; a running index and the .whm extension are appended.
 */
class MapBagWriter : public rclcpp::Node
{
public:
  MapBagWriter() : Node( "map_bag_writer" )
  {
    write_path_ = declare_parameter( "write_path", "." );
    file_prefix_ = declare_parameter( "file_prefix", "ground_truth" );

    subscription_ = create_subscription<map_bag_msgs::msg::Map>(
        "ground_truth/elevation_map", 10,
        std::bind( &MapBagWriter::mapCallback, this, std::placeholders::_1 ) );

    RCLCPP_INFO( get_logger(), "Writing maps received on \"%s\" to \"%s\"",
                 subscription_->get_topic_name(), write_path_.c_str() );
  }

private:
  void mapCallback( const map_bag_msgs::msg::Map &msg )
  {
    std::error_code error;
    std::filesystem::create_directories( write_path_, error );
    if ( error ) {
      RCLCPP_ERROR( get_logger(), "Could not create '%s': %s", write_path_.c_str(),
                    error.message().c_str() );
      return;
    }

    auto map_bag = std::make_shared<map_bag::MapBag>( msg.resolution );
    map_bag::Map::SharedPtr map = map_bag_ros::message_conversions::msgToMap( msg );
    if ( map->frame() != map_bag->frame() )
      map->_setFrame( map_bag->frame() );
    map_bag->lockForWrite()->addMap( map );

    const std::filesystem::path path =
        std::filesystem::path( write_path_ ) /
        ( file_prefix_ + "_" + std::to_string( count_++ ) + ".whm" );
    std::ofstream stream( path, std::ios::binary );
    if ( !stream ) {
      RCLCPP_ERROR( get_logger(), "Could not open '%s' for writing.", path.c_str() );
      return;
    }
    map_bag::io::writeToStream( *map_bag, stream );
    if ( !stream ) {
      RCLCPP_ERROR( get_logger(), "Failed to write '%s'.", path.c_str() );
      return;
    }
    RCLCPP_INFO( get_logger(), "Wrote '%s'", path.c_str() );
  }

  rclcpp::Subscription<map_bag_msgs::msg::Map>::SharedPtr subscription_;
  std::string write_path_;
  std::string file_prefix_;
  size_t count_ = 0;
};

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<MapBagWriter>() );
  rclcpp::shutdown();
  return 0;
}
