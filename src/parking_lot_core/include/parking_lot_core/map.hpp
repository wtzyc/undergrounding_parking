#pragma once

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace parking_lot_core
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  std::optional<double> yaw;
};

struct Neighbor
{
  std::string neighbor_id;
  Pose2D relative_pose;
  std::string direction;
  double cost{0.0};
};

struct LandmarkNode
{
  std::string id;
  std::string type;
  Pose2D pose;
  std::optional<std::string> zone;
  std::optional<int> row;
  std::optional<int> column;
  std::map<std::string, double> geometry;
  std::optional<Pose2D> entrance_pose;
  double confidence{0.0};
  int observation_count{0};
  std::string status{"candidate"};
  std::optional<std::string> source;
};

struct RoadNode
{
  std::string id;
  std::string node_type;
  Pose2D pose;
  std::optional<std::string> anchor_id;
  std::vector<Neighbor> neighbors;
  double confidence{0.0};
  std::optional<std::string> source;
};

struct Edge
{
  std::string id;
  std::string from_id;
  std::string to_id;
  Pose2D relative_pose;
  std::string direction;
  double cost{0.0};
};

struct MapMetadata
{
  std::string version{"1.0"};
  std::string frame_id{"map"};
  std::string vehicle_model_name{"my_orange_car"};
  std::optional<std::string> current_area;
  std::map<std::string, std::string> parameters;
  std::map<std::string, std::string> evaluation_metadata;
  std::map<std::string, std::string> extra;
};

struct ParkingMap
{
  MapMetadata metadata;
  std::vector<LandmarkNode> landmarks;
  std::vector<RoadNode> road_nodes;
  std::vector<Edge> edges;
};

class MapValidationError : public std::runtime_error
{
public:
  explicit MapValidationError(const std::string & message)
: std::runtime_error(message) {}
};

void validate_map(const ParkingMap & parking_map);

ParkingMap load_map(const std::string & path);

std::string serialize_map(const ParkingMap & parking_map);

void save_map(const ParkingMap & parking_map, const std::string & path);

}  // namespace parking_lot_core
