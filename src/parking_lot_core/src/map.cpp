#include "parking_lot_core/map.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

namespace parking_lot_core
{
namespace
{

using JsonDocument = rapidjson::Document;
using JsonValue = rapidjson::Value;
using Allocator = JsonDocument::AllocatorType;

const std::set<std::string> kLandmarkTypes{"parking_slot", "pillar"};
const std::set<std::string> kRoadNodeTypes{"road", "parking_anchor", "pillar_anchor"};
const std::set<std::string> kStatuses{"candidate", "confirmed"};

void require_member(const JsonValue & object, const char * name, const char * context)
{
  if (!object.IsObject() || !object.HasMember(name)) {
    throw MapValidationError(std::string(context) + " is missing: " + name);
  }
}

std::string read_string(const JsonValue & object, const char * name, const char * context)
{
  require_member(object, name, context);
  const auto & value = object[name];
  if (!value.IsString()) {
    throw MapValidationError(std::string(context) + "." + name + " must be a string");
  }
  return value.GetString();
}

double read_number(const JsonValue & object, const char * name, const char * context)
{
  require_member(object, name, context);
  const auto & value = object[name];
  if (!value.IsNumber()) {
    throw MapValidationError(std::string(context) + "." + name + " must be a number");
  }
  return value.GetDouble();
}

void add_string(JsonValue & object, const char * name, const std::string & value, Allocator & allocator)
{
  JsonValue key(name, allocator);
  JsonValue json_value(value.c_str(), allocator);
  object.AddMember(key, json_value, allocator);
}

void add_number(JsonValue & object, const char * name, double value, Allocator & allocator)
{
  JsonValue key(name, allocator);
  JsonValue json_value(value);
  object.AddMember(key, json_value, allocator);
}

void add_optional_string(
  JsonValue & object, const char * name, const std::optional<std::string> & value,
  Allocator & allocator)
{
  if (value.has_value()) {
    add_string(object, name, *value, allocator);
  } else {
    object.AddMember(JsonValue(name, allocator), JsonValue(rapidjson::kNullType), allocator);
  }
}

void add_optional_number(
  JsonValue & object, const char * name, const std::optional<double> & value,
  Allocator & allocator)
{
  if (value.has_value()) {
    add_number(object, name, *value, allocator);
  } else {
    object.AddMember(JsonValue(name, allocator), JsonValue(rapidjson::kNullType), allocator);
  }
}

JsonValue pose_to_json(const Pose2D & pose, Allocator & allocator)
{
  JsonValue object(rapidjson::kObjectType);
  add_number(object, "x", pose.x, allocator);
  add_number(object, "y", pose.y, allocator);
  add_optional_number(object, "yaw", pose.yaw, allocator);
  return object;
}

Pose2D pose_from_json(const JsonValue & object, const std::string & context)
{
  if (!object.IsObject()) {
    throw MapValidationError(context + " must be an object");
  }
  return Pose2D{
    read_number(object, "x", context.c_str()),
    read_number(object, "y", context.c_str()),
    object.HasMember("yaw") && !object["yaw"].IsNull()
      ? std::optional<double>(read_number(object, "yaw", context.c_str()))
      : std::nullopt,
  };
}

std::map<std::string, double> geometry_from_json(const JsonValue & object, const std::string & context)
{
  if (!object.IsObject()) {
    throw MapValidationError(context + " must be an object");
  }
  std::map<std::string, double> result;
  for (auto it = object.MemberBegin(); it != object.MemberEnd(); ++it) {
    if (!it->name.IsString() || !it->value.IsNumber()) {
      throw MapValidationError(context + " values must be numeric");
    }
    result.emplace(it->name.GetString(), it->value.GetDouble());
  }
  return result;
}

std::map<std::string, std::string> string_map_from_json(
  const JsonValue & object, const std::string & context)
{
  if (!object.IsObject()) {
    throw MapValidationError(context + " must be an object");
  }
  std::map<std::string, std::string> result;
  for (auto it = object.MemberBegin(); it != object.MemberEnd(); ++it) {
    if (!it->name.IsString() || !it->value.IsString()) {
      throw MapValidationError(context + " values must be strings");
    }
    result.emplace(it->name.GetString(), it->value.GetString());
  }
  return result;
}

void add_string_map(
  JsonValue & object, const char * name, const std::map<std::string, std::string> & values,
  Allocator & allocator)
{
  JsonValue map_object(rapidjson::kObjectType);
  for (const auto & [key, value] : values) {
    add_string(map_object, key.c_str(), value, allocator);
  }
  object.AddMember(JsonValue(name, allocator), map_object, allocator);
}

Neighbor neighbor_from_json(const JsonValue & object, const std::string & context)
{
  if (!object.IsObject()) {
    throw MapValidationError(context + " must be an object");
  }
  return Neighbor{
    read_string(object, "neighbor_id", context.c_str()),
    pose_from_json(object["relative_pose"], context + ".relative_pose"),
    read_string(object, "direction", context.c_str()),
    read_number(object, "cost", context.c_str()),
  };
}

JsonValue neighbor_to_json(const Neighbor & neighbor, Allocator & allocator)
{
  JsonValue object(rapidjson::kObjectType);
  add_string(object, "neighbor_id", neighbor.neighbor_id, allocator);
  object.AddMember(
    JsonValue("relative_pose", allocator), pose_to_json(neighbor.relative_pose, allocator), allocator);
  add_string(object, "direction", neighbor.direction, allocator);
  add_number(object, "cost", neighbor.cost, allocator);
  return object;
}

LandmarkNode landmark_from_json(const JsonValue & object, std::size_t index)
{
  const std::string context = "landmark[" + std::to_string(index) + "]";
  if (!object.IsObject()) {
    throw MapValidationError(context + " must be an object");
  }
  LandmarkNode result;
  result.id = read_string(object, "id", context.c_str());
  result.type = read_string(object, "type", context.c_str());
  result.pose = pose_from_json(object["pose"], context + ".pose");
  if (object.HasMember("zone") && !object["zone"].IsNull()) {
    result.zone = read_string(object, "zone", context.c_str());
  }
  if (object.HasMember("row") && !object["row"].IsNull()) {
    if (!object["row"].IsInt()) throw MapValidationError(context + ".row must be an integer");
    result.row = object["row"].GetInt();
  }
  if (object.HasMember("column") && !object["column"].IsNull()) {
    if (!object["column"].IsInt()) throw MapValidationError(context + ".column must be an integer");
    result.column = object["column"].GetInt();
  }
  result.geometry = object.HasMember("geometry")
    ? geometry_from_json(object["geometry"], context + ".geometry")
    : std::map<std::string, double>{};
  if (object.HasMember("entrance_pose") && !object["entrance_pose"].IsNull()) {
    result.entrance_pose = pose_from_json(object["entrance_pose"], context + ".entrance_pose");
  }
  result.confidence = object.HasMember("confidence") ? read_number(object, "confidence", context.c_str()) : 0.0;
  if (object.HasMember("observation_count")) {
    if (!object["observation_count"].IsInt()) {
      throw MapValidationError(context + ".observation_count must be an integer");
    }
    result.observation_count = object["observation_count"].GetInt();
  }
  result.status = object.HasMember("status") ? read_string(object, "status", context.c_str()) : "candidate";
  if (object.HasMember("source") && !object["source"].IsNull()) {
    result.source = read_string(object, "source", context.c_str());
  }
  return result;
}

JsonValue landmark_to_json(const LandmarkNode & landmark, Allocator & allocator)
{
  JsonValue object(rapidjson::kObjectType);
  add_string(object, "id", landmark.id, allocator);
  add_string(object, "type", landmark.type, allocator);
  add_optional_string(object, "zone", landmark.zone, allocator);
  if (landmark.row.has_value()) object.AddMember("row", *landmark.row, allocator);
  else object.AddMember("row", JsonValue(rapidjson::kNullType), allocator);
  if (landmark.column.has_value()) object.AddMember("column", *landmark.column, allocator);
  else object.AddMember("column", JsonValue(rapidjson::kNullType), allocator);
  object.AddMember("pose", pose_to_json(landmark.pose, allocator), allocator);
  JsonValue geometry(rapidjson::kObjectType);
  for (const auto & [key, value] : landmark.geometry) add_number(geometry, key.c_str(), value, allocator);
  object.AddMember("geometry", geometry, allocator);
  if (landmark.entrance_pose.has_value()) {
    object.AddMember("entrance_pose", pose_to_json(*landmark.entrance_pose, allocator), allocator);
  } else {
    object.AddMember("entrance_pose", JsonValue(rapidjson::kNullType), allocator);
  }
  add_number(object, "confidence", landmark.confidence, allocator);
  object.AddMember("observation_count", landmark.observation_count, allocator);
  add_string(object, "status", landmark.status, allocator);
  add_optional_string(object, "source", landmark.source, allocator);
  return object;
}

RoadNode road_node_from_json(const JsonValue & object, std::size_t index)
{
  const std::string context = "road_node[" + std::to_string(index) + "]";
  if (!object.IsObject()) throw MapValidationError(context + " must be an object");
  RoadNode result;
  result.id = read_string(object, "id", context.c_str());
  result.node_type = read_string(object, "node_type", context.c_str());
  result.pose = pose_from_json(object["pose"], context + ".pose");
  if (object.HasMember("anchor_id") && !object["anchor_id"].IsNull()) {
    result.anchor_id = read_string(object, "anchor_id", context.c_str());
  }
  if (object.HasMember("neighbors")) {
    if (!object["neighbors"].IsArray()) throw MapValidationError(context + ".neighbors must be an array");
    for (rapidjson::SizeType i = 0; i < object["neighbors"].Size(); ++i) {
      result.neighbors.push_back(neighbor_from_json(object["neighbors"][i], context + ".neighbors"));
    }
  }
  result.confidence = object.HasMember("confidence") ? read_number(object, "confidence", context.c_str()) : 0.0;
  if (object.HasMember("source") && !object["source"].IsNull()) {
    result.source = read_string(object, "source", context.c_str());
  }
  return result;
}

JsonValue road_node_to_json(const RoadNode & node, Allocator & allocator)
{
  JsonValue object(rapidjson::kObjectType);
  add_string(object, "id", node.id, allocator);
  add_string(object, "node_type", node.node_type, allocator);
  add_optional_string(object, "anchor_id", node.anchor_id, allocator);
  object.AddMember("pose", pose_to_json(node.pose, allocator), allocator);
  JsonValue neighbors(rapidjson::kArrayType);
  for (const auto & neighbor : node.neighbors) neighbors.PushBack(neighbor_to_json(neighbor, allocator), allocator);
  object.AddMember("neighbors", neighbors, allocator);
  add_number(object, "confidence", node.confidence, allocator);
  add_optional_string(object, "source", node.source, allocator);
  return object;
}

Edge edge_from_json(const JsonValue & object, std::size_t index)
{
  const std::string context = "edge[" + std::to_string(index) + "]";
  if (!object.IsObject()) throw MapValidationError(context + " must be an object");
  return Edge{
    read_string(object, "id", context.c_str()),
    read_string(object, "from_id", context.c_str()),
    read_string(object, "to_id", context.c_str()),
    pose_from_json(object["relative_pose"], context + ".relative_pose"),
    read_string(object, "direction", context.c_str()),
    read_number(object, "cost", context.c_str()),
  };
}

JsonValue edge_to_json(const Edge & edge, Allocator & allocator)
{
  JsonValue object(rapidjson::kObjectType);
  add_string(object, "id", edge.id, allocator);
  add_string(object, "from_id", edge.from_id, allocator);
  add_string(object, "to_id", edge.to_id, allocator);
  object.AddMember("relative_pose", pose_to_json(edge.relative_pose, allocator), allocator);
  add_string(object, "direction", edge.direction, allocator);
  add_number(object, "cost", edge.cost, allocator);
  return object;
}

void validate_pose(const Pose2D & pose, const std::string & name)
{
  if (!std::isfinite(pose.x)) throw MapValidationError(name + ".x must be finite");
  if (!std::isfinite(pose.y)) throw MapValidationError(name + ".y must be finite");
  if (pose.yaw.has_value() && !std::isfinite(*pose.yaw)) {
    throw MapValidationError(name + ".yaw must be finite");
  }
}

void validate_confidence(double value, const std::string & name)
{
  if (!std::isfinite(value)) throw MapValidationError(name + " must be finite");
  if (value < 0.0 || value > 1.0) throw MapValidationError(name + " must be between 0 and 1");
}

void validate_nonnegative(double value, const std::string & name)
{
  if (!std::isfinite(value)) throw MapValidationError(name + " must be finite");
  if (value < 0.0) throw MapValidationError(name + " must be non-negative");
}

JsonDocument read_document(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) throw MapValidationError("map file does not exist: " + path);
  rapidjson::IStreamWrapper wrapper(stream);
  JsonDocument document;
  document.ParseStream(wrapper);
  if (document.HasParseError()) {
    throw MapValidationError(
      "map file is not valid JSON: " + std::string(GetParseError_En(document.GetParseError())));
  }
  if (!document.IsObject()) throw MapValidationError("map JSON root must be an object");
  return document;
}

bool is_legacy_map(const JsonValue & object)
{
  return object.HasMember("areas") || object.HasMember("transitions") ||
         object.HasMember("visits") || object.HasMember("telemetry");
}

ParkingMap parse_document(const JsonValue & object)
{
  if (is_legacy_map(object)) {
    ParkingMap result;
    if (object.HasMember("version") && object["version"].IsString()) result.metadata.version = object["version"].GetString();
    if (object.HasMember("frame_id") && object["frame_id"].IsString()) result.metadata.frame_id = object["frame_id"].GetString();
    if (object.HasMember("vehicle_model_name") && object["vehicle_model_name"].IsString()) {
      result.metadata.vehicle_model_name = object["vehicle_model_name"].GetString();
    }
    if (object.HasMember("current_area") && object["current_area"].IsString()) {
      result.metadata.current_area = object["current_area"].GetString();
    }
    result.metadata.extra["legacy_format"] = "parking_lot_area_map";
    return result;
  }

  ParkingMap result;
  result.metadata.version = read_string(object, "version", "map");
  result.metadata.frame_id = read_string(object, "frame_id", "map");
  result.metadata.vehicle_model_name = read_string(object, "vehicle_model_name", "map");
  if (object.HasMember("current_area") && !object["current_area"].IsNull()) {
    result.metadata.current_area = read_string(object, "current_area", "map");
  }
  if (object.HasMember("parameters")) result.metadata.parameters = string_map_from_json(object["parameters"], "map.parameters");
  if (object.HasMember("evaluation_metadata")) {
    result.metadata.evaluation_metadata = string_map_from_json(object["evaluation_metadata"], "map.evaluation_metadata");
  }
  if (!object.HasMember("landmarks") || !object["landmarks"].IsArray()) {
    throw MapValidationError("map.landmarks must be an array");
  }
  if (!object.HasMember("road_nodes") || !object["road_nodes"].IsArray()) {
    throw MapValidationError("map.road_nodes must be an array");
  }
  if (!object.HasMember("edges") || !object["edges"].IsArray()) {
    throw MapValidationError("map.edges must be an array");
  }
  for (rapidjson::SizeType i = 0; i < object["landmarks"].Size(); ++i) result.landmarks.push_back(landmark_from_json(object["landmarks"][i], i));
  for (rapidjson::SizeType i = 0; i < object["road_nodes"].Size(); ++i) result.road_nodes.push_back(road_node_from_json(object["road_nodes"][i], i));
  for (rapidjson::SizeType i = 0; i < object["edges"].Size(); ++i) result.edges.push_back(edge_from_json(object["edges"][i], i));
  return result;
}

JsonDocument to_document(const ParkingMap & map)
{
  JsonDocument document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  add_string(document, "version", map.metadata.version, allocator);
  add_string(document, "frame_id", map.metadata.frame_id, allocator);
  add_string(document, "vehicle_model_name", map.metadata.vehicle_model_name, allocator);
  add_optional_string(document, "current_area", map.metadata.current_area, allocator);
  add_string_map(document, "parameters", map.metadata.parameters, allocator);
  add_string_map(document, "evaluation_metadata", map.metadata.evaluation_metadata, allocator);
  JsonValue landmarks(rapidjson::kArrayType);
  for (const auto & item : map.landmarks) landmarks.PushBack(landmark_to_json(item, allocator), allocator);
  document.AddMember("landmarks", landmarks, allocator);
  JsonValue road_nodes(rapidjson::kArrayType);
  for (const auto & item : map.road_nodes) road_nodes.PushBack(road_node_to_json(item, allocator), allocator);
  document.AddMember("road_nodes", road_nodes, allocator);
  JsonValue edges(rapidjson::kArrayType);
  for (const auto & item : map.edges) edges.PushBack(edge_to_json(item, allocator), allocator);
  document.AddMember("edges", edges, allocator);
  for (const auto & [key, value] : map.metadata.extra) add_string(document, key.c_str(), value, allocator);
  return document;
}

}  // namespace

void validate_map(const ParkingMap & parking_map)
{
  if (parking_map.metadata.version.empty()) throw MapValidationError("metadata.version must not be empty");
  if (parking_map.metadata.frame_id.empty()) throw MapValidationError("metadata.frame_id must not be empty");
  if (parking_map.metadata.vehicle_model_name.empty()) {
    throw MapValidationError("metadata.vehicle_model_name must not be empty");
  }

  std::set<std::string> node_ids;
  for (const auto & landmark : parking_map.landmarks) {
    if (landmark.id.empty()) throw MapValidationError("landmark id must not be empty");
    if (!node_ids.insert(landmark.id).second) throw MapValidationError("duplicate node id: " + landmark.id);
    if (!kLandmarkTypes.count(landmark.type)) throw MapValidationError("invalid landmark type: " + landmark.type);
    validate_pose(landmark.pose, "landmark[" + landmark.id + "].pose");
    if (landmark.entrance_pose.has_value()) validate_pose(*landmark.entrance_pose, "landmark[" + landmark.id + "].entrance_pose");
    validate_confidence(landmark.confidence, "landmark[" + landmark.id + "].confidence");
    if (landmark.observation_count < 0) throw MapValidationError("landmark[" + landmark.id + "].observation_count must be non-negative");
    if (!kStatuses.count(landmark.status)) throw MapValidationError("invalid landmark status: " + landmark.status);
  }
  for (const auto & node : parking_map.road_nodes) {
    if (node.id.empty()) throw MapValidationError("road node id must not be empty");
    if (!node_ids.insert(node.id).second) throw MapValidationError("duplicate node id: " + node.id);
    if (!kRoadNodeTypes.count(node.node_type)) throw MapValidationError("invalid road node type: " + node.node_type);
    validate_pose(node.pose, "road_node[" + node.id + "].pose");
    if (node.anchor_id.has_value() && !node_ids.count(*node.anchor_id)) {
      throw MapValidationError("road node " + node.id + " references unknown anchor: " + *node.anchor_id);
    }
    validate_confidence(node.confidence, "road_node[" + node.id + "].confidence");
    std::set<std::string> neighbor_ids;
    for (const auto & neighbor : node.neighbors) {
      if (neighbor.neighbor_id.empty()) throw MapValidationError("road node " + node.id + " has an empty neighbor id");
      if (neighbor.neighbor_id == node.id) {
        throw MapValidationError("road node " + node.id + " cannot reference itself as a neighbor");
      }
      if (!neighbor_ids.insert(neighbor.neighbor_id).second) {
        throw MapValidationError(
          "road node " + node.id + " has duplicate neighbor: " + neighbor.neighbor_id);
      }
      validate_pose(neighbor.relative_pose, "neighbor[" + node.id + "].relative_pose");
      if (neighbor.direction.empty()) throw MapValidationError("neighbor[" + node.id + "] direction must not be empty");
      validate_nonnegative(neighbor.cost, "neighbor[" + node.id + "].cost");
    }
  }
  std::set<std::string> edge_ids;
  for (const auto & edge : parking_map.edges) {
    if (edge.id.empty()) throw MapValidationError("edge id must not be empty");
    if (!edge_ids.insert(edge.id).second) throw MapValidationError("duplicate edge id: " + edge.id);
    if (!node_ids.count(edge.from_id)) throw MapValidationError("edge " + edge.id + " references unknown from_id: " + edge.from_id);
    if (!node_ids.count(edge.to_id)) throw MapValidationError("edge " + edge.id + " references unknown to_id: " + edge.to_id);
    if (edge.from_id == edge.to_id) throw MapValidationError("edge " + edge.id + " cannot connect a node to itself");
    validate_pose(edge.relative_pose, "edge[" + edge.id + "].relative_pose");
    if (edge.direction.empty()) throw MapValidationError("edge " + edge.id + " direction must not be empty");
    validate_nonnegative(edge.cost, "edge " + edge.id + " cost");
  }
  for (const auto & node : parking_map.road_nodes) {
    for (const auto & neighbor : node.neighbors) {
      if (!node_ids.count(neighbor.neighbor_id)) {
        throw MapValidationError("road node " + node.id + " references unknown neighbor: " + neighbor.neighbor_id);
      }
    }
  }
}

ParkingMap load_map(const std::string & path)
{
  const auto document = read_document(path);
  ParkingMap result = parse_document(document);
  validate_map(result);
  return result;
}

void save_map(const ParkingMap & parking_map, const std::string & path)
{
  const std::string serialized = serialize_map(parking_map);
  const std::filesystem::path map_path(path);
  if (!map_path.parent_path().empty()) std::filesystem::create_directories(map_path.parent_path());
  std::ofstream stream(path);
  if (!stream.is_open()) throw MapValidationError("cannot write map file: " + path);
  stream << serialized << '\n';
  if (!stream.good()) throw MapValidationError("failed while writing map file: " + path);
}

std::string serialize_map(const ParkingMap & parking_map)
{
  validate_map(parking_map);
  const auto document = to_document(parking_map);
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

}  // namespace parking_lot_core
