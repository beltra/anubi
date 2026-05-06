#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_nav_recovery
{

struct CostmapGrid
{
  unsigned int width{0};
  unsigned int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::vector<unsigned char> data;
};

struct SectorConfig
{
  int sector_count{8};
  double inner_radius_m{0.15};
  double outer_radius_m{1.25};
  unsigned char occupied_cost_threshold{253};
  bool unknown_is_occupied{false};
  double blocked_fraction_threshold{0.2};
};

struct SectorReport
{
  int index{0};
  double center_angle_rad{0.0};
  unsigned int sample_count{0};
  unsigned int occupied_count{0};
  unsigned char max_cost{0};
  double score{0.0};
  bool clear{false};
};

double normalizeAngle(double angle);

std::vector<SectorReport> analyzeSectors(
  const CostmapGrid & grid,
  double robot_x,
  double robot_y,
  double robot_yaw,
  const SectorConfig & config);

std::optional<double> selectEscapeHeading(
  const std::vector<SectorReport> & sectors);

std::optional<int> sectorIndexForName(
  const std::string & name,
  int sector_count);

bool isSectorClear(
  const std::vector<SectorReport> & sectors,
  const std::string & name,
  int sector_count);

bool isValidGrid(const CostmapGrid & grid);

}  // namespace robot_nav_recovery
