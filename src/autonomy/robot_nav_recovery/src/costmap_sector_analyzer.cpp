#include "robot_nav_recovery/costmap_sector_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "nav2_costmap_2d/cost_values.hpp"

namespace robot_nav_recovery
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double sectorWidth(int sector_count)
{
  return 2.0 * kPi / static_cast<double>(sector_count);
}

int angleToSector(double angle, int sector_count)
{
  int index = static_cast<int>(std::llround(normalizeAngle(angle) / sectorWidth(sector_count)));
  index %= sector_count;
  if (index < 0) {
    index += sector_count;
  }
  return index;
}

double sectorCenter(int index, int sector_count)
{
  return normalizeAngle(static_cast<double>(index) * sectorWidth(sector_count));
}

bool isOccupied(unsigned char cost, const SectorConfig & config)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return config.unknown_is_occupied;
  }
  return cost >= config.occupied_cost_threshold;
}

std::optional<double> namedSectorAngle(const std::string & name)
{
  static const std::unordered_map<std::string, double> kAngles = {
    {"front", 0.0},
    {"front_left", kPi / 4.0},
    {"left", kPi / 2.0},
    {"rear_left", 3.0 * kPi / 4.0},
    {"rear", kPi},
    {"back", kPi},
    {"rear_right", -3.0 * kPi / 4.0},
    {"right", -kPi / 2.0},
    {"front_right", -kPi / 4.0},
  };
  auto iter = kAngles.find(name);
  if (iter == kAngles.end()) {
    return std::nullopt;
  }
  return iter->second;
}

}  // namespace

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool isValidGrid(const CostmapGrid & grid)
{
  return grid.width > 0 && grid.height > 0 && grid.resolution > 0.0 &&
         grid.data.size() == static_cast<size_t>(grid.width * grid.height);
}

std::vector<SectorReport> analyzeSectors(
  const CostmapGrid & grid,
  double robot_x,
  double robot_y,
  double robot_yaw,
  const SectorConfig & config)
{
  if (!isValidGrid(grid) || config.sector_count <= 0 || config.outer_radius_m <= 0.0) {
    return {};
  }

  std::vector<SectorReport> sectors(static_cast<size_t>(config.sector_count));
  for (int index = 0; index < config.sector_count; ++index) {
    sectors[static_cast<size_t>(index)].index = index;
    sectors[static_cast<size_t>(index)].center_angle_rad = sectorCenter(index, config.sector_count);
  }

  const double cos_origin = std::cos(grid.origin_yaw);
  const double sin_origin = std::sin(grid.origin_yaw);
  const double inner_sq = config.inner_radius_m * config.inner_radius_m;
  const double outer_sq = config.outer_radius_m * config.outer_radius_m;

  for (unsigned int my = 0; my < grid.height; ++my) {
    for (unsigned int mx = 0; mx < grid.width; ++mx) {
      const double local_x = (static_cast<double>(mx) + 0.5) * grid.resolution;
      const double local_y = (static_cast<double>(my) + 0.5) * grid.resolution;
      const double world_x = grid.origin_x + local_x * cos_origin - local_y * sin_origin;
      const double world_y = grid.origin_y + local_x * sin_origin + local_y * cos_origin;
      const double dx = world_x - robot_x;
      const double dy = world_y - robot_y;
      const double range_sq = dx * dx + dy * dy;
      if (range_sq < inner_sq || range_sq > outer_sq) {
        continue;
      }

      const double relative_angle = normalizeAngle(std::atan2(dy, dx) - robot_yaw);
      auto & report = sectors[static_cast<size_t>(angleToSector(relative_angle, config.sector_count))];
      const unsigned char cost = grid.data[static_cast<size_t>(my * grid.width + mx)];
      report.sample_count += 1;
      report.max_cost = std::max(report.max_cost, cost);
      if (isOccupied(cost, config)) {
        report.occupied_count += 1;
      }
    }
  }

  for (auto & report : sectors) {
    if (report.sample_count == 0) {
      report.score = std::numeric_limits<double>::infinity();
      report.clear = false;
      continue;
    }
    const double occupied_fraction =
      static_cast<double>(report.occupied_count) / static_cast<double>(report.sample_count);
    report.score = occupied_fraction;
    report.clear = occupied_fraction <= config.blocked_fraction_threshold;
  }

  return sectors;
}

std::optional<double> selectEscapeHeading(const std::vector<SectorReport> & sectors)
{
  if (sectors.empty()) {
    return std::nullopt;
  }

  const SectorReport * best = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  for (const auto & sector : sectors) {
    if (!sector.clear || !std::isfinite(sector.score)) {
      continue;
    }
    // Prefer less rotation only when occupancy scores are nearly equal.
    const double adjusted_score = sector.score + 0.02 * std::fabs(sector.center_angle_rad);
    if (adjusted_score < best_score) {
      best_score = adjusted_score;
      best = &sector;
    }
  }

  if (best == nullptr) {
    return std::nullopt;
  }
  return best->center_angle_rad;
}

std::optional<int> sectorIndexForName(const std::string & name, int sector_count)
{
  if (sector_count <= 0) {
    return std::nullopt;
  }

  auto angle = namedSectorAngle(name);
  if (!angle.has_value()) {
    return std::nullopt;
  }
  return angleToSector(angle.value(), sector_count);
}

bool isSectorClear(
  const std::vector<SectorReport> & sectors,
  const std::string & name,
  int sector_count)
{
  auto index = sectorIndexForName(name, sector_count);
  if (!index.has_value() || index.value() < 0 || index.value() >= static_cast<int>(sectors.size())) {
    return false;
  }
  return sectors[static_cast<size_t>(index.value())].clear;
}

}  // namespace robot_nav_recovery
