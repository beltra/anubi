#include "robot_nav_recovery/costmap_sector_analyzer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace robot_nav_recovery
{
namespace
{

CostmapGrid makeGrid()
{
  CostmapGrid grid;
  grid.width = 10;
  grid.height = 10;
  grid.resolution = 0.2;
  grid.origin_x = -1.0;
  grid.origin_y = -1.0;
  grid.data.assign(100, nav2_costmap_2d::FREE_SPACE);
  return grid;
}

void setCell(CostmapGrid & grid, unsigned int mx, unsigned int my, unsigned char cost)
{
  grid.data[static_cast<size_t>(my * grid.width + mx)] = cost;
}

}  // namespace

TEST(CostmapSectorAnalyzer, ScoresOccupiedFrontSector)
{
  auto grid = makeGrid();
  for (unsigned int mx = 7; mx < 10; ++mx) {
    for (unsigned int my = 4; my < 6; ++my) {
      setCell(grid, mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  SectorConfig config;
  config.sector_count = 8;
  config.outer_radius_m = 1.0;
  config.blocked_fraction_threshold = 0.05;

  auto sectors = analyzeSectors(grid, 0.0, 0.0, 0.0, config);

  ASSERT_EQ(sectors.size(), 8u);
  ASSERT_TRUE(sectorIndexForName("front", 8).has_value());
  const auto front_index = sectorIndexForName("front", 8).value();
  EXPECT_FALSE(sectors[static_cast<size_t>(front_index)].clear);
  EXPECT_GT(sectors[static_cast<size_t>(front_index)].occupied_count, 0u);
}

TEST(CostmapSectorAnalyzer, SectorClearRejectsOccupiedRear)
{
  auto grid = makeGrid();
  for (unsigned int mx = 0; mx < 3; ++mx) {
    for (unsigned int my = 4; my < 6; ++my) {
      setCell(grid, mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  SectorConfig config;
  config.sector_count = 8;
  config.outer_radius_m = 1.0;
  config.blocked_fraction_threshold = 0.05;

  auto sectors = analyzeSectors(grid, 0.0, 0.0, 0.0, config);

  EXPECT_FALSE(isSectorClear(sectors, "rear", config.sector_count));
  EXPECT_TRUE(isSectorClear(sectors, "front", config.sector_count));
}

TEST(CostmapSectorAnalyzer, SelectsLeastObstructedEscapeHeading)
{
  std::vector<SectorReport> sectors(8);
  for (int i = 0; i < 8; ++i) {
    sectors[static_cast<size_t>(i)].index = i;
    sectors[static_cast<size_t>(i)].center_angle_rad = normalizeAngle(i * M_PI / 4.0);
    sectors[static_cast<size_t>(i)].sample_count = 10;
    sectors[static_cast<size_t>(i)].occupied_count = 5;
    sectors[static_cast<size_t>(i)].score = 0.5;
    sectors[static_cast<size_t>(i)].clear = true;
  }
  sectors[2].score = 0.0;
  sectors[2].occupied_count = 0;

  auto heading = selectEscapeHeading(sectors);

  ASSERT_TRUE(heading.has_value());
  EXPECT_NEAR(heading.value(), M_PI / 2.0, 1e-6);
}

TEST(CostmapSectorAnalyzer, InvalidGridReturnsNoSectors)
{
  CostmapGrid grid;
  grid.width = 2;
  grid.height = 2;
  grid.resolution = 0.1;
  grid.data = {0, 0, 0};

  SectorConfig config;

  EXPECT_TRUE(analyzeSectors(grid, 0.0, 0.0, 0.0, config).empty());
}

}  // namespace robot_nav_recovery
