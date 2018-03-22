#include "local_grid_map/Algorithm.hpp"

// gtest
#include <gtest/gtest.h>

// STD
#include <vector>

using namespace local_grid_map;

TEST(Algorithm, getWithoutSet)
{
  Algorithm algorithm;
  const bool istoconf = algorithm.getCamIsToConf();
  EXPECT_EQ(false, istoconf);
}

TEST(Algorithm, setConfCameraTrue)
{
  const bool conf_camera = true;
  Algorithm algorithm;
  algorithm.setCamIsToConf(conf_camera);
  const double istoconf = algorithm.getCamIsToConf();
  EXPECT_EQ(conf_camera, istoconf);
}

TEST(Algorithm, setConfCameraFalse)
{
  const bool conf_camera = false;
  Algorithm algorithm;
  algorithm.setCamIsToConf(conf_camera);
  const double istoconf = algorithm.getCamIsToConf();
  EXPECT_EQ(conf_camera, istoconf);
}

