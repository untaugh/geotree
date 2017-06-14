#include "gtest/gtest.h"

#include "geotree.h"

namespace {

  class GeotreeTest : public ::testing::Test {
  protected:
  
  };

  TEST_F(GeotreeTest, First) {
    Geotree *g = new Geotree();
    g->test = 2;
    EXPECT_EQ(2, g->test);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
