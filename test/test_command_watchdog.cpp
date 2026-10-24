#include <gtest/gtest.h>
#include "rabcl/controller/command_watchdog.hpp"

namespace rabcl
{

TEST(CommandWatchdog, ExpiredBeforeFirstFeed)
{
  CommandWatchdog watchdog(0.5f);
  EXPECT_TRUE(watchdog.Expired(0.0f));
}

TEST(CommandWatchdog, NotExpiredWithinTimeout)
{
  CommandWatchdog watchdog(0.5f);
  watchdog.Feed(1.0f);
  EXPECT_FALSE(watchdog.Expired(1.4f));
}

TEST(CommandWatchdog, ExpiredAfterTimeout)
{
  CommandWatchdog watchdog(0.5f);
  watchdog.Feed(1.0f);
  EXPECT_TRUE(watchdog.Expired(1.6f));
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
