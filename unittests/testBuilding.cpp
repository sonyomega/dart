#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

//using namespace std;

/* ********************************************************************************************* */
TEST(BUILDING, BASIC)
{
}

/* ********************************************************************************************* */
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
