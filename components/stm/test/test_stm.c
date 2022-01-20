#include "unity.h"
#include "stm.h"

TEST_CASE("tat min pair", "[null]") { 
    TEST_ASSERT_EQUAL((1 + 4), 14); 
}

TEST_CASE("interp: 0, 100, 1/2 => 50", "[interp]") {
    TEST_ASSERT_EQUAL(interp(0, 100, 0, 100, 50), 50);
}

TEST_CASE("interp_hue_cw: 0, 3, 1/3 => 1", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_cw(0, 3, 0, 3, 1), 1);
}

TEST_CASE("interp_hue_cw: 255, 2, 1/3 => 0", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_cw(255, 2, 0, 3, 1), 0);
}

TEST_CASE("interp_hue_cw: 254, 1, 1/3 => 255", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_cw(254, 1, 0, 3, 1), 255);
}

TEST_CASE("interp_hue_cw: 253, 0, 1/3 => 254", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_cw(253, 0, 0, 3, 1), 254);
}

TEST_CASE("interp_hue_ccw: 3, 0, 2/3 => 1", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_ccw(3, 0, 0, 3, 2), 1);
}

TEST_CASE("interp_hue_ccw: 2, 255, 2/3 => 0", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_ccw(2, 255, 0, 3, 2), 0);
} 

TEST_CASE("interp_hue_ccw: 1, 254, 2/3 => 255", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_ccw(1, 254, 0, 3, 2), 255);
}

TEST_CASE("interp_hue_ccw: 0, 253, 2/3 => 254", "[interp]") {
    TEST_ASSERT_EQUAL(interp_hue_ccw(0, 253, 0, 3, 2), 254);
}


