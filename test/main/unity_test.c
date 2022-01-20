#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"

static void print_banner(const char *text) {
    printf("\n#### %s #####\n\n", text);
}

void app_main(void) {
    /*
        print_banner("Executing one test by its name");
        UNITY_BEGIN();
        unity_run_test_by_name("Mean of an empty array is zero");
        UNITY_END();

        print_banner("Running tests with [mean] tag");
        UNITY_BEGIN();
        unity_run_tests_by_tag("[mean]", false);
        UNITY_END();

        print_banner("Running tests without [fails] tag");
        UNITY_BEGIN();
        unity_run_tests_by_tag("[fails]", true);
        UNITY_END();
    */
    print_banner("Running all the registered tests");
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();

    vTaskDelay(portMAX_DELAY);

    print_banner("Starting interactive test menu");

    /* This function will not return, and will be busy waiting for UART input.
     * Make sure that task watchdog is disabled if you use this function.
     */
    unity_run_menu();
}
