#include <stdio.h>
#include <stdbool.h>
#include "board.h"

// Mock for toggle_led function
static bool led1_state = false;
static int toggle_led_called = 0;
static const char* last_led_name = NULL;

void toggle_led(const char* name) {
    toggle_led_called++;
    last_led_name = name;
    led1_state = !led1_state;
}

// Test cases
void test_show_heartbeat_should_toggle_led1(void) {
    // Reset mock state
    toggle_led_called = 0;
    last_led_name = NULL;
    led1_state = false;
    
    // Call the function under test
    show_heartbeat();
    
    // Verify behavior
    if (toggle_led_called != 1) {
        printf("FAIL: toggle_led should be called once, was called %d times\n", toggle_led_called);
        return;
    }
    
    if (last_led_name == NULL || strcmp(last_led_name, "led1") != 0) {
        printf("FAIL: toggle_led should be called with 'led1', got '%s'\n", 
               last_led_name ? last_led_name : "NULL");
        return;
    }
    
    if (led1_state != true) {
        printf("FAIL: LED state should be toggled to true\n");
        return;
    }
    
    printf("PASS: test_show_heartbeat_should_toggle_led1\n");
}

int main(void) {
    test_show_heartbeat_should_toggle_led1();
    return 0;
}