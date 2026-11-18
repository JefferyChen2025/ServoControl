#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

constexpr int SERVO_PIN = 14;
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_BITS = 14;
constexpr int PWM_HZ = 50;

constexpr int32_t PWM_INITIAL_US = 1500;
constexpr uint32_t PWM_PERIOD_US = 1000000UL / PWM_HZ;
constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_BITS) - 1;

constexpr uint32_t MOTION_INTERVAL_MS = 20;
constexpr int32_t MAX_STEP_US = 10;
constexpr int32_t SLOWDOWN_DIVISOR = 20;

int32_t current_pwm_us = PWM_INITIAL_US;
int32_t target_pwm_us = PWM_INITIAL_US;
uint32_t last_motion_update_ms = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t pwm_subscription;
rcl_subscription_t step_subscription;
rcl_publisher_t current_publisher;

std_msgs__msg__Int32 pwm_msg;
std_msgs__msg__Int32 step_msg;
std_msgs__msg__Int32 current_msg;

void stop_on_error(rcl_ret_t result)
{
    if (result != RCL_RET_OK) {
        while (true) {
            delay(100);
        }
    }
}

void set_pwm_us(int32_t pulse_us)
{
    if (pulse_us <= 0 ||
        pulse_us >= static_cast<int32_t>(PWM_PERIOD_US)) {
        return;
    }

    uint32_t duty = static_cast<uint32_t>(
        static_cast<uint64_t>(pulse_us) * PWM_MAX_DUTY
        / PWM_PERIOD_US
    );

    ledcWrite(PWM_CHANNEL, duty);
    current_pwm_us = pulse_us;
}

void publish_current_pwm()
{
    current_msg.data = current_pwm_us;

    rcl_ret_t result =
        rcl_publish(&current_publisher, &current_msg, nullptr);

    (void)result;
}

void pwm_callback(const void *message)
{
    const auto *command =
        static_cast<const std_msgs__msg__Int32 *>(message);

    int32_t requested_pwm_us = command->data;

    if (requested_pwm_us > 0 &&
        requested_pwm_us < static_cast<int32_t>(PWM_PERIOD_US)) {
        target_pwm_us = requested_pwm_us;
    }

    publish_current_pwm();
}

void step_callback(const void *message)
{
    const auto *command =
        static_cast<const std_msgs__msg__Int32 *>(message);

    if (command->data != 0) {
        int64_t requested_pwm_us =
            static_cast<int64_t>(target_pwm_us) + command->data;

        if (requested_pwm_us > 0 &&
            requested_pwm_us < static_cast<int64_t>(PWM_PERIOD_US)) {
            target_pwm_us = static_cast<int32_t>(requested_pwm_us);
        }
    }

    publish_current_pwm();
}

void update_servo_motion()
{
    uint32_t now = millis();

    if (now - last_motion_update_ms < MOTION_INTERVAL_MS) {
        return;
    }

    last_motion_update_ms = now;

    int32_t error_us = target_pwm_us - current_pwm_us;

    if (error_us == 0) {
        return;
    }

    int32_t distance_us =
        (error_us > 0) ? error_us : -error_us;

    int32_t step_us =
        (distance_us + SLOWDOWN_DIVISOR - 1)
        / SLOWDOWN_DIVISOR;

    if (step_us > MAX_STEP_US) {
        step_us = MAX_STEP_US;
    }

    if (step_us > distance_us) {
        step_us = distance_us;
    }

    int32_t next_pwm_us =
        current_pwm_us + ((error_us > 0) ? step_us : -step_us);

    set_pwm_us(next_pwm_us);
    publish_current_pwm();
}

void setup()
{
    if (ledcSetup(PWM_CHANNEL, PWM_HZ, PWM_BITS) == 0) {
        while (true) {
            delay(100);
        }
    }

    ledcAttachPin(SERVO_PIN, PWM_CHANNEL);
    set_pwm_us(PWM_INITIAL_US);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        delay(100);
    }

    allocator = rcl_get_default_allocator();

    stop_on_error(
        rclc_support_init(&support, 0, nullptr, &allocator)
    );

    stop_on_error(
        rclc_node_init_default(
            &node, "esp32_servo_node", "", &support
        )
    );

    stop_on_error(
        rclc_subscription_init_default(
            &pwm_subscription,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/servo_pwm_us"
        )
    );

    stop_on_error(
        rclc_subscription_init_default(
            &step_subscription,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/servo_pwm_step_us"
        )
    );

    stop_on_error(
        rclc_publisher_init_default(
            &current_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/servo_pwm_current_us"
        )
    );

    executor = rclc_executor_get_zero_initialized_executor();

    stop_on_error(
        rclc_executor_init(
            &executor, &support.context, 2, &allocator
        )
    );

    stop_on_error(
        rclc_executor_add_subscription(
            &executor,
            &pwm_subscription,
            &pwm_msg,
            &pwm_callback,
            ON_NEW_DATA
        )
    );

    stop_on_error(
        rclc_executor_add_subscription(
            &executor,
            &step_subscription,
            &step_msg,
            &step_callback,
            ON_NEW_DATA
        )
    );

    last_motion_update_ms = millis();
}

void loop()
{
    rcl_ret_t result =
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    (void)result;

    update_servo_motion();

    delay(1);
}
