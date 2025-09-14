#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from deap import base, creator, tools
import random
import datetime
from std_msgs.msg import Float64
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl, WorldReset
from sensor_msgs.msg import JointState
import math

# -------------------------------
# Globals
# -------------------------------
cart_pose_x = 0.0
y_angular = 0.0
pole_tip_pose_z = 0.0

pub_cart = None
reset_client = None
node = None

# -------------------------------
# DEAP Setup
# -------------------------------
try:
    creator.create("FitnessMulti", base.Fitness, weights=(-1.0, 1.0))
except RuntimeError:
    pass  # already created

try:
    creator.create("Individual", list, fitness=creator.FitnessMulti)
except RuntimeError:
    pass  # already created

# -------------------------------
# Callbacks
# -------------------------------
def get_cart_pose(data: JointState):
    """Extracts cart position, pole angular velocity, and pole tip z."""
    global cart_pose_x, y_angular, pole_tip_pose_z

    try:
        ind = data.name.index("cart_pole::cart_joint")
        cart_pose_x = data.position[ind]

        ind_pitch = data.name.index("cart_pole::pole_joint")
        y_angular = data.velocity[ind_pitch]

        ind_tip = data.name.index("cart_pole::tip_joint")
        pole_tip_pose_z = data.position[ind_tip]

    except ValueError:
        # joint not found yet
        return

# -------------------------------
# Evaluation Function
# -------------------------------
def gain_evaluation(individual):
    global y_angular, cart_pose_x, pole_tip_pose_z, pub_cart, reset_client, node
    
    node.get_logger().info(f"Evaluating individual: {individual}")

    Kp_y, Ki_y, Kd_y, Kp_p, Ki_p, Kd_p = individual
    time_interval = 0.005

    # Reset world before simulation
    while not reset_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Reset service not available, waiting...")

    request = ControlWorld.Request()
    request.world_control = WorldControl()
    request.world_control.reset = WorldReset()
    request.world_control.reset.all = True    # Reset everything
    request.world_control.pause = False       # Don't pause after reset
    
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result().success:
        node.get_logger().info("World reset successful")
    else:
        node.get_logger().warn("World reset failed")

    # PID state variables
    yaw_angle = 0.0
    target_yaw_angle = 0.0
    target_cart_pose_x = 0.0
    last_error_yaw = 0.0
    last_error_pos = 0.0
    integral_position_error = 0.0
    integral_yaw_error = 0.0
    pole_tip_height_sum = 0.0
    x_displacement_sum = 0.0

    # Run simulation loop
    for _ in range(1000):
        time1 = time.time()

        pole_tip_height_sum += pole_tip_pose_z
        yaw_angle += y_angular * time_interval

        # yaw PID
        error_yaw = target_yaw_angle - yaw_angle
        integral_yaw_error += (error_yaw + last_error_yaw) * time_interval / 2
        effort_yaw = -(Kp_y * error_yaw +
                       Ki_y * integral_yaw_error +
                       Kd_y * (error_yaw - last_error_yaw) / time_interval)

        # position PID
        error_pos = target_cart_pose_x - cart_pose_x
        integral_position_error += (error_pos + last_error_pos) * time_interval / 2
        x_displacement_sum += abs(cart_pose_x)
        effort_pos = -(Kp_p * error_pos +
                       Ki_p * integral_position_error +
                       Kd_p * (error_pos - last_error_pos) / time_interval)

        # combine efforts
        effort = effort_yaw + effort_pos
        last_error_yaw = error_yaw
        last_error_pos = error_pos

        # publish command
        msg = Float64()
        msg.data = effort
        pub_cart.publish(msg)

        # process ROS callbacks
        rclpy.spin_once(node, timeout_sec=0.0)

        # maintain loop timing
        time2 = time.time()
        interval = time2 - time1
        if interval < time_interval:
            time.sleep(time_interval - interval)

    # clear integrals
    integral_position_error = 0.0
    integral_yaw_error = 0.0

    node.get_logger().info(
        f"x_disp={x_displacement_sum:.3f}, pole_tip_height={pole_tip_height_sum:.3f}"
    )

    return x_displacement_sum, pole_tip_height_sum

# -------------------------------
# Main
# -------------------------------
def main(args=None):
    global pub_cart, reset_client, node
    
    rclpy.init(args=args)
    node = rclpy.create_node("pid_gain_optimizer")

    pub_cart = node.create_publisher(Float64, "/cart_controller/command", 10)
    reset_client = node.create_client(ControlWorld, "/world/robomaster_rale/control/state")
    node.create_subscription(JointState, "/joint_states", get_cart_pose, 10)

    node.get_logger().info("##### Genetic optimization started #####")

    toolbox = base.Toolbox()
    toolbox.register("attr_gene", random.uniform, 0, 20)
    toolbox.register("evaluate", gain_evaluation)
    toolbox.register("mate", tools.cxBlend, alpha=0.2)
    toolbox.register("select", tools.selTournament, tournsize=3)
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.5, indpb=0.2)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_gene, 6)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # GA parameters
    N_GEN = 6
    POP_SIZE = 60
    CX_PB = 0.5
    MUT_PB = 0.3

    random.seed(datetime.datetime.now())

    # Create initial population
    pop = toolbox.population(n=POP_SIZE)

    # Evaluate initial population
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
    node.get_logger().info(f"Evaluated {len(pop)} individuals")

    # Evolution loop
    for g in range(1, N_GEN + 1):
        node.get_logger().info(f"-- Generation {g} --")

        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        # Crossover
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CX_PB:
                toolbox.mate(child1, child2)
                del child1.fitness.values, child2.fitness.values

        # Mutation
        for mutant in offspring:
            if random.random() < MUT_PB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Re-evaluate invalid offspring
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        pop[:] = offspring

        # Population stats
        fits = [ind.fitness.values[0] for ind in pop]
        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x * x for x in fits)
        std = abs(sum2 / length - mean**2) ** 0.5

        node.get_logger().info(
            f"  Min={min(fits):.3f}, Max={max(fits):.3f}, Avg={mean:.3f}, Std={std:.3f}"
        )

    # Choose best
    best_ind = tools.selBest(pop, 1)[0]
    node.get_logger().info(f"Best individual: {best_ind}, Fitness={best_ind.fitness.values}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()