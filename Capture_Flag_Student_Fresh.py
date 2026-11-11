"""
CDS215 — CAPTURE THE FLAG (Game 2)

Blue Husky (you): take the gold flag and bring it back to your blue base.
Red guard: patrols near the flag, then chases you once the flag is captured.
Being caught or running out of time loses the game.

Requires: pip install pybullet
Run: python CDS215_Capture_the_Flag_Student.py
Controls: W/Up = forward, S/Down = reverse, A/Left = steer left,
          D/Right = steer right, R = restart, Q = quit.

Student exercise: complete three short tasks together in update_mission().
All robot driving, guard AI, physics, obstacles, and graphics are provided.
Optional no-GUI rule check: python filename.py --smoke-test
"""

import math
import sys
import time
import pybullet as p
import pybullet_data


# =====================================================================
# PART 1 — GAME SETTINGS: EASY FOR STUDENTS TO EXPERIMENT WITH
# =====================================================================
MISSION_SECONDS = 120.0            # Capture the flag and return before time is up.
PLAYER_SPEED = 7              # Student Husky forward motor speed.
PLAYER_TURN_SPEED = 3          # Student Husky turning rate setting.
HUNTER_SPEED = 14              # Hunter motor speed.
HUNTER_TURN_SPEED = 6.2          # Hunter turn speed.
HUNTER_START = (6.0, 3.5)
PLAYER_START = (-6.0, -6.0)

# Game 2 mission settings (coordinates in meters).
HOME_BASE = (-6.0, -6.0)
ENEMY_FLAG = (6.0, 5.8)
BASE_RADIUS = 1.25            # How close you must get to return the flag.
FLAG_RADIUS = 1.25            # How close you must get to pick up the flag.
PATROL_POINTS = [(6.4, 2.0), (6.7, 6.4), (5.0, 6.3)]
PATROL_RADIUS = 0.95
GUARD_PATROL_SPEED_FACTOR = 0.62   # A slower guard before the flag is stolen.

# These settings affect PERFORMANCE and PLAYBACK, not just wheel speeds.
PHYSICS_HZ = 120                 # Try 120 or 240. More Hz = more physics work.
RAY_DRAW_HZ = 10                 # Try 2, 10, 30, 120: how often rays are drawn.
MAX_PLAYBACK_SPEED = 1.0        # 1.0 = normal; 1.5/2.0 = faster; 0 = unlimited.
SHOW_HUNTER_RAYS = True          # Turn off if graphics are slow.
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 750

# Instructor tuning: deliberately separate from the student settings.
ARENA_HALF_SIZE = 9.0
WALL_THICKNESS = 0.6
HUNTER_SENSOR_RANGE = 2.6
# Three forward-looking rays: center plus left and right FRONT CORNERS.
# Wide-angle rays used to miss obstacles directly in front of a corner.
HUNTER_SENSOR_ANGLE = math.radians(12)  # Slight outward sweep, not 35 degrees.
HUNTER_STOP_DISTANCE = 1.5
HUNTER_SIDE_DISTANCE = 1.15           # Trigger when EITHER corner gets too close.
HUNTER_SENSOR_OFFSET = 0.72         # Forward of the chassis to reduce self-hits.
HUNTER_CORNER_OFFSET = 0.36         # Left/right position measured from centerline.

# A single avoidance TURN is not enough: without forward follow-through the
# hunter immediately aims at the player and swings back toward the same box.
HUNTER_AVOID_TURN_SECONDS = 0.55     # Rotate away from the detected box.
HUNTER_ADVANCE_SECONDS = 1.2         # Then drive past its corner before chasing.
HUNTER_ADVANCE_SPEED_FACTOR = 0.70   # Moderate, controlled forward motion.
HUNTER_ADVANCE_FRONT_CLEARANCE = 0.70  # Emergency stop/turn if box is too close.
HUNTER_ADVANCE_SIDE_CLEARANCE = 0.35   # Keep the chassis corners safe.

# STUCK RECOVERY — applies during chase AND obstacle-avoidance sequences.
HUNTER_STUCK_CHECK_SECONDS = 2.0    # How often to check actual forward progress.
HUNTER_STUCK_MIN_MOVEMENT = 0.30    # Less movement means physically trapped.
HUNTER_REVERSE_SECONDS = 0.95       # Back away from the obstacle.
HUNTER_RECOVERY_SPIN_SECONDS = 1.10 # Turn in place after reversing.
HUNTER_RECOVERY_ADVANCE_SECONDS = 1.5  # Move away before resuming pursuit.
SENSOR_HEIGHT = 0.38
LEFT_WHEELS = (2, 4)
RIGHT_WHEELS = (3, 5)

# Each item: center x, center y, FULL width, FULL depth, RGB color.
# The course deliberately provides space to weave around the obstacles.
OBSTACLES = [
    (-3.0,  1.2, 2.6, 1.2, (0.15, 0.75, 0.95)),
    ( 1.1, -2.1, 3.0, 1.25, (0.95, 0.62, 0.15)),
    ( 3.5,  2.8, 1.3, 2.8, (0.68, 0.34, 0.91)),
    (-4.1, -3.3, 1.6, 1.5, (0.94, 0.74, 0.18)),
    (-0.6,  4.1, 2.5, 1.2, (0.40, 0.88, 0.54)),
    ( 4.3, -4.1, 1.7, 1.7, (0.91, 0.41, 0.61)),
]

SMOKE_TEST = "--smoke-test" in sys.argv
if not isinstance(PHYSICS_HZ, int) or PHYSICS_HZ < 30:
    raise ValueError("PHYSICS_HZ must be an integer of at least 30")
if not isinstance(RAY_DRAW_HZ, int) or RAY_DRAW_HZ < 1:
    raise ValueError("RAY_DRAW_HZ must be an integer of at least 1")
if MAX_PLAYBACK_SPEED < 0:
    raise ValueError("MAX_PLAYBACK_SPEED cannot be negative")
if MISSION_SECONDS <= 0:
    raise ValueError("MISSION_SECONDS must be positive")


# =====================================================================
# PART 2 — REUSABLE ROBOT COMMANDS (FROM OUR PREVIOUS HUSKY LABS)
# =====================================================================
_last_commands = {}


def set_wheels(robot_id, left_speed, right_speed):
    """Only send a command if it changes (our motor optimization)."""
    speeds = (round(left_speed, 4), round(right_speed, 4))
    if _last_commands.get(robot_id) == speeds:
        return
    for joint in LEFT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=left_speed, force=100)
    for joint in RIGHT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=right_speed, force=100)
    _last_commands[robot_id] = speeds


def stop_robot(robot_id):
    set_wheels(robot_id, 0, 0)


def robot_xy_heading(robot_id):
    position, rotation = p.getBasePositionAndOrientation(robot_id)
    return position[0], position[1], p.getEulerFromQuaternion(rotation)[2]


def angle_difference(target_angle, current_angle):
    return math.atan2(math.sin(target_angle - current_angle),
                      math.cos(target_angle - current_angle))


def pressed(keys, *codes):
    return any(keys.get(code, 0) & p.KEY_IS_DOWN for code in codes)


def player_teleop(robot_id, keys):
    """WASD / arrow controls; holding forward and turn at once makes an arc."""
    up = getattr(p, "B3G_UP_ARROW", -100)
    down = getattr(p, "B3G_DOWN_ARROW", -101)
    left_arrow = getattr(p, "B3G_LEFT_ARROW", -102)
    right_arrow = getattr(p, "B3G_RIGHT_ARROW", -103)

    forward = pressed(keys, ord('w'), ord('W'), up)
    backward = pressed(keys, ord('s'), ord('S'), down)
    left = pressed(keys, ord('a'), ord('A'), left_arrow)
    right = pressed(keys, ord('d'), ord('D'), right_arrow)

    drive = (int(forward) - int(backward)) * PLAYER_SPEED
    turn = (int(left) - int(right)) * PLAYER_TURN_SPEED
    set_wheels(robot_id, drive - turn, drive + turn)


# =====================================================================
# PART 3 — ROBOT COLORS, ARENA, AND SCENE SETUP
# =====================================================================
def make_box(x, y, width, depth, color, height=1.2):
    half = [width / 2, depth / 2, height / 2]
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                 rgbaColor=[*color, 1])
    return p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=collision,
                             baseVisualShapeIndex=visual,
                             basePosition=[x, y, height / 2])


def tint_husky(robot_id, rgb):
    """Give the two otherwise-identical Husky models distinct team colors."""
    for link in [-1] + list(range(p.getNumJoints(robot_id))):
        try:
            p.changeVisualShape(robot_id, link, rgbaColor=[*rgb, 1])
        except p.error:
            pass  # Some URDF links have no visible shape.


def create_arena():
    size = ARENA_HALF_SIZE
    thick = WALL_THICKNESS
    blue_gray = (0.28, 0.35, 0.48)
    make_box(0, size, size * 2 + thick, thick, blue_gray)
    make_box(0, -size, size * 2 + thick, thick, blue_gray)
    make_box(size, 0, thick, size * 2 + thick, blue_gray)
    make_box(-size, 0, thick, size * 2 + thick, blue_gray)
    for x, y, w, d, color in OBSTACLES:
        make_box(x, y, w, d, color)


def make_husky(start, rgb):
    rid = p.loadURDF("husky/husky.urdf", [start[0], start[1], 0.22])
    tint_husky(rid, rgb)
    return rid


# =====================================================================
# PART 4 — RED GUARD: PATROL WAYPOINTS THEN CHASE THE PLAYER
# =====================================================================
# The guard follows fixed waypoint coordinates until the flag is stolen.
# Then it switches to chasing the student's LIVE coordinates every step.
# Students do not modify the navigation or obstacle avoidance below.

_ray_ids = {"center": -1, "left": -1, "right": -1}


def hunter_sensor(hunter_id, player_id, x, y, heading, angle, step):
    """Return the distance to the nearest wall/box using a front-facing ray.

    The center ray originates at the nose. Left/right rays originate near
    the front corners and fan slightly outward to protect the full width.
    The player is not an obstacle to avoid because catching it is the goal.
    Detection runs EVERY physics step; visuals refresh less frequently.
    """
    # The center ray starts at the nose. The two side rays start at the
    # FRONT CORNERS of the Husky instead of on the 35-degree diagonal.
    # "side" is signed: + means left, - means right, 0 means center.
    side = HUNTER_CORNER_OFFSET if angle > 0 else (
        -HUNTER_CORNER_OFFSET if angle < 0 else 0.0
    )
    start = [x + HUNTER_SENSOR_OFFSET * math.cos(heading) - side * math.sin(heading),
             y + HUNTER_SENSOR_OFFSET * math.sin(heading) + side * math.cos(heading),
             SENSOR_HEIGHT]
    direction = heading + angle
    end = [start[0] + HUNTER_SENSOR_RANGE * math.cos(direction),
           start[1] + HUNTER_SENSOR_RANGE * math.sin(direction), SENSOR_HEIGHT]
    hit_id, _, fraction, _, _ = p.rayTest(start, end)[0]
    # Treat the player as clear so the enemy runs into the player to win.
    is_wall = hit_id >= 0 and hit_id not in (hunter_id, player_id)
    distance = max(0.0, fraction * HUNTER_SENSOR_RANGE) if is_wall else HUNTER_SENSOR_RANGE

    draw_interval = max(1, round(PHYSICS_HZ / RAY_DRAW_HZ))
    if SHOW_HUNTER_RAYS and not SMOKE_TEST and step % draw_interval == 0:
        label = "center" if angle == 0 else ("left" if angle > 0 else "right")
        rgb = [1, 0.15, 0.1] if is_wall else [0.1, 0.9, 0.2]
        _ray_ids[label] = p.addUserDebugLine(
            start, end, rgb, lineWidth=2,
            replaceItemUniqueId=_ray_ids[label])
    return distance


def update_hunter(hunter_id, player_id, game, target_xy=None, speed_factor=1.0):
    """Follow a patrol waypoint or the player's LIVE position; avoid obstacles."""
    hunter_x, hunter_y, hunter_heading = robot_xy_heading(hunter_id)
    if target_xy is None:
        target_x, target_y, _ = robot_xy_heading(player_id)  # CHASE moving player!
    else:
        target_x, target_y = target_xy                 # PATROL fixed waypoint.
    target_angle = math.atan2(target_y - hunter_y, target_x - hunter_x)
    error = angle_difference(target_angle, hunter_heading)
    step = game["step"]

    # Detect a physically trapped hunter, including when it switches between
    # AVOID and ADVANCE without actually moving around the box. Previously
    # this check only ran during CHASE, so that loop could last indefinitely.
    if step - game["anchor_step"] >= round(HUNTER_STUCK_CHECK_SECONDS * PHYSICS_HZ):
        anchor_x, anchor_y = game["anchor"]
        moved = math.hypot(hunter_x - anchor_x, hunter_y - anchor_y)
        to_target = math.hypot(target_x - hunter_x, target_y - hunter_y)
        if (moved < HUNTER_STUCK_MIN_MOVEMENT
                and to_target > 0.75
                and game["mode"] in ("chase", "avoid", "advance")):
            old_mode = game["mode"]
            game["mode"] = "reverse"
            game["mode_until"] = step + round(HUNTER_REVERSE_SECONDS * PHYSICS_HZ)
            game["recover_direction"] *= -1  # Alternate to escape dead ends.
            if not SMOKE_TEST:
                print(f"HUNTER STUCK in {old_mode.upper()}: moved {moved:.2f} m "
                      f"in {HUNTER_STUCK_CHECK_SECONDS:.1f}s. Reversing and turning.")
        game["anchor"] = (hunter_x, hunter_y)
        game["anchor_step"] = step

    if game["mode"] == "reverse":
        if step < game["mode_until"]:
            set_wheels(hunter_id, -HUNTER_SPEED * 0.6, -HUNTER_SPEED * 0.6)
            return
        game["mode"] = "spin"
        game["mode_until"] = step + round(HUNTER_RECOVERY_SPIN_SECONDS * PHYSICS_HZ)

    if game["mode"] == "spin":
        if step < game["mode_until"]:
            d = game["recover_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        # Do not point straight at the player again after getting unstuck;
        # commit to moving away from the box before returning to CHASE.
        game["mode"] = "advance"
        game["mode_until"] = step + round(HUNTER_RECOVERY_ADVANCE_SECONDS * PHYSICS_HZ)
        game["avoid_direction"] = game["recover_direction"]
        game["anchor"] = (hunter_x, hunter_y)
        game["anchor_step"] = step

    if game["mode"] == "avoid":
        if step < game["mode_until"]:
            d = game["avoid_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        # Do NOT immediately aim back at the player. First get past the box.
        game["mode"] = "advance"
        game["mode_until"] = step + round(HUNTER_ADVANCE_SECONDS * PHYSICS_HZ)

    # Sensors monitor the path every step. Drawing the rays is optional.
    front = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                          hunter_heading, 0, step)
    left = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                         hunter_heading, HUNTER_SENSOR_ANGLE, step)
    right = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                          hunter_heading, -HUNTER_SENSOR_ANGLE, step)

    # Finish the maneuver with a committed forward move. We intentionally
    # ignore the usual EARLY-warning distances during this short stage: they
    # may still see the box alongside the Husky. But all three rays remain
    # active, and emergency clearances can interrupt forward travel.
    if game["mode"] == "advance":
        too_close = (front < HUNTER_ADVANCE_FRONT_CLEARANCE
                     or left < HUNTER_ADVANCE_SIDE_CLEARANCE
                     or right < HUNTER_ADVANCE_SIDE_CLEARANCE)
        if too_close:
            game["mode"] = "avoid"
            # Maintain the chosen detour direction rather than oscillating.
            game["mode_until"] = step + round(0.32 * PHYSICS_HZ)
            d = game["avoid_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        if step < game["mode_until"]:
            speed = HUNTER_SPEED * HUNTER_ADVANCE_SPEED_FACTOR * speed_factor
            set_wheels(hunter_id, speed, speed)
            return
        game["mode"] = "chase"

    # During the normal chase, ANY ray can trigger a full avoidance maneuver.
    if (front < HUNTER_STOP_DISTANCE
            or left < HUNTER_SIDE_DISTANCE
            or right < HUNTER_SIDE_DISTANCE):
        game["mode"] = "avoid"
        game["avoid_direction"] = 1 if left >= right else -1
        game["mode_until"] = step + round(HUNTER_AVOID_TURN_SECONDS * PHYSICS_HZ)
        d = game["avoid_direction"] * HUNTER_TURN_SPEED
        set_wheels(hunter_id, -d, d)
        return

    # Pure pursuit: steer toward the player's current x/y coordinates.
    # Slower turning creates opportunities for the student to outmaneuver it.
    if abs(error) > 0.72:
        turn = HUNTER_TURN_SPEED if error > 0 else -HUNTER_TURN_SPEED
        set_wheels(hunter_id, -turn, turn)
    else:
        turn = max(-HUNTER_TURN_SPEED, min(HUNTER_TURN_SPEED, error * 5.0))
        drive = HUNTER_SPEED * speed_factor * (0.78 if abs(error) > 0.38 else 1.0)
        set_wheels(hunter_id, drive - turn, drive + turn)



# =====================================================================
# PART 5 — MARKERS, FLAG RULES, AND THE STUDENT CODING CHALLENGE
# =====================================================================
# The markers are VISUAL ONLY; they do not block the robot or its sensors.
def make_floor_marker(position, radius, color, label):
    x, y = position
    visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius,
                                 length=0.035, rgbaColor=[*color, 0.84])
    body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=visual,
                             basePosition=[x, y, 0.035])
    text_id = p.addUserDebugText(label, [x - 0.9, y + 0.2, 0.65],
                                 textColorRGB=list(color), textSize=1.15)
    return body, text_id


def celebrate_flag_pickup(game):
    """Prewritten graphics/game-event helper. Students just call it."""
    if game["flag_body"] is not None:
        p.removeBody(game["flag_body"])
        game["flag_body"] = None
    if game["flag_label"] is not None:
        if game["flag_label"] >= 0:  # DIRECT mode may have no debug label.
            p.removeUserDebugItem(game["flag_label"])
        game["flag_label"] = None
    print("FLAG CAPTURED! Guard alarm activated. Get back to the BLUE base!")


def update_mission(player_id, game):
    """This is the ONLY section students need to code."""
    player_x, player_y, _ = robot_xy_heading(player_id)
    # ==================================================================
    # INDEPENDENT CODING CHALLENGE — COMPLETE TASKS 1–4
    # ==================================================================
    # The movement controls, guard behavior, collision detection,
    # and game graphics are already programmed.
    #
    # Your job is to complete the Python statements below.
    # Use your previous notebooks and Python notes for help.
    #
    # IMPORTANT:
    # Some statements contain temporary values such as False, 0,
    # or pass. Replace these with working Python code.


    # ------------------------------------------------------------------
    # TASK 1 — CALCULATE DISTANCES TO THE FLAG AND HOME BASE
    # ------------------------------------------------------------------
    # The program must determine how far the Husky is from:
    #   1. The enemy flag
    #   2. The home base
    #
    # The player's current coordinates are:
    #   player_x = X coordinate
    #   player_y = Y coordinate
    #
    # The flag coordinates are stored in ENEMY_FLAG:
    #   ENEMY_FLAG[0] = X coordinate
    #   ENEMY_FLAG[1] = Y coordinate
    #
    # The home coordinates are stored in HOME_BASE:
    #   HOME_BASE[0] = X coordinate
    #   HOME_BASE[1] = Y coordinate
    #
    # Your job:
    #   1. Calculate the X and Y differences to the flag.
    #   2. Use math.hypot() to calculate distance_to_flag.
    #   3. Calculate the X and Y differences to home base.
    #   4. Use math.hypot() to calculate distance_to_home.
    #
    # Hint: math.hypot() accepts TWO arguments:
    #       an X difference and a Y difference.
    
    # DISTANCE TO ENEMY FLAG
    

    flag_x_difference = player_x -ENEMY_FLAG[0]  # TODO 1A: Calculate X difference.

    flag_y_difference = player_y -ENEMY_FLAG[1]  # TODO 1B: Calculate Y difference.



    #distance_to_flag = float("inf")  # TODO 1C: Use math.hypot().

    distance_to_flag = float(math.hypot(flag_x_difference, flag_y_difference))  # Initialize distance to flag.
    #distance_to_flag = float(math.hypot(flag_x_difference, flag_y_difference))  # REPLACE
    #distance_to_home = float(math.hypot(home_x_difference, home_y_difference))  # REPLACE
    # DISTANCE TO HOME BASE

    home_x_difference = player_x - HOME_BASE[0]  # TODO 1D: Calculate X difference.

    home_y_difference = player_y - HOME_BASE[1]  # TODO 1E: Calculate Y difference.

    #distance_to_home = float("inf")  # TODO 1F: Use math.hypot().
    distance_to_home = float(math.hypot(home_x_difference, home_y_difference))  # Initialize distance to home.


    # ------------------------------------------------------------------
    # TASK 2 — PICK UP THE ENEMY FLAG
    # ------------------------------------------------------------------
    # Write an if statement that checks TWO conditions:
    #
    #   1. Is the player close enough to the enemy flag?
    #   2. Does the player NOT already have the flag?
    #
    # BOTH conditions must be true before collecting the flag.
    #
    # If the conditions are true:
    #   1. Change game["has_flag"] to True.
    #   2. Call celebrate_flag_pickup(game).
    #
    # Hints:
    #   - Compare distance_to_flag with FLAG_RADIUS.
    #   - game["has_flag"] is a Boolean variable.
    #   - The 'not' operator reverses a Boolean value.
    #   - Decide whether to use 'and' or 'or'.
    #   - Indent both actions inside the if statement.
    if distance_to_flag <= FLAG_RADIUS and not game["has_flag"]:
        game["has_flag"] = True
        celebrate_flag_pickup(game)


    #if False:  # TODO 2A: Replace False with your condition.

        pass  # TODO 2B: Update game["has_flag"].

        pass  # TODO 2C: Call the celebration function.


    # ------------------------------------------------------------------
    # TASK 3 — RETURN HOME TO WIN
    # ------------------------------------------------------------------
    # Collecting the flag does NOT automatically win the game.
    #
    # The player must return to home base WITH the flag.
    #
    # Write an if statement that checks TWO conditions:
    #
    #   1. Does the player have the enemy flag?
    #   2. Is the player close enough to home base?
    #
    # BOTH conditions must be true to win.
    #
    # If the conditions are true:
    #   1. Change game["result"] to "WIN".
    #   2. Print a victory message.
    #
    # Hints:
    #   - Check game["has_flag"].
    #   - Compare distance_to_home with BASE_RADIUS.
    #   - Decide which Boolean operator to use.
    #   - Indent both actions inside your if statement.


    if game["has_flag"] and distance_to_home <= BASE_RADIUS:  # TODO 3A: Replace False with your winning condition.

        game["result"] = "WIN"  # TODO 3B: Update game["result"].

        print("Player has returned home with the flag. Victory!")     # TODO 3C: Print the victory message.


        # --------------------------------------------------------------
        # TASK 4 — CALCULATE AND DISPLAY COMPLETION TIME
        # --------------------------------------------------------------
        # This task belongs INSIDE the winning condition from Task 3.
        #
        # When the Husky successfully returns home with the flag,
        # display how long it took to complete the mission.
        #
        # Your job:
        #   1. Retrieve the simulation step count from game["step"].
        #   2. Convert those steps into simulated seconds.
        #   3. Store the result in completion_time.
        #   4. Print the completion time using an f-string.
        #   5. Format the result to ONE decimal place.
        #
        # Example output:
        #   Completion time: 38.5 seconds
        #
        # Hints:
        #   - PHYSICS_HZ represents simulation steps per second.
        #   - Divide steps by steps per second to obtain seconds.
        #   - Use :.1f inside an f-string for one decimal place.
        #
        # IMPORTANT:
        #   This code should execute ONLY after the player wins.


        completion_time = game["step"] / PHYSICS_HZ  # TODO 4A: Calculate elapsed seconds.
        print(f"Completion time: {completion_time:.1f} seconds")  # TODO 4B: Print completion_time using an f-string.

def update_guard(player_id, guard_id, game):
    """Patrol before flag pickup; chase the player after the flag is captured."""
    if game["has_flag"]:
        update_hunter(guard_id, player_id, game)
        return

    hx, hy, _ = robot_xy_heading(guard_id)
    goal = PATROL_POINTS[game["patrol_index"]]
    if math.hypot(hx - goal[0], hy - goal[1]) < PATROL_RADIUS:
        game["patrol_index"] = (game["patrol_index"] + 1) % len(PATROL_POINTS)
        goal = PATROL_POINTS[game["patrol_index"]]
    update_hunter(guard_id, player_id, game, target_xy=goal,
                  speed_factor=GUARD_PATROL_SPEED_FACTOR)


# =====================================================================
# PART 6 — GAME STATUS, RESET, AND THE MAIN LOOP (PREWRITTEN)
# =====================================================================
_hud_id = -1
_mode_hud_id = -1
_carry_label_id = -1


def status_hud(game, player_id, guard_id, force=False):
    """Update the status board twice per simulated second."""
    global _hud_id, _mode_hud_id, _carry_label_id
    if SMOKE_TEST:
        return
    update_period = max(1, PHYSICS_HZ // 2)
    if not force and game["step"] % update_period != 0:
        return
    remaining = max(0, MISSION_SECONDS - game["step"] / PHYSICS_HZ)
    px, py, _ = robot_xy_heading(player_id)
    hx, hy, _ = robot_xy_heading(guard_id)
    separation = math.hypot(px - hx, py - hy)
    if game["result"] == "WIN":
        label = "YOU WIN! FLAG RETURNED!   R = RESTART   Q = QUIT"
        color = [0.2, 1.0, 0.4]
    elif game["result"] == "CAUGHT":
        label = "CAUGHT BY GUARD!  R = RETRY   Q = QUIT"
        color = [1.0, 0.3, 0.25]
    elif game["result"] == "TIMEOUT":
        label = "TIME UP! FLAG NOT RETURNED.  R = RETRY   Q = QUIT"
        color = [1.0, 0.3, 0.25]
    elif game["has_flag"]:
        label = f"FLAG CAPTURED! GET HOME | {remaining:05.1f}s | GUARD {separation:.1f}m"
        color = [1, 0.95, 0.2]
    else:
        label = f"CAPTURE GOLD FLAG | {remaining:05.1f}s | GUARD {separation:.1f}m"
        color = [1, 0.95, 0.2]

    _hud_id = p.addUserDebugText(label, [-8.2, 8.05, 1.7],
                                 textColorRGB=color, textSize=1.18,
                                 replaceItemUniqueId=_hud_id)
    behavior = "CHASING!" if game["has_flag"] else "PATROLLING"
    _mode_hud_id = p.addUserDebugText(
        f"RED GUARD: {behavior} | NAV MODE: {game['mode'].upper()}",
        [-8.2, 7.35, 1.7], textColorRGB=[0.85, 0.95, 1.0], textSize=0.96,
        replaceItemUniqueId=_mode_hud_id)
    if game["has_flag"]:
        _carry_label_id = p.addUserDebugText(
            "FLAG!", [px - 0.35, py, 1.5], textColorRGB=[1.0, 0.9, 0.1],
            textSize=1.0, replaceItemUniqueId=_carry_label_id)


def reset_game():
    """Build fresh arena and dictionary when the player starts or presses R."""
    global _hud_id, _mode_hud_id, _carry_label_id
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / PHYSICS_HZ)
    p.setRealTimeSimulation(0)
    p.loadURDF("plane.urdf")
    create_arena()
    make_floor_marker(HOME_BASE, BASE_RADIUS, (0.15, 0.50, 1.0), "BLUE HOME")
    flag_body, flag_label = make_floor_marker(
        ENEMY_FLAG, FLAG_RADIUS, (1.0, 0.77, 0.10), "GOLD FLAG")
    player_id = make_husky(PLAYER_START, (0.16, 0.49, 1.0))
    guard_id = make_husky(HUNTER_START, (1.0, 0.16, 0.18))
    p.resetDebugVisualizerCamera(cameraDistance=25, cameraYaw=0,
                                  cameraPitch=-88, cameraTargetPosition=[0, 0, 0])
    _last_commands.clear()
    for name in _ray_ids:
        _ray_ids[name] = -1
    _hud_id = -1
    _mode_hud_id = -1
    _carry_label_id = -1
    game = {
        "step": 0,
        "result": "PLAYING",
        "has_flag": False,
        "flag_body": flag_body,
        "flag_label": flag_label,
        "patrol_index": 0,
        "mode": "chase",        # Guard's navigation mode (not the mission phase).
        "mode_until": 0,
        "avoid_direction": 1,
        "recover_direction": 1,
        "anchor": HUNTER_START,
        "anchor_step": 0,
        "wall_start": time.monotonic(),
        "report_start": time.monotonic(),
        "report_step": 0,
    }
    status_hud(game, player_id, guard_id, force=True)
    print("\nCAPTURE THE FLAG — NEW GAME")
    print("Blue = YOU. Capture the GOLD FLAG and return to BLUE HOME before time runs out.")
    print("Red guard PATROLS until you take the flag, then CHASES you.")
    print("WASD / arrows = drive | R = restart | Q = quit")
    print("Click the PyBullet window so keyboard controls work.")
    return player_id, guard_id, game


def smoke_test_mission():
    """Developer check without a GUI. Starter is expected to be incomplete."""
    player_id, guard_id, game = reset_game()
    for _ in range(6):
        p.stepSimulation()
    p.resetBasePositionAndOrientation(
        player_id, [ENEMY_FLAG[0], ENEMY_FLAG[1], 0.22], [0, 0, 0, 1])
    update_mission(player_id, game)
    first_step_ok = game["has_flag"]
    p.resetBasePositionAndOrientation(
        player_id, [HOME_BASE[0], HOME_BASE[1], 0.22], [0, 0, 0, 1])
    update_mission(player_id, game)
    if first_step_ok and game["result"] == "WIN":
        print("SMOKE TEST PASS: capture -> return -> win")
        return 0
    if not first_step_ok and game["result"] == "PLAYING":
        print("SMOKE TEST: starter is correctly unfinished; complete TASKS 1-3.")
        return 0
    print("SMOKE TEST FAIL: flag / home state transition is inconsistent")
    return 1


def run():
    if SMOKE_TEST:
        client = p.connect(p.DIRECT)
    else:
        client = p.connect(p.GUI, options=f"--width={WINDOW_WIDTH} --height={WINDOW_HEIGHT}")
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    if client < 0:
        raise RuntimeError("Cannot open PyBullet. Install pybullet and use a desktop GUI.")
    if not SMOKE_TEST:
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    try:
        if SMOKE_TEST:
            return smoke_test_mission()

        player_id, guard_id, game = reset_game()
        while p.isConnected():
            keys = p.getKeyboardEvents()
            if any(keys.get(k, 0) & p.KEY_WAS_TRIGGERED for k in (ord('q'), ord('Q'))):
                break
            if any(keys.get(k, 0) & p.KEY_WAS_TRIGGERED for k in (ord('r'), ord('R'))):
                player_id, guard_id, game = reset_game()
                continue

            if game["result"] != "PLAYING":
                time.sleep(0.04)
                continue

            player_teleop(player_id, keys)
            update_guard(player_id, guard_id, game)
            p.stepSimulation()
            game["step"] += 1
            elapsed = game["step"] / PHYSICS_HZ

            # Contact and time limit override a mission win on the same frame.
            if p.getContactPoints(bodyA=player_id, bodyB=guard_id):
                game["result"] = "CAUGHT"
                print(f"CAUGHT! The guard tagged you after {elapsed:.1f}s.")
            elif elapsed >= MISSION_SECONDS:
                game["result"] = "TIMEOUT"
                print(f"TIME UP! The {MISSION_SECONDS:.0f}-second limit expired.")
            else:
                update_mission(player_id, game)

            if game["result"] != "PLAYING":
                stop_robot(player_id)
                stop_robot(guard_id)
                status_hud(game, player_id, guard_id, force=True)

            status_hud(game, player_id, guard_id)
            now = time.monotonic()
            if now - game["report_start"] >= 5.0:
                sim_time = (game["step"] - game["report_step"]) / PHYSICS_HZ
                rate = sim_time / (now - game["report_start"])
                print(f"SIM SPEED: {rate:.2f}x | {max(0.0, MISSION_SECONDS - elapsed):.1f}s remain")
                game["report_start"] = now
                game["report_step"] = game["step"]

            if MAX_PLAYBACK_SPEED > 0:
                block = max(1, PHYSICS_HZ // 20)
                if game["step"] % block == 0:
                    target_wall = game["step"] / (PHYSICS_HZ * MAX_PLAYBACK_SPEED)
                    delay = target_wall - (now - game["wall_start"])
                    if delay > 0:
                        time.sleep(delay)
        return 0
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    sys.exit(run())
