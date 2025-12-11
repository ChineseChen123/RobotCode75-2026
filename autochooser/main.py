import os
import sys
import math
import json
import pygame
import pygame.gfxdraw

# --- Helper Functions ---

def snap_angle(angle):
    """Snap angle if within 0.05 radians of a candidate.
    Candidates: 0, π/4, π/2, -π/4, -π/2, and ±π."""
    if abs(angle - math.pi) < 0.05 or abs(angle + math.pi) < 0.05:
        return math.pi
    candidates = [0, math.pi/4, math.pi/2, -math.pi/4, -math.pi/2]
    for cand in candidates:
        if abs(angle - cand) < 0.05:
            return cand
    return angle

def rotate_point(x, y, theta):
    """
    Rotate point (x, y) clockwise by theta radians.
    Uses the matrix:
         [ cosθ   sinθ ]
         [ -sinθ  cosθ ]
    """
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    new_x = x * cos_theta + y * sin_theta
    new_y = -x * sin_theta + y * cos_theta
    return new_x, new_y

def draw_aa_thick_line(surface, color, start, end, thickness):
    """
    Draw a thick anti-aliased line between start and end using super-sampling.
    """
    scale = 4  # Super-sampling factor
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return
    dx /= length
    dy /= length
    offset_x = -dy * (thickness / 2.0)
    offset_y = dx * (thickness / 2.0)
    pts = [
        (start[0] + offset_x, start[1] + offset_y),
        (start[0] - offset_x, start[1] - offset_y),
        (end[0] - offset_x, end[1] - offset_y),
        (end[0] + offset_x, end[1] + offset_y)
    ]
    scaled_pts = [(int(x * scale), int(y * scale)) for (x, y) in pts]
    xs = [p[0] for p in scaled_pts]
    ys = [p[1] for p in scaled_pts]
    min_x = min(xs)
    min_y = min(ys)
    width = max(xs) - min_x + 2
    height = max(ys) - min_y + 2
    temp_surf = pygame.Surface((width, height), pygame.SRCALPHA)
    adjusted_pts = [(p[0] - min_x, p[1] - min_y) for p in scaled_pts]
    pygame.gfxdraw.filled_polygon(temp_surf, adjusted_pts, color)
    smooth_surf = pygame.transform.smoothscale(temp_surf, (width // scale, height // scale))
    dest_x = int(min_x / scale)
    dest_y = int(min_y / scale)
    surface.blit(smooth_surf, (dest_x, dest_y))

def draw_smooth_polygon(dest_surf, points, color, scale=8):
    """
    Draw a filled polygon with smooth edges using super-sampling.
    Draws on the entire dest_surf.
    """
    high_res_size = (dest_surf.get_width()*scale, dest_surf.get_height()*scale)
    temp_surf = pygame.Surface(high_res_size, pygame.SRCALPHA)
    scaled_points = [(int(x*scale), int(y*scale)) for (x,y) in points]
    pygame.gfxdraw.filled_polygon(temp_surf, scaled_points, color)
    smooth = pygame.transform.smoothscale(temp_surf, dest_surf.get_size())
    dest_surf.blit(smooth, (0,0))

def draw_smooth_polygon_outline(dest_surf, points, color, thickness, scale=4):
    """
    Draw an outlined polygon with smooth edges using super-sampling.
    """
    high_res_size = (dest_surf.get_width()*scale, dest_surf.get_height()*scale)
    temp_surf = pygame.Surface(high_res_size, pygame.SRCALPHA)
    scaled_points = [(int(x*scale), int(y*scale)) for (x,y) in points]
    # Multiply thickness by scale.
    pygame.draw.polygon(temp_surf, color, scaled_points, thickness*scale)
    smooth = pygame.transform.smoothscale(temp_surf, dest_surf.get_size())
    dest_surf.blit(smooth, (0,0))

# --- Data Loading ---

traj_files = [
    "trajectories/hb-rbl.traj",
    "trajectories/ht-rtl.traj",
    "trajectories/p-hb.traj",
    "trajectories/p-rbr.traj",
    "trajectories/rbl-hb.traj",
    "trajectories/rbl-p.traj",
    "trajectories/rbr-hb.traj",
    "trajectories/rr-p.traj",
    "trajectories/rtl-ht.traj",
    "trajectories/rtr-ht.traj",
    "trajectories/sb-rbr.traj",
    "trajectories/sm-rr.traj",
    "trajectories/st-rtr.traj"
]

# Each waypoint is expected to have "x", "y", and "heading" (radians)
waypoints = {}
for traj in traj_files:
    if not os.path.exists(traj):
        raise FileNotFoundError(f"Trajectory file not found: {traj}")
    start_label, end_label = traj[len("trajectories/"):-len(".traj")].split("-")
    with open(traj, "r") as f:
        data = json.load(f)
        waypoints[start_label] = data["snapshot"]["waypoints"][0]
        waypoints[end_label] = data["snapshot"]["waypoints"][-1]

starting_trajs = {
    "st": "trajectories/st-rtr.traj",
    "sm": "trajectories/sm-rr.traj",
    "sb": "trajectories/sb-rbr.traj"
}

def getWaypointFromTraj(traj_file):
    """
    Reads a trajectory file and returns two robot states:
      (x, y, heading) for the start and for the end.
    Heading is snapped.
    """
    base = os.path.basename(traj_file)
    name, _ = os.path.splitext(base)
    try:
        start_label, end_label = name.split("-")
    except ValueError:
        raise ValueError("Filename must be in the format 'x-y.traj'")
    wp1 = waypoints[start_label]
    wp2 = waypoints[end_label]
    return ((wp1["x"], wp1["y"], snap_angle(wp1["heading"])),
            (wp2["x"], wp2["y"], snap_angle(wp2["heading"])))

def get_possible_next(current_label):
    """
    Returns a list of next ghost options.
    Each option is a tuple: (destination_label, (x, y, heading), traj_file).
    For starting state (None), return starting options.
    """
    options = []
    if current_label is None:
        for label in ["st", "sm", "sb"]:
            traj_file = starting_trajs[label]
            wp = waypoints[label]
            options.append((label, (wp["x"], wp["y"], snap_angle(wp["heading"])), traj_file))
    else:
        for traj_file in traj_files:
            base = os.path.basename(traj_file)
            name, _ = os.path.splitext(base)
            try:
                start_label, end_label = name.split("-")
            except ValueError:
                continue
            if start_label == current_label:
                _, dest = getWaypointFromTraj(traj_file)
                options.append((end_label, dest, traj_file))
    return options

# --- Drawing Robot Representations ---

def draw_robot(surface, center, size, heading, base_color, alpha):
    """
    Draws a filled, rotated square (representing the robot) with a smooth edge,
    a center dot, and an arrow indicating the front.
    Rotation is done using clockwise rotation so that 0 rad means facing right.
    """
    heading = snap_angle(heading)
    pad = 10
    robot_surf_size = int(size + pad)
    robot_surf = pygame.Surface((robot_surf_size, robot_surf_size), pygame.SRCALPHA)
    robot_center = (robot_surf_size/2, robot_surf_size/2)
    half = size/2

    # Define square corners relative to center.
    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
    rotated = []
    for (x, y) in corners:
        rx, ry = rotate_point(x, y, heading)
        rotated.append((robot_center[0] + rx, robot_center[1] + ry))
    
    color = (*base_color, alpha)
    # Use super-sampling to draw a smooth filled polygon.
    draw_smooth_polygon(robot_surf, rotated, color, scale=4)
    
    # Draw a center dot.
    dot_radius = 2
    pygame.gfxdraw.filled_circle(robot_surf, int(robot_center[0]), int(robot_center[1]), dot_radius, (0,0,0,255))
    pygame.gfxdraw.aacircle(robot_surf, int(robot_center[0]), int(robot_center[1]), dot_radius, (0,0,0,255))
    
    # Draw arrow indicating the front.
    arrow_base = (robot_center[0] + half * math.cos(heading),
                  robot_center[1] - half * math.sin(heading))
    arrow_tip = (robot_center[0] + (half+4) * math.cos(heading),
                 robot_center[1] - (half+4) * math.sin(heading))
    arrow_color = (0,0,0,alpha)
    pygame.draw.line(robot_surf, arrow_color, arrow_base, arrow_tip, 2)
    ah_size = 4
    dir_vec = (math.cos(heading), -math.sin(heading))
    perp_vec = (dir_vec[1], -dir_vec[0])
    base_pt = (arrow_tip[0] - 4*dir_vec[0], arrow_tip[1] - 4*dir_vec[1])
    left_pt = (base_pt[0] + ah_size * perp_vec[0], base_pt[1] + ah_size * perp_vec[1])
    right_pt = (base_pt[0] - ah_size * perp_vec[0], base_pt[1] - ah_size * perp_vec[1])
    arrow_head = [arrow_tip, left_pt, right_pt]
    pygame.gfxdraw.filled_polygon(robot_surf, arrow_head, arrow_color)
    pygame.gfxdraw.aapolygon(robot_surf, arrow_head, arrow_color)
    
    blit_pos = (int(center[0] - robot_surf_size/2), int(center[1] - robot_surf_size/2))
    surface.blit(robot_surf, blit_pos)

def draw_ghost_robot(surface, center, size, heading, outline_color, alpha):
    """
    Draws an outlined, rotated square (ghost robot) with smooth edges,
    a center dot, and an arrow indicating the front.
    """
    heading = snap_angle(heading)
    pad = 10
    robot_surf_size = int(size + pad)
    robot_surf = pygame.Surface((robot_surf_size, robot_surf_size), pygame.SRCALPHA)
    robot_center = (robot_surf_size/2, robot_surf_size/2)
    half = size/2

    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
    rotated = []
    for (x, y) in corners:
        rx, ry = rotate_point(x, y, heading)
        rotated.append((robot_center[0] + rx, robot_center[1] + ry))
    
    color = (*outline_color, alpha)
    # Draw a smooth outlined polygon.
    draw_smooth_polygon_outline(robot_surf, rotated, color, thickness=2, scale=4)
    
    dot_radius = 2
    pygame.gfxdraw.filled_circle(robot_surf, int(robot_center[0]), int(robot_center[1]), dot_radius, (0,0,0,255))
    pygame.gfxdraw.aacircle(robot_surf, int(robot_center[0]), int(robot_center[1]), dot_radius, (0,0,0,255))
    
    arrow_base = (robot_center[0] + half * math.cos(heading),
                  robot_center[1] - half * math.sin(heading))
    arrow_tip = (robot_center[0] + (half+4) * math.cos(heading),
                 robot_center[1] - (half+4) * math.sin(heading))
    arrow_color = (0,0,0,alpha)
    pygame.draw.line(robot_surf, arrow_color, arrow_base, arrow_tip, 2)
    ah_size = 4
    dir_vec = (math.cos(heading), -math.sin(heading))
    perp_vec = (dir_vec[1], -dir_vec[0])
    base_pt = (arrow_tip[0] - 4*dir_vec[0], arrow_tip[1] - 4*dir_vec[1])
    left_pt = (base_pt[0] + ah_size * perp_vec[0], base_pt[1] + ah_size * perp_vec[1])
    right_pt = (base_pt[0] - ah_size * perp_vec[0], base_pt[1] - ah_size * perp_vec[1])
    arrow_head = [arrow_tip, left_pt, right_pt]
    pygame.gfxdraw.aapolygon(robot_surf, arrow_head, arrow_color)
    pygame.gfxdraw.polygon(robot_surf, arrow_head, arrow_color)
    
    blit_pos = (int(center[0] - robot_surf_size/2), int(center[1] - robot_surf_size/2))
    surface.blit(robot_surf, blit_pos)

# --- UI Drawing Helper ---

def draw_rounded_rect(surface, rect, color, corner_radius):
    pygame.draw.rect(surface, color, rect, border_radius=corner_radius)

# --- Main UI Code ---

def main():
    pygame.init()
    reduction_factor = 1.25
    image_file = "./reefscapefield.png"  # Update path as needed.
    try:
        field_img = pygame.image.load(image_file)
    except pygame.error as e:
        print("Unable to load image:", e)
        sys.exit()
    orig_rect = field_img.get_rect()
    new_size = (int(orig_rect.width/reduction_factor), int(orig_rect.height/reduction_factor))
    field_img = pygame.transform.smoothscale(field_img, new_size)
    screen = pygame.display.set_mode(new_size)
    pygame.display.set_caption("FRC Field Trajectory Builder")

    # Conversion parameters.
    pixel_offset_x = 255/reduction_factor
    pixel_offset_y = 788/reduction_factor
    meter_x_ref = 17.577
    meter_y_ref = 8.082
    pixel_x_ref = 1653/reduction_factor
    pixel_y_ref = 131/reduction_factor
    scale_x = (pixel_x_ref - pixel_offset_x)/meter_x_ref
    scale_y = (pixel_offset_y - pixel_y_ref)/meter_y_ref

    def meter_to_pixel(meter_pos):
        meter_x, meter_y = meter_pos[:2]
        pixel_x = pixel_offset_x + meter_x * scale_x
        pixel_y = pixel_offset_y - meter_y * scale_y
        return (int(pixel_x), int(pixel_y))

    try:
        font = pygame.font.SysFont("Segoe UI", 18)
        large_font = pygame.font.SysFont("Segoe UI", 24)
    except Exception:
        font = pygame.font.SysFont("Arial", 18)
        large_font = pygame.font.SysFont("Arial", 24)

    BLACK = (0,0,0)
    WHITE = (255,255,255)
    LIGHT_GRAY = (200,200,200)
    DARK_GRAY = (50,50,50)
    RED = (255,0,0)
    GREEN = (0,255,0)
    BLUE = (0,120,255)
    YELLOW = (255,255,0)

    reset_button_rect = pygame.Rect(new_size[0]-360, 20, 100, 40)
    undo_button_rect  = pygame.Rect(new_size[0]-240, 20, 100, 40)
    done_button_rect  = pygame.Rect(new_size[0]-120, 20, 100, 40)

    chain = []       # List of location labels.
    chain_trajs = [] # Trajectory files connecting the chain.
    ghost_options = get_possible_next(None)

    clock = pygame.time.Clock()
    running = True
    robot_size = 32

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                if reset_button_rect.collidepoint(mouse_pos):
                    chain = []
                    chain_trajs = []
                    ghost_options = get_possible_next(None)
                    continue
                if undo_button_rect.collidepoint(mouse_pos):
                    if chain:
                        chain.pop()
                        if chain_trajs:
                            chain_trajs.pop()
                        ghost_options = get_possible_next(chain[-1] if chain else None)
                    continue
                if done_button_rect.collidepoint(mouse_pos):
                    print("\nTrajectory Chain Built:")
                    if chain:
                        print("Chain:", " -> ".join(chain))
                        if chain_trajs:
                            print("Traj files:")
                            for t in chain_trajs:
                                print("  ", t)
                        else:
                            print("(Only a starting point selected.)")
                    else:
                        print("(No trajectory built.)")
                    running = False
                    continue
                clicked_option = None
                for option in ghost_options:
                    label, state, traj_file = option
                    ghost_pixel = meter_to_pixel(state)
                    if math.hypot(mouse_pos[0]-ghost_pixel[0], mouse_pos[1]-ghost_pixel[1]) < 15:
                        clicked_option = option
                        break
                if clicked_option is not None:
                    selected_label, state, traj_file = clicked_option
                    if not chain:
                        chain.append(selected_label)
                    else:
                        chain_trajs.append(traj_file)
                        chain.append(selected_label)
                    ghost_options = get_possible_next(chain[-1])
        
        screen.fill(DARK_GRAY)
        screen.blit(field_img, (0,0))
        

        for option in ghost_options:
            label, state, traj_file = option
            ghost_pos = meter_to_pixel(state)
            draw_ghost_robot(screen, ghost_pos, robot_size, state[2], GREEN, 220)
            label_surf = font.render(label, True, GREEN)
            screen.blit(label_surf, (ghost_pos[0]+20, ghost_pos[1]-14))

        if chain:
            n = len(chain)
            chain_alphas = []
            chain_pixels = [meter_to_pixel((waypoints[label]["x"], waypoints[label]["y"])) for label in chain]
            for idx, label in enumerate(chain):
                alpha = 255 if n == 1 else int(128 + (idx/(n-1))*(255-128))
                chain_alphas.append(alpha)
            if len(chain_pixels) > 1:
                for i in range(len(chain_pixels)-1):
                    avg_alpha = int((chain_alphas[i]+chain_alphas[i+1])/2)
                    line_color = (BLUE[0], BLUE[1], BLUE[2], avg_alpha)
                    temp_line = pygame.Surface(new_size, pygame.SRCALPHA)
                    draw_aa_thick_line(temp_line, line_color, chain_pixels[i], chain_pixels[i+1], 6)
                    screen.blit(temp_line, (0,0))
            for idx, label in enumerate(chain):
                alpha = 255 if n == 1 else int(128 + (idx/(n-1))*(255-128))
                wp = waypoints[label]
                pos = meter_to_pixel((wp["x"], wp["y"]))
                draw_robot(screen, pos, robot_size, wp["heading"], BLUE, alpha)

            chain_text = "Chain: " + " -> ".join(chain)
            text_surf = font.render(chain_text, True, WHITE)
            screen.blit(text_surf, (20,20))
        
        
        for rect, text in [(reset_button_rect, "Reset"), (undo_button_rect, "Undo"), (done_button_rect, "Done")]:
            button_surf = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
            draw_rounded_rect(button_surf, pygame.Rect(0,0,rect.width,rect.height), LIGHT_GRAY, 8)
            txt_surf = large_font.render(text, True, BLACK)
            txt_rect = txt_surf.get_rect(center=(rect.width//2, rect.height//2))
            button_surf.blit(txt_surf, txt_rect)
            shadow = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
            shadow.fill((0,0,0,50))
            screen.blit(shadow, (rect.x+3, rect.y+3))
            screen.blit(button_surf, (rect.x, rect.y))
        
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()

if __name__ == "__main__":
    main()
