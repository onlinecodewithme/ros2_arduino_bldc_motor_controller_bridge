#!/usr/bin/env python3

import numpy as np
from PIL import Image, ImageDraw
import os

def create_complex_map(width=400, height=400, output_dir='maps'):
    """
    Create a complex environment map with various obstacles.
    
    Args:
        width: Width of the map in pixels
        height: Height of the map in pixels
        output_dir: Directory to save the map files
    """
    # Create a white background (free space)
    map_image = Image.new('L', (width, height), color=255)
    draw = ImageDraw.Draw(map_image)
    
    # Add perimeter walls (10 pixels thick)
    wall_thickness = 10
    draw.rectangle([0, 0, width-1, wall_thickness], fill=0)  # Top wall
    draw.rectangle([0, 0, wall_thickness, height-1], fill=0)  # Left wall
    draw.rectangle([0, height-wall_thickness, width-1, height-1], fill=0)  # Bottom wall
    draw.rectangle([width-wall_thickness, 0, width-1, height-1], fill=0)  # Right wall
    
    # Add internal walls to create rooms and corridors
    
    # Horizontal walls
    draw.rectangle([100, 100, 300, 110], fill=0)  # Upper horizontal wall
    draw.rectangle([50, 200, 150, 210], fill=0)   # Middle-left horizontal wall
    draw.rectangle([250, 200, 350, 210], fill=0)  # Middle-right horizontal wall
    draw.rectangle([100, 300, 300, 310], fill=0)  # Lower horizontal wall
    
    # Vertical walls
    draw.rectangle([100, 110, 110, 200], fill=0)  # Upper-left vertical wall
    draw.rectangle([300, 110, 310, 200], fill=0)  # Upper-right vertical wall
    draw.rectangle([150, 210, 160, 300], fill=0)  # Lower-left vertical wall
    draw.rectangle([250, 210, 260, 300], fill=0)  # Lower-right vertical wall
    
    # Add doorways (gaps in walls)
    draw.rectangle([195, 100, 205, 110], fill=255)  # Door in upper horizontal wall
    draw.rectangle([100, 145, 110, 165], fill=255)  # Door in upper-left vertical wall
    draw.rectangle([300, 145, 310, 165], fill=255)  # Door in upper-right vertical wall
    draw.rectangle([90, 200, 110, 210], fill=255)   # Door in middle-left horizontal wall
    draw.rectangle([290, 200, 310, 210], fill=255)  # Door in middle-right horizontal wall
    draw.rectangle([150, 245, 160, 265], fill=255)  # Door in lower-left vertical wall
    draw.rectangle([250, 245, 260, 265], fill=255)  # Door in lower-right vertical wall
    draw.rectangle([195, 300, 205, 310], fill=255)  # Door in lower horizontal wall
    
    # Add complex obstacles
    
    # Circular obstacles (representing pillars or round furniture)
    for center, radius in [
        ((50, 50), 15),    # Upper left
        ((350, 50), 15),   # Upper right
        ((50, 350), 15),   # Lower left
        ((350, 350), 15),  # Lower right
        ((200, 250), 20),  # Center
    ]:
        x, y = center
        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                if dx*dx + dy*dy <= radius*radius:
                    draw.point((x+dx, y+dy), fill=0)
    
    # L-shaped obstacles
    draw.rectangle([50, 120, 70, 170], fill=0)
    draw.rectangle([50, 150, 90, 170], fill=0)
    
    draw.rectangle([330, 120, 350, 170], fill=0)
    draw.rectangle([310, 150, 350, 170], fill=0)
    
    draw.rectangle([50, 230, 70, 280], fill=0)
    draw.rectangle([50, 230, 90, 250], fill=0)
    
    draw.rectangle([330, 230, 350, 280], fill=0)
    draw.rectangle([310, 230, 350, 250], fill=0)
    
    # Narrow passages
    draw.rectangle([195, 195, 205, 300], fill=0)
    draw.rectangle([160, 195, 250, 205], fill=0)
    draw.rectangle([160, 195, 170, 245], fill=0)
    draw.rectangle([240, 195, 250, 245], fill=0)
    
    # Add some random small obstacles
    np.random.seed(42)  # For reproducibility
    for _ in range(30):
        x = np.random.randint(20, width-20)
        y = np.random.randint(20, height-20)
        size = np.random.randint(5, 15)
        
        # Skip if too close to doorways or existing paths
        if ((abs(x-200) < 30 and abs(y-105) < 30) or
            (abs(x-105) < 30 and abs(y-155) < 30) or
            (abs(x-305) < 30 and abs(y-155) < 30) or
            (abs(x-100) < 30 and abs(y-205) < 30) or
            (abs(x-300) < 30 and abs(y-205) < 30) or
            (abs(x-155) < 30 and abs(y-255) < 30) or
            (abs(x-255) < 30 and abs(y-255) < 30) or
            (abs(x-200) < 30 and abs(y-305) < 30)):
            continue
        
        shape = np.random.choice(['circle', 'square'])
        if shape == 'circle':
            for dx in range(-size, size+1):
                for dy in range(-size, size+1):
                    if dx*dx + dy*dy <= size*size:
                        if 0 <= x+dx < width and 0 <= y+dy < height:
                            draw.point((x+dx, y+dy), fill=0)
        else:
            draw.rectangle([x-size//2, y-size//2, x+size//2, y+size//2], fill=0)
    
    # Make sure the start position is clear (center of the map)
    start_area_radius = 30
    for dx in range(-start_area_radius, start_area_radius+1):
        for dy in range(-start_area_radius, start_area_radius+1):
            if dx*dx + dy*dy <= start_area_radius*start_area_radius:
                if 0 <= width//2+dx < width and 0 <= height//2+dy < height:
                    draw.point((width//2+dx, height//2+dy), fill=255)
    
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Save the map as a PGM file
    map_file = os.path.join(output_dir, 'complex_map.pgm')
    map_image.save(map_file, format='PPM')
    
    # Create the YAML metadata file
    yaml_file = os.path.join(output_dir, 'complex_map.yaml')
    resolution = 0.05  # meters per pixel
    origin = [-10.0, -10.0, 0.0]  # [x, y, yaw]
    
    with open(yaml_file, 'w') as f:
        f.write(f"image: complex_map.pgm\n")
        f.write(f"resolution: {resolution}  # meters per pixel\n")
        f.write(f"origin: {origin}  # [x, y, yaw]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.196\n")
        f.write(f"mode: scale\n")
    
    print(f"Created complex map at {map_file}")
    print(f"Created YAML metadata at {yaml_file}")
    print(f"Map dimensions: {width}x{height} pixels")
    print(f"Map size: {width*resolution:.1f}x{height*resolution:.1f} meters")
    print(f"Origin: {origin}")

if __name__ == '__main__':
    create_complex_map(output_dir='src/robot_navigation/maps')
