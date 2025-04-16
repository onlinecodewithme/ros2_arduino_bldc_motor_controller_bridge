# Complex Navigation Map

This directory contains a complex environment map designed to test advanced navigation capabilities of the robot.

## Map Features

The complex map (`complex_map.pgm` and `complex_map.yaml`) includes:

- **Overall Size**: 20m x 20m (400x400 pixels at 0.05m/pixel resolution)
- **Room Structure**: Multi-room layout with wall separations
- **Doorways**: Strategic openings between rooms requiring precise navigation
- **Varied Obstacles**:
  - Round obstacles (pillars)
  - L-shaped obstacles
  - Narrow passages requiring careful planning
  - Random small obstacles of different shapes scattered throughout
- **Clear Starting Area**: The center of the map has a clear space for the robot to start

## Map Visual Description

The map contains:
- Perimeter walls (10 pixels thick)
- Internal horizontal and vertical walls creating rooms and corridors
- Strategic doorways providing paths between different areas
- Circular obstacles in corners and a larger one in the center
- L-shaped obstacles in multiple locations
- A complex narrow passage system in the middle
- Random obstacles scattered throughout the environment

## Using the Map

To use this map with navigation:

1. Make sure the map files are in place:
   - `complex_map.pgm` (the image file)
   - `complex_map.yaml` (the configuration file)

2. Run the navigation system with this map using:
   ```bash
   ./src/robot_navigation/run_complex_map_navigation.sh
   ```

3. In RViz:
   - The map will be displayed showing all obstacles
   - Use the "2D Nav Goal" button to set destinations
   - Watch as the robot plans and navigates through the complex environment

## Map Parameters

The map configuration is defined in `complex_map.yaml`:
- **Resolution**: 0.05 meters per pixel
- **Origin**: [-10.0, -10.0, 0.0] (x, y, yaw)
- **Occupation Thresholds**:
  - occupied_thresh: 0.65
  - free_thresh: 0.196

## Regenerating the Map

If needed, you can regenerate the map by running:
```bash
python3 src/robot_navigation/create_complex_map.py
```

This will recreate both the `.pgm` and `.yaml` files.

## Navigation Challenges

This map presents several navigation challenges:
- Path planning through narrow doorways
- Avoiding closely placed obstacles
- Finding optimal routes through complex room arrangements
- Handling dead-ends and requiring re-planning
