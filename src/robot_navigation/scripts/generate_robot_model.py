#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
import tempfile
from pathlib import Path

def check_hi3dgen_installed():
    """Check if Hi3DGen is available and install if needed."""
    try:
        # Check if the repository is already cloned
        hi3dgen_dir = os.path.expanduser("~/Hi3DGen")
        if not os.path.exists(hi3dgen_dir):
            print("Hi3DGen not found. Cloning from GitHub...")
            subprocess.run(
                ["git", "clone", "https://github.com/Stable-X/Hi3DGen.git", hi3dgen_dir], 
                check=True
            )
            print("Hi3DGen cloned successfully.")
        
        # Check for Python dependencies
        print("Installing/checking Python dependencies...")
        subprocess.run(
            ["pip", "install", "-r", os.path.join(hi3dgen_dir, "requirements.txt")],
            check=True
        )
        
        return hi3dgen_dir
    except subprocess.CalledProcessError as e:
        print(f"Error setting up Hi3DGen: {e}")
        sys.exit(1)

def generate_3d_model(image_path, output_dir, hi3dgen_dir):
    """Generate 3D model from image using Hi3DGen."""
    try:
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Prepare the command to run Hi3DGen
        cmd = [
            "python", 
            os.path.join(hi3dgen_dir, "main.py"),
            "--image_path", image_path,
            "--output_dir", output_dir,
            "--mode", "image",  # Assuming the tool has an image mode
            "--model_type", "tracked_vehicle",  # May need adjustment based on actual parameters
            "--export_format", "obj,dae",  # Export in formats compatible with ROS
        ]
        
        print(f"Running Hi3DGen with command: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        print(f"3D model generation completed. Models saved to {output_dir}")
        
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error generating 3D model: {e}")
        return False

def update_urdf_with_new_model(urdf_path, model_path):
    """Update the URDF file to use the newly generated model."""
    try:
        # Read the current URDF content
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        
        # Look for mesh filename references
        if "simple_track.dae" in urdf_content:
            # Replace with the new model
            new_model_filename = os.path.basename(model_path)
            updated_content = urdf_content.replace(
                "package://robot_navigation/urdf/meshes/simple_track.dae",
                f"package://robot_navigation/urdf/meshes/{new_model_filename}"
            )
            
            # Save the updated URDF
            with open(urdf_path, 'w') as f:
                f.write(updated_content)
            
            print(f"URDF file {urdf_path} updated to use new model.")
            return True
        else:
            print("Could not find mesh references in URDF to update.")
            return False
    except Exception as e:
        print(f"Error updating URDF: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Generate 3D model from robot image using Hi3DGen")
    parser.add_argument("--image", required=True, help="Path to robot image")
    parser.add_argument("--output", default=None, help="Output directory for the 3D model")
    parser.add_argument("--urdf", default=None, help="Path to URDF file to update")
    
    args = parser.parse_args()
    
    # Check if image exists
    if not os.path.exists(args.image):
        print(f"Error: Image file {args.image} not found.")
        sys.exit(1)
    
    # Set default output directory if not provided
    if args.output is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.output = os.path.join(script_dir, "..", "urdf", "meshes")
    
    # Set default URDF path if not provided
    if args.urdf is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.urdf = os.path.join(script_dir, "..", "urdf", "tracked_robot.urdf.xacro")
    
    # Check Hi3DGen installation
    hi3dgen_dir = check_hi3dgen_installed()
    
    # Generate 3D model
    print(f"Generating 3D model from image: {args.image}")
    if generate_3d_model(args.image, args.output, hi3dgen_dir):
        # Find the generated model
        model_files = list(Path(args.output).glob("*.dae")) + list(Path(args.output).glob("*.obj"))
        if model_files:
            model_path = str(model_files[0])
            print(f"Generated model: {model_path}")
            
            # Update URDF
            if args.urdf and os.path.exists(args.urdf):
                update_urdf_with_new_model(args.urdf, model_path)
            else:
                print(f"URDF file not found: {args.urdf}")
        else:
            print("No model files generated.")
    else:
        print("Failed to generate 3D model.")

if __name__ == "__main__":
    main()
