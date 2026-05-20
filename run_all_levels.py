import subprocess
import os

# Define the levels you want to run tonight
LEVELS = [
    "interface_level",
    "behavior_level",
    "system_level"
]

def main():
    root_dir = os.getcwd()
    
    for level in LEVELS:
        print(f"\n{'#'*40}")
        print(f" STARTING LEVEL: {level} ".center(40, "#"))
        print(f"{'#'*40}\n")
        
        # Path to the run_all.py inside each level
        script_path = os.path.join(root_dir, level, "run_all.py")
        
        if os.path.exists(script_path):
            # Change directory to the level folder so paths inside run_all.py stay relative
            try:
                subprocess.run(["python3", "run_all.py"], cwd=os.path.join(root_dir, level), check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error running {level}: {e}")
        else:
            print(f"Warning: No run_all.py found in {level}")

    print("\nALL LEVELS COMPLETED! Check the 'results' directory.")

if __name__ == "__main__":
    main()
