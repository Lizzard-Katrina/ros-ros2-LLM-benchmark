import subprocess
import os
import argparse

LEVELS = [
#    "interface_level",
#    "behavior_level",
    "system_level"
]

def main():
    parser = argparse.ArgumentParser(description="Multi-level ROS benchmark runner")
    parser.add_argument("--reasoning_effort", type=str, default="none", help="Reasoning effort override")
    parser.add_argument("--runs", type=int, default=4, help="Total loops to execute")
    parser.add_argument("--start_run", type=int, default=2, help="Start index for results directory")
    args = parser.parse_args()

    root_dir = os.getcwd()
    
    for level in LEVELS:
        print(f"\n{'#'*40}")
        print(f" STARTING LEVEL: {level} ".center(40, "#"))
        print(f"{'#'*40}\n")
        
        script_path = os.path.join(root_dir, level, "run_all_5.py")
        
        if os.path.exists(script_path):
            try:
                cmd = [
                    "python3", "run_all_5.py",
                    "--reasoning_effort", args.reasoning_effort,
                    "--runs", str(args.runs),
                    "--start_run", str(args.start_run)
                ]
                subprocess.run(cmd, cwd=os.path.join(root_dir, level), check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error running {level}: {e}")
        else:
            print(f"Warning: No run_all_5.py found in {level}")

    print("\nALL LEVELS COMPLETED! Check output directories.")

if __name__ == "__main__":
    main()
