import os
import json
import csv

def export_run_to_csv(run_dirs, output_file):
    """
    Reads data from a list of directories and exports them to a single CSV.
    """
    headers = ["Model", "Category", "Task ID", "Status", "Score", "Pass Rate", "Tokens", "Failed Functions"]
    rows = []

    # Iterate through all provided directories for this specific run
    for results_dir in run_dirs:
        if not os.path.exists(results_dir):
            print(f"  [Warning] Directory '{results_dir}' not found, skipping...")
            continue

        # Iterate through model folders (e.g., openai_gpt-4o)
        for model_folder in os.listdir(results_dir):
            model_path = os.path.join(results_dir, model_folder)
            if not os.path.isdir(model_path): continue
            
            # Iterate through category folders (e.g., action_server)
            for cat_folder in os.listdir(model_path):
                summary_path = os.path.join(model_path, cat_folder, "summary.json")
                
                if os.path.exists(summary_path):
                    with open(summary_path, "r", encoding="utf-8") as f:
                        tasks = json.load(f)
                        for task in tasks:
                            # Extract failed function names into a single string
                            failed_funcs = ", ".join([fc['test_function'] for fc in task.get('failed_cases', [])])
                            
                            rows.append([
                                model_folder,
                                cat_folder,
                                task['task_id'],
                                task['status'],
                                task['score'],
                                task['pass_rate'],
                                task['tokens'],
                                failed_funcs if failed_funcs else "None"
                            ])

    # Only write the CSV if we actually found rows
    if rows:
        with open(output_file, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(rows)
        print(f"✅ Success! Exported {len(rows)} tasks to: {output_file}")
    else:
        print(f"⚠️ No data found to export for {output_file}")

if __name__ == "__main__":
    print("Starting data export for all 5 runs...\n")
    
    # Run 1: We must combine the old 'results' (4 models) and new 'results1' (2 models)
    print("Processing Run 1...")
    export_run_to_csv(["results", "results1"], "benchmark_system_run1.csv")
    
    # Run 2 to 5: Directly use results2, results3, results4, results5
    print("\nProcessing Run 2...")
    export_run_to_csv(["results2"], "benchmark_system_run2.csv")
    
    print("\nProcessing Run 3...")
    export_run_to_csv(["results3"], "benchmark_system_run3.csv")
    
    print("\nProcessing Run 4...")
    export_run_to_csv(["results4"], "benchmark_system_run4.csv")
    
    print("\nProcessing Run 5...")
    export_run_to_csv(["results5"], "benchmark_system_run5.csv")
    
    print("\n🎉 All exports completed!")

