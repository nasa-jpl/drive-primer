#!/usr/bin/env python3
import argparse
import subprocess
import sys
import shutil

def main():
    available_commands = {
        'config' : ('dp-gui', 'tkinter GUI to generate bayopt definition files'),
        'optimize': ('dp-opt', 'Bayesian optimization from downlink telemetry'),
        'dashboard' : ('optuna-dashboard', 'Run optuna-dashboard'),
        'csv2rksml': ('dp-csv2rksml', 'Log CSV to RKSML converter'),
        'table': ('dp-table', 'Generate slip table from soil parameters'),
        'table-gp' : ('dp-table-gp', 'Use slip table output to fit GP') 
    }
    
    parser = argparse.ArgumentParser(
        description='High fidelity simulation in the loop analysis',
        prog='drive-primer'
    )
    
    if len(sys.argv) < 2:
        print("Available commands:")
        for alias, (_, desc) in available_commands.items():
            print(f"  {alias:<20} - {desc}")
        print("\nUsage: drive-primer <command> [args...]")
        return
    
    command = sys.argv[1]
    remaining_args = sys.argv[2:]
    
    if command in ['-h', '--help']:
        print("Available commands:")
        for alias, (_, desc) in available_commands.items():
            print(f"  {alias:<20} - {desc}")
        print("\nUsage: drive-primer <command> [args...]")
        return
    
    if command in available_commands:
        actual_command, _ = available_commands[command]
        
        if not shutil.which(actual_command):
            print(f"Error: {actual_command} not found. Make sure the pip package is installed locally.")
            print(f"(Trying to run '{command}' which maps to '{actual_command}')")
            sys.exit(1)
        
        cmd = [actual_command] + remaining_args
        result = subprocess.run(cmd)
        sys.exit(result.returncode)
        
    else:
        print(f"Unknown command: {command}")
        # print("Available commands:", ", ".join(available_commands.keys()))
        sys.exit(1)

if __name__ == "__main__":
    main()