#!/usr/bin/env python3
import os
import glob
import re
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

def parse_result_file(filename):
    """Parse a result file and extract metrics"""
    results = {}
    
    with open(filename, 'r') as f:
        for line in f:
            # Extract FPS
            if 'Average FPS:' in line:
                fps = float(re.search(r'Average FPS: (\d+\.\d+)', line).group(1))
                results['fps'] = fps
            
            # Extract latency
            if 'Average latency:' in line:
                latency = float(re.search(r'Average latency: (\d+\.\d+)', line).group(1))
                results['latency'] = latency
            
            # Extract CPU usage
            if 'Average CPU usage:' in line:
                cpu = float(re.search(r'Average CPU usage: (\d+\.\d+)', line).group(1))
                results['cpu'] = cpu
            
            # Extract memory usage
            if 'Average memory usage:' in line:
                memory = float(re.search(r'Average memory usage: (\d+\.\d+)', line).group(1))
                results['memory'] = memory
    
    return results

def main():
    # Find the most recent result files for both implementations
    python_files = sorted(glob.glob('yolo_python_performance_*.txt'))
    cpp_files = sorted(glob.glob('yolo_cpp_performance_*.txt'))
    
    if not python_files or not cpp_files:
        print("Could not find result files for both implementations")
        return
    
    # Get the most recent files
    python_file = python_files[-1]
    cpp_file = cpp_files[-1]
    
    print(f"Comparing results from:")
    print(f"  - Python: {python_file}")
    print(f"  - C++: {cpp_file}")
    
    # Parse the files
    python_results = parse_result_file(python_file)
    cpp_results = parse_result_file(cpp_file)
    
    # Create comparison report
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = f"yolo_comparison_report_{timestamp}.txt"
    
    with open(report_file, 'w') as f:
        f.write("YOLO Performance Comparison: Python vs C++\n")
        f.write("=========================================\n\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        
        # Compare FPS
        if 'fps' in python_results and 'fps' in cpp_results:
            f.write("Frames Per Second (FPS):\n")
            f.write(f"  - Python: {python_results['fps']:.2f} FPS\n")
            f.write(f"  - C++: {cpp_results['fps']:.2f} FPS\n")
            
            speedup = cpp_results['fps'] / python_results['fps'] if python_results['fps'] > 0 else float('inf')
            f.write(f"  - Speedup: {speedup:.2f}x\n\n")
        
        # Compare latency
        if 'latency' in python_results and 'latency' in cpp_results:
            f.write("Processing Latency (ms):\n")
            f.write(f"  - Python: {python_results['latency']:.2f} ms\n")
            f.write(f"  - C++: {cpp_results['latency']:.2f} ms\n")
            
            reduction = (python_results['latency'] - cpp_results['latency']) / python_results['latency'] * 100 if python_results['latency'] > 0 else float('inf')
            f.write(f"  - Reduction: {reduction:.2f}%\n\n")
        
        # Compare CPU usage
        if 'cpu' in python_results and 'cpu' in cpp_results:
            f.write("CPU Usage (%):\n")
            f.write(f"  - Python: {python_results['cpu']:.2f}%\n")
            f.write(f"  - C++: {cpp_results['cpu']:.2f}%\n")
            
            reduction = (python_results['cpu'] - cpp_results['cpu']) / python_results['cpu'] * 100 if python_results['cpu'] > 0 else float('inf')
            f.write(f"  - Reduction: {reduction:.2f}%\n\n")
        
        # Compare memory usage
        if 'memory' in python_results and 'memory' in cpp_results:
            f.write("Memory Usage (MB):\n")
            f.write(f"  - Python: {python_results['memory']:.2f} MB\n")
            f.write(f"  - C++: {cpp_results['memory']:.2f} MB\n")
            
            reduction = (python_results['memory'] - cpp_results['memory']) / python_results['memory'] * 100 if python_results['memory'] > 0 else float('inf')
            f.write(f"  - Reduction: {reduction:.2f}%\n\n")
        
        f.write("Conclusion:\n")
        if 'fps' in python_results and 'fps' in cpp_results and cpp_results['fps'] > python_results['fps']:
            f.write("The C++ implementation shows better performance in terms of FPS.\n")
        elif 'fps' in python_results and 'fps' in cpp_results:
            f.write("The Python implementation shows better performance in terms of FPS.\n")
        
        f.write("\nDetailed results can be found in the individual test output files.\n")
    
    print(f"Comparison report saved to {report_file}")
    
    # Create comparison visualization
    create_comparison_visualization(python_results, cpp_results, timestamp)

def create_comparison_visualization(python_results, cpp_results, timestamp):
    """Create a visual comparison of the Python and C++ implementations"""
    metrics = ['fps', 'latency', 'cpu', 'memory']
    labels = ['FPS', 'Latency (ms)', 'CPU Usage (%)', 'Memory (MB)']
    
    # Prepare data
    data = []
    for metric in metrics:
        if metric in python_results and metric in cpp_results:
            data.append((metric, python_results[metric], cpp_results[metric]))
    
    if not data:
        print("Not enough data for visualization")
        return
    
    # Create visualization
    fig, axs = plt.subplots(len(data), 1, figsize=(10, 3*len(data)), squeeze=False)
    
    for i, (metric, python_val, cpp_val) in enumerate(data):
        ax = axs[i, 0]
        
        # Create bar chart
        x = np.arange(2)
        width = 0.6
        
        bars = ax.bar(x, [python_val, cpp_val], width)
        
        # Add labels and title
        ax.set_ylabel(labels[metrics.index(metric)])
        ax.set_title(f"{labels[metrics.index(metric)]} Comparison")
        ax.set_xticks(x)
        ax.set_xticklabels(['Python', 'C++'])
        
        # Add values on bars
        for bar in bars:
            height = bar.get_height()
            ax.annotate(f'{height:.2f}',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom')
        
        # For FPS, higher is better, for others lower is better
        if metric == 'fps':
            if cpp_val > python_val:
                ax.text(0.5, 0.9, f'C++ is {cpp_val/python_val:.2f}x faster', 
                       horizontalalignment='center', verticalalignment='center', 
                       transform=ax.transAxes, bbox=dict(facecolor='green', alpha=0.2))
            else:
                ax.text(0.5, 0.9, f'Python is {python_val/cpp_val:.2f}x faster', 
                       horizontalalignment='center', verticalalignment='center', 
                       transform=ax.transAxes, bbox=dict(facecolor='red', alpha=0.2))
        else:
            if python_val > cpp_val:
                reduction = (python_val - cpp_val) / python_val * 100
                ax.text(0.5, 0.9, f'C++ reduces {metric} by {reduction:.2f}%', 
                       horizontalalignment='center', verticalalignment='center', 
                       transform=ax.transAxes, bbox=dict(facecolor='green', alpha=0.2))
            else:
                increase = (cpp_val - python_val) / python_val * 100
                ax.text(0.5, 0.9, f'C++ increases {metric} by {increase:.2f}%', 
                       horizontalalignment='center', verticalalignment='center', 
                       transform=ax.transAxes, bbox=dict(facecolor='red', alpha=0.2))
    
    plt.tight_layout()
    plt.savefig(f"yolo_comparison_{timestamp}.png")
    print(f"Comparison visualization saved to yolo_comparison_{timestamp}.png")

if __name__ == "__main__":
    main()