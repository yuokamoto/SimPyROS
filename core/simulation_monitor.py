#!/usr/bin/env python3
"""
SimPyROS Simulation Monitor Window

Displays real-time simulation statistics and timing information in a separate tkinter window.
Works with all visualization backends including process_separated_pyvista.
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import json
import os
from typing import Dict, Any, Optional
import tempfile


class SimulationMonitor:
    """Real-time simulation data monitoring window for SimPyROS"""
    
    def __init__(self, title="SimPyROS Monitor"):
        self.title = title
        self.running = False
        self.window = None
        self.labels = {}
        self.buttons = {}
        self.data_file = os.path.join(tempfile.gettempdir(), "simpyros_monitor_data.json")
        self.update_interval = 0.2  # Update every 200ms for responsive display
        
        # Control state
        self.control_enabled = False
        self.control_callback = None
        
    def start(self, enable_controls=False, control_callback=None):
        """Start the monitor window in a separate thread
        
        Args:
            enable_controls: Enable play/pause/reset controls
            control_callback: Function to call for control commands
        """
        if self.running:
            return
            
        self.control_enabled = enable_controls
        self.control_callback = control_callback
        self.running = True
        self.monitor_thread = threading.Thread(target=self._run_monitor, daemon=True)
        self.monitor_thread.start()
        
    def stop(self):
        """Stop the monitor window"""
        self.running = False
        if self.window:
            try:
                self.window.quit()
                self.window.destroy()
            except:
                pass
                
        # Clean up data file
        try:
            if os.path.exists(self.data_file):
                os.remove(self.data_file)
        except:
            pass
                
    def _run_monitor(self):
        """Run the tkinter monitor window"""
        self.window = tk.Tk()
        self.window.title(self.title)
        self.window.geometry("500x400")
        self.window.configure(bg='black')
        
        # Make window stay on top
        self.window.attributes('-topmost', True)
        
        # Create main frame
        main_frame = ttk.Frame(self.window, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure style for better visibility
        style = ttk.Style()
        style.configure('Monitor.TLabel', foreground='lime', background='black', font=('Courier', 10))
        style.configure('MonitorTitle.TLabel', foreground='cyan', background='black', font=('Courier', 12, 'bold'))
        style.configure('MonitorButton.TButton', font=('Arial', 9))
        
        # Title
        title_label = ttk.Label(main_frame, text="🚀 SimPyROS Monitor", style='MonitorTitle.TLabel')
        title_label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        # Create labels for different data fields
        self.labels = {}
        fields = [
            ("Simulation Time", "sim_time", "s"),
            ("Real Time", "real_time", "s"), 
            ("Target RT Factor", "target_rt_factor", "x"),
            ("Actual RT Factor", "actual_rt_factor", "x"),
            ("Timing Accuracy", "timing_accuracy", "%"),
            ("Update Rate", "update_rate", "Hz"),
            ("Time Step", "time_step", "s"),
            ("Visualization", "visualization", ""),
            ("Active Robots", "active_robots", ""),
            ("Active Objects", "active_objects", ""),
            ("Architecture", "architecture", "")
        ]
        
        for i, (display_name, field_key, unit) in enumerate(fields):
            label = ttk.Label(main_frame, text=f"{display_name}: --", style='Monitor.TLabel')
            label.grid(row=i+1, column=0, sticky=tk.W, pady=2)
            self.labels[field_key] = (label, display_name, unit)
            
        # Add control buttons if enabled
        if self.control_enabled:
            control_frame = ttk.Frame(main_frame)
            control_frame.grid(row=len(fields)+2, column=0, sticky=tk.W, pady=10)
            
            self.buttons['play_pause'] = ttk.Button(control_frame, text="▶️ Play", 
                                                   command=self._toggle_play_pause,
                                                   style='MonitorButton.TButton')
            self.buttons['play_pause'].pack(side=tk.LEFT, padx=(0, 5))
            
            self.buttons['reset'] = ttk.Button(control_frame, text="🔄 Reset",
                                              command=self._reset_simulation,
                                              style='MonitorButton.TButton')
            self.buttons['reset'].pack(side=tk.LEFT, padx=(0, 5))
            
        # Add status label
        self.status_label = ttk.Label(main_frame, text="Status: Waiting for data...", 
                                     style='Monitor.TLabel')
        self.status_label.grid(row=len(fields)+3, column=0, sticky=tk.W, pady=10)
        
        # Start periodic update
        self.window.after(int(self.update_interval * 1000), self._update_display)
        
        # Start the tkinter main loop
        try:
            self.window.mainloop()
        except:
            pass
        finally:
            self.running = False
            
    def _update_display(self):
        """Update the display with latest data"""
        if not self.running:
            return
            
        try:
            # Read data from shared file
            if os.path.exists(self.data_file):
                with open(self.data_file, 'r') as f:
                    data = json.load(f)
                    
                # Update labels
                for field_key, (label, display_name, unit) in self.labels.items():
                    value = data.get(field_key, None)
                    if value is not None:
                        if isinstance(value, float):
                            if field_key in ['sim_time', 'real_time']:
                                formatted_value = f"{value:.1f}"
                            elif field_key in ['target_rt_factor', 'actual_rt_factor']:
                                formatted_value = f"{value:.2f}"
                            elif field_key == 'timing_accuracy':
                                formatted_value = f"{value:.1f}"
                            elif field_key == 'update_rate':
                                formatted_value = f"{value:.1f}"
                            elif field_key == 'time_step':
                                formatted_value = f"{value:.3f}"
                            else:
                                formatted_value = f"{value:.2f}"
                        else:
                            formatted_value = str(value)
                        
                        display_text = f"{display_name}: {formatted_value}{unit}"
                        label.config(text=display_text)
                    else:
                        label.config(text=f"{display_name}: --")
                
                # Update control buttons if enabled
                if self.control_enabled and 'simulation_state' in data:
                    state = data['simulation_state']
                    if 'play_pause' in self.buttons:
                        if state == 'running':
                            self.buttons['play_pause'].config(text="⏸️ Pause")
                        else:
                            self.buttons['play_pause'].config(text="▶️ Play")
                
                self.status_label.config(text=f"Status: Connected (Updated: {time.strftime('%H:%M:%S')})")
            else:
                self.status_label.config(text="Status: No data file found")
                
        except Exception as e:
            self.status_label.config(text=f"Status: Error reading data - {str(e)}")
            
        # Schedule next update
        if self.running:
            self.window.after(int(self.update_interval * 1000), self._update_display)
            
    def _toggle_play_pause(self):
        """Toggle play/pause state"""
        if self.control_callback:
            self.control_callback('toggle_play_pause')
            
    def _reset_simulation(self):
        """Reset simulation"""
        if self.control_callback:
            self.control_callback('reset')
            
    def update_data(self, sim_data: Dict[str, Any]):
        """Update simulation data from SimulationManager
        
        Args:
            sim_data: Dictionary containing simulation statistics
        """
        try:
            with open(self.data_file, 'w') as f:
                json.dump(sim_data, f)
        except:
            pass  # Ignore errors in data writing


def create_simulation_monitor(title="SimPyROS Monitor", enable_controls=False, control_callback=None):
    """Factory function to create a simulation monitor
    
    Args:
        title: Window title
        enable_controls: Enable control buttons
        control_callback: Function to call for control commands
        
    Returns:
        SimulationMonitor instance
    """
    monitor = SimulationMonitor(title)
    monitor.start(enable_controls, control_callback)
    return monitor


# Standalone monitor launcher for testing
def main():
    """Run standalone simulation monitor"""
    monitor = SimulationMonitor("SimPyROS Simulation Monitor")
    print("Starting SimPyROS monitor window...")
    print("Press Ctrl+C to exit")
    
    try:
        monitor.start(enable_controls=True)
        # Keep main thread alive
        while monitor.running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping monitor...")
        monitor.stop()


if __name__ == "__main__":
    main()