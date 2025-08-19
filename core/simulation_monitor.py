#!/usr/bin/env python3
"""
SimPyROS Simulation Monitor Window

Displays real-time simulation statistics and timing information in a separate tkinter window.
Works with all visualization backends including process_separated_pyvista.

Features:
- Live simulation time, real-time factor, and timing accuracy monitoring
- Active robot and object counters
- Play/Pause/Reset control buttons
- X11-safe implementation using basic tk widgets (no ttk)
- Automatic error handling and graceful window cleanup
- Cross-platform compatibility with robust display handling

The monitor was redesigned to eliminate X11 RENDER errors by:
- Using basic tk.Label instead of ttk.Label
- Removing all font specifications that trigger X11 glyph rendering
- Avoiding emoji characters in button text
- Using minimal styling (bg='white', fg='black' only)

Usage:
    python examples/beginner/basic_simulation.py --enable-monitor
"""

import tkinter as tk
# Removed ttk import to avoid X11 RENDER issues - using only basic tk widgets
import threading
import time
import json
import os
from typing import Dict, Any, Optional
import tempfile
import signal
import sys


class SimulationMonitor:
    """Real-time simulation data monitoring window for SimPyROS"""
    
    def __init__(self, title="SimPyROS Monitor"):
        self.title = title
        self.running = False
        self.window = None
        self.labels = {}
        self.buttons = {}
        self.data_file = os.path.join(tempfile.gettempdir(), "simpyros_monitor_data.json")
        self.update_interval = 0.5  # Reduced update frequency to avoid X11 issues
        
        # Control state
        self.control_enabled = False
        self.control_callback = None
        
        # Error handling
        self.x11_error_count = 0
        self.max_x11_errors = 3
        
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
        """Stop the monitor window with enhanced error handling"""
        print("🛑 Stopping simulation monitor...")
        self.running = False
        
        # Force close window with multiple attempts
        if self.window:
            for attempt in range(3):
                try:
                    # Try different cleanup methods
                    if attempt == 0:
                        self.window.quit()
                        self.window.destroy()
                    elif attempt == 1:
                        self.window.wm_withdraw()
                        self.window.quit()
                    else:
                        # Force destroy
                        self.window.destroy()
                    print(f"✅ Monitor window closed (attempt {attempt + 1})")
                    break
                except Exception as e:
                    print(f"⚠️ Monitor cleanup attempt {attempt + 1} failed: {e}")
                    if attempt == 2:
                        print("❌ Failed to close monitor window gracefully")
        
        # Clean up data file
        try:
            if os.path.exists(self.data_file):
                os.remove(self.data_file)
                print("🧹 Monitor data file cleaned up")
        except Exception as e:
            print(f"⚠️ Failed to clean up monitor data file: {e}")
            
        self.window = None
                
    def _run_monitor(self):
        """Run the tkinter monitor window with X11 error handling"""
        try:
            # Pre-test X11 connection
            import os
            display = os.environ.get('DISPLAY', '')
            if not display:
                print("❌ No DISPLAY environment variable, cannot create monitor window")
                self.running = False
                return
            
            # Initialize tkinter with minimal, safe settings
            self.window = tk.Tk()
            
            # Immediately configure basic properties without complex styling
            self.window.title(self.title)
            self.window.geometry("400x300")  # Smaller window
            self.window.configure(bg='white')  # Use white background to avoid render issues
            
            # Test basic functionality before proceeding
            test_label = tk.Label(self.window, text="Initializing...", bg='white', fg='black')
            test_label.pack()
            self.window.update_idletasks()  # Force a render test
            
            # Set up error handling for X11 issues
            def on_closing():
                self.running = False
                self.stop()
            
            self.window.protocol("WM_DELETE_WINDOW", on_closing)
            
            # Remove the test label
            test_label.destroy()
            
            # Skip topmost setting to avoid X11 issues
            print("📊 Creating simple monitor layout...")
            
            # Create main frame using basic tkinter (no ttk styling)
            main_frame = tk.Frame(self.window, bg='white', padx=10, pady=10)
            main_frame.pack(fill=tk.BOTH, expand=True)
            
            # Title - ultra-minimal tk.Label to avoid X11 RENDER issues
            title_label = tk.Label(main_frame, text="SimPyROS Monitor", 
                                  bg='white', fg='black')
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
                # Ultra-minimal tk.Label to avoid X11 RENDER issues
                label = tk.Label(main_frame, text=f"{display_name}: --", 
                                bg='white', fg='black')
                label.grid(row=i+1, column=0, sticky=tk.W, pady=2)
                self.labels[field_key] = (label, display_name, unit)
                
            # Add control buttons if enabled - using basic tk widgets
            if self.control_enabled:
                control_frame = tk.Frame(main_frame, bg='white')
                control_frame.grid(row=len(fields)+2, column=0, sticky=tk.W, pady=10)
                
                self.buttons['play_pause'] = tk.Button(control_frame, text="Play", 
                                                       command=self._toggle_play_pause,
                                                       bg='lightgray', fg='black')
                self.buttons['play_pause'].pack(side=tk.LEFT, padx=(0, 5))
                
                self.buttons['reset'] = tk.Button(control_frame, text="Reset",
                                                  command=self._reset_simulation,
                                                  bg='lightgray', fg='black')
                self.buttons['reset'].pack(side=tk.LEFT, padx=(0, 5))
                
            # Add status label - ultra-minimal tk.Label
            self.status_label = tk.Label(main_frame, text="Status: Waiting for data...", 
                                        bg='white', fg='black')
            self.status_label.grid(row=len(fields)+3, column=0, sticky=tk.W, pady=10)
            
            # Start periodic update
            self.window.after(int(self.update_interval * 1000), self._update_display)
            
            print("✅ Monitor window created successfully")
            
            # Start the tkinter main loop with error handling
            try:
                self.window.mainloop()
            except Exception as e:
                print(f"❌ Monitor window mainloop error: {e}")
                self.x11_error_count += 1
                if self.x11_error_count >= self.max_x11_errors:
                    print("❌ Too many X11 errors, disabling monitor")
                    self.running = False
            finally:
                self.running = False
                
        except Exception as e:
            print(f"❌ Monitor window creation failed: {e}")
            self.running = False
            if self.window:
                try:
                    self.window.destroy()
                except:
                    pass
                self.window = None
            
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
                        try:
                            label.config(text=display_text)
                        except tk.TclError as e:
                            # Handle X11 errors during label updates
                            self.x11_error_count += 1
                            if self.x11_error_count >= self.max_x11_errors:
                                print(f"❌ Too many X11 errors in label updates, stopping monitor")
                                self.running = False
                                return
                    else:
                        try:
                            label.config(text=f"{display_name}: --")
                        except tk.TclError:
                            pass  # Ignore errors on default values
                
                # Update control buttons if enabled
                if self.control_enabled and 'simulation_state' in data:
                    state = data['simulation_state']
                    if 'play_pause' in self.buttons:
                        try:
                            if state == 'running':
                                self.buttons['play_pause'].config(text="Pause")
                            else:
                                self.buttons['play_pause'].config(text="Play")
                        except tk.TclError:
                            pass  # Ignore button update errors
                
                try:
                    self.status_label.config(text=f"Status: Connected (Updated: {time.strftime('%H:%M:%S')})")
                except tk.TclError:
                    pass  # Ignore status label errors
            else:
                try:
                    self.status_label.config(text="Status: No data file found")
                except tk.TclError:
                    pass
                
        except Exception as e:
            try:
                self.status_label.config(text=f"Status: Error reading data - {str(e)}")
            except tk.TclError:
                pass  # If we can't even update the status, just continue
            
        # Schedule next update with error handling
        if self.running and self.window:
            try:
                self.window.after(int(self.update_interval * 1000), self._update_display)
            except tk.TclError:
                # If scheduling fails, stop the monitor
                print("❌ Failed to schedule next update, stopping monitor")
                self.running = False
            
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