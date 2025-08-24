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
from typing import Dict, Any, Optional, Callable
import tempfile
import signal
import sys
from abc import ABC, abstractmethod


class BaseMonitor(ABC):
    """Base class for simulation monitors with shared UI creation logic"""
    
    def __init__(self, title="SimPyROS Monitor"):
        self.title = title
        self.running = False
        self.window = None
        self.labels = {}
        self.buttons = {}
        self.data_file = os.path.join(tempfile.gettempdir(), "simpyros_monitor_data.json")
        self.update_interval = 0.5  # Default update frequency
        
        # Control state
        self.control_enabled = False
        self.control_callback = None
        self.simulation_paused = False
        
        # Display options
        self.show_debug_info = False  # Show debug info (Architecture, Visualization)
        
        # Error handling for X11 issues
        self.x11_error_count = 0
        self.max_x11_errors = 3
    
    @abstractmethod
    def start(self, enable_controls=False, control_callback=None):
        """Start the monitor - must be implemented by subclasses"""
        pass
    
    @abstractmethod
    def stop(self):
        """Stop the monitor - must be implemented by subclasses"""
        pass
    
    def _create_ui_elements(self, parent_frame):
        """Create common UI elements - shared between implementations"""
        # Title - ultra-minimal tk.Label to avoid X11 RENDER issues
        title_label = tk.Label(parent_frame, text="SimPyROS Monitor", 
                              bg='white', fg='black')
        title_label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        # Create labels for different data fields
        self.labels = {}
        # Basic fields (always shown)
        fields = [
            ("Simulation Time", "sim_time", "s"),
            ("Real Time", "real_time", "s"), 
            ("Target RT Factor", "target_rt_factor", "x"),
            ("Actual RT Factor", "actual_rt_factor", "x"),
            ("Timing Accuracy", "timing_accuracy", "%"),
            ("Time Step", "time_step", "s"),  # Remove Update Rate - TimeStep is sufficient
            ("Active Robots", "active_robots", ""),
            ("Active Objects", "active_objects", ""),
        ]
        
        # Debug fields (only shown if enabled)
        if self.show_debug_info:
            fields.extend([
                ("Visualization", "visualization", ""),
                ("Architecture", "architecture", "")
            ])
        
        for i, (display_name, field_key, unit) in enumerate(fields):
            # Ultra-minimal tk.Label to avoid X11 RENDER issues
            label = tk.Label(parent_frame, text=f"{display_name}: --", 
                            bg='white', fg='black')
            label.grid(row=i+1, column=0, sticky=tk.W, pady=2)
            self.labels[field_key] = (label, display_name, unit)
            
        # Add control buttons if enabled - using basic tk widgets
        if self.control_enabled:
            control_frame = tk.Frame(parent_frame, bg='white')
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
        self.status_label = tk.Label(parent_frame, text="Status: Waiting for data...", 
                                    bg='white', fg='black')
        self.status_label.grid(row=len(fields)+3, column=0, sticky=tk.W, pady=10)
        
        return len(fields) + 4  # Return number of rows created
    
    def _update_ui_with_data(self, data: Dict[str, Any]):
        """Update UI elements with data - shared logic"""
        try:
            for key, (label, display_name, unit) in self.labels.items():
                value = data.get(key, "N/A")
                
                # Format different data types appropriately
                if key == "sim_time" and isinstance(value, (int, float)):
                    text = f"{display_name}: {value:.1f} {unit}"
                elif key in ["target_rt_factor", "actual_rt_factor"] and isinstance(value, (int, float)):
                    text = f"{display_name}: {value:.2f} {unit}"
                elif key == "timing_accuracy" and isinstance(value, (int, float)):
                    text = f"{display_name}: {value:.1f} {unit}"
                elif key == "time_step" and isinstance(value, (int, float)):
                    text = f"{display_name}: {value:.1f} {unit}"
                else:
                    text = f"{display_name}: {value} {unit}".strip()
                
                label.config(text=text)
            
            # Update status
            if hasattr(self, 'status_label'):
                sim_state = data.get('simulation_state', 'unknown')
                self.status_label.config(text=f"Status: {sim_state.title()}")
            
            # Update control button states
            if 'play_pause' in self.buttons:
                sim_state = data.get('simulation_state', 'running')
                if sim_state == 'paused':
                    self.buttons['play_pause'].config(text="Play")
                else:
                    self.buttons['play_pause'].config(text="Pause")
        
        except Exception as e:
            print(f"⚠️ Monitor UI update error: {e}")
    
    def _toggle_play_pause(self):
        """Toggle play/pause state"""
        if self.control_callback:
            command = 'resume' if self.simulation_paused else 'pause'
            self.control_callback(command)
            self.simulation_paused = not self.simulation_paused
    
    def _reset_simulation(self):
        """Reset simulation"""
        if self.control_callback:
            self.control_callback('reset')
    
    def update_data(self, sim_data: Dict[str, Any]):
        """Update monitor data - must be implemented by subclasses for data delivery"""
        try:
            with open(self.data_file, 'w') as f:
                json.dump(sim_data, f)
        except Exception as e:
            print(f"⚠️ Monitor data file write error: {e}")


class SimulationMonitor(BaseMonitor):
    """Real-time simulation data monitoring window for SimPyROS using threading"""
    
    def __init__(self, title="SimPyROS Monitor"):
        super().__init__(title)
        self.monitor_thread = None
        
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
        self.monitor_thread = threading.Thread(target=self._run_monitor, daemon=False)
        self.monitor_thread.start()
        
    def stop(self):
        """Stop the monitor window with thread-safe handling"""
        print("🛑 Stopping simulation monitor...")
        self.running = False
        
        # Signal monitor thread to close gracefully
        self._signal_close()
        
        # Wait for monitor thread to finish
        if hasattr(self, 'monitor_thread') and self.monitor_thread and self.monitor_thread.is_alive():
            print("⏳ Waiting for monitor thread to finish...")
            self.monitor_thread.join(timeout=3.0)
            if self.monitor_thread.is_alive():
                print("⚠️ Monitor thread did not finish gracefully")
        
        # Clean up data file
        try:
            if os.path.exists(self.data_file):
                os.remove(self.data_file)
                print("🧹 Monitor data file cleaned up")
        except Exception as e:
            print(f"⚠️ Failed to clean up monitor data file: {e}")
            
        self.window = None
        print("✅ Monitor stopped successfully")
        
    def _signal_close(self):
        """Signal the monitor thread to close (thread-safe)"""
        try:
            # Create a signal file to communicate with monitor thread
            signal_file = os.path.join(tempfile.gettempdir(), "simpyros_monitor_close_signal.txt")
            with open(signal_file, 'w') as f:
                f.write("close")
            print("📡 Close signal sent to monitor thread")
        except Exception as e:
            print(f"⚠️ Failed to send close signal: {e}")
                
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
                """Handle window close event"""
                print("🔴 Monitor window close requested")
                self.running = False
                try:
                    if self.window:
                        self.window.quit()  # Exit mainloop
                except Exception as e:
                    print(f"⚠️ Error during window close: {e}")
            
            self.window.protocol("WM_DELETE_WINDOW", on_closing)
            
            # Remove the test label
            test_label.destroy()
            
            # Skip topmost setting to avoid X11 issues
            print("📊 Creating simple monitor layout...")
            
            # Create main frame using basic tkinter (no ttk styling)
            main_frame = tk.Frame(self.window, bg='white', padx=10, pady=10)
            main_frame.pack(fill=tk.BOTH, expand=True)
            
            # Use shared UI creation from BaseMonitor
            self._create_ui_elements(main_frame)
            
            # Start periodic update
            self.window.after(int(self.update_interval * 1000), self._update_display)
            
            print("✅ Monitor window created successfully")
            
            # Start the tkinter main loop with error handling
            try:
                self.window.mainloop()
                print("✅ Monitor window mainloop exited gracefully")
            except Exception as e:
                print(f"❌ Monitor window mainloop error: {e}")
                self.x11_error_count += 1
                if self.x11_error_count >= self.max_x11_errors:
                    print("❌ Too many X11 errors, disabling monitor")
                    self.running = False
            finally:
                self.running = False
                # Clean up window within the same thread
                if self.window:
                    try:
                        self.window.destroy()
                        print("✅ Monitor window destroyed in thread")
                    except Exception as e:
                        print(f"⚠️ Error destroying window in thread: {e}")
                    finally:
                        self.window = None
                
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
        if not self.running or not self.window:
            return
            
        # Check for close signal
        signal_file = os.path.join(tempfile.gettempdir(), "simpyros_monitor_close_signal.txt")
        if os.path.exists(signal_file):
            try:
                os.remove(signal_file)
                print("📡 Received close signal, shutting down monitor")
                self.running = False
                if self.window:
                    self.window.quit()  # Exit mainloop from within the thread
                return
            except Exception as e:
                print(f"⚠️ Error processing close signal: {e}")
            
        try:
            # Read data from shared file
            if os.path.exists(self.data_file):
                with open(self.data_file, 'r') as f:
                    data = json.load(f)
                    
                # Use shared UI update logic from BaseMonitor
                self._update_ui_with_data(data)
                
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
            
    # Inherited methods from BaseMonitor:
    # - _toggle_play_pause()
    # - _reset_simulation() 
    # - update_data()


def create_simulation_monitor(title="SimPyROS Monitor", enable_controls=False, control_callback=None, show_debug_info=False):
    """Factory function to create a simulation monitor
    
    Args:
        title: Window title
        enable_controls: Enable control buttons
        control_callback: Function to call for control commands
        show_debug_info: Show debug info (Architecture, Visualization)
        
    Returns:
        Monitor instance (process-separated for better thread safety)
    """
    try:
        from core.process_separated_monitor import create_process_separated_monitor
        print("📊 Using process-separated monitor for better thread safety")
        monitor = create_process_separated_monitor(title)
        monitor.show_debug_info = show_debug_info  # Set debug flag
        monitor.start(enable_controls, control_callback)
        return monitor
    except Exception as e:
        print(f"⚠️ Process-separated monitor not available, using threaded version: {e}")
        monitor = SimulationMonitor(title)
        monitor.show_debug_info = show_debug_info  # Set debug flag
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