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
    
    @classmethod
    def create_monitor_ui(cls, parent_frame: tk.Frame, 
                         show_debug_info: bool = False,
                         enable_controls: bool = False,
                         control_callback: Optional[Callable] = None) -> tuple:
        """Create shared monitor UI elements (class method version)
        
        Args:
            parent_frame: Parent tkinter frame to add elements to
            show_debug_info: Whether to show debug fields (Architecture, Visualization)
            enable_controls: Whether to add control buttons (Play/Pause/Reset)
            control_callback: Callback function for control buttons
            
        Returns:
            Tuple of (labels_dict, buttons_dict, status_label):
            - labels_dict: Dictionary mapping field_key to (label, display_name, unit)
            - buttons_dict: Dictionary of control buttons
            - status_label: Status label widget
        """
        
        # Title label
        title_label = tk.Label(parent_frame, text="SimPyROS Monitor", 
                              bg='white', fg='black')
        title_label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        # Create labels for different data fields
        labels_dict = {}
        
        # Basic fields (always shown)
        fields = [
            ("Simulation Time", "sim_time", "s"),
            ("Real Time", "real_time", "s"), 
            ("Target RT Factor", "target_rt_factor", "x"),
            ("Actual RT Factor", "actual_rt_factor", "x"),
            ("Time Step", "time_step", "s"),
            ("Active Robots", "active_robots", ""),
            ("Active Objects", "active_objects", ""),
        ]
        
        # Debug fields (only shown if enabled)
        if show_debug_info:
            fields.extend([
                ("Visualization", "visualization", ""),
                ("Architecture", "architecture", "")
            ])
        
        for i, (display_name, field_key, unit) in enumerate(fields):
            label = tk.Label(parent_frame, text=f"{display_name}: --", 
                            bg='white', fg='black')
            label.grid(row=i+1, column=0, sticky=tk.W, pady=2)
            labels_dict[field_key] = (label, display_name, unit)
            
        # Add control buttons if enabled
        buttons_dict = {}
        if enable_controls and control_callback:
            control_frame = tk.Frame(parent_frame, bg='white')
            control_frame.grid(row=len(fields)+2, column=0, sticky=tk.W, pady=10)
            
            def toggle_play_pause():
                """Toggle play/pause - assumes control_callback handles state"""
                control_callback('toggle')
            
            def reset_simulation():
                """Reset simulation"""
                control_callback('reset')
            
            buttons_dict['play_pause'] = tk.Button(control_frame, text="Play", 
                                                   command=toggle_play_pause,
                                                   bg='lightgray', fg='black')
            buttons_dict['play_pause'].pack(side=tk.LEFT, padx=(0, 5))
            
            buttons_dict['reset'] = tk.Button(control_frame, text="Reset",
                                              command=reset_simulation,
                                              bg='lightgray', fg='black')
            buttons_dict['reset'].pack(side=tk.LEFT, padx=(0, 5))
        
        # Add status label
        status_label = tk.Label(parent_frame, text="Status: Waiting for data...", 
                               bg='white', fg='black')
        status_label.grid(row=len(fields)+3, column=0, sticky=tk.W, pady=10)
        
        return labels_dict, buttons_dict, status_label
    
    @classmethod
    def update_monitor_ui(cls, labels_dict: Dict[str, Any], 
                         status_label: tk.Label,
                         data: Dict[str, Any],
                         buttons_dict: Optional[Dict[str, tk.Button]] = None) -> None:
        """Update monitor UI elements with data (class method version)
        
        Args:
            labels_dict: Dictionary mapping field_key to (label, display_name, unit)
            status_label: Status label widget to update
            data: Data dictionary to display
            buttons_dict: Optional control buttons to update
        """
        try:
            # Update data labels
            for key, (label, display_name, unit) in labels_dict.items():
                value = data.get(key, "N/A")
                
                # Format different data types appropriately
                if key == "sim_time" and isinstance(value, (int, float)):
                    text = f"{display_name}: {value:.1f} {unit}"
                elif key in ["target_rt_factor", "actual_rt_factor"] and isinstance(value, (int, float)):
                    text = f"{display_name}: {value:.2f} {unit}"
                elif key == "time_step" and isinstance(value, (int, float)):
                    text = f"{display_name}: {value:.3f} {unit}"
                else:
                    text = f"{display_name}: {value} {unit}".strip()
                
                label.config(text=text)
            
            # Update status
            sim_state = data.get('simulation_state', 'unknown')
            status_label.config(text=f"Status: {sim_state.title()}")
            
            # Update control button states
            if buttons_dict and 'play_pause' in buttons_dict:
                sim_state = data.get('simulation_state', 'running')
                if sim_state == 'paused':
                    buttons_dict['play_pause'].config(text="Play")
                else:
                    buttons_dict['play_pause'].config(text="Pause")
        
        except Exception as e:
            print(f"⚠️ Monitor UI update error: {e}")
    
    @staticmethod
    def format_field_simple(field_key: str, value: Any) -> str:
        """Simple field formatting for process-separated monitor (static method)
        
        Args:
            field_key: Field identifier
            value: Value to format
            
        Returns:
            Formatted text string
        """
        if field_key == "sim_time" and isinstance(value, (int, float)):
            return f"Simulation Time: {value:.1f}s"
        elif field_key in ["target_rt_factor", "actual_rt_factor"] and isinstance(value, (int, float)):
            display_name = field_key.replace('_', ' ').title()
            return f"{display_name}: {value:.2f}x"
        elif field_key == "real_time" and isinstance(value, (int, float)):
            return f"Real Time: {value:.1f}s"
        elif field_key == "time_step" and isinstance(value, (int, float)):
            return f"Time Step: {value:.3f}s"
        else:
            display_name = field_key.replace('_', ' ').title()
            return f"{display_name}: {value}"
    
    @classmethod
    def read_monitor_data(cls, data_file: str) -> tuple:
        """Read and validate monitor data from file (class method)
        
        Args:
            data_file: Path to the data file
            
        Returns:
            Tuple of (data_dict, status_message, status_color):
            - data_dict: Parsed JSON data or None if error
            - status_message: Status message for display
            - status_color: Color for status ('green', 'orange', 'red')
        """
        import os
        import json
        
        try:
            if os.path.exists(data_file):
                with open(data_file, 'r') as f:
                    data = json.load(f)
                return data, "Status: Active", 'green'
            else:
                return None, "Status: No data", 'orange'
        except Exception as e:
            return None, f"Status: Error - {e}", 'red'
    
    @classmethod 
    def update_labels_simple(cls, labels_dict: Dict[str, Any], data: Dict[str, Any]) -> None:
        """Update simple labels with data (for process-separated monitor)
        
        Args:
            labels_dict: Dictionary mapping field_key to label widget
            data: Data dictionary to display
        """
        for field_key, label in labels_dict.items():
            value = data.get(field_key, "N/A")
            text = cls.format_field_simple(field_key, value)
            label.config(text=text)
    
    @abstractmethod
    def start(self, enable_controls=False, control_callback=None):
        """Start the monitor - must be implemented by subclasses"""
        pass
    
    @abstractmethod
    def stop(self):
        """Stop the monitor - must be implemented by subclasses"""
        pass
    
    def _create_ui_elements(self, parent_frame):
        """Create common UI elements using class methods"""
        # Use class method for UI creation
        self.labels, self.buttons, self.status_label = self.__class__.create_monitor_ui(
            parent_frame=parent_frame,
            show_debug_info=self.show_debug_info,
            enable_controls=self.control_enabled,
            control_callback=self._handle_control_command
        )
        
        # Calculate number of fields for return value
        num_fields = len(self.labels)
        return num_fields + 4  # Title + fields + controls + status
    
    def _update_ui_with_data(self, data: Dict[str, Any]):
        """Update UI elements with data using class methods"""
        # Use class method for UI updates
        self.__class__.update_monitor_ui(
            labels_dict=self.labels,
            status_label=self.status_label,
            data=data,
            buttons_dict=self.buttons
        )
    
    def _handle_control_command(self, command: str):
        """Handle control commands from shared UI"""
        if command == 'toggle':
            self._toggle_play_pause()
        elif command == 'reset':
            self._reset_simulation()
    
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
        if not self.running:
            return
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
        self._stop_event = threading.Event()  # Thread-safe stop signaling
        
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
        self._stop_event.clear()  # Clear any previous stop signals
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
        print("📡 Setting stop event for monitor thread")
        self._stop_event.set()  # Thread-safe signaling
                
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
            
            # Start the tkinter main loop with improved error handling
            try:
                print("🎬 Starting tkinter mainloop...")
                self.window.mainloop()
                print("✅ Monitor window mainloop exited gracefully")
            except Exception as e:
                print(f"❌ Monitor window mainloop error: {e}")
                # Don't increment error count for normal shutdown
                if not self._stop_event.is_set():
                    self.x11_error_count += 1
                    if self.x11_error_count >= self.max_x11_errors:
                        print("❌ Too many X11 errors, disabling monitor")
            finally:
                self.running = False
                # Clean up window within the same thread
                if self.window:
                    try:
                        if self.window.winfo_exists():
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
            
        # Check for close signal (thread-safe event)
        if self._stop_event.is_set():
            print("📡 Received stop event, shutting down monitor")
            self.running = False
            if self.window:
                self.window.quit()  # Exit mainloop from within the thread
            return
            
        # Use BaseMonitor class method for data reading
        data, status_message, status_color = self.__class__.read_monitor_data(self.data_file)
        
        if data is not None:
            # Use shared UI update logic from BaseMonitor
            self._update_ui_with_data(data)
            
            # Enhanced status with timestamp for threaded monitor
            try:
                timestamp = time.strftime('%H:%M:%S')
                self.status_label.config(text=f"Status: Connected (Updated: {timestamp})")
            except tk.TclError:
                pass  # Ignore status label errors
        else:
            # Handle error cases
            try:
                self.status_label.config(text=status_message)
            except tk.TclError:
                pass  # If we can't update the status, just continue
            
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