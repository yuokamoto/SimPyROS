#!/usr/bin/env python3
"""
Process-Separated Simulation Monitor for SimPyROS

This monitor runs in a completely separate process to avoid tkinter threading issues
and Tcl_AsyncDelete errors. Communicates via shared files.
"""

import multiprocessing as mp
import os
import sys
import time
import json
import tempfile
from typing import Dict, Any, Optional
from core.multiprocessing_cleanup import register_multiprocessing_process
from core.simulation_monitor import BaseMonitor


def run_monitor_process(data_file: str, title: str = "SimPyROS Monitor"):
    """Run monitor in separate process"""
    try:
        import tkinter as tk
        import signal
        
        print(f"🚀 Starting process-separated monitor (PID: {os.getpid()})")
        
        # Global shutdown flag for clean termination
        shutdown_requested = False
        
        # Install signal handlers for proper Ctrl-C handling in child process
        def signal_handler(signum, frame):
            nonlocal shutdown_requested
            print(f"📡 Monitor process received signal {signum} - initiating graceful shutdown")
            shutdown_requested = True
            try:
                root.quit()  # Exit the main loop gracefully
            except Exception:
                pass  # Window might already be destroyed
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        print("✅ Signal handlers installed in monitor process")
        
        # Create window
        root = tk.Tk()
        root.title(title)
        root.geometry("400x300")
        root.configure(bg='white')
        
        # UI elements
        labels = {}
        
        # Create main frame
        main_frame = tk.Frame(root, bg='white')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create labels for data fields (simplified version of BaseMonitor logic)
        fields = [
            ("Simulation Time", "sim_time", "s"),
            ("Real Time", "real_time", "s"), 
            ("Target RT Factor", "target_rt_factor", "x"),
            ("Actual RT Factor", "actual_rt_factor", "x"),
            ("Timing Accuracy", "timing_accuracy", "%"),
            ("Update Rate", "update_frequency", "Hz"),
            ("Time Step", "time_step", "s"),
            ("Visualization", "visualization", ""),
            ("Active Robots", "active_robots", ""),
            ("Active Objects", "active_objects", ""),
            ("Architecture", "architecture", "")
        ]
        
        labels = {}
        for i, (display_name, field_key, unit) in enumerate(fields):
            label = tk.Label(main_frame, text=f"{display_name}: --", 
                            bg='white', fg='black')
            label.grid(row=i+1, column=0, sticky=tk.W, pady=2)
            labels[field_key] = label
            
        # Status label
        status_label = tk.Label(main_frame, text="Status: Waiting for data...", 
                               bg='white', fg='black')
        status_label.grid(row=len(fields)+2, column=0, sticky=tk.W, pady=10)
        
        def update_display():
            """Update display with data from file"""
            # Check if shutdown was requested
            if shutdown_requested:
                return  # Don't schedule further updates
                
            try:
                if os.path.exists(data_file):
                    with open(data_file, 'r') as f:
                        data = json.load(f)
                    
                    # Update labels with data (simplified version of BaseMonitor logic)
                    for field_key, label in labels.items():
                        value = data.get(field_key, "N/A")
                        
                        # Format different data types appropriately
                        if field_key == "sim_time" and isinstance(value, (int, float)):
                            text = f"Simulation Time: {value:.1f}s"
                        elif field_key in ["target_rt_factor", "actual_rt_factor"] and isinstance(value, (int, float)):
                            text = f"{field_key.replace('_', ' ').title()}: {value:.2f}x"
                        elif field_key == "timing_accuracy" and isinstance(value, (int, float)):
                            text = f"Timing Accuracy: {value:.1f}%"
                        elif field_key == "update_frequency" and isinstance(value, (int, float)):
                            text = f"Update Rate: {value:.1f} Hz"
                        elif field_key == "time_step" and isinstance(value, (int, float)):
                            text = f"Time Step: {value:.3f}s"
                        else:
                            display_name = field_key.replace('_', ' ').title()
                            text = f"{display_name}: {value}"
                        
                        label.config(text=text)
                    
                    status_label.config(text="Status: Active", fg='green')
                else:
                    status_label.config(text="Status: No data", fg='orange')
                
            except Exception as e:
                if not shutdown_requested:  # Only report errors if not shutting down
                    status_label.config(text=f"Status: Error - {e}", fg='red')
            
            # Schedule next update only if not shutting down
            if not shutdown_requested:
                try:
                    root.after(500, update_display)
                except Exception:
                    pass  # Window might be destroyed
        
        # Handle close
        def on_closing():
            """Handle window close"""
            nonlocal shutdown_requested
            print("🔴 Process monitor window closed")
            shutdown_requested = True
            try:
                root.quit()
            except Exception:
                pass
            try:
                root.destroy()
            except Exception:
                pass
            
        root.protocol("WM_DELETE_WINDOW", on_closing)
        
        # Start updates
        root.after(100, update_display)
        
        print("✅ Process-separated monitor window ready")
        
        # Run mainloop with improved error handling
        try:
            root.mainloop()
        except KeyboardInterrupt:
            print("⌨️ Monitor interrupted by signal - shutting down gracefully")
        except Exception as e:
            if not shutdown_requested:  # Only report unexpected errors
                print(f"❌ Monitor process error: {e}")
        finally:
            # Ensure clean shutdown
            shutdown_requested = True
            try:
                if root.winfo_exists():  # Check if window still exists
                    root.quit()
            except Exception:
                pass
            try:
                if root.winfo_exists():  # Check if window still exists
                    root.destroy()
            except Exception:
                pass
            print("🛑 Process-separated monitor exiting gracefully")
            
    except Exception as e:
        print(f"❌ Process monitor error: {e}")
        import traceback
        traceback.print_exc()


class ProcessSeparatedMonitor(BaseMonitor):
    """Process-separated simulation monitor using multiprocessing"""
    
    def __init__(self, title="SimPyROS Process Monitor"):
        super().__init__(title)
        self.process = None
        self.data_file = os.path.join(tempfile.gettempdir(), "simpyros_process_monitor_data.json")
        
    def start(self, enable_controls=False, control_callback=None):
        """Start the monitor process"""
        try:
            print("🚀 Starting process-separated monitor...")
            
            # Start monitor process
            self.process = mp.Process(
                target=run_monitor_process,
                args=(self.data_file, self.title),
                daemon=False
            )
            register_multiprocessing_process(self.process)  # Register for cleanup
            self.process.start()
            self.running = True
            
            print(f"✅ Process-separated monitor started (PID: {self.process.pid})")
            
        except Exception as e:
            print(f"❌ Failed to start process monitor: {e}")
            
    def stop(self):
        """Stop the monitor process"""
        print("🛑 Stopping process-separated monitor...")
        self.running = False
        
        if self.process and self.process.is_alive():
            try:
                self.process.terminate()
                self.process.join(timeout=3.0)
                
                if self.process.is_alive():
                    print("⚠️ Force killing monitor process")
                    self.process.kill()
                    self.process.join(timeout=1.0)
                    
                print("✅ Process-separated monitor stopped")
                
            except Exception as e:
                print(f"⚠️ Error stopping monitor process: {e}")
        
        # Clean up data file
        try:
            if os.path.exists(self.data_file):
                os.remove(self.data_file)
                print("🧹 Monitor data file cleaned up")
        except Exception as e:
            print(f"⚠️ Failed to clean up data file: {e}")
            
        self.process = None
        
    def update_data(self, data: Dict[str, Any]):
        """Update monitor data (with change detection to reduce file I/O)"""
        if not self.running:
            return
        
        # Skip update if data hasn't changed significantly
        if hasattr(self, '_last_data'):
            # Check if important values have changed
            important_keys = ['sim_time', 'actual_rt_factor', 'simulation_state']
            data_changed = False
            for key in important_keys:
                if key in data and key in self._last_data:
                    if isinstance(data[key], (int, float)):
                        # For numbers, check if change is significant (>1% for sim_time, >5% for others)
                        threshold = 0.01 if key == 'sim_time' else 0.05
                        if abs(data[key] - self._last_data[key]) / max(abs(self._last_data[key]), 1e-6) > threshold:
                            data_changed = True
                            break
                    else:
                        # For strings/others, check direct equality
                        if data[key] != self._last_data[key]:
                            data_changed = True
                            break
                else:
                    data_changed = True  # New key or missing key
                    break
            
            if not data_changed:
                # print(\"📊 Monitor update skipped - no significant data changes\")  # Debug
                return  # Skip file write if no significant changes
            
        try:
            with open(self.data_file, 'w') as f:
                json.dump(data, f)
            
            # Store last data for comparison
            self._last_data = data.copy()
            
        except Exception as e:
            print(f"⚠️ Failed to update monitor data: {e}")


def create_process_separated_monitor(title="SimPyROS Process Monitor"):
    """Factory function to create process-separated monitor"""
    return ProcessSeparatedMonitor(title)


if __name__ == "__main__":
    # Test the process-separated monitor
    import time
    
    monitor = ProcessSeparatedMonitor("Test Monitor")
    monitor.start()
    
    try:
        # Simulate data updates
        for i in range(10):
            test_data = {
                'sim_time': i * 0.5,
                'rtf': 1.0,
                'robots': 1,
                'objects': 0,
                'fps': 60.0
            }
            monitor.update_data(test_data)
            time.sleep(1.0)
    finally:
        monitor.stop()
        print("Test completed")