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


def run_monitor_process(data_file: str, title: str = "SimPyROS Monitor"):
    """Run monitor in separate process"""
    try:
        import tkinter as tk
        import signal
        
        print(f"🚀 Starting process-separated monitor (PID: {os.getpid()})")
        
        # Install signal handlers for proper Ctrl-C handling in child process
        def signal_handler(signum, frame):
            print(f"📡 Monitor process received signal {signum}")
            raise KeyboardInterrupt(f"Signal {signum} received")
        
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
        
        # Create labels for data fields
        fields = [
            ("Simulation Time", "sim_time", "0.0s"),
            ("Target RTF", "target_rtf", "1.0x"),
            ("Actual RTF", "actual_rtf", "N/A"),
            ("Time Step", "time_step", "0.0ms"),
            ("Active Robots", "robots", "0"),
            ("Active Objects", "objects", "0"),
            ("Update Rate", "fps", "0.0 Hz"),
            ("Timing Accuracy", "timing_accuracy", "N/A"),
        ]
        
        for i, (display_name, key, default) in enumerate(fields):
            tk.Label(main_frame, text=f"{display_name}:", bg='white', fg='black').grid(
                row=i, column=0, sticky=tk.W, pady=2
            )
            labels[key] = tk.Label(main_frame, text=default, bg='white', fg='blue')
            labels[key].grid(row=i, column=1, sticky=tk.W, padx=(10, 0), pady=2)
        
        # Status label
        status_label = tk.Label(main_frame, text="Status: Waiting for data...", 
                               bg='white', fg='black')
        status_label.grid(row=len(fields), column=0, sticky=tk.W, pady=10)
        
        def update_display():
            """Update display with data from file"""
            try:
                if os.path.exists(data_file):
                    with open(data_file, 'r') as f:
                        data = json.load(f)
                    
                    # Update labels with enhanced information
                    labels['sim_time'].config(text=f"{data.get('sim_time', 0.0):.1f}s")
                    labels['target_rtf'].config(text=f"{data.get('target_rtf', 1.0):.1f}x")
                    labels['actual_rtf'].config(text=f"{data.get('actual_rtf', 'N/A')}")
                    labels['time_step'].config(text=f"{data.get('time_step', 0.0):.1f}ms")
                    labels['robots'].config(text=str(data.get('robots', 0)))
                    labels['objects'].config(text=str(data.get('objects', 0)))
                    labels['fps'].config(text=f"{data.get('fps', 0.0):.1f} Hz")
                    
                    # Color-coded timing accuracy
                    accuracy = data.get('timing_accuracy', 'N/A')
                    labels['timing_accuracy'].config(text=accuracy)
                    
                    # Set color based on accuracy
                    if isinstance(accuracy, str) and 'EXCELLENT' in accuracy:
                        labels['timing_accuracy'].config(fg='green')
                    elif isinstance(accuracy, str) and 'GOOD' in accuracy:
                        labels['timing_accuracy'].config(fg='blue')
                    elif isinstance(accuracy, str) and 'FAIR' in accuracy:
                        labels['timing_accuracy'].config(fg='orange')
                    elif isinstance(accuracy, str) and 'POOR' in accuracy:
                        labels['timing_accuracy'].config(fg='red')
                    else:
                        labels['timing_accuracy'].config(fg='black')
                    
                    status_label.config(text="Status: Active", fg='green')
                else:
                    status_label.config(text="Status: No data", fg='orange')
                
            except Exception as e:
                status_label.config(text=f"Status: Error - {e}", fg='red')
            
            # Schedule next update
            root.after(500, update_display)
        
        # Handle close
        def on_closing():
            """Handle window close"""
            print("🔴 Process monitor window closed")
            root.quit()
            root.destroy()
            
        root.protocol("WM_DELETE_WINDOW", on_closing)
        
        # Start updates
        root.after(100, update_display)
        
        print("✅ Process-separated monitor window ready")
        
        # Run mainloop
        try:
            root.mainloop()
        except KeyboardInterrupt:
            print("⌨️ Monitor interrupted by signal")
        except Exception as e:
            print(f"❌ Monitor process error: {e}")
        finally:
            # Ensure clean shutdown
            try:
                root.quit()
                root.destroy()
            except:
                pass
            print("🛑 Process-separated monitor exiting")
            
    except Exception as e:
        print(f"❌ Process monitor error: {e}")
        import traceback
        traceback.print_exc()


class ProcessSeparatedMonitor:
    """Process-separated simulation monitor"""
    
    def __init__(self, title="SimPyROS Process Monitor"):
        self.title = title
        self.process = None
        self.data_file = os.path.join(tempfile.gettempdir(), "simpyros_process_monitor_data.json")
        self.running = False
        
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
        """Update monitor data"""
        if not self.running:
            return
            
        try:
            with open(self.data_file, 'w') as f:
                json.dump(data, f)
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