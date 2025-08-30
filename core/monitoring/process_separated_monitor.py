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
from ..utils.multiprocessing_cleanup import register_multiprocessing_process
from .simulation_monitor import BaseMonitor


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
        
        # Use BaseMonitor class methods for UI creation
        from ..simulation_monitor import BaseMonitor
        
        labels_dict, buttons_dict, status_label = BaseMonitor.create_monitor_ui(
            parent_frame=main_frame,
            show_debug_info=False,  # Debug info disabled by default
            enable_controls=False   # Controls not supported in process-separated
        )
        
        # Convert labels_dict to simple format for compatibility
        labels = {key: label_tuple[0] for key, label_tuple in labels_dict.items()}
        
        def update_display():
            """Update display with data from file"""
            # Check if shutdown was requested
            if shutdown_requested:
                return  # Don't schedule further updates
                
            # Use BaseMonitor class method for data reading and processing
            data, status_message, status_color = BaseMonitor.read_monitor_data(data_file)
            
            if data is not None:
                # Update labels using BaseMonitor class method
                BaseMonitor.update_labels_simple(labels, data)
            
            # Update status label
            if not shutdown_requested:  # Only update status if not shutting down
                status_label.config(text=status_message, fg=status_color)
            
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
        self.show_debug_info = False  # Default off for process-separated monitor
        
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