#!/usr/bin/env python3
"""
Multiprocessing resource cleanup utilities

Provides functions to properly clean up multiprocessing resources
to prevent orphaned resource_tracker processes.
"""

import multiprocessing as mp
import atexit
import signal
import os
from typing import List
from core.logger import get_logger, log_info, log_warning

logger = get_logger('simpyros.multiprocessing_cleanup')


class MultiprocessingCleaner:
    """
    Class to manage proper cleanup of multiprocessing resources
    """
    
    def __init__(self):
        self.active_processes: List[mp.Process] = []
        self.active_queues: List[mp.Queue] = []
        self.cleanup_registered = False
        
    def register_process(self, process: mp.Process):
        """Register process for cleanup management"""
        self.active_processes.append(process)
        self._ensure_cleanup_registered()
        
    def register_queue(self, queue: mp.Queue):
        """Register queue for cleanup management"""
        self.active_queues.append(queue)
        self._ensure_cleanup_registered()
        
    def _ensure_cleanup_registered(self):
        """Ensure cleanup handlers are registered"""
        if not self.cleanup_registered:
            atexit.register(self.cleanup_all)
            signal.signal(signal.SIGTERM, self._signal_handler)
            signal.signal(signal.SIGINT, self._signal_handler)
            self.cleanup_registered = True
            log_info(logger, "Multiprocessing cleanup handlers registered")
            
    def _signal_handler(self, signum, frame):
        """Signal handler for cleanup"""
        log_info(logger, f"Received signal {signum}, cleaning up multiprocessing resources")
        self.cleanup_all()
        # Exit after cleanup
        if signum == signal.SIGINT:
            log_info(logger, "Exiting due to SIGINT")
            import sys
            sys.exit(0)
        elif signum == signal.SIGTERM:
            log_info(logger, "Exiting due to SIGTERM")
            import sys
            sys.exit(0)
        
    def cleanup_all(self):
        """Cleanup all multiprocessing resources"""
        try:
            # Process cleanup
            for process in self.active_processes[:]:
                self.cleanup_process(process)
            self.active_processes.clear()
            
            # Queue cleanup
            for queue in self.active_queues[:]:
                self.cleanup_queue(queue)
            self.active_queues.clear()
            
            log_info(logger, "All multiprocessing resources cleaned up")
            
        except Exception as e:
            log_warning(logger, f"Error during multiprocessing cleanup: {e}")
            
    def cleanup_process(self, process: mp.Process):
        """Cleanup single process"""
        try:
            if process and process.is_alive():
                log_info(logger, f"Terminating process PID: {process.pid}")
                
                # Send SIGINT first to allow graceful shutdown in child processes
                try:
                    import os
                    os.kill(process.pid, signal.SIGINT)
                    process.join(timeout=1.0)  # Wait for graceful shutdown
                except (OSError, ProcessLookupError):
                    pass  # Process might have already exited
                
                # Fast graceful termination
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=0.5)  # Reduced from 2.0s to 0.5s
                
                # Force kill if still alive
                if process.is_alive():
                    log_warning(logger, f"Force killing process PID: {process.pid}")
                    process.kill()
                    process.join(timeout=0.2)  # Reduced from 1.0s to 0.2s
                    
                if process.is_alive():
                    log_warning(logger, f"Process PID: {process.pid} still alive after cleanup")
                else:
                    log_info(logger, f"Process PID: {process.pid} successfully terminated")
                    
        except Exception as e:
            log_warning(logger, f"Error cleaning up process: {e}")
            
    def cleanup_queue(self, queue: mp.Queue):
        """Cleanup single queue"""
        try:
            if queue:
                # Clear remaining items
                while not queue.empty():
                    try:
                        queue.get_nowait()
                    except:
                        break
                        
                # Close and join
                queue.close()
                queue.join_thread()
                log_info(logger, "Queue cleaned up successfully")
                
        except Exception as e:
            log_warning(logger, f"Error cleaning up queue: {e}")
            
    def remove_process(self, process: mp.Process):
        """Remove process from list"""
        if process in self.active_processes:
            self.active_processes.remove(process)
            
    def remove_queue(self, queue: mp.Queue):
        """Remove queue from list"""
        if queue in self.active_queues:
            self.active_queues.remove(queue)


# Global cleaner instance
_global_cleaner = MultiprocessingCleaner()


def register_multiprocessing_process(process: mp.Process):
    """Register process with global cleaner"""
    _global_cleaner.register_process(process)
    

def register_multiprocessing_queue(queue: mp.Queue):
    """Register queue with global cleaner"""
    _global_cleaner.register_queue(queue)
    

def cleanup_multiprocessing_resources():
    """Manually cleanup all multiprocessing resources"""
    _global_cleaner.cleanup_all()
    

def force_cleanup_resource_tracker():
    """
    Force cleanup of resource_tracker process
    
    Warning: Use this as a last resort.
    """
    try:
        import subprocess
        # Use ps command to find resource_tracker processes
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        current_pid = os.getpid()
        
        for line in result.stdout.split('\n'):
            if 'resource_tracker' in line and 'python' in line:
                parts = line.split()
                if len(parts) > 1:
                    try:
                        pid = int(parts[1])
                        if pid != current_pid:
                            log_warning(logger, f"Force terminating resource_tracker process PID: {pid}")
                            subprocess.run(['kill', '-TERM', str(pid)], check=False)
                    except ValueError:
                        continue
                        
    except Exception as e:
        log_warning(logger, f"Error during resource_tracker cleanup: {e}")