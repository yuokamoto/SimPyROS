#!/usr/bin/env python3
"""
Test script for Ctrl-C termination fixes

This script tests that all processes terminate properly when Ctrl-C is pressed.
"""

import sys
import os
import time
import signal
import subprocess

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.multiprocessing_cleanup import MultiprocessingCleaner

def test_termination_timing():
    """Test basic termination timing without running full simulation"""
    print("🧪 Testing termination timing...")
    
    cleaner = MultiprocessingCleaner()
    
    # Create a dummy process for testing
    import multiprocessing as mp
    
    def dummy_worker():
        """Dummy worker with signal handling"""
        import signal
        
        def signal_handler(signum, frame):
            print(f"🔔 Dummy worker received signal {signum}")
            raise KeyboardInterrupt("Terminated by signal")
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("💀 Dummy worker terminating gracefully")
    
    process = mp.Process(target=dummy_worker)
    cleaner.register_process(process)
    process.start()
    
    print(f"✅ Test process started (PID: {process.pid})")
    
    # Test termination timing
    start_time = time.time()
    try:
        # Wait a bit then trigger cleanup
        time.sleep(1.0)
        print("🔄 Testing process cleanup...")
        cleaner.cleanup_process(process)
        
        end_time = time.time()
        cleanup_time = end_time - start_time
        
        print(f"⏱️ Cleanup completed in {cleanup_time:.2f} seconds")
        
        if cleanup_time < 3.0:
            print("✅ Fast termination test PASSED")
        else:
            print("❌ Fast termination test FAILED (too slow)")
            
        return cleanup_time < 3.0
        
    except KeyboardInterrupt:
        print("🔴 Test interrupted")
        return False

def test_performance_demo_termination():
    """Test termination of the actual performance demo"""
    print("\n🧪 Testing performance demo termination...")
    
    try:
        # Run the basic simulation in a subprocess
        cmd = [
            sys.executable, 
            "examples/beginner/basic_simulation.py",
            "--example", "performance",
            "--num-robots", "10",  # Smaller number for test
            "--duration", "30",     # Long enough to test interruption
            "--rtf", "2.0"
        ]
        
        print(f"🚀 Starting subprocess: {' '.join(cmd)}")
        
        # Start process
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        
        # Let it run for a few seconds
        print("⏳ Letting simulation run for 3 seconds...")
        time.sleep(3.0)
        
        # Send SIGINT (Ctrl-C)
        print("🔴 Sending SIGINT to test termination...")
        start_time = time.time()
        
        process.send_signal(signal.SIGINT)
        
        # Wait for termination with timeout
        try:
            stdout, _ = process.communicate(timeout=10.0)
            end_time = time.time()
            termination_time = end_time - start_time
            
            print(f"⏱️ Process terminated in {termination_time:.2f} seconds")
            print(f"🔍 Exit code: {process.returncode}")
            
            # Show last few lines of output
            if stdout:
                lines = stdout.strip().split('\n')
                print("📄 Last output lines:")
                for line in lines[-5:]:
                    print(f"   {line}")
            
            if termination_time < 5.0:
                print("✅ Performance demo termination test PASSED")
                return True
            else:
                print("❌ Performance demo termination test FAILED (too slow)")
                return False
                
        except subprocess.TimeoutExpired:
            print("❌ Process did not terminate within timeout - FAILED")
            process.kill()
            return False
            
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False

def main():
    """Run all termination tests"""
    print("🧪 SimPyROS Termination Fix Test Suite")
    print("="*50)
    
    tests = [
        ("Basic Termination Timing", test_termination_timing),
        ("Performance Demo Termination", test_performance_demo_termination)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🔬 Running: {test_name}")
        try:
            if test_func():
                print(f"✅ {test_name}: PASSED")
                passed += 1
            else:
                print(f"❌ {test_name}: FAILED")
        except Exception as e:
            print(f"💥 {test_name}: ERROR - {e}")
    
    print(f"\n📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All termination tests PASSED!")
        print("💡 Ctrl-C should now work properly with SimPyROS")
        return 0
    else:
        print("⚠️ Some tests failed - termination issues may persist")
        return 1

if __name__ == "__main__":
    exit(main())