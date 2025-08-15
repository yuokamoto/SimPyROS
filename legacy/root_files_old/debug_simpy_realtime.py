#!/usr/bin/env python3
"""
SimPy RealtimeEnvironment Debug Test

Tests how SimPy RealtimeEnvironment behaves with different factors
"""

import simpy.rt
import time

def test_simpy_realtime_behavior():
    """Test pure SimPy RealtimeEnvironment behavior"""
    
    factors = [0.5, 1.0, 2.0]
    
    for factor in factors:
        print(f"\n{'='*50}")
        print(f"Testing SimPy RealtimeEnvironment with factor={factor}")
        print(f"{'='*50}")
        
        env = simpy.rt.RealtimeEnvironment(factor=factor, strict=True)
        
        def test_process(duration):
            print(f"Process starting: sim_time={env.now:.3f}")
            wall_start = time.time()
            
            yield env.timeout(duration)
            
            wall_elapsed = time.time() - wall_start
            print(f"Process finished: sim_time={env.now:.3f}, wall_time={wall_elapsed:.3f}s")
            print(f"Expected wall time: {duration/factor:.3f}s")
            print(f"Wall time error: {abs(wall_elapsed - duration/factor)/max(duration/factor, 0.001)*100:.1f}%")
        
        # Start process
        env.process(test_process(3.0))
        
        # Record timing
        wall_start = time.time()
        
        print(f"Starting simulation (sim_duration=3.0s, expected_wall={3.0/factor:.3f}s)")
        
        try:
            env.run()
        except Exception as e:
            print(f"Error: {e}")
        
        wall_total = time.time() - wall_start
        print(f"Total wall time: {wall_total:.3f}s")
        print(f"Total sim time: {env.now:.3f}s")
        print(f"Actual factor: {env.now/wall_total:.3f}x (target: {factor}x)")
        
        time.sleep(1)  # Brief pause between tests

if __name__ == "__main__":
    print("🧪 SimPy RealtimeEnvironment Behavior Test")
    test_simpy_realtime_behavior()
    print("\n✅ SimPy RealtimeEnvironment test complete!")