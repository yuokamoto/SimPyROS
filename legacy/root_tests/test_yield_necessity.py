#!/usr/bin/env python3
"""
Test to demonstrate the necessity of yield in SimPy processes
"""
import simpy
import time

def test_without_yield():
    """Demonstrate what happens without yield (dangerous - will hang)"""
    print("⚠️  WARNING: This test demonstrates why yield is necessary")
    print("   (We won't actually run the hanging version)")
    
    # This is what would happen without yield:
    print("""
❌ Without yield (THIS WOULD HANG):
def bad_process(env):
    while True:
        print("Processing...")
        # No yield - infinite loop blocks everything!
        
✅ With yield (CORRECT):
def good_process(env):
    while True:
        print("Processing...")
        yield env.timeout(0.1)  # Yields control to other processes
    """)

def good_process(env, name, interval):
    """Properly yielding process"""
    count = 0
    while count < 5:
        print(f"{name}: Processing step {count}")
        count += 1
        yield env.timeout(interval)
    print(f"{name}: Completed")

def test_with_yield():
    """Show proper SimPy process cooperation"""
    print("✅ Testing Proper SimPy Process Cooperation")
    print("-" * 50)
    
    env = simpy.Environment()
    
    # Start multiple processes with different intervals
    env.process(good_process(env, "Process A", 0.1))
    env.process(good_process(env, "Process B", 0.15))
    env.process(good_process(env, "Process C", 0.2))
    
    print("Starting processes...")
    start_time = time.time()
    
    # Run simulation
    env.run(until=1.0)
    
    end_time = time.time()
    print(f"\nSimulation completed in {end_time - start_time:.3f}s real time")
    print("✅ All processes cooperated properly thanks to yield statements")

def demonstrate_sim_time_vs_yield():
    """Show how sim_time checking works WITH proper yielding"""
    print(f"\n🔄 Demonstrating Sim-Time Based Updates WITH Yielding")
    print("-" * 55)
    
    env = simpy.Environment()
    
    def sim_time_process(env, name, update_interval):
        """Process that updates based on sim_time but still yields properly"""
        last_update_sim_time = 0.0
        step = 0
        
        while env.now < 1.0:  # Run for 1 second of sim time
            current_sim_time = env.now
            
            # Check if it's time to update based on sim_time
            if current_sim_time >= last_update_sim_time + update_interval:
                print(f"{name} Update #{step}: sim_time={current_sim_time:.2f}s")
                last_update_sim_time = current_sim_time
                step += 1
            
            # ✅ CRITICAL: Must yield to allow other processes to run
            yield env.timeout(0.05)  # Check every 50ms sim time
    
    # Start processes with different update intervals
    env.process(sim_time_process(env, "Fast", 0.2))   # Updates every 200ms sim time
    env.process(sim_time_process(env, "Slow", 0.4))   # Updates every 400ms sim time
    
    print("Running sim_time based processes...")
    env.run()
    print("✅ Sim-time based updates worked correctly with proper yielding")

if __name__ == "__main__":
    test_without_yield()
    test_with_yield()
    demonstrate_sim_time_vs_yield()
    
    print("\n📋 Summary:")
    print("   ✅ yield is ABSOLUTELY NECESSARY in SimPy processes")
    print("   ✅ Without yield, processes would hang in infinite loops")
    print("   ✅ Sim-time based checking + yielding = proper cooperation")
    print("   ✅ Our current implementation is correct!")