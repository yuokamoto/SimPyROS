#!/usr/bin/env python3
"""
Test 10 robots performance demo
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the fixed function
from examples.beginner.basic_simulation import multi_robots_performance_demo


def test_10_robots():
    """Test 10 robots performance demo"""
    print("🧪 Testing 10 robots performance demo...")
    
    try:
        # Test with 10 robots, no frequency grouping, visualization enabled
        result = multi_robots_performance_demo(
            num_robots=10,
            use_frequency_grouping=False,  # Use working approach
            real_time_factor=1.0,
            visualization=True
        )
        
        print("✅ 10 robots test completed successfully!")
        if result:
            print(f"📊 Performance results: {result}")
        
    except Exception as e:
        print(f"❌ 10 robots test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_10_robots()