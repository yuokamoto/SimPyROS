#!/usr/bin/env python3
"""
Test timing performance after fixes
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from examples.beginner.basic_simulation import simple_control_example


def test_fixed_timing():
    """Test the fixed simple control example"""
    print("🧪 Testing Fixed Timing Performance...")
    print("This will run simple_control_example and show timing statistics")
    print("=" * 60)
    
    try:
        # Run the fixed simple control example
        simple_control_example(unified_process=False)
        
        print("\n✅ Test completed!")
        print("Check the timing statistics above to see real-time factor accuracy")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_fixed_timing()