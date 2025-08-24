#!/usr/bin/env python3
"""
Common utilities for SimPyROS

Minimal shared utilities to avoid code duplication.
"""


def safe_divide(numerator: float, denominator: float, default: float = 0.0) -> float:
    """Safe division with default value for zero denominator"""
    return numerator / denominator if denominator != 0 else default


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value between min and max"""
    return max(min_val, min(max_val, value))


def format_timing_accuracy(actual_factor: float, target_factor: float) -> str:
    """Format timing accuracy with color-coded categories"""
    if target_factor <= 0:
        return "N/A"
        
    error_percent = abs(actual_factor - target_factor) / target_factor * 100
    accuracy = 100.0 - error_percent
    
    if accuracy >= 95:
        return f"EXCELLENT ({error_percent:.1f}% error)"
    elif accuracy >= 85:
        return f"GOOD ({error_percent:.1f}% error)"
    elif accuracy >= 70:
        return f"FAIR ({error_percent:.1f}% error)"
    else:
        return f"POOR ({error_percent:.1f}% error)"