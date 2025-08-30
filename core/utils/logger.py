#!/usr/bin/env python3
"""
SimPyROS Centralized Logging Configuration

Provides consistent logging functionality across all modules with configurable
levels and formats for better debugging and production use.
"""

import logging
import os
import sys
from typing import Optional

# Default logging configuration
DEFAULT_LOG_LEVEL = os.getenv('SIMPYROS_LOG_LEVEL', 'INFO').upper()
DEFAULT_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
SIMPLE_FORMAT = '%(levelname)s: %(message)s'

# Logger cache to prevent duplicates
_loggers = {}

# Component-specific logging configuration
COMPONENT_LOG_CONFIG = {
    'visualization': {'file': 'logs/visualization.log', 'console': True},
    'simulation': {'file': 'logs/simulation.log', 'console': True},
    'robot': {'file': 'logs/robot.log', 'console': True},
    'monitor': {'file': 'logs/monitor.log', 'console': False},
    'performance': {'file': 'logs/performance.log', 'console': False},
    'multiprocessing': {'file': 'logs/multiprocessing.log', 'console': False},
    'examples': {'file': None, 'console': True}  # Examples only to console
}


def _ensure_log_directory():
    """Ensure logs directory exists"""
    log_dir = 'logs'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir, exist_ok=True)


def _get_component_from_name(name: str) -> str:
    """Extract component type from logger name"""
    if 'pyvista' in name or 'visualizer' in name:
        return 'visualization'
    elif 'simulation_manager' in name or 'simulation_object' in name:
        return 'simulation'
    elif 'robot' in name:
        return 'robot'
    elif 'monitor' in name:
        return 'monitor'
    elif 'performance' in name or 'benchmark' in name:
        return 'performance'
    elif 'multiprocessing' in name or 'cleanup' in name:
        return 'multiprocessing'
    elif 'examples' in name:
        return 'examples'
    else:
        return 'simulation'  # Default fallback


def get_logger(name: str, level: Optional[str] = None) -> logging.Logger:
    """
    Get or create logger with consistent formatting
    
    Args:
        name: Logger name (usually __name__)
        level: Optional log level override
        
    Returns:
        Configured logger instance
    """
    if name in _loggers:
        return _loggers[name]
    
    logger = logging.getLogger(name)
    
    # Level configuration
    log_level = getattr(logging, (level or DEFAULT_LOG_LEVEL).upper(), logging.INFO)
    logger.setLevel(log_level)
    
    # Add handlers only if none exists (avoid duplicate handlers)
    if not logger.handlers:
        _ensure_log_directory()
        
        # Get component configuration
        component = _get_component_from_name(name)
        config = COMPONENT_LOG_CONFIG.get(component, COMPONENT_LOG_CONFIG['simulation'])
        
        # Console handler (if enabled for this component)
        if config['console']:
            console_handler = logging.StreamHandler(sys.stdout)
            
            # Use simple format for user-facing messages
            if name.startswith('simpyros'):
                formatter = logging.Formatter(SIMPLE_FORMAT)
            else:
                formatter = logging.Formatter(DEFAULT_FORMAT)
                
            console_handler.setFormatter(formatter)
            logger.addHandler(console_handler)
        
        # File handler (if specified for this component)
        if config['file']:
            try:
                file_handler = logging.FileHandler(config['file'], mode='a')
                file_formatter = logging.Formatter(DEFAULT_FORMAT)
                file_handler.setFormatter(file_formatter)
                logger.addHandler(file_handler)
            except Exception as e:
                # Fallback to console if file logging fails
                if not config['console']:
                    console_handler = logging.StreamHandler(sys.stdout)
                    console_handler.setFormatter(logging.Formatter(SIMPLE_FORMAT))
                    logger.addHandler(console_handler)
                print(f"Warning: Failed to create file handler for {config['file']}: {e}")
    
    # Prevent propagation to root logger
    logger.propagate = False
    
    _loggers[name] = logger
    return logger


def set_log_level(level: str):
    """
    Set log level for all SimPyROS logs
    
    Args:
        level: Log level ('DEBUG', 'INFO', 'WARNING', 'ERROR')
    """
    log_level = getattr(logging, level.upper(), logging.INFO)
    
    for logger in _loggers.values():
        logger.setLevel(log_level)
    
    print(f"🔧 SimPyROS logging level set to: {level.upper()}")


def enable_debug():
    """Enable debug logging for all SimPyROS components"""
    set_log_level('DEBUG')


def suppress_verbose():
    """Suppress verbose output (WARNING level and above only)"""
    set_log_level('WARNING')


# Convenience functions for common logging patterns
def log_success(logger: logging.Logger, message: str):
    """Log success message with consistent format"""
    logger.info(f"✅ {message}")


def log_warning(logger: logging.Logger, message: str):
    """Log warning message with consistent format"""
    logger.warning(f"⚠️ {message}")


def log_error(logger: logging.Logger, message: str):
    """Log error message with consistent format"""
    logger.error(f"❌ {message}")


def log_debug(logger: logging.Logger, message: str):
    """Log debug message with consistent format"""
    logger.debug(f"🔧 {message}")


def log_info(logger: logging.Logger, message: str):
    """Log info message with consistent format"""
    logger.info(f"ℹ️ {message}")


# Initialize default logger for module
logger = get_logger(__name__)