#!/usr/bin/env python3
"""
External Mesh Manager for SimPyROS

Manages external robot repositories and mesh files for robots like TurtleBot3 and UR5.
Handles automatic repository cloning, URDF path resolution, and mesh file management.

Features:
- Automatic git repository cloning
- Pre-configured support for popular robots
- ROS package path resolution
- Mesh file validation and caching
"""

import os
import sys
import subprocess
import urllib.parse
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass
from pathlib import Path
import warnings

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))


@dataclass
class RepositoryInfo:
    """Information about an external robot repository"""
    name: str
    url: str
    branch: str = "main"
    urdf_paths: List[str] = None
    mesh_paths: List[str] = None
    variants: Dict[str, str] = None
    description: str = ""

    def __post_init__(self):
        if self.urdf_paths is None:
            self.urdf_paths = []
        if self.mesh_paths is None:
            self.mesh_paths = []
        if self.variants is None:
            self.variants = {}


class ExternalMeshManager:
    """
    Manager for external robot repositories and mesh files
    
    This class handles:
    - Automatic cloning of external robot repositories
    - URDF path resolution for different robot variants
    - Mesh file management and validation
    - ROS package path resolution
    """

    # Pre-configured repository information
    SUPPORTED_REPOS = {
        'turtlebot3': RepositoryInfo(
            name='turtlebot3',
            url='https://github.com/ROBOTIS-GIT/turtlebot3.git',
            branch='ros2-devel',
            urdf_paths=[
                'turtlebot3_description/urdf/turtlebot3_burger.urdf',
                'turtlebot3_description/urdf/turtlebot3_waffle.urdf', 
                'turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf'
            ],
            mesh_paths=[
                'turtlebot3_description/meshes/'
            ],
            variants={
                'burger': 'turtlebot3_description/urdf/turtlebot3_burger.urdf',
                'waffle': 'turtlebot3_description/urdf/turtlebot3_waffle.urdf',
                'waffle_pi': 'turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf'
            },
            description="TurtleBot3 mobile robot family"
        ),
        
        'ur5': RepositoryInfo(
            name='ur5',
            url='https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git',
            branch='ros2',
            urdf_paths=[
                'urdf/ur3.urdf.xacro',
                'urdf/ur3e.urdf.xacro',
                'urdf/ur5.urdf.xacro',
                'urdf/ur5e.urdf.xacro',
                'urdf/ur10.urdf.xacro',
                'urdf/ur10e.urdf.xacro',
                'urdf/ur16e.urdf.xacro'
            ],
            mesh_paths=[
                'meshes/ur3/',
                'meshes/ur3e/',
                'meshes/ur5/',
                'meshes/ur5e/',
                'meshes/ur10/',
                'meshes/ur10e/',
                'meshes/ur16e/'
            ],
            variants={
                'ur3': 'urdf/ur3.urdf.xacro',
                'ur3e': 'urdf/ur3e.urdf.xacro',
                'ur5': 'urdf/ur5.urdf.xacro',
                'ur5e': 'urdf/ur5e.urdf.xacro',
                'ur10': 'urdf/ur10.urdf.xacro',
                'ur10e': 'urdf/ur10e.urdf.xacro',
                'ur16e': 'urdf/ur16e.urdf.xacro'
            },
            description="Universal Robots industrial robot arms"
        )
    }

    def __init__(self, repos_dir: str = "external_repos"):
        """
        Initialize external mesh manager
        
        Args:
            repos_dir: Directory to store cloned repositories
        """
        self.repos_dir = Path(repos_dir)
        self.repos_dir.mkdir(exist_ok=True)
        
        # Custom repositories added by user
        self.custom_repos: Dict[str, RepositoryInfo] = {}
        
        print(f"🗂️ External mesh manager initialized (repos: {self.repos_dir})")

    def add_custom_repository(self, repo_info: RepositoryInfo) -> bool:
        """
        Add a custom repository configuration
        
        Args:
            repo_info: Repository information
            
        Returns:
            Success status
        """
        if repo_info.name in self.SUPPORTED_REPOS:
            warnings.warn(f"Repository '{repo_info.name}' already exists in supported repos")
            return False
            
        self.custom_repos[repo_info.name] = repo_info
        print(f"✅ Added custom repository: {repo_info.name}")
        return True

    def get_repository_info(self, repo_name: str) -> Optional[RepositoryInfo]:
        """Get repository information by name"""
        if repo_name in self.SUPPORTED_REPOS:
            return self.SUPPORTED_REPOS[repo_name]
        elif repo_name in self.custom_repos:
            return self.custom_repos[repo_name]
        else:
            return None

    def list_available_repositories(self) -> List[str]:
        """List all available repository names"""
        return list(self.SUPPORTED_REPOS.keys()) + list(self.custom_repos.keys())

    def clone_repository(self, repo_name: str, force_update: bool = False) -> bool:
        """
        Clone or update external repository
        
        Args:
            repo_name: Name of repository to clone
            force_update: Force update if repository already exists
            
        Returns:
            Success status
        """
        repo_info = self.get_repository_info(repo_name)
        if not repo_info:
            print(f"❌ Unknown repository: {repo_name}")
            print(f"Available repositories: {self.list_available_repositories()}")
            return False

        repo_path = self.repos_dir / repo_name
        
        try:
            if repo_path.exists():
                if force_update:
                    print(f"🔄 Updating repository: {repo_name}")
                    result = subprocess.run(
                        ["git", "pull"],
                        cwd=repo_path,
                        capture_output=True,
                        text=True,
                        check=True
                    )
                    print(f"✅ Updated {repo_name}: {result.stdout.strip()}")
                else:
                    print(f"✅ Repository {repo_name} already exists (use force_update=True to update)")
                return True
            else:
                print(f"📦 Cloning repository: {repo_name}")
                print(f"   URL: {repo_info.url}")
                print(f"   Branch: {repo_info.branch}")
                
                result = subprocess.run(
                    ["git", "clone", "-b", repo_info.branch, repo_info.url, str(repo_path)],
                    capture_output=True,
                    text=True,
                    check=True
                )
                
                print(f"✅ Successfully cloned {repo_name}")
                return True
                
        except subprocess.CalledProcessError as e:
            print(f"❌ Git operation failed for {repo_name}: {e.stderr}")
            return False
        except Exception as e:
            print(f"❌ Failed to clone {repo_name}: {e}")
            return False

    def get_repository_path(self, repo_name: str) -> Optional[Path]:
        """Get the local path to a repository"""
        repo_path = self.repos_dir / repo_name
        return repo_path if repo_path.exists() else None

    def get_urdf_path(self, repo_name: str, variant: Optional[str] = None) -> Optional[str]:
        """
        Get URDF file path for a specific robot variant
        
        Args:
            repo_name: Repository name
            variant: Robot variant (if None, uses first available)
            
        Returns:
            Absolute path to URDF file or None
        """
        repo_info = self.get_repository_info(repo_name)
        if not repo_info:
            return None

        repo_path = self.get_repository_path(repo_name)
        if not repo_path:
            print(f"⚠️ Repository {repo_name} not cloned yet")
            return None

        # Try variant-specific path first
        if variant and variant in repo_info.variants:
            urdf_relative = repo_info.variants[variant]
            urdf_path = repo_path / urdf_relative
            
            if urdf_path.exists():
                return str(urdf_path.absolute())
            else:
                print(f"⚠️ URDF not found: {urdf_path}")

        # Try first available URDF
        for urdf_relative in repo_info.urdf_paths:
            urdf_path = repo_path / urdf_relative
            if urdf_path.exists():
                if variant:
                    print(f"⚠️ Variant '{variant}' not found, using: {urdf_relative}")
                return str(urdf_path.absolute())

        print(f"❌ No URDF files found for {repo_name}")
        return None

    def list_robot_variants(self, repo_name: str) -> List[str]:
        """List available variants for a robot"""
        repo_info = self.get_repository_info(repo_name)
        if not repo_info:
            return []
        
        return list(repo_info.variants.keys())

    def resolve_mesh_path(self, mesh_path: str, repo_name: str) -> Optional[str]:
        """
        Resolve mesh path from URDF reference to absolute path
        
        Args:
            mesh_path: Original mesh path from URDF (may be package:// URL)
            repo_name: Repository context for resolution
            
        Returns:
            Absolute path to mesh file or None
        """
        repo_path = self.get_repository_path(repo_name)
        if not repo_path:
            return None

        repo_info = self.get_repository_info(repo_name)
        if not repo_info:
            return None

        # Handle package:// URLs
        if mesh_path.startswith('package://'):
            # Remove package:// prefix
            package_relative = mesh_path[10:]
            
            # Try to find the mesh file
            for mesh_dir in repo_info.mesh_paths:
                potential_path = repo_path / mesh_dir / package_relative.split('/', 1)[-1]
                if potential_path.exists():
                    return str(potential_path.absolute())
            
            # Try direct resolution within repository
            potential_path = repo_path / package_relative
            if potential_path.exists():
                return str(potential_path.absolute())

        # Handle relative paths
        elif not os.path.isabs(mesh_path):
            for mesh_dir in repo_info.mesh_paths:
                potential_path = repo_path / mesh_dir / mesh_path
                if potential_path.exists():
                    return str(potential_path.absolute())

        # Handle absolute paths
        elif os.path.exists(mesh_path):
            return mesh_path

        return None

    def validate_repository(self, repo_name: str) -> Dict[str, bool]:
        """
        Validate repository structure and files
        
        Returns:
            Dictionary with validation results
        """
        results = {
            'exists': False,
            'has_urdf': False,
            'has_meshes': False,
            'variants_valid': False
        }

        repo_path = self.get_repository_path(repo_name)
        if not repo_path:
            return results

        results['exists'] = True

        repo_info = self.get_repository_info(repo_name)
        if not repo_info:
            return results

        # Check URDF files
        urdf_found = 0
        for urdf_relative in repo_info.urdf_paths:
            urdf_path = repo_path / urdf_relative
            if urdf_path.exists():
                urdf_found += 1

        results['has_urdf'] = urdf_found > 0

        # Check mesh directories
        mesh_dirs_found = 0
        for mesh_dir in repo_info.mesh_paths:
            mesh_path = repo_path / mesh_dir
            if mesh_path.exists() and mesh_path.is_dir():
                mesh_dirs_found += 1

        results['has_meshes'] = mesh_dirs_found > 0

        # Check variants
        variants_valid = 0
        for variant, urdf_relative in repo_info.variants.items():
            urdf_path = repo_path / urdf_relative
            if urdf_path.exists():
                variants_valid += 1

        results['variants_valid'] = variants_valid > 0

        return results

    def print_repository_status(self, repo_name: Optional[str] = None):
        """Print status of repositories"""
        repos_to_check = [repo_name] if repo_name else self.list_available_repositories()
        
        print(f"\n📊 Repository Status")
        print("=" * 50)
        
        for repo in repos_to_check:
            repo_info = self.get_repository_info(repo)
            if not repo_info:
                continue
                
            validation = self.validate_repository(repo)
            
            status_icon = "✅" if validation['exists'] else "❌"
            print(f"\n{status_icon} {repo.upper()}")
            print(f"   Description: {repo_info.description}")
            print(f"   URL: {repo_info.url}")
            print(f"   Branch: {repo_info.branch}")
            print(f"   Cloned: {'Yes' if validation['exists'] else 'No'}")
            
            if validation['exists']:
                print(f"   URDF files: {'Found' if validation['has_urdf'] else 'Missing'}")
                print(f"   Mesh files: {'Found' if validation['has_meshes'] else 'Missing'}")
                print(f"   Variants: {len(repo_info.variants)} configured")
                
                if repo_info.variants:
                    print(f"   Available variants: {', '.join(repo_info.variants.keys())}")

    def cleanup_repositories(self, repo_names: Optional[List[str]] = None):
        """
        Clean up repository directories
        
        Args:
            repo_names: List of specific repositories to clean (None for all)
        """
        import shutil
        
        if repo_names is None:
            # Clean all repositories
            if self.repos_dir.exists():
                shutil.rmtree(self.repos_dir)
                self.repos_dir.mkdir()
                print(f"🗑️ Cleaned all repositories from {self.repos_dir}")
        else:
            # Clean specific repositories
            for repo_name in repo_names:
                repo_path = self.repos_dir / repo_name
                if repo_path.exists():
                    shutil.rmtree(repo_path)
                    print(f"🗑️ Cleaned repository: {repo_name}")


# Convenience functions
def setup_external_repositories(repo_names: List[str], force_update: bool = False, repos_dir: str = "external_repos") -> bool:
    """
    Setup multiple external repositories at once
    
    Args:
        repo_names: List of repository names to setup
        force_update: Force update existing repositories
        repos_dir: Directory for repositories
        
    Returns:
        Success status (True if all repositories were setup successfully)
    """
    manager = ExternalMeshManager(repos_dir)
    success_count = 0
    
    print(f"📦 Setting up {len(repo_names)} external repositories...")
    
    for repo_name in repo_names:
        if manager.clone_repository(repo_name, force_update):
            success_count += 1
        else:
            print(f"⚠️ Failed to setup repository: {repo_name}")
    
    print(f"✅ Successfully setup {success_count}/{len(repo_names)} repositories")
    return success_count == len(repo_names)


def get_robot_urdf_from_external(repo_name: str, variant: Optional[str] = None, repos_dir: str = "external_repos") -> Optional[str]:
    """
    Convenience function to get URDF path from external repository
    
    Args:
        repo_name: Repository name
        variant: Robot variant
        repos_dir: Directory for repositories
        
    Returns:
        Absolute path to URDF file or None
    """
    manager = ExternalMeshManager(repos_dir)
    
    # Ensure repository is cloned
    if not manager.clone_repository(repo_name):
        return None
        
    return manager.get_urdf_path(repo_name, variant)


if __name__ == "__main__":
    # Demo usage
    print("🚀 External Mesh Manager Demo")
    
    manager = ExternalMeshManager()
    manager.print_repository_status()
    
    # Setup TurtleBot3 as example
    if manager.clone_repository('turtlebot3'):
        urdf_path = manager.get_urdf_path('turtlebot3', 'waffle_pi')
        if urdf_path:
            print(f"\n✅ TurtleBot3 URDF: {urdf_path}")
        
        variants = manager.list_robot_variants('turtlebot3')
        print(f"Available variants: {variants}")