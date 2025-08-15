#!/usr/bin/env python3
"""
Visual Demos Launcher - 3つの視覚デモを簡単に起動
"""

import sys
import os
import subprocess
import time

def print_demo_menu():
    """デモメニュー表示"""
    print("🎮 SimPyROS Visual Demos Launcher")
    print("=" * 50)
    print()
    print("Available visual comparison demos:")
    print()
    print("1️⃣ Enhanced PyVista Demo (Recommended)")
    print("   - Unified backend switching in single simulation")
    print("   - Existing PyVistaVisualizer integration")
    print("   - Interactive keyboard controls")
    print("   - Requirements: pyvista")
    print()
    print("2️⃣ Tkinter Interactive UI Demo")
    print("   - Desktop GUI with real-time charts")
    print("   - Interactive backend switching")
    print("   - Performance graphs and controls")
    print("   - Requirements: matplotlib (tkinter included)")
    print()
    print("3️⃣ Web Dashboard Demo")
    print("   - Modern browser-based interface")
    print("   - Real-time WebSocket updates")
    print("   - Multi-backend comparison")
    print("   - Requirements: flask, flask-socketio, eventlet")
    print()
    print("4️⃣ Console Unified Demo")
    print("   - Interactive backend switching")
    print("   - Real-time performance comparison")
    print("   - No additional dependencies required")
    print("   - Works on any system")
    print()
    print("5️⃣ Basic Performance Test")
    print("   - Simple mock performance test")
    print("   - No dependencies, always works")
    print("   - Quick validation")
    print()

def check_dependencies():
    """依存関係チェック"""
    deps = {
        'pyvista': False,
        'matplotlib': False,
        'flask': False,
        'tkinter': False
    }
    
    try:
        import pyvista
        deps['pyvista'] = True
    except ImportError:
        pass
    
    try:
        import matplotlib
        deps['matplotlib'] = True
    except ImportError:
        pass
    
    try:
        import flask
        deps['flask'] = True
    except ImportError:
        pass
    
    try:
        import tkinter
        deps['tkinter'] = True
    except ImportError:
        pass
    
    return deps

def print_dependency_status(deps):
    """依存関係状況表示"""
    print("📋 Dependency Status:")
    print("-" * 30)
    
    for dep, available in deps.items():
        status = "✅ Available" if available else "❌ Missing"
        print(f"{dep:<12}: {status}")
    
    print()
    
    if not deps['pyvista']:
        print("💡 To install PyVista: pip install pyvista")
    if not deps['matplotlib']:
        print("💡 To install Matplotlib: pip install matplotlib")
    if not deps['flask']:
        print("💡 To install Flask: pip install flask flask-socketio eventlet")
    
    print()

def run_demo(demo_choice: str, deps: dict):
    """デモ実行"""
    
    if demo_choice == '1':
        # Enhanced PyVista Demo
        if not deps['pyvista']:
            print("❌ PyVista not available. Please install: pip install pyvista")
            return False
        
        print("🚀 Launching Enhanced PyVista Demo...")
        subprocess.run([
            sys.executable, 
            "examples/unified/enhanced_pyvista_demo.py"
        ])
        
    elif demo_choice == '2':
        # Tkinter UI Demo
        if not deps['matplotlib'] or not deps['tkinter']:
            missing = []
            if not deps['matplotlib']:
                missing.append("matplotlib")
            if not deps['tkinter']:
                missing.append("tkinter")
            print(f"❌ Missing dependencies: {', '.join(missing)}")
            print("💡 Install matplotlib: pip install matplotlib")
            print("💡 tkinter should be included with Python")
            return False
        
        print("🚀 Launching Tkinter Interactive UI Demo...")
        subprocess.run([
            sys.executable,
            "examples/unified/interactive_ui_demo.py"
        ])
        
    elif demo_choice == '3':
        # Web Dashboard Demo
        if not deps['flask']:
            print("❌ Flask not available. Please install: pip install flask flask-socketio eventlet")
            return False
        
        print("🚀 Launching Web Dashboard Demo...")
        print("   Opening on http://localhost:5000")
        print("   Press Ctrl+C to stop the server")
        subprocess.run([
            sys.executable,
            "examples/unified/web_dashboard_demo.py"
        ])
        
    elif demo_choice == '4':
        # Console Unified Demo
        print("🚀 Launching Console Unified Demo...")
        subprocess.run([
            sys.executable,
            "examples/unified/console_unified_demo.py"
        ])
        
    elif demo_choice == '5':
        # Basic Performance Test
        print("🚀 Launching Basic Performance Test...")
        subprocess.run([
            sys.executable,
            "test_unified_simple.py"
        ])
        
    else:
        print("❌ Invalid choice")
        return False
    
    return True

def quick_install_guide():
    """クイックインストールガイド"""
    print("📦 Quick Installation Guide")
    print("=" * 40)
    print()
    print("For all visual demos:")
    print("pip install pyvista matplotlib flask flask-socketio eventlet")
    print()
    print("For individual demos:")
    print("  PyVista Demo: pip install pyvista")
    print("  Tkinter Demo: pip install matplotlib")
    print("  Web Demo:     pip install flask flask-socketio eventlet")
    print("  Console Demo: No additional dependencies")
    print()

def main():
    """メイン実行"""
    
    try:
        print_demo_menu()
        
        # 依存関係チェック
        deps = check_dependencies()
        print_dependency_status(deps)
        
        # 利用可能なデモの推奨
        available_demos = []
        if deps['pyvista']:
            available_demos.append("1 (Enhanced PyVista)")
        if deps['matplotlib'] and deps['tkinter']:
            available_demos.append("2 (Tkinter UI)")
        if deps['flask']:
            available_demos.append("3 (Web Dashboard)")
        available_demos.append("4 (Console)")
        available_demos.append("5 (Basic Test)")
        
        if available_demos:
            print(f"🎯 Available demos: {', '.join(available_demos)}")
        else:
            print("⚠️ No visual demos available. Only console demo can run.")
        
        print()
        
        # ユーザー選択
        while True:
            try:
                choice = input("Select demo (1-5) or 'i' for install guide, 'q' to quit: ").strip().lower()
                
                if choice == 'q':
                    print("👋 Goodbye!")
                    break
                elif choice == 'i':
                    quick_install_guide()
                    continue
                elif choice in ['1', '2', '3', '4', '5']:
                    success = run_demo(choice, deps)
                    if success:
                        print("\n✅ Demo completed!")
                    else:
                        print("\n❌ Demo failed to start")
                    
                    # 再実行するか確認
                    again = input("\nRun another demo? (y/n): ").strip().lower()
                    if not again.startswith('y'):
                        break
                else:
                    print("❌ Invalid choice. Please enter 1-5, 'i', or 'q'")
                    
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break
            except EOFError:
                print("\n👋 Goodbye!")
                break
    
    except Exception as e:
        print(f"❌ Launcher error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()