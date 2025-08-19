#!/usr/bin/env python3
"""
SimPyROS 中央集権ログ設定

全モジュールで一貫したログ機能を提供し、設定可能なレベルと
フォーマットでより良いデバッグと本番利用をサポートします。
"""

import logging
import os
import sys
from typing import Optional

# デフォルトログ設定
DEFAULT_LOG_LEVEL = os.getenv('SIMPYROS_LOG_LEVEL', 'INFO').upper()
DEFAULT_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
SIMPLE_FORMAT = '%(levelname)s: %(message)s'

# 重複ログ防止用のログキャッシュ
_loggers = {}


def get_logger(name: str, level: Optional[str] = None) -> logging.Logger:
    """
    一貫したフォーマットでログを取得または作成
    
    Args:
        name: ログ名（通常は __name__）
        level: オプションのログレベル上書き
        
    Returns:
        設定されたログインスタンス
    """
    if name in _loggers:
        return _loggers[name]
    
    logger = logging.getLogger(name)
    
    # レベル設定
    log_level = getattr(logging, (level or DEFAULT_LOG_LEVEL).upper(), logging.INFO)
    logger.setLevel(log_level)
    
    # ハンドラが存在しない場合のみ追加（重複ハンドラ回避）
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        
        # ユーザー向けメッセージにはシンプルフォーマットを使用
        if name.startswith('simpyros'):
            formatter = logging.Formatter(SIMPLE_FORMAT)
        else:
            formatter = logging.Formatter(DEFAULT_FORMAT)
            
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    
    # ルートログへの伝播を防止
    logger.propagate = False
    
    _loggers[name] = logger
    return logger


def set_log_level(level: str):
    """
    全SimPyROSログのログレベルを設定
    
    Args:
        level: ログレベル ('DEBUG', 'INFO', 'WARNING', 'ERROR')
    """
    log_level = getattr(logging, level.upper(), logging.INFO)
    
    for logger in _loggers.values():
        logger.setLevel(log_level)
    
    print(f"🔧 SimPyROS logging level set to: {level.upper()}")


def enable_debug():
    """全SimPyROSコンポーネントのデバッグログを有効化"""
    set_log_level('DEBUG')


def suppress_verbose():
    """詳細出力を抑制（WARNINGレベル以上のみ）"""
    set_log_level('WARNING')


# 一般的なログパターン用の便利関数
def log_success(logger: logging.Logger, message: str):
    """一貫したフォーマットで成功メッセージをログ出力"""
    logger.info(f"✅ {message}")


def log_warning(logger: logging.Logger, message: str):
    """一貫したフォーマットで警告メッセージをログ出力"""
    logger.warning(f"⚠️ {message}")


def log_error(logger: logging.Logger, message: str):
    """一貫したフォーマットでエラーメッセージをログ出力"""
    logger.error(f"❌ {message}")


def log_debug(logger: logging.Logger, message: str):
    """一貫したフォーマットでデバッグメッセージをログ出力"""
    logger.debug(f"🔧 {message}")


def log_info(logger: logging.Logger, message: str):
    """一貫したフォーマットで情報メッセージをログ出力"""
    logger.info(f"ℹ️ {message}")


# モジュール用デフォルトログの初期化
logger = get_logger(__name__)