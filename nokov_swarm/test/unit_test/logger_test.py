import pytest
import os
import tempfile
import glob
from unittest.mock import Mock, MagicMock, patch, call
from PySide6.QtCore import QObject, Signal
from nokov_swarm.src.utils.logger import ROSLogRedirector, get_log_redirector, setup_ros_log_redirect, add_custom_log

class TestROSLogRedirector:
    """ROSLogRedirector类的单元测试"""
    
    def test_init(self):
        """测试ROSLogRedirector初始化"""
        redirector = ROSLogRedirector()
        
        assert isinstance(redirector, QObject)
        assert redirector._original_loggers == {}
        assert redirector._ui_handlers == []
        assert redirector._is_initialized is False
        assert redirector._rosout_subscriber is None
        assert redirector._rosout_thread is None
        assert redirector._stop_rosout is False
        assert len(redirector._level_colors) == 4  # INFO, WARN, ERROR, FATAL
    
    def test_clean_ansi_codes(self):
        """测试清理ANSI转义码"""
        redirector = ROSLogRedirector()
        
        # 测试包含ANSI转义码的文本
        text_with_ansi = "\x1b[31mError message\x1b[0m"
        cleaned = redirector._clean_ansi_codes(text_with_ansi)
        assert cleaned == "Error message"
        
        # 测试包含回车符的文本
        text_with_cr = "Line 1\r\nLine 2\r\n"
        cleaned = redirector._clean_ansi_codes(text_with_cr)
        assert cleaned == "Line 1\nLine 2"  # 修复：strip()会移除末尾的换行符
        
        # 测试包含控制字符的文本
        text_with_control = "Text\x00\x01\x02with\x03control\x04chars"
        cleaned = redirector._clean_ansi_codes(text_with_control)
        assert cleaned == "Textwithcontrolchars"
        
        # 测试空文本
        assert redirector._clean_ansi_codes("") == ""
        assert redirector._clean_ansi_codes(None) is None
    
    def test_save_logs_to_file(self):
        """测试保存日志到文件"""
        redirector = ROSLogRedirector()
        log_content = "Test log content\nLine 2\nLine 3"
        
        with patch('os.path.expanduser') as mock_expanduser:
            with patch('os.makedirs') as mock_makedirs:
                with patch('builtins.open', create=True) as mock_open:
                    with patch('nokov_swarm.src.utils.logger.datetime') as mock_datetime:
                        # 模拟时间戳
                        mock_now = Mock()
                        mock_now.strftime.return_value = "2023-01-01T12-00-00"
                        mock_datetime.now.return_value = mock_now
                        
                        mock_expanduser.return_value = "/home/user/.ros/log"
                        mock_file = Mock()
                        mock_open.return_value.__enter__.return_value = mock_file
                        
                        result = redirector.save_logs_to_file(log_content)
                        
                        # 修复：使用实际的时间戳格式
                        expected_path = "/home/user/.ros/log/nokov_swarm_logs_2023-01-01T12-00-00.txt"
                        assert result == expected_path
                        mock_expanduser.assert_called_once_with("~/.ros/log")
                        mock_makedirs.assert_called_once_with("/home/user/.ros/log", exist_ok=True)
                        mock_file.write.assert_called_once()
    
    def test_save_logs_to_file_with_filename(self):
        """测试保存日志到文件（指定文件名）"""
        redirector = ROSLogRedirector()
        log_content = "Test log content"
        filename = "test_log.txt"
        
        with patch('os.path.expanduser') as mock_expanduser:
            with patch('os.makedirs') as mock_makedirs:
                with patch('builtins.open', create=True) as mock_open:
                    mock_expanduser.return_value = "/home/user/.ros/log"
                    mock_file = Mock()
                    mock_open.return_value.__enter__.return_value = mock_file
                    
                    result = redirector.save_logs_to_file(log_content, filename)
                    
                    assert result == f"/home/user/.ros/log/{filename}"
                    mock_file.write.assert_called_once()
    
    def test_save_logs_to_file_exception(self):
        """测试保存日志到文件异常处理"""
        redirector = ROSLogRedirector()
        log_content = "Test log content"
        
        with patch('os.path.expanduser') as mock_expanduser:
            with patch('os.makedirs') as mock_makedirs:
                mock_expanduser.side_effect = Exception("Permission denied")
                
                with pytest.raises(Exception) as exc_info:
                    redirector.save_logs_to_file(log_content)
                
                assert "save log failed" in str(exc_info.value)
    
    def test_open_latest_log_file(self):
        """测试打开最新日志文件"""
        redirector = ROSLogRedirector()
        
        with patch('os.path.expanduser') as mock_expanduser:
            with patch('glob.glob') as mock_glob:
                with patch('os.path.getmtime') as mock_getmtime:
                    with patch('os.system') as mock_system:
                        mock_expanduser.return_value = "/home/user/.ros/log"
                        mock_glob.return_value = [
                            "/home/user/.ros/log/nokov_swarm_logs_2023-01-01.txt",
                            "/home/user/.ros/log/nokov_swarm_logs_2023-01-02.txt"
                        ]
                        mock_getmtime.side_effect = [1000, 2000]  # 第二个文件更新
                        
                        result = redirector.open_latest_log_file()
                        
                        assert result == "/home/user/.ros/log/nokov_swarm_logs_2023-01-02.txt"
                        mock_system.assert_called_once()
    
    def test_open_latest_log_file_no_files(self):
        """测试打开最新日志文件（没有文件）"""
        redirector = ROSLogRedirector()
        
        with patch('os.path.expanduser') as mock_expanduser:
            with patch('glob.glob') as mock_glob:
                mock_expanduser.return_value = "/home/user/.ros/log"
                mock_glob.return_value = []
                
                result = redirector.open_latest_log_file()
                
                assert result is None
    
    def test_get_log_files_list(self):
        """测试获取日志文件列表"""
        redirector = ROSLogRedirector()
        
        with patch('os.path.expanduser') as mock_expanduser:
            with patch('glob.glob') as mock_glob:
                with patch('os.stat') as mock_stat:
                    mock_expanduser.return_value = "/home/user/.ros/log"
                    mock_glob.return_value = [
                        "/home/user/.ros/log/nokov_swarm_logs_2023-01-01.txt",
                        "/home/user/.ros/log/nokov_swarm_logs_2023-01-02.txt"
                    ]
                    
                    # 模拟文件状态
                    stat1 = Mock()
                    stat1.st_size = 1024
                    stat1.st_mtime = 1000
                    stat2 = Mock()
                    stat2.st_size = 2048
                    stat2.st_mtime = 2000
                    mock_stat.side_effect = [stat1, stat2]
                    
                    result = redirector.get_log_files_list()
                    
                    assert len(result) == 2
                    assert result[0]['name'] == "nokov_swarm_logs_2023-01-02.txt"  # 按时间排序
                    assert result[0]['size'] == 2048
                    assert result[0]['size_human'] == "2.0KB"
    
    def test_format_file_size(self):
        """测试格式化文件大小"""
        redirector = ROSLogRedirector()
        
        assert redirector._format_file_size(0) == "0B"
        assert redirector._format_file_size(1024) == "1.0KB"
        assert redirector._format_file_size(1024 * 1024) == "1.0MB"
        assert redirector._format_file_size(1024 * 1024 * 1024) == "1.0GB"
        assert redirector._format_file_size(1536) == "1.5KB"
    
    def test_setup_ros_log_redirect(self):
        """测试设置ROS日志重定向"""
        redirector = ROSLogRedirector()
        mock_node = Mock()
        mock_node.get_name.return_value = "test_node"
        
        with patch.object(redirector, '_override_ros_logger') as mock_override:
            with patch.object(redirector, '_add_root_logger_handler') as mock_add_root:
                with patch.object(redirector, '_start_ros_log_listener') as mock_start_listener:
                    with patch.object(redirector, '_add_global_log_interceptor') as mock_add_interceptor:
                        with patch.object(redirector, '_add_rosout_subscriber') as mock_add_rosout:
                            redirector.setup_ros_log_redirect(mock_node)
                            
                            assert redirector._is_initialized is True
                            mock_override.assert_called_once_with(mock_node)
                            mock_add_root.assert_called_once()
                            mock_start_listener.assert_called_once_with(mock_node)
                            mock_add_interceptor.assert_called_once()
                            mock_add_rosout.assert_called_once_with(mock_node)
    
    def test_setup_ros_log_redirect_already_initialized(self):
        """测试设置ROS日志重定向（已经初始化）"""
        redirector = ROSLogRedirector()
        redirector._is_initialized = True
        mock_node = Mock()
        
        with patch.object(redirector, 'logMessage') as mock_signal:
            redirector.setup_ros_log_redirect(mock_node)
            
            # 应该发送警告消息
            mock_signal.emit.assert_called_with("WARN", "ROS2 log redirection already initialized, skipping duplicate initialization", "#f39c12")
    
    def test_override_ros_logger(self):
        """测试重写ROS日志器"""
        redirector = ROSLogRedirector()
        mock_node = Mock()
        mock_logger = Mock()
        mock_node.get_logger.return_value = mock_logger
        mock_node.get_name.return_value = "test_node"
        
        with patch.object(redirector, 'logMessage') as mock_signal:
            redirector._override_ros_logger(mock_node)
            
            # 验证原始方法被保存
            assert 'ros_node' in redirector._original_loggers
            assert 'info' in redirector._original_loggers['ros_node']
            assert 'warn' in redirector._original_loggers['ros_node']
            assert 'error' in redirector._original_loggers['ros_node']
            assert 'debug' in redirector._original_loggers['ros_node']
            assert 'fatal' in redirector._original_loggers['ros_node']
            
            # 验证方法被重写
            assert mock_logger.info != redirector._original_loggers['ros_node']['info']
            assert mock_logger.warn != redirector._original_loggers['ros_node']['warn']
            assert mock_logger.error != redirector._original_loggers['ros_node']['error']
            assert mock_logger.debug != redirector._original_loggers['ros_node']['debug']
            assert mock_logger.fatal != redirector._original_loggers['ros_node']['fatal']
    
    def test_format_ros_log_message(self):
        """测试格式化ROS日志消息"""
        redirector = ROSLogRedirector()
        
        # 测试复杂格式
        complex_message = "[crazyflie] [crazyflie_server-2] [WARN] [1756361337.611954922] [crazyflie_server]: Test message"
        result = redirector._format_ros_log_message(complex_message, "crazyflie", "WARN")
        assert result == "[crazyflie-crazyflie_server-2] Test message"
        
        # 测试标准重复格式
        standard_message = "[crazyflie] [WARN] [1756361337.611954922] [crazyflie] Test message"
        result = redirector._format_ros_log_message(standard_message, "crazyflie", "WARN")
        assert result == "[crazyflie] Test message"
        
        # 测试包含日志级别的消息
        level_message = "[INFO] Test message"
        result = redirector._format_ros_log_message(level_message, "crazyflie", "INFO")
        assert result == "[crazyflie] Test message"
        
        # 测试普通消息
        normal_message = "Test message"
        result = redirector._format_ros_log_message(normal_message, "crazyflie", "INFO")
        assert result == "[crazyflie] Test message"
    
    def test_add_custom_log(self):
        """测试添加自定义日志"""
        redirector = ROSLogRedirector()
        
        with patch.object(redirector, 'logMessage') as mock_signal:
            redirector.add_custom_log("INFO", "Test message", "#3498db")
            
            mock_signal.emit.assert_called_once_with("INFO", "Test message", "#3498db")
    
    def test_add_custom_log_default_color(self):
        """测试添加自定义日志（默认颜色）"""
        redirector = ROSLogRedirector()
        
        with patch.object(redirector, 'logMessage') as mock_signal:
            redirector.add_custom_log("INFO", "Test message")
            
            mock_signal.emit.assert_called_once_with("INFO", "Test message", "#3498db")
    
    def test_cleanup(self):
        """测试清理日志重定向器"""
        redirector = ROSLogRedirector()
        redirector._is_initialized = True
        redirector._rosout_thread = Mock()
        redirector._rosout_thread.is_alive.return_value = True
        redirector._ui_handlers = [Mock(), Mock()]
        redirector._original_loggers = {'ros_node': {}}
        
        redirector.cleanup()
        
        assert redirector._stop_rosout is True
        assert redirector._ui_handlers == []
        assert redirector._original_loggers == {}
        assert redirector._is_initialized is False


class TestROSLogRedirectorGlobalFunctions:
    """ROSLogRedirector全局函数的单元测试"""
    
    def test_get_log_redirector(self):
        """测试获取全局日志重定向器实例"""
        redirector1 = get_log_redirector()
        redirector2 = get_log_redirector()
        
        assert redirector1 is redirector2  # 应该是同一个实例
    
    def test_setup_ros_log_redirect_global(self):
        """测试设置全局ROS日志重定向"""
        mock_node = Mock()
        mock_node.get_name.return_value = "test_node"
        
        with patch('nokov_swarm.src.utils.logger.get_log_redirector') as mock_get_redirector:
            mock_redirector = Mock()
            mock_get_redirector.return_value = mock_redirector
            
            result = setup_ros_log_redirect(mock_node)
            
            assert result == mock_redirector
            mock_redirector.setup_ros_log_redirect.assert_called_once_with(mock_node)
    
    def test_add_custom_log_global(self):
        """测试添加全局自定义日志"""
        with patch('nokov_swarm.src.utils.logger.get_log_redirector') as mock_get_redirector:
            mock_redirector = Mock()
            # 修复：模拟_clean_ansi_codes方法
            mock_redirector._clean_ansi_codes.return_value = "Test message"
            mock_get_redirector.return_value = mock_redirector
            
            add_custom_log("INFO", "Test message", "#3498db")
            
            # 修复：验证调用参数，第二个参数应该是清理后的消息
            mock_redirector.add_custom_log.assert_called_once_with("INFO", "Test message", "#3498db")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])