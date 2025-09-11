import pytest
import yaml
import tempfile
import os
from pathlib import Path
from unittest.mock import Mock, MagicMock, patch
from src.core.config_handler import ConfigLoader

# 测试配置
TEST_ROS_CONFIG = {
    '/crazyflie_server': {
        'qos': {
            'history': 'keep_last',
            'depth': 10,
            'reliability': 'reliable',
            'durability': 'volatile',
            'deadline': 1000,
            'lifespan': 5000,
            'liveliness': 'automatic',
            'liveliness_lease_duration': 3000,
            'avoid_ros_namespace_conventions': False
        },
        'dds': {
            'rmw_implementation': 'rmw_fastrtps_cpp',
            'domain_id': 0
        }
    }
}

TEST_UI_CONFIG = {
    'General': {
        'language': 'zh_CN',
        'theme': 'dark',
        'auto_save': 'true'
    },
    'Display': {
        'show_grid': 'true',
        'grid_size': '1.0',
        'camera_follow': 'false'
    }
}

@pytest.fixture
def temp_ros_config_file():
    """创建临时ROS配置文件"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(TEST_ROS_CONFIG, f)
        temp_file = f.name
    
    yield temp_file
    
    # 清理
    if os.path.exists(temp_file):
        os.remove(temp_file)

@pytest.fixture
def temp_ui_config_file():
    """创建临时UI配置文件"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write('[General]\n')
        f.write('language = zh_CN\n')
        f.write('theme = dark\n')
        f.write('auto_save = true\n\n')
        f.write('[Display]\n')
        f.write('show_grid = true\n')
        f.write('grid_size = 1.0\n')
        f.write('camera_follow = false\n')
        temp_file = f.name
    
    yield temp_file
    
    # 清理
    if os.path.exists(temp_file):
        os.remove(temp_file)

@pytest.fixture
def temp_invalid_ui_config_file():
    """创建无效的UI配置文件（INI格式错误）"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write('invalid ini content without sections')
        temp_file = f.name
    
    yield temp_file
    
    # 清理
    if os.path.exists(temp_file):
        os.remove(temp_file)

class TestConfigLoader:
    """ConfigLoader测试类"""
    
    def test_load_configs(self, temp_ros_config_file, temp_ui_config_file):
        """测试加载所有配置文件"""
        config = ConfigLoader.load_configs(temp_ros_config_file, temp_ui_config_file)
        
        assert 'ros' in config
        assert 'ui' in config
        assert 'ros_config_path' in config
        assert config['ros_config_path'] == temp_ros_config_file
        assert config['ros'] == TEST_ROS_CONFIG
        assert config['ui'] == TEST_UI_CONFIG
    
    def test_load_ros_config(self, temp_ros_config_file):
        """测试加载ROS配置文件"""
        config = ConfigLoader.load_ros_config(temp_ros_config_file)
        
        assert config == TEST_ROS_CONFIG
        assert '/crazyflie_server' in config
        assert 'qos' in config['/crazyflie_server']
        assert 'dds' in config['/crazyflie_server']
    
    def test_load_ros_config_nonexistent(self):
        """测试加载不存在的ROS配置文件"""
        with pytest.raises(FileNotFoundError):
            ConfigLoader.load_ros_config('/nonexistent/file.yaml')
    
    def test_load_ui_config(self, temp_ui_config_file):
        """测试加载UI配置文件"""
        config = ConfigLoader.load_ui_config(temp_ui_config_file)
        
        assert 'General' in config
        assert 'Display' in config
        assert config['General']['language'] == 'zh_CN'
        assert config['General']['theme'] == 'dark'
        assert config['Display']['show_grid'] == 'true'
    
    def test_load_ui_config_nonexistent(self):
        """测试加载不存在的UI配置文件"""
        config = ConfigLoader.load_ui_config('/nonexistent/file.ini')
        assert config == {}
    
    def test_load_ui_config_invalid(self, temp_invalid_ui_config_file):
        """测试加载无效的UI配置文件"""
        # 使用pytest.raises来捕获configparser异常
        with pytest.raises(Exception):  # configparser.MissingSectionHeaderError
            ConfigLoader.load_ui_config(temp_invalid_ui_config_file)
    
    @patch.dict(os.environ, {}, clear=True)
    def test_setup_dds_from_config(self, temp_ros_config_file):
        """测试从配置文件设置DDS环境变量"""
        ConfigLoader.setup_dds_from_config(temp_ros_config_file)
        
        assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
        assert os.environ['ROS_DOMAIN_ID'] == '0'
    
    @patch.dict(os.environ, {}, clear=True)
    def test_setup_dds_from_config_nonexistent(self):
        """测试从不存在配置文件设置DDS环境变量"""
        ConfigLoader.setup_dds_from_config('/nonexistent/file.yaml')
        
        # 环境变量不应该被设置
        assert 'RMW_IMPLEMENTATION' not in os.environ
        assert 'ROS_DOMAIN_ID' not in os.environ
    
    @patch.dict(os.environ, {}, clear=True)
    def test_setup_dds_from_config_no_dds_section(self, temp_ros_config_file):
        """测试从没有DDS配置的配置文件设置DDS环境变量"""
        # 创建没有DDS配置的文件
        config_without_dds = {
            '/crazyflie_server': {
                'qos': {
                    'history': 'keep_last',
                    'depth': 10
                }
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_without_dds, f)
            temp_file = f.name
        
        try:
            ConfigLoader.setup_dds_from_config(temp_file)
            
            # 环境变量不应该被设置
            assert 'RMW_IMPLEMENTATION' not in os.environ
            assert 'ROS_DOMAIN_ID' not in os.environ
        finally:
            os.remove(temp_file)
    
    def test_get_qos_profile_from_config_nonexistent(self):
        """测试从不存在配置文件获取QoS配置"""
        qos_profile = ConfigLoader.get_qos_profile_from_config('/nonexistent/file.yaml')
        
        assert qos_profile is None
    
    def test_get_qos_profile_from_config_no_server_section(self, temp_ros_config_file):
        """测试从没有server配置的配置文件获取QoS配置"""
        # 创建没有server配置的文件
        config_without_server = {
            'other_section': {
                'some_key': 'some_value'
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_without_server, f)
            temp_file = f.name
        
        try:
            qos_profile = ConfigLoader.get_qos_profile_from_config(temp_file)
            assert qos_profile is None
        finally:
            os.remove(temp_file)
    
    def test_get_qos_profile_from_config_no_qos_section(self, temp_ros_config_file):
        """测试从没有QoS配置的配置文件获取QoS配置"""
        # 创建没有QoS配置的文件
        config_without_qos = {
            '/crazyflie_server': {
                'dds': {
                    'rmw_implementation': 'rmw_fastrtps_cpp',
                    'domain_id': 0
                }
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_without_qos, f)
            temp_file = f.name
        
        try:
            qos_profile = ConfigLoader.get_qos_profile_from_config(temp_file)
            assert qos_profile is None
        finally:
            os.remove(temp_file)
    
    @patch('builtins.__import__')
    def test_get_qos_profile_from_config_import_error(self, mock_import, temp_ros_config_file):
        """测试QoS配置 - 导入错误"""
        # 模拟导入错误
        mock_import.side_effect = ImportError("ROS 2 modules not available")
        
        qos_profile = ConfigLoader.get_qos_profile_from_config(temp_ros_config_file)
        
        assert qos_profile is None
    
    @patch('builtins.__import__')
    def test_get_qos_profile_from_config_general_error(self, mock_import, temp_ros_config_file):
        """测试QoS配置 - 一般错误"""
        # 模拟导入成功但创建QoS时出错
        def side_effect(name, *args, **kwargs):
            if name == 'rclpy.qos':
                raise Exception("QoS creation failed")
            return Mock()
        
        mock_import.side_effect = side_effect
        
        qos_profile = ConfigLoader.get_qos_profile_from_config(temp_ros_config_file)
        
        assert qos_profile is None

if __name__ == "__main__":
    pytest.main([__file__, "-v"])