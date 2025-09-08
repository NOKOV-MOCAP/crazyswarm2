import pytest
import time
import threading
from unittest.mock import Mock, MagicMock, patch, call
from nokov_swarm.src.core.system_monitor import SystemMonitor, get_system_monitor, set_latency_target, start_system_monitoring, stop_system_monitoring

class TestSystemMonitor:
    """SystemMonitor类的单元测试"""
    
    def test_init(self):
        """测试SystemMonitor初始化"""
        monitor = SystemMonitor()
        
        assert monitor._monitoring is False
        assert monitor._monitor_thread is None
        assert monitor.callbacks == []
        assert monitor.update_interval == 1.0
        assert monitor.latency_target == "8.8.8.8"
        assert monitor.latency_update_interval == 2.0
        assert monitor.last_network_io is None
        assert isinstance(monitor.system_info, dict)
    
    def test_add_callback(self):
        """测试添加回调函数"""
        monitor = SystemMonitor()
        callback1 = Mock()
        callback2 = Mock()
        
        # 添加第一个回调
        monitor.add_callback(callback1)
        assert callback1 in monitor.callbacks
        assert len(monitor.callbacks) == 1
        
        # 添加第二个回调
        monitor.add_callback(callback2)
        assert callback2 in monitor.callbacks
        assert len(monitor.callbacks) == 2
        
        # 重复添加同一个回调
        monitor.add_callback(callback1)
        assert len(monitor.callbacks) == 2  # 不应该重复添加
    
    def test_remove_callback(self):
        """测试移除回调函数"""
        monitor = SystemMonitor()
        callback1 = Mock()
        callback2 = Mock()
        
        # 添加回调
        monitor.add_callback(callback1)
        monitor.add_callback(callback2)
        assert len(monitor.callbacks) == 2
        
        # 移除第一个回调
        monitor.remove_callback(callback1)
        assert callback1 not in monitor.callbacks
        assert callback2 in monitor.callbacks
        assert len(monitor.callbacks) == 1
        
        # 移除不存在的回调
        monitor.remove_callback(Mock())
        assert len(monitor.callbacks) == 1
    
    @patch('psutil.cpu_percent')
    def test_get_cpu_usage(self, mock_cpu_percent):
        """测试获取CPU使用率"""
        monitor = SystemMonitor()
        mock_cpu_percent.return_value = 25.5
        
        result = monitor.get_cpu_usage()
        
        assert result == 25.5
        mock_cpu_percent.assert_called_once_with(interval=0.1)
    
    @patch('psutil.cpu_percent')
    def test_get_cpu_usage_exception(self, mock_cpu_percent):
        """测试获取CPU使用率异常处理"""
        monitor = SystemMonitor()
        mock_cpu_percent.side_effect = Exception("CPU error")
        
        result = monitor.get_cpu_usage()
        
        assert result == 0.0
    
    @patch('psutil.virtual_memory')
    def test_get_memory_usage(self, mock_virtual_memory):
        """测试获取内存使用率"""
        monitor = SystemMonitor()
        mock_memory = Mock()
        mock_memory.percent = 60.5
        mock_virtual_memory.return_value = mock_memory
        
        result = monitor.get_memory_usage()
        
        assert result == 60.5
    
    @patch('psutil.virtual_memory')
    def test_get_memory_usage_exception(self, mock_virtual_memory):
        """测试获取内存使用率异常处理"""
        monitor = SystemMonitor()
        mock_virtual_memory.side_effect = Exception("Memory error")
        
        result = monitor.get_memory_usage()
        
        assert result == 0.0
    
    @patch('psutil.disk_partitions')
    @patch('psutil.disk_usage')
    def test_get_disk_usage(self, mock_disk_usage, mock_disk_partitions):
        """测试获取磁盘使用率"""
        monitor = SystemMonitor()
        
        # 模拟磁盘分区
        mock_partition1 = Mock()
        mock_partition1.device = '/dev/sda1'
        mock_partition1.mountpoint = '/'
        mock_partition2 = Mock()
        mock_partition2.device = '/dev/sda2'
        mock_partition2.mountpoint = '/home'
        mock_disk_partitions.return_value = [mock_partition1, mock_partition2]
        
        # 模拟磁盘使用情况
        mock_usage1 = Mock()
        mock_usage1.total = 1000000000
        mock_usage1.used = 500000000
        mock_usage1.free = 500000000
        mock_usage1.percent = 50.0
        mock_usage2 = Mock()
        mock_usage2.total = 2000000000
        mock_usage2.used = 1000000000
        mock_usage2.free = 1000000000
        mock_usage2.percent = 50.0
        mock_disk_usage.side_effect = [mock_usage1, mock_usage2]
        
        result = monitor.get_disk_usage()
        
        assert len(result) == 2
        assert '/dev/sda1' in result
        assert '/dev/sda2' in result
        assert result['/dev/sda1']['percent'] == 50.0
        assert result['/dev/sda2']['percent'] == 50.0
    
    @patch('psutil.disk_partitions')
    def test_get_disk_usage_exception(self, mock_disk_partitions):
        """测试获取磁盘使用率异常处理"""
        monitor = SystemMonitor()
        mock_disk_partitions.side_effect = Exception("Disk error")
        
        result = monitor.get_disk_usage()
        
        assert result == {}
    
    @patch('psutil.net_io_counters')
    def test_get_network_io_first_call(self, mock_net_io):
        """测试获取网络IO统计（第一次调用）"""
        monitor = SystemMonitor()
        mock_net_io.return_value = Mock(
            bytes_sent=1000,
            bytes_recv=2000,
            packets_sent=10,
            packets_recv=20
        )
        
        result = monitor.get_network_io()
        
        assert result['bytes_sent'] == 0
        assert result['bytes_recv'] == 0
        assert result['packets_sent'] == 0
        assert result['packets_recv'] == 0
        assert result['bytes_sent_rate'] == 0.0
        assert result['bytes_recv_rate'] == 0.0
        assert monitor.last_network_io is not None
    
    @patch('psutil.net_io_counters')
    def test_get_network_io_subsequent_calls(self, mock_net_io):
        """测试获取网络IO统计（后续调用）"""
        monitor = SystemMonitor()
        
        # 第一次调用
        mock_net_io.return_value = Mock(
            bytes_sent=1000,
            bytes_recv=2000,
            packets_sent=10,
            packets_recv=20
        )
        monitor.get_network_io()
        
        # 模拟时间过去1秒
        time.sleep(0.1)
        
        # 第二次调用
        mock_net_io.return_value = Mock(
            bytes_sent=2000,
            bytes_recv=4000,
            packets_sent=20,
            packets_recv=40
        )
        result = monitor.get_network_io()
        
        assert result['bytes_sent'] == 2000
        assert result['bytes_recv'] == 4000
        assert result['packets_sent'] == 20
        assert result['packets_recv'] == 40
        assert result['bytes_sent_rate'] > 0
        assert result['bytes_recv_rate'] > 0
    
    @patch('subprocess.run')
    def test_measure_network_latency_success(self, mock_run):
        """测试测量网络延迟（成功）"""
        monitor = SystemMonitor()
        
        # 模拟ping成功
        mock_result = Mock()
        mock_result.returncode = 0
        mock_result.stdout = "PING 8.8.8.8: time=5.123 ms"
        mock_run.return_value = mock_result
        
        result = monitor.measure_network_latency("8.8.8.8")
        
        assert result['status'] == 'success'
        assert result['latency'] == 5.123
        assert '网络延迟: 5.123ms' in result['message']
    
    @patch('subprocess.run')
    def test_measure_network_latency_failure(self, mock_run):
        """测试测量网络延迟（失败）"""
        monitor = SystemMonitor()
        
        # 模拟ping失败
        mock_result = Mock()
        mock_result.returncode = 1
        mock_result.stdout = "PING failed"
        mock_run.return_value = mock_result
        
        result = monitor.measure_network_latency("8.8.8.8")
        
        assert result['status'] == 'error'
        assert result['latency'] == 0.0
        assert result['message'] == '网络连接失败'
    
    @patch('subprocess.run')
    def test_measure_network_latency_timeout(self, mock_run):
        """测试测量网络延迟（超时）"""
        monitor = SystemMonitor()
        
        # 模拟ping超时
        mock_run.side_effect = Exception("timeout")
        
        result = monitor.measure_network_latency("8.8.8.8")
        
        assert result['status'] == 'error'
        assert result['latency'] == 0.0
        assert '测量失败' in result['message']
    
    def test_set_latency_target(self):
        """测试设置延迟测量目标"""
        monitor = SystemMonitor()
        
        monitor.set_latency_target("1.1.1.1")
        
        assert monitor.latency_target == "1.1.1.1"
        assert monitor._latency_cache["timestamp"] == 0
    
    def test_set_latency_update_interval(self):
        """测试设置延迟更新间隔"""
        monitor = SystemMonitor()
        
        monitor.set_latency_update_interval(5.0)
        
        assert monitor.latency_update_interval == 5.0
    
    def test_get_latency_update_interval(self):
        """测试获取延迟更新间隔"""
        monitor = SystemMonitor()
        
        interval = monitor.get_latency_update_interval()
        
        assert interval == 2.0
    
    def test_is_pa_device(self):
        """测试判断是否是PA设备"""
        monitor = SystemMonitor()
        
        # 测试Nordic Semiconductor设备
        assert monitor._is_pa_device("1915", "7777", "Nordic Semiconductor ASA CrazyRadio PA")
        assert monitor._is_pa_device("1915", "1234", "CrazyRadio PA")
        
        # 测试其他设备
        assert monitor._is_pa_device("1234", "5678", "CrazyRadio PA")
        assert monitor._is_pa_device("1234", "5678", "PA Device")
        assert monitor._is_pa_device("1234", "5678", "Nordic Device")
        
        # 测试非PA设备
        assert not monitor._is_pa_device("1234", "5678", "USB Mouse")
        assert not monitor._is_pa_device("1915", "1234", "Other Device")
    
    @patch('subprocess.run')
    def test_detect_pa_devices(self, mock_run):
        """测试检测PA设备"""
        monitor = SystemMonitor()
        
        # 模拟lsusb输出
        mock_result = Mock()
        mock_result.returncode = 0
        mock_result.stdout = """Bus 001 Device 002: ID 1915:7777 Nordic Semiconductor ASA CrazyRadio PA
Bus 001 Device 003: ID 1234:5678 USB Mouse"""
        mock_run.return_value = mock_result
        
        devices = monitor.detect_pa_devices()
        
        assert len(devices) == 1
        assert devices[0]['vendor_id'] == '1915'
        assert devices[0]['product_id'] == '7777'
        assert devices[0]['description'] == 'Nordic Semiconductor ASA CrazyRadio PA'
        assert devices[0]['status'] == '已连接'
    
    @patch('subprocess.run')
    def test_detect_pa_devices_failure(self, mock_run):
        """测试检测PA设备失败"""
        monitor = SystemMonitor()
        
        # 模拟lsusb失败
        mock_result = Mock()
        mock_result.returncode = 1
        mock_run.return_value = mock_result
        
        devices = monitor.detect_pa_devices()
        
        assert devices == []
    
    @patch('subprocess.run')
    def test_detect_pa_devices_timeout(self, mock_run):
        """测试检测PA设备超时"""
        monitor = SystemMonitor()
        
        # 模拟lsusb超时
        mock_run.side_effect = Exception("timeout")
        
        devices = monitor.detect_pa_devices()
        
        assert devices == []
    
    def test_refresh_pa_devices(self):
        """测试刷新PA设备检测"""
        monitor = SystemMonitor()
        
        # 设置缓存
        monitor._pa_devices_cache["timestamp"] = time.time()
        monitor._pa_devices_cache["devices"] = [{"test": "device"}]
        
        # 刷新应该清除缓存
        with patch.object(monitor, 'detect_pa_devices') as mock_detect:
            mock_detect.return_value = []
            result = monitor.refresh_pa_devices()
            
            assert result == []
            mock_detect.assert_called_once()
    
    def test_get_pa_device_count(self):
        """测试获取PA设备数量"""
        monitor = SystemMonitor()
        
        # 设置缓存
        monitor._pa_devices_cache["timestamp"] = time.time()
        monitor._pa_devices_cache["devices"] = [{"test": "device1"}, {"test": "device2"}]
        
        count = monitor.get_pa_device_count()
        
        assert count == 2
    
    @patch('psutil.cpu_percent')
    @patch('psutil.virtual_memory')
    @patch('psutil.disk_partitions')
    @patch('psutil.disk_usage')
    @patch('psutil.net_io_counters')
    @patch('psutil.pids')
    def test_get_system_load(self, mock_pids, mock_net_io, mock_disk_usage, mock_disk_partitions, mock_virtual_memory, mock_cpu_percent):
        """测试获取系统负载信息"""
        monitor = SystemMonitor()
        
        # 模拟各种系统信息
        mock_cpu_percent.return_value = 25.5
        mock_memory = Mock()
        mock_memory.percent = 60.5
        mock_virtual_memory.return_value = mock_memory
        
        mock_partition = Mock()
        mock_partition.device = '/dev/sda1'
        mock_partition.mountpoint = '/'
        mock_disk_partitions.return_value = [mock_partition]
        
        mock_usage = Mock()
        mock_usage.total = 1000000000
        mock_usage.used = 500000000
        mock_usage.free = 500000000
        mock_usage.percent = 50.0
        mock_disk_usage.return_value = mock_usage
        
        mock_net_io.return_value = Mock(
            bytes_sent=1000,
            bytes_recv=2000,
            packets_sent=10,
            packets_recv=20
        )
        
        mock_pids.return_value = [1, 2, 3, 4, 5]
        
        # 模拟网络延迟
        with patch.object(monitor, '_get_network_latency') as mock_latency:
            mock_latency.return_value = {
                "latency": 5.0,
                "status": "success",
                "message": "网络延迟: 5.000ms"
            }
            
            result = monitor.get_system_load()
            
            assert 'timestamp' in result
            assert result['cpu_usage'] == 25.5
            assert result['memory_usage'] == 60.5
            assert 'disk_usage' in result
            assert 'network_io' in result
            assert result['network_latency'] == 5.0
            assert result['network_latency_status'] == 'success'
            assert result['process_count'] == 5
            assert 'system_info' in result
    
    def test_get_network_latency_cached(self):
        """测试获取网络延迟（使用缓存）"""
        monitor = SystemMonitor()
        
        # 设置缓存
        monitor._latency_cache = {
            "value": 10.0,
            "status": "success",
            "message": "网络延迟: 10.000ms",
            "timestamp": time.time(),
            "cache_duration": 2
        }
        
        result = monitor._get_network_latency()
        
        assert result['latency'] == 10.0
        assert result['status'] == 'success'
        assert result['message'] == '网络延迟: 10.000ms'
    
    def test_get_network_latency_expired(self):
        """测试获取网络延迟（缓存过期）"""
        monitor = SystemMonitor()
        
        # 设置过期缓存
        monitor._latency_cache = {
            "value": 10.0,
            "status": "success",
            "message": "网络延迟: 10.000ms",
            "timestamp": time.time() - 10,  # 10秒前
            "cache_duration": 2
        }
        
        with patch.object(monitor, 'measure_network_latency') as mock_measure:
            mock_measure.return_value = {
                "latency": 15.0,
                "status": "success",
                "message": "网络延迟: 15.000ms"
            }
            
            result = monitor._get_network_latency()
            
            assert result['latency'] == 15.0
            assert result['status'] == 'success'
            mock_measure.assert_called_once_with(monitor.latency_target)
    
    def test_start_monitoring(self):
        """测试开始监控"""
        monitor = SystemMonitor()
        
        monitor.start_monitoring()
        
        assert monitor._monitoring is True
        assert monitor._monitor_thread is not None
        assert monitor._monitor_thread.is_alive()
    
    def test_stop_monitoring(self):
        """测试停止监控"""
        monitor = SystemMonitor()
        
        # 先启动监控
        monitor.start_monitoring()
        assert monitor._monitoring is True
        
        # 停止监控
        monitor.stop_monitoring()
        
        assert monitor._monitoring is False
    
    def test_get_summary(self):
        """测试获取系统摘要"""
        monitor = SystemMonitor()
        
        with patch.object(monitor, 'get_system_load') as mock_load:
            mock_load.return_value = {
                'cpu_usage': 25.5,
                'memory_usage': 60.5,
                'disk_usage': {'/dev/sda1': {'percent': 50.0}},
                'network_latency': 5.0,
                'network_latency_status': 'success',
                'network_latency_message': '网络延迟: 5.000ms',
                'process_count': 100
            }
            
            result = monitor.get_summary()
            
            assert result['cpu_usage'] == '25.5%'
            assert result['memory_usage'] == '60.5%'
            assert result['disk_usage'] == '50.0%'
            assert result['network_latency'] == '5ms'
            assert result['process_count'] == 100
    
    def test_get_summary_exception(self):
        """测试获取系统摘要异常处理"""
        monitor = SystemMonitor()
        
        with patch.object(monitor, 'get_system_load') as mock_load:
            mock_load.side_effect = Exception("System error")
            
            result = monitor.get_summary()
            
            assert result['cpu_usage'] == 'N/A'
            assert result['memory_usage'] == 'N/A'
            assert result['disk_usage'] == 'N/A'
            assert result['network_latency'] == 'N/A'
            assert result['process_count'] == 0


class TestSystemMonitorGlobalFunctions:
    """SystemMonitor全局函数的单元测试"""
    
    def test_get_system_monitor(self):
        """测试获取全局系统监控器实例"""
        monitor1 = get_system_monitor()
        monitor2 = get_system_monitor()
        
        assert monitor1 is monitor2  # 应该是同一个实例
    
    def test_set_latency_target(self):
        """测试设置全局延迟目标"""
        monitor = get_system_monitor()
        original_target = monitor.latency_target
        
        set_latency_target("1.1.1.1")
        
        assert monitor.latency_target == "1.1.1.1"
        
        # 恢复原始值
        monitor.latency_target = original_target
    
    def test_start_system_monitoring(self):
        """测试启动全局系统监控"""
        monitor = get_system_monitor()
        original_monitoring = monitor._monitoring
        
        start_system_monitoring()
        
        assert monitor._monitoring is True
        
        # 停止监控
        monitor.stop_monitoring()
        monitor._monitoring = original_monitoring
    
    def test_stop_system_monitoring(self):
        """测试停止全局系统监控"""
        monitor = get_system_monitor()
        
        # 先启动监控
        monitor.start_monitoring()
        
        stop_system_monitoring()
        
        assert monitor._monitoring is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])