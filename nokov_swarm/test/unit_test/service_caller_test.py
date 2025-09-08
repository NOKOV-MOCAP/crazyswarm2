import pytest
import numpy as np
import array
from unittest.mock import Mock, MagicMock, patch, call
from nokov_swarm.src.ros_intergration.service_caller import ServiceCaller, arrayToGeometryPoint
from geometry_msgs.msg import Point
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
from std_srvs.srv import Empty
from builtin_interfaces.msg import Time

class TestServiceCaller:
    """ServiceCaller类的单元测试"""
    
    @pytest.fixture
    def mock_node(self):
        """创建模拟的ROS节点"""
        node = Mock()
        node.create_client = Mock()
        node.create_publisher = Mock()
        node.create_timer = Mock()
        node.get_clock = Mock()
        node.get_logger = Mock()
        node.destroy_client = Mock()
        
        # 模拟时钟
        clock = Mock()
        now = Mock()
        # 修复：返回真实的Time对象而不是Mock
        time_msg = Time()
        time_msg.sec = 1234567890
        time_msg.nanosec = 123456789
        now.to_msg.return_value = time_msg
        clock.now.return_value = now
        node.get_clock.return_value = clock
        
        # 模拟日志器
        logger = Mock()
        node.get_logger.return_value = logger
        
        return node
    
    @pytest.fixture
    def mock_services(self, mock_node):
        """创建模拟的服务客户端"""
        services = {}
        service_types = [
            'Empty', 'Takeoff', 'Land', 'GoTo', 'UploadTrajectory', 
            'StartTrajectory', 'SetParameters', 'GetParameters'
        ]
        
        for service_type in service_types:
            service = Mock()
            service.wait_for_service = Mock()
            services[service_type] = service
        
        # 模拟create_client方法
        def create_client_mock(service_type, service_name):
            if 'emergency' in service_name:
                return services['Empty']
            elif 'takeoff' in service_name:
                return services['Takeoff']
            elif 'land' in service_name:
                return services['Land']
            elif 'go_to' in service_name:
                return services['GoTo']
            elif 'upload_trajectory' in service_name:
                return services['UploadTrajectory']
            elif 'start_trajectory' in service_name:
                return services['StartTrajectory']
            elif 'set_parameters' in service_name:
                return services['SetParameters']
            elif 'get_parameters' in service_name:
                return services['GetParameters']
            return Mock()
        
        mock_node.create_client.side_effect = create_client_mock
        return services
    
    @pytest.fixture
    def service_caller(self, mock_node, mock_services):
        """创建ServiceCaller实例"""
        param_type_dict = {
            'hlCommander.groupmask': ParameterType.PARAMETER_INTEGER,
            'test.param': ParameterType.PARAMETER_DOUBLE
        }
        
        # 模拟get_parameters响应
        mock_response = Mock()
        mock_response.values = [
            Mock(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[1.0, 2.0, 3.0]),
            Mock(string_value="radio://0/80/2M/E7E7E7E703")
        ]
        
        def get_params_callback(future):
            future.result = Mock(return_value=mock_response)
        
        # 模拟异步调用
        with patch('rclpy.callback_groups.MutuallyExclusiveCallbackGroup'):
            caller = ServiceCaller(mock_node, "cf1", param_type_dict)
            # 手动调用回调
            get_params_callback(Mock())
            caller._handle_get_params_response(Mock())
        
        return caller
    
    def test_array_to_geometry_point(self):
        """测试arrayToGeometryPoint函数"""
        array = [1.0, 2.0, 3.0]
        point = arrayToGeometryPoint(array)
        
        assert isinstance(point, Point)
        assert point.x == 1.0
        assert point.y == 2.0
        assert point.z == 3.0
    
    def test_init(self, mock_node, mock_services):
        """测试ServiceCaller初始化"""
        param_type_dict = {'test.param': ParameterType.PARAMETER_INTEGER}
        
        with patch('rclpy.callback_groups.MutuallyExclusiveCallbackGroup'):
            caller = ServiceCaller(mock_node, "cf1", param_type_dict)
        
        assert caller.node == mock_node
        assert caller.cfname == "cf1"
        assert caller.prefix == "/cf1"
        assert caller.paramTypeDict == param_type_dict
        assert caller.is_active is False
        assert caller._vel_islanding is False
        assert caller._isair is False
        assert caller.publish_frequency == -1.0
    
    def test_emergency(self, service_caller):
        """测试紧急停止服务"""
        service_caller.emergency()
        
        # 验证调用了emergency服务
        service_caller.emergencyService.call_async.assert_called_once()
        call_args = service_caller.emergencyService.call_async.call_args[0][0]
        assert isinstance(call_args, Empty.Request)
    
    def test_takeoff(self, service_caller):
        """测试起飞服务"""
        service_caller.takeoff(1.0, 2.0, 1)
        
        # 验证调用了takeoff服务
        service_caller.takeoffService.call_async.assert_called_once()
        call_args = service_caller.takeoffService.call_async.call_args[0][0]
        assert call_args.group_mask == 1
        assert call_args.height == 1.0
        assert call_args.duration.sec == 2
        assert call_args.duration.nanosec == 0
    
    def test_land(self, service_caller):
        """测试降落服务"""
        service_caller.land(0.5, 3.0, 2)
        
        # 验证调用了land服务
        service_caller.landService.call_async.assert_called_once()
        call_args = service_caller.landService.call_async.call_args[0][0]
        assert call_args.group_mask == 2
        assert call_args.height == 0.5
        assert call_args.duration.sec == 3
        assert call_args.duration.nanosec == 0
    
    def test_go_to(self, service_caller):
        """测试GoTo服务"""
        goal = [1.0, 2.0, 3.0]
        yaw = 1.57
        duration = 5.0
        
        service_caller.goTo(goal, yaw, duration, relative=True, groupMask=3)
        
        # 验证调用了goTo服务
        service_caller.goToService.call_async.assert_called_once()
        call_args = service_caller.goToService.call_async.call_args[0][0]
        assert call_args.group_mask == 3
        assert call_args.relative is True
        assert call_args.goal.x == 1.0
        assert call_args.goal.y == 2.0
        assert call_args.goal.z == 3.0
        assert call_args.yaw == 1.57
        assert call_args.duration.sec == 5
        assert call_args.duration.nanosec == 0
    
    def test_set_group_mask(self, service_caller):
        """测试设置组掩码"""
        with patch.object(service_caller, 'setParam') as mock_set_param:
            service_caller.setGroupMask(5)
            mock_set_param.assert_called_once_with('hlCommander.groupmask', 5)
    
    def test_set_group_mask_key_error(self, service_caller):
        """测试设置组掩码时参数不存在"""
        with patch.object(service_caller, 'setParam') as mock_set_param:
            mock_set_param.side_effect = KeyError("Parameter not found")
            
            # 应该不会抛出异常，只是记录错误
            service_caller.setGroupMask(5)
            mock_set_param.assert_called_once_with('hlCommander.groupmask', 5)
    
    def test_set_param_integer(self, service_caller):
        """测试设置整数参数"""
        service_caller.setParam('hlCommander.groupmask', 10)
        
        # 验证调用了set_parameters服务
        service_caller.setParamsService.call_async.assert_called_once()
        call_args = service_caller.setParamsService.call_async.call_args[0][0]
        assert len(call_args.parameters) == 1
        
        param = call_args.parameters[0]
        assert param.name == 'cf1.params.hlCommander.groupmask'
        assert param.value.type == ParameterType.PARAMETER_INTEGER
        assert param.value.integer_value == 10
    
    def test_set_param_double(self, service_caller):
        """测试设置浮点数参数"""
        service_caller.setParam('test.param', 3.14)
        
        # 验证调用了set_parameters服务
        service_caller.setParamsService.call_async.assert_called_once()
        call_args = service_caller.setParamsService.call_async.call_args[0][0]
        assert len(call_args.parameters) == 1
        
        param = call_args.parameters[0]
        assert param.name == 'cf1.params.test.param'
        assert param.value.type == ParameterType.PARAMETER_DOUBLE
        assert param.value.double_value == 3.14
    
    def test_set_param_key_error(self, service_caller):
        """测试设置不存在的参数"""
        with patch.object(service_caller.node.get_logger(), 'warn') as mock_warn:
            service_caller.setParam('nonexistent.param', 123)
            mock_warn.assert_called()
    
    def test_set_param_exception(self, service_caller):
        """测试设置参数时发生异常"""
        with patch.object(service_caller.setParamsService, 'call_async') as mock_call:
            mock_call.side_effect = Exception("Service error")
            
            with patch.object(service_caller.node.get_logger(), 'warn') as mock_warn:
                service_caller.setParam('test.param', 123)
                mock_warn.assert_called()
    
    def test_upload_trajectory(self, service_caller):
        """测试上传轨迹"""
        # 创建模拟轨迹
        trajectory = Mock()
        trajectory.polynomials = []
        
        # 添加多项式
        poly = Mock()
        poly.duration = 2.0
        # 修复：返回浮点数列表而不是整数列表
        poly.px = Mock(p=Mock(tolist=Mock(return_value=[1.0, 2.0, 3.0])))
        poly.py = Mock(p=Mock(tolist=Mock(return_value=[4.0, 5.0, 6.0])))
        poly.pz = Mock(p=Mock(tolist=Mock(return_value=[7.0, 8.0, 9.0])))
        poly.pyaw = Mock(p=Mock(tolist=Mock(return_value=[10.0, 11.0, 12.0])))
        trajectory.polynomials = [poly]
        
        callback = Mock()
        service_caller.uploadTrajectory(1, 0, trajectory, callback)
        
        # 验证调用了upload_trajectory服务
        service_caller.uploadTrajectoryService.call_async.assert_called_once()
        call_args = service_caller.uploadTrajectoryService.call_async.call_args[0][0]
        assert call_args.trajectory_id == 1
        assert call_args.piece_offset == 0
        assert len(call_args.pieces) == 1
        
        piece = call_args.pieces[0]
        assert piece.duration.sec == 2
        assert piece.duration.nanosec == 0
        # 修复：比较array.array对象
        assert list(piece.poly_x) == [1.0, 2.0, 3.0]
        assert list(piece.poly_y) == [4.0, 5.0, 6.0]
        assert list(piece.poly_z) == [7.0, 8.0, 9.0]
        assert list(piece.poly_yaw) == [10.0, 11.0, 12.0]
    
    def test_upload_trajectory_callback_success(self, service_caller):
        """测试上传轨迹回调（成功）"""
        callback = Mock()
        future = Mock()
        future.result = Mock(return_value=Mock())
        
        service_caller._upload_trajectory_callback(future, 1, callback)
        
        callback.assert_called_once_with(True, "Trajectory 1 uploaded successfully")
    
    def test_upload_trajectory_callback_failure(self, service_caller):
        """测试上传轨迹回调（失败）"""
        callback = Mock()
        future = Mock()
        future.result = Mock(side_effect=Exception("Upload failed"))
        
        service_caller._upload_trajectory_callback(future, 1, callback)
        
        callback.assert_called_once_with(False, "Failed to upload trajectory 1: Upload failed")
    
    def test_start_trajectory(self, service_caller):
        """测试开始轨迹"""
        service_caller.startTrajectory(1, timescale=2.0, reverse=True, relative=False, groupMask=4)
        
        # 验证调用了start_trajectory服务
        service_caller.startTrajectoryService.call_async.assert_called_once()
        call_args = service_caller.startTrajectoryService.call_async.call_args[0][0]
        assert call_args.group_mask == 4
        assert call_args.trajectory_id == 1
        assert call_args.timescale == 2.0
        assert call_args.reversed is True
        assert call_args.relative is False
    
    def test_cmd_velocity_world(self, service_caller):
        """测试速度命令"""
        vel = [1.0, 2.0, 3.0]
        yaw_rate = 0.5
        frequency = 10.0
        
        with patch.object(service_caller, 'set_frequency') as mock_set_freq:
            with patch.object(service_caller, 'set_velocity') as mock_set_vel:
                service_caller.cmdVelocityWorld(vel, yaw_rate, frequency)
                
                mock_set_freq.assert_called_once_with(frequency)
                mock_set_vel.assert_called_once_with(vel, yaw_rate)
    
    def test_set_pose(self, service_caller):
        """测试设置姿态"""
        pose = Mock()
        pose.position = Mock(x=0.5, y=0.5, z=0.1)  # 修复：z=0.1 不会触发空中状态
        
        service_caller.setpose(pose)
        
        assert service_caller.cfpose == pose
        assert service_caller._vel_islanding is False
        assert service_caller._isair is False  # z=0.1 < 0.2，不会在空中
    
    def test_set_pose_out_of_bounds(self, service_caller):
        """测试设置姿态（超出边界）"""
        pose = Mock()
        pose.position = Mock(x=2.0, y=2.0, z=0.5)  # 超出边界
        
        service_caller.setpose(pose)
        
        assert service_caller.cfpose == pose
        assert service_caller._vel_islanding is True
    
    def test_set_pose_in_air(self, service_caller):
        """测试设置姿态（在空中）"""
        pose = Mock()
        pose.position = Mock(x=0.5, y=0.5, z=0.3)  # 在空中
        
        service_caller.setpose(pose)
        
        assert service_caller.cfpose == pose
        assert service_caller._isair is True
    
    def test_set_pose_landing(self, service_caller):
        """测试设置姿态（降落）"""
        pose = Mock()
        pose.position = Mock(x=0.5, y=0.5, z=0.05)  # 接近地面
        
        service_caller._isair = True
        service_caller.is_active = True
        
        with patch.object(service_caller, 'stop') as mock_stop:
            service_caller.setpose(pose)
            
            assert service_caller._vel_islanding is False
            assert service_caller._isair is False
            mock_stop.assert_called_once()
    
    def test_set_frequency(self, service_caller):
        """测试设置频率"""
        with patch.object(service_caller, 'update_publish_timer') as mock_update:
            service_caller.set_frequency(20.0)
            
            assert service_caller.publish_frequency == 20.0
            assert service_caller.is_active is True
            mock_update.assert_called_once()
    
    def test_set_frequency_zero(self, service_caller):
        """测试设置频率为0"""
        service_caller.set_frequency(0.0)
        
        assert service_caller._vel_islanding is True
    
    def test_set_velocity(self, service_caller):
        """测试设置速度"""
        vel = [1.0, 2.0, 3.0]
        yaw_rate = 0.5
        
        service_caller.set_velocity(vel, yaw_rate)
        
        assert service_caller.current_vel == vel
        assert service_caller.current_yaw_rate == yaw_rate
    
    def test_stop(self, service_caller):
        """测试停止"""
        service_caller.is_active = True
        service_caller.publish_timer = Mock()
        
        with patch.object(service_caller, 'publish_velocity') as mock_publish:
            with patch.object(service_caller, 'emergency') as mock_emergency:
                service_caller.stop()
                
                assert service_caller.is_active is False
                assert service_caller.publish_timer is None
                mock_publish.assert_called_once_with([0.0, 0.0, 0.0], 0.0)
                mock_emergency.assert_called_once()
    
    def test_publish_velocity_callback(self, service_caller):
        """测试发布速度回调"""
        service_caller.current_vel = [1.0, 2.0, 3.0]
        service_caller.current_yaw_rate = 0.5
        
        with patch.object(service_caller, 'publish_velocity') as mock_publish:
            service_caller.publish_velocity_callback()
            
            mock_publish.assert_called_once_with([1.0, 2.0, 3.0], 0.5)
    
    def test_publish_velocity(self, service_caller):
        """测试发布速度"""
        vel = [1.0, 2.0, 3.0]
        yaw_rate = 0.5
        
        service_caller.publish_velocity(vel, yaw_rate)
        
        # 验证消息内容
        msg = service_caller.cmdVelocityWorldMsg
        assert msg.vel.x == 1.0
        assert msg.vel.y == 2.0
        assert msg.vel.z == 3.0
        assert msg.yaw_rate == 0.5
        
        # 验证发布了消息
        service_caller.cmdVelocityWorldPublisher.publish.assert_called_once_with(msg)
    
    def test_publish_velocity_landing(self, service_caller):
        """测试发布速度（降落模式）"""
        service_caller._vel_islanding = True
        
        service_caller.publish_velocity([1.0, 2.0, 3.0], 0.5)
        
        # 验证消息内容（应该被覆盖为降落模式）
        msg = service_caller.cmdVelocityWorldMsg
        assert msg.vel.x == 0.0
        assert msg.vel.y == 0.0
        assert msg.vel.z == -0.1
        assert msg.yaw_rate == 0.0
    
    def test_destroy(self, service_caller):
        """测试销毁服务客户端"""
        service_caller.destroy()
        
        # 验证所有服务客户端都被销毁
        assert service_caller.emergencyService is None
        assert service_caller.takeoffService is None
        assert service_caller.landService is None
        assert service_caller.goToService is None
        assert service_caller.setParamsService is None
        assert service_caller.getParamsService is None
        
        # 验证destroy_client被调用
        assert service_caller.node.destroy_client.call_count >= 5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])