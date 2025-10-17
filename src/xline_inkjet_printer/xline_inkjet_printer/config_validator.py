"""
配置验证工具

提供配置参数的验证和规范化功能。
"""

import ipaddress
from typing import Dict, Any, Tuple, Optional
import re


class ValidationError(Exception):
    """配置验证错误"""
    pass


class ConfigValidator:
    """配置验证器"""

    # 参数范围定义
    PORT_MIN = 1
    PORT_MAX = 65535
    TIMEOUT_MIN = 0.1
    TIMEOUT_MAX = 300.0
    RECONNECT_INTERVAL_MIN = 0.1
    RECONNECT_INTERVAL_MAX = 3600.0
    RECONNECT_TIMES_MIN = 0
    RECONNECT_TIMES_MAX = 1000

    @staticmethod
    def validate_ip(ip: str) -> Tuple[bool, str]:
        """
        验证IP地址

        Args:
            ip: IP地址字符串

        Returns:
            (是否有效, 错误信息/规范化后的IP)

        Examples:
            >>> ConfigValidator.validate_ip('192.168.1.1')
            (True, '192.168.1.1')
            >>> ConfigValidator.validate_ip('invalid')
            (False, 'Invalid IP address format')
        """
        if not ip or not isinstance(ip, str):
            return False, "IP address must be a non-empty string"

        try:
            # 使用 ipaddress 模块验证
            ip_obj = ipaddress.ip_address(ip.strip())
            return True, str(ip_obj)
        except ValueError as e:
            return False, f"Invalid IP address: {str(e)}"

    @staticmethod
    def validate_hostname(hostname: str) -> Tuple[bool, str]:
        """
        验证主机名

        Args:
            hostname: 主机名字符串

        Returns:
            (是否有效, 错误信息/规范化后的主机名)
        """
        if not hostname or not isinstance(hostname, str):
            return False, "Hostname must be a non-empty string"

        hostname = hostname.strip()

        # 主机名规则：字母、数字、点、连字符，长度1-253
        if len(hostname) > 253:
            return False, "Hostname too long (max 253 characters)"

        # 允许的字符：字母、数字、点、连字符
        pattern = re.compile(r'^[a-zA-Z0-9.-]+$')
        if not pattern.match(hostname):
            return False, "Hostname contains invalid characters"

        # 不能以点或连字符开头/结尾
        if hostname.startswith('.') or hostname.startswith('-'):
            return False, "Hostname cannot start with '.' or '-'"
        if hostname.endswith('.') or hostname.endswith('-'):
            return False, "Hostname cannot end with '.' or '-'"

        return True, hostname

    @staticmethod
    def validate_host(host: str) -> Tuple[bool, str]:
        """
        验证主机（IP地址或主机名）

        Args:
            host: 主机字符串（IP或主机名）

        Returns:
            (是否有效, 错误信息/规范化后的主机)
        """
        # 先尝试作为IP验证
        is_valid_ip, result_ip = ConfigValidator.validate_ip(host)
        if is_valid_ip:
            return True, result_ip

        # 再尝试作为主机名验证
        is_valid_hostname, result_hostname = ConfigValidator.validate_hostname(host)
        if is_valid_hostname:
            return True, result_hostname

        return False, f"Invalid host (neither valid IP nor hostname): {result_ip}"

    @staticmethod
    def validate_port(port: Any) -> Tuple[bool, str]:
        """
        验证端口号

        Args:
            port: 端口号（int或可转换为int的值）

        Returns:
            (是否有效, 错误信息/规范化后的端口字符串)
        """
        try:
            port_int = int(port)
            if ConfigValidator.PORT_MIN <= port_int <= ConfigValidator.PORT_MAX:
                return True, str(port_int)
            else:
                return False, f"Port must be between {ConfigValidator.PORT_MIN} and {ConfigValidator.PORT_MAX}"
        except (ValueError, TypeError):
            return False, f"Port must be an integer, got: {type(port).__name__}"

    @staticmethod
    def validate_timeout(timeout: Any) -> Tuple[bool, str]:
        """
        验证超时时间

        Args:
            timeout: 超时时间（秒）

        Returns:
            (是否有效, 错误信息/规范化后的超时字符串)
        """
        try:
            timeout_float = float(timeout)
            if ConfigValidator.TIMEOUT_MIN <= timeout_float <= ConfigValidator.TIMEOUT_MAX:
                return True, str(timeout_float)
            else:
                return False, f"Timeout must be between {ConfigValidator.TIMEOUT_MIN} and {ConfigValidator.TIMEOUT_MAX} seconds"
        except (ValueError, TypeError):
            return False, f"Timeout must be a number, got: {type(timeout).__name__}"

    @staticmethod
    def validate_reconnect_times(times: Any) -> Tuple[bool, str]:
        """
        验证重连次数

        Args:
            times: 重连次数（0表示无限重连）

        Returns:
            (是否有效, 错误信息/规范化后的次数字符串)
        """
        try:
            times_int = int(times)
            if ConfigValidator.RECONNECT_TIMES_MIN <= times_int <= ConfigValidator.RECONNECT_TIMES_MAX:
                return True, str(times_int)
            else:
                return False, f"Reconnect times must be between {ConfigValidator.RECONNECT_TIMES_MIN} and {ConfigValidator.RECONNECT_TIMES_MAX}"
        except (ValueError, TypeError):
            return False, f"Reconnect times must be an integer, got: {type(times).__name__}"

    @staticmethod
    def validate_reconnect_interval(interval: Any) -> Tuple[bool, str]:
        """
        验证重连间隔

        Args:
            interval: 重连间隔（秒）

        Returns:
            (是否有效, 错误信息/规范化后的间隔字符串)
        """
        try:
            interval_float = float(interval)
            if ConfigValidator.RECONNECT_INTERVAL_MIN <= interval_float <= ConfigValidator.RECONNECT_INTERVAL_MAX:
                return True, str(interval_float)
            else:
                return False, f"Reconnect interval must be between {ConfigValidator.RECONNECT_INTERVAL_MIN} and {ConfigValidator.RECONNECT_INTERVAL_MAX} seconds"
        except (ValueError, TypeError):
            return False, f"Reconnect interval must be a number, got: {type(interval).__name__}"

    @staticmethod
    def validate_auto_connect(auto_connect: Any) -> Tuple[bool, str]:
        """
        验证自动连接标志

        Args:
            auto_connect: 自动连接标志（bool或可转换的值）

        Returns:
            (是否有效, 错误信息/规范化后的布尔字符串)
        """
        if isinstance(auto_connect, bool):
            return True, str(auto_connect)

        # 尝试转换常见的布尔值表示
        if isinstance(auto_connect, str):
            auto_connect_lower = auto_connect.lower().strip()
            if auto_connect_lower in ('true', 'yes', '1', 'on'):
                return True, 'True'
            elif auto_connect_lower in ('false', 'no', '0', 'off'):
                return True, 'False'

        if isinstance(auto_connect, int):
            return True, str(bool(auto_connect))

        return False, f"auto_connect must be a boolean value, got: {type(auto_connect).__name__}"

    @classmethod
    def validate_connection_config(cls, config: Dict[str, Any], section_name: str = "unknown") -> Dict[str, Any]:
        """
        验证完整的连接配置

        Args:
            config: 配置字典
            section_name: 配置节名称（用于错误信息）

        Returns:
            验证并规范化后的配置字典

        Raises:
            ValidationError: 如果配置无效
        """
        validated = {}
        errors = []

        # 验证 IP/主机
        if 'ip' in config:
            is_valid, result = cls.validate_host(config['ip'])
            if is_valid:
                validated['ip'] = result
            else:
                errors.append(f"[{section_name}] IP: {result}")
        else:
            errors.append(f"[{section_name}] Missing required field: ip")

        # 验证端口
        if 'port' in config:
            is_valid, result = cls.validate_port(config['port'])
            if is_valid:
                validated['port'] = int(result)
            else:
                errors.append(f"[{section_name}] Port: {result}")
        else:
            errors.append(f"[{section_name}] Missing required field: port")

        # 验证超时（可选）
        if 'timeout' in config:
            is_valid, result = cls.validate_timeout(config['timeout'])
            if is_valid:
                validated['timeout'] = float(result)
            else:
                errors.append(f"[{section_name}] Timeout: {result}")

        # 验证重连次数（可选）
        if 'reconnect_times' in config:
            is_valid, result = cls.validate_reconnect_times(config['reconnect_times'])
            if is_valid:
                validated['reconnect_times'] = int(result)
            else:
                errors.append(f"[{section_name}] Reconnect times: {result}")

        # 验证重连间隔（可选）
        if 'reconnect_interval' in config:
            is_valid, result = cls.validate_reconnect_interval(config['reconnect_interval'])
            if is_valid:
                validated['reconnect_interval'] = float(result)
            else:
                errors.append(f"[{section_name}] Reconnect interval: {result}")

        # 验证自动连接标志（可选，支持向后兼容）
        if 'auto_connect' in config:
            is_valid, result = cls.validate_auto_connect(config['auto_connect'])
            if is_valid:
                validated['auto_connect'] = result.lower() == 'true'
            else:
                errors.append(f"[{section_name}] auto_connect: {result}")
        elif 'enabled' in config:
            # 向后兼容：支持旧的 'enabled' 字段
            is_valid, result = cls.validate_auto_connect(config['enabled'])
            if is_valid:
                validated['auto_connect'] = result.lower() == 'true'
            else:
                errors.append(f"[{section_name}] enabled (deprecated, use auto_connect): {result}")

        # 如果有错误，抛出异常
        if errors:
            raise ValidationError("\n".join(errors))

        return validated

    @classmethod
    def validate_yaml_config(cls, data: Dict[str, Any]) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
        """
        验证完整的YAML配置文件

        Args:
            data: YAML数据字典

        Returns:
            (是否有效, 错误信息/成功信息, 验证后的配置)
        """
        try:
            validated_config = {
                'global_defaults': {},
                'connections': {}
            }

            # 验证全局默认值
            if 'global_defaults' in data:
                defaults = data['global_defaults']
                if 'timeout' in defaults:
                    is_valid, result = cls.validate_timeout(defaults['timeout'])
                    if not is_valid:
                        return False, f"Global defaults - Timeout: {result}", None
                    validated_config['global_defaults']['timeout'] = float(result)

                if 'reconnect_times' in defaults:
                    is_valid, result = cls.validate_reconnect_times(defaults['reconnect_times'])
                    if not is_valid:
                        return False, f"Global defaults - Reconnect times: {result}", None
                    validated_config['global_defaults']['reconnect_times'] = int(result)

                if 'reconnect_interval' in defaults:
                    is_valid, result = cls.validate_reconnect_interval(defaults['reconnect_interval'])
                    if not is_valid:
                        return False, f"Global defaults - Reconnect interval: {result}", None
                    validated_config['global_defaults']['reconnect_interval'] = float(result)

            # 验证各个连接
            if 'connections' in data:
                connections = data['connections']
                if not isinstance(connections, dict):
                    return False, "Connections must be a dictionary", None

                for conn_name, conn_config in connections.items():
                    try:
                        validated_conn = cls.validate_connection_config(conn_config, conn_name)
                        validated_config['connections'][conn_name] = validated_conn
                    except ValidationError as e:
                        return False, str(e), None

            return True, "Configuration validated successfully", validated_config

        except Exception as e:
            return False, f"Validation error: {str(e)}", None
