# rclcpp/C++ Convention

## 1. Common

- 포인터 선언 시 `*<변수명>` 스타일로 통일한다.
예시:
before)
SerialPort * serial_port

after)
SerialPort *serial_port

- reference 선언 시 `&<변수명>` 스타일로 통일한다.
예시:
const std::string &port

- const reference 적극 활용.
복사 불필요한 객체는 최대한 `const &<변수명>` 사용.

- auto 사용 최대 지양.
명시 가능한 타입은 전부 타입 명시.
예외:
  - iterator 타입이 지나치게 긴 경우
  - lambda 반환 타입 추론이 자연스러운 경우

- 함수 선언/호출 시 불필요한 개행 최대 지양.
가급적 한 줄 유지.

좋은 예시:
Pose2D refine_pose_with_scan_matching(const sensor_msgs::msg::LaserScan &scan, const Pose2D &predicted_pose) const;

나쁜 예시:
Pose2D refine_pose_with_scan_matching(
    const sensor_msgs::msg::LaserScan &scan,
    const Pose2D &predicted_pose) const;

- BSD style brace/indent 사용.
모든 if/for/while/function/class/namespace에서 개행 후 `{` 시작.

예시:

namespace robot::hw::lidar
{

class LidarDriverNode
{
private:
protected:
public:
};

}

예시:

if (is_connected_)
{

}

for (std::size_t i = 0; i < size; ++i)
{

}

- tab 기반 indent 사용.
space indent 사용 지양.

- ternary operator 과도 사용 금지.
가독성 우선.

- magic number 사용 금지.
constexpr 또는 static constexpr 사용.

- enum 대신 enum class 우선 사용.

- nullptr 사용.
NULL 사용 금지.

- C-style cast 금지.
static_cast, reinterpret_cast 명시.

- using namespace 최소화.
cpp에서만 제한적으로 허용.

## 2. Header (.hpp)

- namespace는 반드시:
robot::<category>::<actor_noun>

형태 사용.

예시:
robot::hw::lidar
robot::hw::serial
robot::slam::mapper
robot::map::server

- include 규칙:

외부 헤더:
#include <...>

프로젝트 내부 헤더:
#include "..."

예시:
#include <cstdint>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_lidar_driver/ring_buffer.hpp"

- pragma once 사용.

- 생성자는 explicit 사용.

예시:
explicit LidarDriverNode(const rclcpp::NodeOptions &options);

- 소멸자는 virtual 명시.

예시:
virtual ~LidarDriverNode();

- class 내부 순서 통일:

private:
protected:
public:

순으로 고정.

public 내부 순서:
1. 생성자
2. 소멸자
3. using/type alias
4. public method

예시:

class LidarDriverNode : public rclcpp::Node
{
private:
protected:
public:
	explicit LidarDriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
	virtual ~LidarDriverNode();

	using SharedPtr = std::shared_ptr<LidarDriverNode>;

	void initialize();
};

- 멤버 변수 postfix 규칙:
m_<name> 사용.

예시:
m_serial_port
m_ring_buffer
m_scan_publisher

- bool 변수는:
is_, has_, can_, use_ prefix 사용.

예시:
m_is_connected
m_has_scan
m_use_epoll

- static constexpr는 hpp 상단 class 내부에 정리.

## 3. Source (.cpp)

- cpp include는 반드시 pair 되는 hpp 하나만 include.

좋은 예시:
#include "robot_lidar_driver/lidar_driver_node.hpp"

나쁜 예시:
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "robot_lidar_driver/lidar_driver_node.hpp"

- 외부 헤더 include는 전부 pair hpp에서 처리.

- using namespace는 cpp 최상단에서 제한적으로 사용 가능.

예시:
using namespace robot::hw::lidar;

- 함수 구현도 BSD brace style 유지.

예시:

LidarDriverNode::LidarDriverNode(const rclcpp::NodeOptions &options)
: Node("robot_lidar_driver", options)
{

}

- 긴 initializer list는 개행 허용.

- logging format 통일.

예시:
RCLCPP_INFO(get_logger(), "Opened serial port: %s", m_port.c_str());

- std::cout 사용 금지.
ROS logging만 사용.

- 예외 남발 금지.
recover 가능한 경우 bool/error code 기반 처리 우선.

## 4. ROS2 Style

- Node 이름:
snake_case 사용.

예시:
robot_lidar_driver

- Topic 이름:
snake_case 사용.

예시:
/scan
/debug/raw_packet

- frame_id:
lower_snake_case 사용.

예시:
base_scan
laser_link

- parameter declare/get 분리 명확히.

- publisher/subscriber/service 생성은 initialize() 또는 별도 setup 함수로 정리.

- timer callback, serial callback, parser callback 함수 분리.

- callback 내부에서 긴 로직 직접 처리 금지.
별도 private 함수로 분리.