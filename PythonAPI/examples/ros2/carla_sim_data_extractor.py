# ==================== 주요 라이브러리 임포트 ====================
import os
import argparse
import datetime
import json
import logging
import numpy as np
import carla
from pathlib import Path
import csv

# ROS2 및 센서 메시지 관련 라이브러리
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, PointCloud2
from builtin_interfaces.msg import Time

# ==================== TopicRePublisher 클래스 ====================
# CARLA로부터 받은 센서 데이터를 가공하여 새로운 토픽으로 재퍼블리시하는 ROS2 노드
class TopicRePublisher(Node):
    def __init__(self, fps, simlen, sensor_ticks):
        super().__init__('topic_republisher')

        # 센서주기 및 시뮬레이션 길이 기반 퍼블리쉬 개수 한도 계산
        # +1 : FAST-LIO2는 LIDAR 스캔 종료 시점 이후의 IMU가 입력되어야 그 스캔을 처리함
        # +1 : ROS 메시지 시간과 시뮬레이션 시간 한틱 차이나는 것 보정
        self.total_epoch = int(simlen * fps) + 1 + 1 
        self.limit_cam1 = int(simlen * (1 / sensor_ticks['gray_left']))
        self.limit_cam2 = int(simlen * (1 / sensor_ticks['gray_right']))
        self.limit_lidar = int(simlen * (1 / sensor_ticks['lidar'])) + 1
        self.limit_imu = int(simlen * (1 / sensor_ticks['imuTrue'])) + 1 
        
        # 각 센서별 현재 퍼블리시 횟수 기록
        self.count_cam1 = 0
        self.count_cam2 = 0
        self.count_lidar = 0
        self.count_imu = 0

        # 원본 센서 토픽 구독 (CARLA ROS2 Native에서 제공)
        self.create_subscription(Image, '/carla/raw/gray_left/image', self.cam1_callback, 10)
        self.create_subscription(Image, '/carla/raw/gray_right/image', self.cam2_callback, 10)
        self.create_subscription(PointCloud2, '/carla/raw/lidar', self.lidar_callback, 10)
        self.create_subscription(Imu, '/carla/raw/imuTrue', self.imu_callback, 10)

        # 수정 후 발행할 새로운 토픽 퍼블리셔
        self.pub_cam1 = self.create_publisher(Image, '/carla/sensors/gray_left/image', 10)
        self.pub_cam2 = self.create_publisher(Image, '/carla/sensors/gray_right/image', 10)
        self.pub_lidar = self.create_publisher(PointCloud2, '/carla/sensors/lidar', 10)
        self.pub_imu = self.create_publisher(Imu, '/carla/sensors/imuTrue', 10)

        # 시뮬레이션 상태 변수
        self.sim_time = None # 현재 시뮬레이션 시간
        self.ros_time = None # 현재 ROS 시간
        self.publish_enabled = False # 퍼블리시 허용 여부
        self.timestamp_logs = []  # 모든 센서 메시지 타임스탬프 기록 리스트
        self.current_epoch = -1   # 현재 epoch 인덱스 (Tick 번호)

        self.get_logger().info('Topic Re-Publisher Node Initialized')
    
    # 현재 시뮬레이션 에폭 업데이트
    def set_current_epoch(self, epoch):
        self.current_epoch = epoch    

    # 현재 시뮬레이션 시간 업데이트
    def update_sim_time(self, snapshot_time):
        self.sim_time = snapshot_time

    # 현재 ROS 시간 업데이트 
    def update_ros_time(self, ros_time):
        self.ros_time = ros_time

    def get_ros_time(self):
        return self.ros_time
    
    def adjust_timestamp(self, msg, offset_sec):
        # """ROS 메시지의 타임스탬프를 오프셋 (초 단위)만큼 보정하여 재설정"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + offset_sec
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t - int(t)) * 1e9)

    # 좌측 카메라 콜백 함수
    def cam1_callback(self, msg):
        if self.publish_enabled and self.count_cam1 < self.limit_cam1:
            # 재퍼블리시
            self.adjust_timestamp(msg, 0.0)
            self.pub_cam1.publish(msg)
            self.count_cam1 += 1

            # ROS 시간 계산 및 로그 기록
            ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.timestamp_logs.append(("gray_left", self.count_cam1, ros_time))
            print(f"[PUBLISH @ Epoch: {self.current_epoch+1}/{self.total_epoch}] "
              f"gray_left ({self.count_cam1}/{self.limit_cam1}) @ ROS time: {ros_time:.9f} "
              f"@ Sim time: {self.sim_time:.9f}")
    
    # 우측 카메라 콜백 함수
    def cam2_callback(self, msg):
        if self.publish_enabled and self.count_cam2 < self.limit_cam2:

            self.adjust_timestamp(msg, 0.0)
            self.pub_cam2.publish(msg)
            self.count_cam2 += 1

            ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.timestamp_logs.append(("gray_right", self.count_cam2, ros_time))
            print(f"[PUBLISH @ Epoch: {self.current_epoch+1}/{self.total_epoch}] "
              f"gray_right ({self.count_cam2}/{self.limit_cam2}) @ ROS time: {ros_time:.9f} "
              f"@ Sim time: {self.sim_time:.9f}")

    # 라이다 콜백 함수       
    def lidar_callback(self, msg):
        if self.publish_enabled and self.count_lidar < self.limit_lidar:

            self.adjust_timestamp(msg, -0.1)
            self.pub_lidar.publish(msg)
            self.count_lidar += 1

            ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.timestamp_logs.append(("lidar", self.count_lidar, ros_time))
            print(f"[PUBLISH @ Epoch: {self.current_epoch+1}/{self.total_epoch}] "
              f"lidar ({self.count_lidar}/{self.limit_lidar}) @ ROS time: {ros_time:.9f} "
              f"@ Sim time: {self.sim_time:.9f}")

    # IMU 콜백 함수
    def imu_callback(self, msg):
        if self.publish_enabled and self.count_imu < self.limit_imu:

            self.adjust_timestamp(msg, 1e-9)
            self.pub_imu.publish(msg)
            self.count_imu += 1
            
            ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.timestamp_logs.append(("imu", self.count_imu, ros_time))
            print(f"[PUBLISH @ Epoch: {self.current_epoch+1}/{self.total_epoch}] "
              f"imu ({self.count_imu}/{self.limit_imu}) @ ROS time: {ros_time:.9f} "
              f"@ Sim time: {self.sim_time:.9f}")

# ============================ 차량 스폰 함수 ============================
def setup_vehicle(world, config):
    # """
    # 차량을 맵 위에 생성하고 ROS2 통신을 위한 설정을 적용함.
    # - 차량 블루프린트를 선택하고 role_name/ros_name을 설정
    # - 첫 번째 spawn point에 차량을 생성
    # - 생성 실패 시 예외 발생

    # :param world: CARLA 월드 객체
    # :param config: JSON 설정 파일 내 차량 설정 정보
    # :return: 생성된 차량 액터
    # """
    logging.debug(f"Spawning vehicle: {config.get('type')}")
    bp_library = world.get_blueprint_library()
    bp_list = bp_library.filter(config.get("type"))
    if not bp_list:
        raise RuntimeError(f"No blueprint found for type: {config.get('type')}")
    
    bp = bp_list[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    spawn_point = world.get_map().get_spawn_points()[11]
    actor = world.try_spawn_actor(bp, spawn_point)
    if actor is None:
        raise RuntimeError("Vehicle spawn failed. Possibly due to collision or invalid blueprint.")
    return actor

# ============================ 센서 전체 스폰 함수 ============================
def setup_sensors(world, vehicle, sensors_config):
    # """
    # 차량에 센서를 부착하고 ROS2 퍼블리시를 활성화함.
    # - 센서 ID 및 속성에 따라 blueprint 설정
    # - Roll/Pitch/Yaw 축은 ROS-좌표계에 맞게 부호 변환
    # - 센서 생성 후 enable_for_ros() 호출

    # :param world: CARLA 월드 객체
    # :param vehicle: 차량 액터
    # :param sensors_config: JSON 설정 파일의 센서 목록
    # :return: 생성된 센서 액터 리스트
    # """
    sensors = []
    bp_library = world.get_blueprint_library()

    for sensor in sensors_config:
        logging.debug(f"Spawning sensor: {sensor}")
        bp_list = bp_library.filter(sensor.get("type"))
        if not bp_list:
            logging.warning(f"No blueprint found for sensor type: {sensor.get('type')}")
            continue

        bp = bp_list[0]
        bp.set_attribute("ros_name", sensor.get("id"))
        bp.set_attribute("role_name", sensor.get("id"))

        # 센서 속성 (sensor_tick 등)
        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        # 센서 부착 위치 (CARLA 좌표계 기준)
        spawn = sensor["spawn_point"]
        tf = carla.Transform(
            location=carla.Location(x=spawn["x"], y=-spawn["y"], z=spawn["z"]),
            rotation=carla.Rotation(roll=spawn["roll"], pitch=-spawn["pitch"], yaw=-spawn["yaw"])
        )

        actor = world.try_spawn_actor(bp, tf, attach_to=vehicle)
        if actor:
            actor.enable_for_ros()
            print(f"Publish {actor}")
            sensors.append(actor)

        else:
            logging.warning(f"Failed to spawn sensor {sensor['id']}")
    
    return sensors

def main(args):   
    # """
    # CARLA 시뮬레이터와 ROS 2를 연동하여 차량과 센서 데이터를 생성하고 기록하는 메인 함수.

    # - CARLA 클라이언트 및 맵 초기화
    # - 차량 및 센서 스폰
    # - ROS 2 노드 초기화 및 센서 데이터 재퍼블리싱
    # - 시뮬레이션 루프를 통해 센서 데이터 수집 및 저장
    # """

    # ============================ 시뮬레이션 월드 초기화 함수 ============================
    def initialize_world(client, fps):
        # """
        # CARLA 시뮬레이션을 동기화 모드로 설정하고 고정 시간 간격으로 실행하도록 구성.
        # - synchronous_mode = True
        # - fixed_delta_seconds = 1/fps

        # :param client: carla.Client 객체
        # :param fps: 시뮬레이션 프레임 속도 (Hz)
        # :return: (world, old_settings, map)
        # """
        world = client.get_world()
        old_settings = world.get_settings()
        new_settings = carla.WorldSettings(
            synchronous_mode = True,
            fixed_delta_seconds = 1.0 / fps
        )
        world.apply_settings(new_settings)
        return world, old_settings, world.get_map()
    
    # ============================ ROS2 노드 초기화 함수 ============================
    def initialize_ros_node(fps, simlen, sensors_config):
        # """
        # TopicRePublisher ROS2 노드를 생성하고, ROS2 spin을 별도 스레드로 실행함.
        # - 각 센서 ID별 sensor_tick 추출
        # - TopicRePublisher 노드에 전달

        # :param simlen: 시뮬레이션 길이 (초 단위)
        # :param sensors_config: 센서 설정 리스트
        # :return: (ROS2 노드 객체, 스레드 객체)
        # """
        sensor_ticks = {
            sensor["id"]: float(sensor["attributes"].get("sensor_tick", 0.05))
            for sensor in sensors_config
        }
        rclpy.init()
        node = TopicRePublisher(fps, simlen, sensor_ticks=sensor_ticks)
        # node.use_sim_time = False

        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()
        return node, thread

    # 상태 변수 초기화
    node = world = vehicle = None
    sensors, ros_thread = [], None

    try:
        # 결과 저장 폴더 생성
        Path(args.result_path).mkdir(parents=True, exist_ok=True)

        # CARLA 클라이언트 생성 및 월드 획득
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world, old_settings, carla_map = initialize_world(client, args.fps)

        # Tick 개수 계산 (fps * 시뮬레이션 시간 + 1)
        ticklen = args.simlen * args.fps + 1 + 1
        algntick_len = args.algnlen * args.fps

        # 데이터 기록용 배열 초기화 (tick 수 만큼 row 확보)
        timeArr = np.zeros((ticklen, 2))     # [tick index, sim_time]
        posxyz = np.zeros((ticklen, 3))      # 차량 위치 (x, y, z)
        poslla = np.zeros((ticklen, 3))      # 차량 위치 (lat, lon, alt)
        vel = np.zeros((ticklen, 3))         # 속도
        acc = np.zeros((ticklen, 3))         # 가속도
        pyr = np.zeros((ticklen, 3))         # 자세 (pitch, yaw, roll)
        angvel = np.zeros((ticklen, 3))      # 각속도

        # JSON 센서 설정 로드
        with open(args.file) as f:
            config = json.load(f)

        # 차량 및 트래픽 매니저 설정
        vehicle = setup_vehicle(world, config)
        tm = client.get_trafficmanager()
        tm.set_synchronous_mode(True)
        tm.ignore_lights_percentage(vehicle, 100.0)
        tm.vehicle_percentage_speed_difference(vehicle, 60.0)
        tm.set_random_device_seed(0)

        # 시뮬레이션 워밍업 (센서 초기화 안정화)
        for i in range(500):
            world.tick()
            print(f"Warming up simulation... {i + 1}/500")

        # ROS2 노드 및 스레드 실행         
        node, ros_thread = initialize_ros_node(args.fps, args.simlen, config.get("sensors", []))
        sensors = setup_sensors(world, vehicle, config.get("sensors", []))

        # ============================ 시뮬레이션 루프 시작 ============================
        logging.info("[SIMULATION] Running main loop...")
        for epoch in range(ticklen):
            world.tick()

            # 현재 epoch 및 시뮬레이션 시간 갱신
            node.set_current_epoch(epoch)
            sim_time = world.get_snapshot().timestamp.elapsed_seconds
            node.update_sim_time(sim_time)

            # 첫 epoch에서는 퍼블리셔 활성화
            if epoch == 0:
                print(f"[Initial Sim Time: {sim_time:.9f}]")
                node.publish_enabled = True

            print(f"[Epoch: {epoch+1}] @ Sim Time: {sim_time:.9f}")
            node.timestamp_logs.append(("simulation_tick", int(epoch+1), float(sim_time)))

            # 정지 상태 해제 후 autopilot 활성화
            if epoch == algntick_len + 1:
                vehicle.set_autopilot(True)

            # 차량 위치, 속도, 자세 등 상태 저장
            try:
                transform = vehicle.get_transform()
                velocity = vehicle.get_velocity()
                acceleration = vehicle.get_acceleration()
                angular_velocity = vehicle.get_angular_velocity()
                geo = carla_map.transform_to_geolocation(transform.location)

                timeArr[epoch] = [epoch, sim_time]
                posxyz[epoch] = [transform.location.x, transform.location.y, transform.location.z]
                vel[epoch] = [velocity.x, velocity.y, velocity.z]
                acc[epoch] = [acceleration.x, acceleration.y, acceleration.z]
                pyr[epoch] = [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]
                angvel[epoch] = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
                poslla[epoch] = [geo.latitude, geo.longitude, geo.altitude]

            except Exception as e:
                logging.warning(f"[Epoch {epoch}] Data collection error: {e}")

        # ============================ 결과 저장 ============================

        # true_pva.bin: 전체 PVA 및 자세/속도 정보 이진 저장
        data = np.hstack([timeArr, posxyz, poslla, vel, acc, pyr, angvel])
        data.astype('<f8').tofile(os.path.join(args.result_path, "true_pva.bin"))
        print(f"[SAVE] File: {os.path.join(args.result_path, 'true_pva.bin')}")

        # timestamps_all.csv: 센서 및 시뮬레이션 시간 기록 저장
        sorted_log = sorted(node.timestamp_logs, key=lambda x: x[2])  # 시간 기준 정렬
        with open(os.path.join(args.result_path, "timestamps_all.csv"), mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Sensor", "FrameIndex", "Time(sec)"])
            writer.writerows(sorted_log)

    finally:
        # 종료 및 자원 정리
        if world and old_settings:
            world.apply_settings(old_settings)
        if vehicle:
            vehicle.destroy()
        for sensor in sensors:
            if sensor:
                sensor.destroy()
        if node:
            node.destroy_node()
        rclpy.shutdown()
        if ros_thread:
            ros_thread.join(timeout=5.0)
        print("LOGGING COMPLETE")
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CARLA Data Extractor')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', default=2000, type=int)
    parser.add_argument('--fps', default=200, type=int)
    parser.add_argument('-s', '--simlen', default=200, type=int)
    parser.add_argument('-a', '--algnlen', default=5, type=int)
    parser.add_argument('-f', '--file', default=os.path.join(os.path.dirname(__file__), 'sensors.json'))
    
    now_str = datetime.datetime.now().strftime('%Y-%m-%d_%H%M')
    folder_name = f"binary_{now_str}"
    default_result_path = os.path.expanduser(os.path.join('~/CARLA/Results', folder_name))
    parser.add_argument('--result_path', default=default_result_path)
    
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    try:
        main(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
