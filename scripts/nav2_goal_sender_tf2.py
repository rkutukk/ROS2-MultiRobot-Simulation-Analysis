#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
NAV2 AKSİYON İSTEMCİSİ - GELİŞMİŞ TF2 VE TURTLEBOT3 SENARYOLARI
=================================================================

Bu script, Nav2'ye hedef gönderirken TF2 (Transform Library) gücünü kullanarak
farklı referans çerçevelerinden (frame) dönüşüm yapar.

ÖZELLİK:
--------
Sadece "robotun 2 metre önü" değil, "kameranın baktığı yer", "lidar sensörünün konumu"
veya "odometry başlangıç noktası" gibi farklı çerçevelere göre hedef belirleyebilirsiniz.

DESTEKLENEN SENARYOLAR (TurtleBot3 İçin):
-----------------------------------------
1. BASE_FOOTPRINT: Robotun yere bastığı nokta (Standart navigasyon).
2. CAMERA_LINK: Robotun kamerasına göre hedef (Görsel inceleme/Inspection).
3. BASE_SCAN: Lidar sensörüne göre hedef (Hassas tarama pozisyonu).
4. ODOM: Odometry çerçevesine göre hedef (Başlangıç noktasına dönüş).
5. MAP: Global harita koordinatı.

"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from action_msgs.msg import GoalStatus

# TF2 Kütüphaneleri
import tf2_ros
import tf2_geometry_msgs 
from tf_transformations import quaternion_from_euler 
import math

class AdvancedNav2Client(Node):
    def __init__(self):
        super().__init__('advanced_nav2_client')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF2 Buffer ve Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """RPY -> Quaternion dönüşümü"""
        q = quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def send_goal_from_frame(self, frame_id, x, y, yaw, description=""):
        """
        Belirtilen referans çerçevesine (frame_id) göre hedef gönderir.
        
        Parametreler:
        - frame_id: Hedefin hangi çerçeveye göre tanımlandığı (örn: 'base_footprint', 'camera_link')
        - x, y: O çerçevedeki konum (metre)
        - yaw: O çerçevedeki yönelim (radyan)
        """
        self.get_logger().info(f'\n--- {description} ---')
        self.get_logger().info(f'KAYNAK FRAME: {frame_id} -> HEDEF: X={x}, Y={y}, Yaw={yaw}')

        # 1. Hedefi kaynak çerçevede oluştur
        goal_pose_local = PoseStamped()
        goal_pose_local.header.frame_id = frame_id
        goal_pose_local.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose_local.pose.position.x = float(x)
        goal_pose_local.pose.position.y = float(y)
        goal_pose_local.pose.position.z = 0.0
        goal_pose_local.pose.orientation = self.get_quaternion_from_euler(0.0, 0.0, yaw)

        # 2. Dönüşüm (source_frame -> map)
        try:
            # Transform'un hazır olmasını bekle
            # 'map' hedef çerçevedir. Nav2 'map' çerçevesinde hedef bekler.
            transform = self.tf_buffer.lookup_transform(
                'map',
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Dönüşümü uygula
            goal_pose_map = self.tf_buffer.transform(goal_pose_local, 'map')
            
            self.get_logger().info(f"DÖNÜŞÜM BAŞARILI: {frame_id} -> map")
            self.get_logger().info(f"  Global Hedef: ({goal_pose_map.pose.position.x:.2f}, {goal_pose_map.pose.position.y:.2f})")
            
            # 3. Gönder
            self.send_goal_pose(goal_pose_map)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF2 Dönüşüm Hatası ({frame_id} -> map): {str(e)}')
            self.get_logger().warn('İPUCU: Robot simülasyonu ve TF ağacı (robot_state_publisher) çalışıyor mu?')
            return

    def send_goal_pose(self, pose_stamped):
        """Hazırlanan PoseStamped mesajını gönderir"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 sunucusu bulunamadı!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info('Nav2 Eylem Sunucusuna Gönderiliyor...')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('HEDEF REDDEDİLDİ.')
            return

        self.get_logger().info('HEDEF KABUL EDİLDİ. Robot hareket ediyor...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('BAŞARI: Hedefe ulaşıldı.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('BAŞARISIZ: Görev iptal edildi (Aborted).')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('İPTAL: Görev kullanıcı tarafından iptal edildi.')
        else:
            self.get_logger().error(f'Bilinmeyen Durum: {status}')
        
        # Demo bitti, kapat.
        # rclpy.shutdown() # Seri denemeler için kapatmıyoruz, kullanıcı Ctrl+C yapmalı.

    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    
    client = AdvancedNav2Client()

    # TF buffer'ın dolması için bekleme
    import time
    print("TF Buffer dolması için bekleniyor (2 sn)...")
    time.sleep(2.0) 

    print("\n=================================================")
    print("   TURTLEBOT3 GELİŞMİŞ TF2 NAVİGASYON DEMOSU")
    print("=================================================")
    print("Lütfen kod içindeki senaryolardan birini seçin (uncomment yapın).")
    
    # =================================================================================
    # SENARYO 1: STANDART HAREKET (base_footprint)
    # Robotun yere bastığı noktaya göre 2 metre ileri.
    # En güvenli yöntemdir, çünkü footprint daima yerdedir (z=0).
    # =================================================================================
    # client.send_goal_from_frame('base_footprint', 2.0, 0.0, 0.0, "SENARYO 1: 2m İleri (Footprint)")

    # =================================================================================
    # SENARYO 2: KAMERA İNCELEMESİ (camera_link)
    # "Kameranın 1 metre önündeki nesneye git."
    # Bu, robotun kamerasıyla bir objeye yaklaşması için kullanılır.
    # Not: TurtleBot3'te camera_link genellikle X-ekseni ileri bakar.
    # =================================================================================
    # client.send_goal_from_frame('camera_link', 1.0, 0.0, 0.0, "SENARYO 2: Kameranın 1m Önü")

    # =================================================================================
    # SENARYO 3: LIDAR KONUMLANDIRMA (base_scan)
    # "Lidar sensörünü tam olarak şu konuma (x=1, y=1) getir."
    # Lidar'ın robot merkezine göre ofseti (örn: 10cm geride) otomatik hesaplanır.
    # Robot, lidar o noktaya gelecek şekilde durur.
    # =================================================================================
    # client.send_goal_from_frame('base_scan', 1.0, 1.0, 0.0, "SENARYO 3: Lidar Konumlandırma")

    # =================================================================================
    # SENARYO 4: BAŞLANGIÇ NOKTASINA DÖNÜŞ (odom)
    # Odometry çerçevesindeki (0,0) noktasına git.
    # Robot başladığı yere geri döner.
    # =================================================================================
    client.send_goal_from_frame('odom', 0.0, 0.0, 0.0, "SENARYO 4: Başlangıca Dönüş (Home)")

    # =================================================================================
    # SENARYO 5: YAN YÜRÜYÜŞ (base_footprint)
    # Robotun 1 metre sağına git.
    # =================================================================================
    # client.send_goal_from_frame('base_footprint', 0.0, -1.0, 0.0, "SENARYO 5: 1m Sağa Git")

    # =================================================================================
    # SENARYO 6: GERİ GERİ PARK ETME (base_footprint)
    # 1 metre arkaya git, ama yönün arkaya baksın (180 derece dönüş).
    # =================================================================================
    # client.send_goal_from_frame('base_footprint', -1.0, 0.0, 3.14, "SENARYO 6: Geri Park Et")


    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Hata: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
